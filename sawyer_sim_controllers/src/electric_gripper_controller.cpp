/***************************************************************************
* Copyright (c) 2014, Kei Okada
* Copyright (c) 2018, Rethink Robotics Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/

#include <sawyer_sim_controllers/electric_gripper_controller.h>
#include <intera_core_msgs/IODeviceStatus.h>
#include <intera_core_msgs/IODeviceConfiguration.h>
#include <intera_core_msgs/IODataStatus.h>
#include <intera_core_msgs/IOStatus.h>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>
#include <yaml-cpp/yaml.h>

namespace sawyer_sim_controllers {

ElectricGripperController::ElectricGripperController()
    : new_command_(true),
      update_counter_(0),
      mimic_idx_(0),
      main_idx_(0),
      start_position_(0),
      last_position_(start_position_),
      cmd_times_list_(100){
}

ElectricGripperController::~ElectricGripperController() {
    gripper_command_sub_.shutdown();
}

bool ElectricGripperController::init(sawyer_hardware_interface::SharedJointInterface* robot,
                                ros::NodeHandle &nh) {

  // Store nodehandle
  nh_ = nh;

  // Get joint sub-controllers
  XmlRpc::XmlRpcValue xml_struct;
  if (!nh_.getParam("joints", xml_struct)) {
    ROS_ERROR("No 'joints' parameter in controller (namespace '%s')",
              nh_.getNamespace().c_str());
    return false;
  }

  // Make sure it's a struct
  if (xml_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("The 'joints' parameter is not a struct (namespace '%s')",
              nh_.getNamespace().c_str());
    return false;
  }

  // Get number of joints
  n_joints_ = xml_struct.size();
  ROS_ASSERT_MSG(n_joints_ == 2, "Electric Gripper expects exactly 2 joints and %zu found. Exiting.", n_joints_);
  ROS_INFO_STREAM(
      "Initializing ElectricGripperController with "<<n_joints_<<" joints.");

  gripper_controllers_.resize(n_joints_);
  int i = 0;  // track the joint id
  // Iterate over both joints, and store off the main joint and mimic indeces.
  // We're expecting to find one main joint and one mimic joint.
  // Gazebo ROS interfaces do not currently support mimic joints, so we will
  // take care of them manually.
  for (const auto joint_it : xml_struct) {
    // Get joint controller
    if (joint_it.second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR(
          "The 'joints/joint_controller' parameter is not a struct (namespace '%s')",
          nh_.getNamespace().c_str());
      return false;
    }
    // Get joint controller name
    std::string joint_controller_name = joint_it.first;
    // Get the joint-namespace nodehandle
    {
      ros::NodeHandle joint_nh(nh_, "joints/" + joint_controller_name);
      ROS_INFO_STREAM_NAMED(
          "init",
          "Loading sub-controller '" << joint_controller_name << "', Namespace: " << joint_nh.getNamespace());

      gripper_controllers_[i].reset(
          new sawyer_effort_controllers::JointPositionController());
      gripper_controllers_[i]->init(robot, joint_nh);

    }  // end of joint-namespaces
    // Set mimic indices
    if ( gripper_controllers_[i]->joint_urdf_->mimic ) {
      mimic_idx_ = i;
    }
    else{
      main_idx_ = i;
    }
    // increment joint i
    ++i;
  }
  gripper_command_buffer_.set(std::make_shared<double>(start_position_));
  std::string command_topic_name;
  // Get controller topic name that it will subscribe to
  if (nh_.getParam("topic_command", command_topic_name)) { // They provided a custom topic to subscribe to

    // Get a node handle that is relative to the base path
    ros::NodeHandle nh_base("~");

    // Create command subscriber custom to sawyer
    gripper_command_sub_ = nh_base.subscribe<intera_core_msgs::IOComponentCommand>(
        command_topic_name, 1, &ElectricGripperController::commandCB, this);
  } else  // default "command" topic
  {
    // Create command subscriber custom to intera
    gripper_command_sub_ = nh_.subscribe<intera_core_msgs::IOComponentCommand>(
        "command", 1, &ElectricGripperController::commandCB, this);
  }
  std::string state_topic_name;
  if (nh_.getParam("topic_state", state_topic_name))
  {// They provided a custom topic to subscribe to
    // Get a node handle that is relative to the base path
    ros::NodeHandle nh_base("~");
    // Create command subscriber custom to sawyer
    gripper_state_pub_ = nh_base.advertise<intera_core_msgs::IODeviceStatus>(state_topic_name, 1, true);
  }
  else
  {
    gripper_state_pub_ = nh.advertise<intera_core_msgs::IODeviceStatus>("state", 1, true);
  }

  end_effector_config_pub_ = nh.advertise<intera_core_msgs::IONodeConfiguration>("/io/end_effector/config", 1, true);
  std::string config_topic_name;
  if (nh_.getParam("topic_config", config_topic_name))
  {// They provided a custom topic to subscribe to
    // Get a node handle that is relative to the base path
    ros::NodeHandle nh_base("~");
    // Create command subscriber custom to sawyer
    gripper_config_pub_ = nh_base.advertise<intera_core_msgs::IODeviceConfiguration>(config_topic_name, 1, true);
  }
  else
  {
    gripper_config_pub_ = nh.advertise<intera_core_msgs::IODeviceConfiguration>("config", 1, true);
  }

  // Populate End Effector Config
  config_.time = ros::Time::now();
  config_.node.name = "EndEffector";
  auto device = intera_core_msgs::IOComponentConfiguration();
  device.name = "right_gripper";
  config_.devices.push_back(device);

  // Set gripper signals defaults
  {
    std::lock_guard<std::mutex> lock(gripper_signals_.mutex);
    gripper_signals_.calibrate=false;
    gripper_signals_.cmd_grip=false;
    gripper_signals_.dead_zone_m=0.002;
    gripper_signals_.force_response_n=-2.0;
    gripper_signals_.go=true;
    gripper_signals_.has_error=false;
    gripper_signals_.is_calibrated=true;
    gripper_signals_.is_gripping=false;
    gripper_signals_.is_moving=false;
    gripper_signals_.position_m=start_position_;
    gripper_signals_.position_response_m=start_position_;
    gripper_signals_.reboot=false;
    gripper_signals_.right_gripper_tip_object_kg=0.0;
    gripper_signals_.speed_mps=1.5;
    gripper_signals_.should_publish=true;
  }
  // Update at 5Hz
  pub_timer_ = nh.createTimer(ros::Duration(0.2), &ElectricGripperController::timerUpdate, this);
  publishConfig();
  return true;
}

void ElectricGripperController::timerUpdate(const ros::TimerEvent& event) {
    // Publish at 5Hz
    end_effector_config_pub_.publish(config_);
    double current_position = std::abs(gripper_controllers_[main_idx_]->getPosition())+
                         std::abs(gripper_controllers_[mimic_idx_]->getPosition());
    bool should_publish = false;
    {
      std::lock_guard<std::mutex> lock(gripper_signals_.mutex);
      if(std::abs(current_position - last_position_) > gripper_signals_.dead_zone_m)
      {
        gripper_signals_.position_response_m = current_position;
        gripper_signals_.should_publish = true;
        last_position_ = current_position;
      }
      should_publish = gripper_signals_.should_publish;
    }
    if(should_publish){
      publishState();
    }
}

void ElectricGripperController::publishState() {
    auto gripper_state = intera_core_msgs::IODeviceStatus();

    gripper_state.time = ros::Time::now();
    if(gripper_state.time.sec == 0){
        gripper_state.time.sec = 1; // Hack to make the config valid when starting
    }
    gripper_state.device.name = "right_gripper";
    auto ready_status = intera_core_msgs::IOStatus();
    ready_status.tag = ready_status.READY;
    ready_status.detail = "{}";

    auto calibrate = intera_core_msgs::IODataStatus();
    calibrate.name = "calibrate";
    calibrate.format = "{\"role\":\"output\",\"type\":\"bool\"}";
    calibrate.status = ready_status;

    auto cmd_grip = intera_core_msgs::IODataStatus();
    cmd_grip.name = "cmd_grip";
    cmd_grip.format = "{\"role\":\"output\",\"type\":\"bool\"}";
    cmd_grip.status = ready_status;

    auto dead_zone_m = intera_core_msgs::IODataStatus();
    dead_zone_m.name = "dead_zone_m";
    dead_zone_m.format = "{\"role\":\"output\",\"type\":\"float\",\"units\":\"meters\"}";
    dead_zone_m.status = ready_status;

    auto force_response_n = intera_core_msgs::IODataStatus();
    force_response_n.name = "force_response_n";
    force_response_n.format = "{\"role\":\"input\",\"type\":\"float\"}";
    force_response_n.status = ready_status;

    auto go = intera_core_msgs::IODataStatus();
    go.name = "go";
    go.format = "{\"role\":\"output\",\"type\":\"bool\"}";
    go.status = ready_status;

    auto has_error = intera_core_msgs::IODataStatus();
    has_error.name = "has_error";
    has_error.format = "{\"role\":\"output\",\"type\":\"bool\"}";
    has_error.status = ready_status;

    auto is_calibrated = intera_core_msgs::IODataStatus();
    is_calibrated.name = "is_calibrated";
    is_calibrated.format = "{\"role\":\"output\",\"type\":\"bool\"}";
    is_calibrated.status = ready_status;

    auto is_gripping = intera_core_msgs::IODataStatus();
    is_gripping.name = "is_gripping";
    is_gripping.format = "{\"role\":\"output\",\"type\":\"bool\"}";
    is_gripping.status = ready_status;

    auto is_moving = intera_core_msgs::IODataStatus();
    is_moving.name = "is_moving";
    is_moving.format = "{\"role\":\"output\",\"type\":\"bool\"}";
    is_moving.status = ready_status;

    auto position_m = intera_core_msgs::IODataStatus();
    position_m.name = "position_m";
    position_m.format = "{\"role\":\"output\",\"type\":\"float\",\"units\":\"meters\"}";
    position_m.status = ready_status;

    auto position_response_m = intera_core_msgs::IODataStatus();
    position_response_m.name = "position_response_m";
    position_response_m.format = "{\"role\":\"output\",\"type\":\"float\",\"units\":\"meters\"}";
    position_response_m.status = ready_status;

    auto reboot = intera_core_msgs::IODataStatus();
    reboot.name = "reboot";
    reboot.format = "{\"role\":\"output\",\"type\":\"bool\"}";
    reboot.status = ready_status;

    auto right_gripper_tip_object_kg = intera_core_msgs::IODataStatus();
    right_gripper_tip_object_kg.name = "right_gripper_tip_object_kg";
    right_gripper_tip_object_kg.format = "{\"role\":\"output\",\"type\":\"bool\"}";
    right_gripper_tip_object_kg.status = ready_status;

    auto speed_mps = intera_core_msgs::IODataStatus();
    speed_mps.name = "speed_mps";
    speed_mps.format = "{\"role\":\"output\",\"type\":\"float\",\"units\":\"metersPerSecond\"}";
    speed_mps.status = ready_status;

    {
      std::lock_guard<std::mutex> lock(gripper_signals_.mutex);
      gripper_signals_.should_publish = false;
      cmd_grip.data = "[" + std::string(gripper_signals_.cmd_grip ? "true" : "false") + "]";
      dead_zone_m.data = "[" + std::to_string(gripper_signals_.dead_zone_m) + "]";
      force_response_n.data = "[" + std::to_string(gripper_signals_.force_response_n) + "]";
      go.data = "[" + std::string(gripper_signals_.go ? "true" : "false")  + "]";
      is_calibrated.data = "[" + std::string(gripper_signals_.is_calibrated ? "true" : "false")  + "]";
      is_gripping.data = "[" + std::string(gripper_signals_.is_gripping ? "true" : "false")  + "]";
      is_moving.data = "[" + std::string(gripper_signals_.is_moving ? "true" : "false")  + "]";
      has_error.data = "[" + std::string(gripper_signals_.has_error ? "true" : "false")  + "]";
      position_m.data = "[" + std::to_string(gripper_signals_.position_m) + "]";
      position_response_m.data = "[" + std::to_string(gripper_signals_.position_response_m) + "]";
      right_gripper_tip_object_kg.data = "[" + std::to_string(gripper_signals_.right_gripper_tip_object_kg) + "]";
      speed_mps.data = "[" + std::to_string(gripper_signals_.speed_mps) + "]";
      // Reset cyclic data
      reboot.data = "[" + std::string(gripper_signals_.reboot ? "true" : "false")  + "]";
      if(gripper_signals_.reboot){
        gripper_signals_.reboot = false;
        gripper_signals_.is_calibrated = false;
        gripper_signals_.should_publish = true;
      }
      calibrate.data = "[" + std::string(gripper_signals_.calibrate ? "true" : "false") + "]";
      if(gripper_signals_.calibrate){
        gripper_signals_.calibrate = false;
        gripper_signals_.is_calibrated = true;
        gripper_signals_.should_publish = true;
      }
    }
    gripper_state.signals= {calibrate, cmd_grip, dead_zone_m, force_response_n,
                            go, has_error, is_calibrated, is_gripping,
                            is_moving, position_m, position_response_m, reboot,
                            right_gripper_tip_object_kg, speed_mps};
    gripper_state.commands = cmd_times_list_.getVector();
    gripper_state.responses = std::vector<std::string>(gripper_state.commands.size());
    gripper_state_pub_.publish(gripper_state);
}

void ElectricGripperController::publishConfig(){
    auto gripper_config = intera_core_msgs::IODeviceConfiguration();
    gripper_config.time = ros::Time::now();
    if(gripper_config.time.sec == 0){
        gripper_config.time.sec = 1; // Hack to make the config valid when starting
    }
    gripper_config.device.name = "right_gripper";
    gripper_config.device.config = "{}";
    gripper_config_pub_.publish(gripper_config);
}

void ElectricGripperController::update(const ros::Time& time, const ros::Duration& period)
{
  update_counter_++;
  //TODO: Change to ROS Param (20 Hz)
  if (update_counter_ % 5 == 0) {
    updateCommands();
  }
  // Apply joint commands
  for (size_t i = 0; i < n_joints_; i++) {
    // Update the individual joint controllers
    gripper_controllers_[i]->update(time, period);
  }
}

void ElectricGripperController::updateCommands() {
  // Check if we have a new command to publish
  if (!new_command_)
    return;
  // Go ahead and assume we have proccessed the current message
  new_command_ = false;
  // Get latest command
  std::shared_ptr<double> cmd;
  gripper_command_buffer_.get(cmd);
  if(cmd.get()){
    auto cmd_position = *(cmd.get());
    // Update the individual joint controllers
    ROS_DEBUG_STREAM(gripper_controllers_[main_idx_]->joint_urdf_->name <<
                     "->setCommand(" << cmd_position << ")");
    gripper_controllers_[main_idx_]->setCommand(cmd_position,
        gripper_signals_.speed_mps);
    gripper_controllers_[mimic_idx_]->setCommand(
        gripper_controllers_[mimic_idx_]->joint_urdf_->mimic->multiplier*
        cmd_position+gripper_controllers_[mimic_idx_]->joint_urdf_->mimic->offset,
        gripper_signals_.speed_mps);
  }
}

void ElectricGripperController::commandCB(
    const intera_core_msgs::IOComponentCommandConstPtr& msg) {
  // Save off Command Time if new
  if(!cmd_times_list_.contains(msg->time)){
    cmd_times_list_.addTime(msg->time);
    // Publish Current State
    ROS_DEBUG_STREAM("Gripper update commands " << msg->args);
    YAML::Node args = YAML::Load(msg->args);
    if(args["signals"]["position_m"])
    {
      auto cmd_position = std::min(std::max(args["signals"]["position_m"]["data"][0].as<double>()/2.0,
                                          gripper_controllers_[main_idx_]->joint_urdf_->limits->lower),
                                          gripper_controllers_[main_idx_]->joint_urdf_->limits->upper);
      {
        std::lock_guard<std::mutex> lock(gripper_signals_.mutex);
        gripper_signals_.position_m = cmd_position*2.0;
        gripper_signals_.should_publish=true;
      }
      gripper_command_buffer_.set(std::make_shared<double>(cmd_position));
      new_command_ = true;
    }
    if(args["signals"]["speed_mps"])
    {
      auto cmd_velocity = std::min(std::max(args["signals"]["speed_mps"]["data"][0].as<double>(),
                                            -gripper_controllers_[main_idx_]->joint_urdf_->limits->velocity),
                                            gripper_controllers_[main_idx_]->joint_urdf_->limits->velocity);
      std::lock_guard<std::mutex> lock(gripper_signals_.mutex);
      gripper_signals_.speed_mps=cmd_velocity;
      gripper_signals_.should_publish=true;
    }
    if(args["signals"]["dead_zone_m"])
    {
      std::lock_guard<std::mutex> lock(gripper_signals_.mutex);
      gripper_signals_.dead_zone_m=args["signals"]["dead_zone_m"]["data"][0].as<double>();
      gripper_signals_.should_publish=true;
    }
    if(args["signals"]["go"])
    {
      std::lock_guard<std::mutex> lock(gripper_signals_.mutex);
      gripper_signals_.go=args["signals"]["go"]["data"][0].as<bool>();
      gripper_signals_.should_publish=true;
    }
    if(args["signals"]["calibrate"])
    {
      std::lock_guard<std::mutex> lock(gripper_signals_.mutex);
      gripper_signals_.calibrate=args["signals"]["calibrate"]["data"][0].as<bool>();
      gripper_signals_.should_publish=true;
    }
    if(args["signals"]["reboot"])
    {
      std::lock_guard<std::mutex> lock(gripper_signals_.mutex);
      gripper_signals_.reboot=args["signals"]["reboot"]["data"][0].as<bool>();
      gripper_signals_.should_publish=true;
    }
    if(args["signals"]["right_gripper_tip_object_kg"])
    {
      std::lock_guard<std::mutex> lock(gripper_signals_.mutex);
      gripper_signals_.right_gripper_tip_object_kg=args["signals"]["right_gripper_tip_object_kg"]["data"][0].as<double>();
      gripper_signals_.should_publish=true;
    }
  }
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::ElectricGripperController,
                       controller_interface::ControllerBase)
