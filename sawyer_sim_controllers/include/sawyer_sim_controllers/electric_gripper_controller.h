/*********************************************************************
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

#ifndef ELECTRIC_GRIPPER_CONTROLLER_H_
#define ELECTRIC_GRIPPER_CONTROLLER_H_

#include <mutex>
#include <ros/node_handle.h>
#include <urdf/model.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_box.h>
#include <intera_core_msgs/IOComponentCommand.h>
#include <intera_core_msgs/IONodeConfiguration.h>
#include <sawyer_sim_controllers/sawyer_joint_position_controller.h>  // used for controlling individual joints

namespace sawyer_sim_controllers
{
  class ElectricGripperController : public controller_interface::Controller<sawyer_hardware_interface::SharedJointInterface>
  {

  public:
    ElectricGripperController();
    virtual ~ElectricGripperController();

    bool init(sawyer_hardware_interface::SharedJointInterface* robot, ros::NodeHandle &n) ;
    void starting(const ros::Time& time){} ;
    void stopping(const ros::Time& time){} ;
    void update(const ros::Time& time, const ros::Duration& period) ;

  private:

    ros::NodeHandle nh_;

    // Last Gripper Command Position
    realtime_tools::RealtimeBox< std::shared_ptr<double> > gripper_command_buffer_;

    // Indices for mimic and orignal joints
    int mimic_idx_;
    int main_idx_;

    size_t n_joints_;
    double start_position_;
    double last_position_;
    // An indicator to evalute to true when an unproccessed new command is in the realtime buffer
    bool new_command_;
    size_t update_counter_;
    intera_core_msgs::IONodeConfiguration config_;

    // Command subscriber
    ros::Subscriber gripper_command_sub_;
    // State publisher
    ros::Publisher gripper_state_pub_;
    // Config publisher
    ros::Publisher gripper_config_pub_;
    // Higher level end effector config publisher
    ros::Publisher end_effector_config_pub_;
    // Timer to update publishers
    ros::Timer pub_timer_;

   /**
    * @brief Regular update for publishing config and state
    * @param event ROS Timer event triggered
    */
    void timerUpdate(const ros::TimerEvent& event);

   /**
    * @brief Retrieves new command and updates both fingers controllers
    */
    void updateCommands();

   /**
    * @brief Construct and publish gripper state based on internal gripper state
    */
    void publishState();

   /**
    * @brief Construct and publish gripper config
    */
    void publishConfig();

    /**
     * @brief Callback from a recieved IO Component command,
     *        storing the new desired internal gripper state
     * @param msg trajectory goal
     */
    void commandCB(const intera_core_msgs::IOComponentCommandConstPtr& msg);

    // Create an effort-based joint position controller for every joint
    std::vector<
      std::shared_ptr<
        sawyer_effort_controllers::JointPositionController> > gripper_controllers_;

    // Class to keep track of command times
    class TimesList{
    public:
        TimesList(size_t max_size){
          max_size_ = max_size;
        };
        bool contains(ros::Time time){
          return std::find(times_.begin(), times_.end(), time) != times_.end();
        };
        void addTime(ros::Time time){
          times_.push_front(time);
          if(times_.size() > max_size_){
            times_.resize(max_size_);
          }
        };
        std::vector<ros::Time> getVector(){
          std::vector<ros::Time> v{ std::begin(times_), std::end(times_) };
          return v;
        };
    private:
        size_t max_size_;
        std::list<ros::Time> times_;
    } cmd_times_list_;

    // Structure to maintain the internal state of gripper signals
    struct GripperSignals{
      bool should_publish;
      std::mutex mutex;
      bool calibrate;
      bool cmd_grip;
      double dead_zone_m;
      double force_response_n;
      bool go;
      bool has_error;
      bool is_calibrated;
      bool is_gripping;
      bool is_moving;
      double position_m;
      double position_response_m;
      bool reboot;
      double right_gripper_tip_object_kg;
      double speed_mps;
    } gripper_signals_;

  };

} // namespace

#endif /* ELECTRIC_GRIPPER_CONTROLLER_H_ */
