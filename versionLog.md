# Version Log

## v1.0.0

_Note: If you have an older version of the package, you may have to do a clean build of the package; several breaking changes in package._

- Added [MoveIt!](https://moveit.ros.org/) and [follow trajectory service](http://wiki.ros.org/joint_trajectory_controller) support.
- Action client and MoveIt! support for Franka Gripper.
- Now fully compatible with [PandaMoveGroupInterface](https://justagist.github.io/franka_ros_interface/DOC.html#pandamovegroupinterface) and [JointTrajectoryActionClient](https://justagist.github.io/franka_ros_interface/DOC.html#jointtrajectoryactionclient) interface classes of the [franka_ros_interface](https://github.com/justagist/franka_ros_interface) and [panda_robot](http://github.com/justagist/panda_robot) packages.
- Several other methods from the [franka_ros_interface](https://github.com/justagist/franka_ros_interface) such as [`move_to_joint_positions`](https://justagist.github.io/franka_ros_interface/DOC.html?highlight=move_to_joint_positions#franka_interface.ArmInterface.move_to_joint_positions) are now available for the simulator (make sure to use the newest version of the interface package).
- Now depends on franka_ros_interface's `franka_moveit` subpackage as well (included in `franka_ros_inteface`).

- **MoveIt! and TrajectoryActionServer usage**:
- The simulator launch file `panda_world.launch` now accepts two additional arguments: `use_custom_action_servers` and `start_moveit`. Both are set to `true` by default, and have to be enabled to be able to use MoveIt! commands and any trajectory action clients (eg. [JointTrajectoryActionClient](https://justagist.github.io/franka_ros_interface/DOC.html#jointtrajectoryactionclient)). To start the simulator without MoveIt! and trajectory action client support (quicker to load), pass as argument `use_custom_action_servers:=false` when launching `panda_world.launch`.
  (_Note: Disabling action service also disables MoveIt! support._)
  _Note: When using launching the simulator `panda_world.launch` with MoveIt!, wait till you see the message `You can start planning now!` before accessing the simulated robot. You can ignore the following error message: 'Exception while loading planning adapter plugin 'default_planner_request_adapters/ResolveConstraintFrames... '_
- A new demo file is added to `panda_simulator_examples`. This can be run using the command `roslaunch panda_simulator_examples demo_moveit.launch` (to disable gripper pass `load_gripper:=false` as argument). This loads the same demo as the `demo_moveit.launch` for the real robot from the franka_ros_interface's `franka_moveit` package. (_Note: If the planning fails, uncheck and check the box next to the display named 'MotionPlanning' in RViz. This seems to be an RViz bug._)

- Other changes:

  - Controller switching is now quicker. Removed redundant controllers and ros parameter loading.
  - Impedance controller and Trajectory controller in the controller manager (switcher) are both 'position_joint_position_controller' in the simulator.
  - Joint position controller gains tuned (very high values; unrealistic) for better trajectory control and position control.
  - Renamed demo file for clarity.
  - Now requires version of 'orocos kdl' package from source.
  - Gravity is now disabled by default for better control when using 'torque controller' and 'velocity controller'.
  - Several other bug fixes.

- Existing/Known Issues:
  - Gravity compensation control is not good with gripper enabled (Fix: disable gravity in simulator (now done by default))

## v0.9

- Joint position, velocity, torque, gravity, gripper controllers fixed
- Controller names specified through parameter server and config file
- Bug in controller switching interface fixed
- Removed redundant impedance controller support
- Compatible with updated [franka_ros_interface](https://github.com/justagist/franka_ros_interface) and [panda_robot](http://github.com/justagist/panda_robot) packages.
