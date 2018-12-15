## cyton_gamma_hardware
Includes hardware drivers for the dynamixel motors 

Syntax for hardware: 
> roslaunch cyton_1500_controllers controller_manager.launch

> roslaunch cyton_1500_controllers start_controller.launch

> rosrun cyton_1500_controllers dynamixel_joint_state_publisher.py

> roslaunch cyton_gamma_1500_moveit_config moveit_planning_execution.launch 2>/dev/null

Then, an optional front-end, either a combined front-end or the separate command and feedback ones.

> rosrun cyton_gamma_pkg combined_front_end.py 2>/dev/null 1500

> rosrun cyton_gamma_pkg command_front_end.py 2>/dev/null 500

> rosrun cyton_gamma_pkg feedback_front_end.py 2>/dev/null 1500

OR Use this shell script, though currently not totally stable. If you try twice and doesn't work, go back to manual launching. Something in the dynamixel drivers doesn't make this happy. In cyton_gamma_pkg/src.

> ./500_hardware_launch.sh

## cyton_gamma_simulation
Syntax for Simulation: 
> roslaunch cyton_gamma_pkg simulation_gamma_1500.launch 

Optionally you can launch the two front-end Qt applications and they work in simulation as well!

This will start up the moveit client with RVIZ command center and simulation output in physics simulator (Gazebo was chosen for this application) 

These have been tested and allow cartesian, joint space, and force control. The gripper action controller is directly commanded since the internal dynamixel controllers do a fine job on a single axis joint. 

## Standup.sh
> standup.sh

This script will move the robot to a zero position. It's convenient for testing purposes and the same syntax can be used to command the robot directly with rostopic pub. 
