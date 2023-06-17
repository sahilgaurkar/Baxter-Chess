# Baxter_Chess
Simulation of Baxter robot playing chess using ROS and Gazebo

Pre-requisite: [baxter-simulator](https://github.com/RethinkRobotics/baxter_simulator)

Steps:

Terminal 1:  
`roslaunch baxter_gazebo baxter_world.launch`

Terminal 2:  
`rosrun baxter_tools enable_robot.py -e`  
`rosrun baxter_interface joint_trajectory_action_server.py`

Terminal 3:  
`roslaunch baxter_moveit_config baxter_grippers.launch`

Terminal 4:  
`rosrun coursework spawn_chessboard.py`
`rosrun coursework gazebo2tfframe.py`

Terminal 5:  
`rosrun coursework pick_and_place_moveit.py`

To delete Chessgame:  
`rosrun coursework delete_chessgame.py`


[Simulation Video](https://youtu.be/i8kd_YJA06E)
