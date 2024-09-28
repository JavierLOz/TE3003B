Puzzlebot Related FIles

Challenge 1. 
- Run puzzlebot URDF using RVIZ whith animated wheels and moving model

  For runnig the simualtion on rviz2 witht the puzzlebot model run the following line after compiling 
  ```console
  $ ros2 launch robot_tf2_broadcaster puzzlebot_simualtion.launch.xml
  ```
  
  The launch file will initiate the ```robot_state_publisher``` node along side the node ```robot_tf2_broadcaster``` to publsh the joint configuration of the robot.
  The script also runs the node ```robot_pose_estimator``` which recieves a twits command on the topic ```/cmd_vel``` and moves the robot accordingly to the recieved speed command.
  All this is visualized on ```rviz2```. 
