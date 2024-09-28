# Puzzlebot Related FIles

## Mini-Challenge 1. 
- Run puzzlebot URDF using RVIZ whith animated wheels and moving model

  For runnig the simualtion on rviz2 with the puzzlebot model run the following line after compiling 
  ```console
  $ ros2 launch robot_tf2_broadcaster puzzlebot_simualtion.launch.xml
  ```
  
  The launch file will initiate the ```robot_state_publisher``` node along side the node ```robot_tf2_broadcaster``` to publsh the joint configuration of the robot.
  
  The script also runs the node ```robot_pose_estimator``` which recieves ```twist``` command on the topic ```/cmd_vel``` and moves the robot accordingly to the recieved speed command.

  The ```twist``` command can be published through a terminal using the command:
    Using autocomplete: 
    ```console
       $ ros2 topic pub /cmd_vel geom<tab> "l <tab>
    ```
  ```console
  $ ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" 

  ```

  All this is visualized on ```rviz2```. 
