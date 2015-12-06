Note: The screen images require baxter_examples in order to run. 

To run in Gazebo/RViz:
`roslaunch cwru_baxter_sim baxter_world.launch` 
`rosrun baxter_traj_streamer  traj_interpolator_as`
`rosrun baxter_tools enable_robot.py -e`
`roslaunch cwru_baxter_sim kinect_xform.launch`
`roslaunch cwru_baxter_launch yale_gripper_xform.launch`
`rosrun baxter_cartesian_moves baxter_cart_move_as`
`rosrun rviz rviz`
`rosrun eecs600_rahul_pcl_utils eecs600Alpha`


To run on Baxter:
Run baxter_master on every terminal in which the below listed commands are run 
Make sure to release the E-stop

`rosrun baxter_traj_streamer  traj_interpolator_as`
`rosrun baxter_cartesian_moves baxter_cart_move_as`
`roslaunch cwru_baxter_launch yale_gripper_xform.launch`
`rosrun rviz rviz`
`rosrun kristina_hmi hand_detect_publisher`
`rosrun baxter_gripper dynamixel_motor_node`
`rosrun baxter_gripper baxter_compliant_gripper`
`rosrun baxter_tools enable_robot.py -e`
`rosrun eecs600_rahul_pcl_utils eecs600AlphaRobot`

If you are running on Atlas4 machine please run 
`roslaunch cwru_baxter_launch kinect.launch`
If you are not on Atlas4 machine, make sure the kinect.launch has been run

