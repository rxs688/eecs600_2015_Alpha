
`roslaunch cwru_baxter_sim baxter_world.launch` 
`rosrun baxter_traj_streamer  traj_interpolator_as`
`rosrun baxter_tools enable_robot.py -e`
`roslaunch cwru_baxter_sim kinect_xform.launch`
`roslaunch cwru_baxter_launch yale_gripper_xform.launch`
`rosrun baxter_cartesian_moves baxter_cart_move_as`
`rosrun rviz rviz`
`rosrun kristina_hmi hand_detect_publisher`
`rosrun baxter_gripper dynamixel_motor_node`
`rosrun baxter_gripper baxter_compliant_gripper`
`rosrun eecs600_rahul_pcl_utils eecs600Alpha`

