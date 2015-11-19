
`roslaunch cwru_baxter_sim baxter_world.launch` 
`rosrun baxter_traj_streamer  traj_interpolator_as`
`rosrun baxter_tools enable_robot.py -e`
`roslaunch cwru_baxter_sim kinect_xform.launch`
`roslaunch cwru_baxter_launch yale_gripper_xform.launch`
`rosrun baxter_cartesian_moves baxter_cart_move_as`
`rosrun rviz rviz`
`rosrun cwru_pcl_utils ps8_eecs600`

Then select a patch of points in the rviz view.
baxter will execute a sweeping motion.
Becasue the gravity compensation is off, I think the hand slowly droops and the 
last move forward goes below the table and drags across the table as it moves forward. 


    
