This is a basic gripper package for operating the Yale open hand on the Baxter robot

The gripper_cmd_subscriber node looks for a boolean input and then commands the gripper_motor_node to either 3000 (OPEN) or 3800 (CLOSED).



To operate run the two following commands:

rosrun marc_gripper gripper_motor_node

rosrun marc_gripper gripper_cmd_subscriber



To manually command the gripper to open or close use the following commands:

rostopic pub manual_cmd std_msgs/Bool 0

rostopic pub manual_cmd std_msgs/Bool 1



1 is open and 0 is close because any input other than 0 and 1 defaults to 1
This should cause the hand to open in the event of a malformed command.
