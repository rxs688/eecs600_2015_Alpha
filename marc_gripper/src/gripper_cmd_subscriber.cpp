#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <math.h>

bool gripper_pos = 1; //initialize gripper to full open
void manualCmdCB(const std_msgs::Bool& manual_input) 
{ 
  ROS_INFO("received value of manual_input is: %d",manual_input.data); 
  gripper_pos = manual_input.data;
} 


int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_gripper_subscriber"); 
    ros::NodeHandle n; 

    //std::cout<<"enter motor_id to test: ";
    int motor_id=1; // hard coded for Yale hand
    //std::cin>>motor_id;
    char cmd_topic_name[50];
    sprintf(cmd_topic_name,"dynamixel_motor%d_cmd",motor_id);
    ROS_INFO("using command topic: %s",cmd_topic_name);

    ros::Publisher dyn_pub = n.advertise<std_msgs::Int16>(cmd_topic_name, 1);
    ros::Subscriber manual_cmd_subscriber = n.subscribe("manual_cmd",1,manualCmdCB);     


    std_msgs::Int16 int_angle; 
   double dt = 0.02; // repeat at freq 1/dt
   ros::Rate naptime(1/dt); //create a ros object from the ros “Rate” class; 

    int_angle.data = 3000; // initialize hand at full open

    int cycle_counter = 0;
    
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {

	// Using the input position as a boolean input
	// 1 is open and 0 is close because any input other than 0 and 1 defaults to 1
	// This should cause the hand to open in the event of a malformed command.

	// If input is 1 open hand
	if (gripper_pos == 1)  
	{
	    int_angle.data = 3000;
            ROS_INFO("sending open command");
	}

	// If input is 2 close hand
	else if (gripper_pos == 0)  
	{
	    int_angle.data = 3800;
            ROS_INFO("sending close command");
	}



        dyn_pub.publish(int_angle); 
	ros::spinOnce();	
	naptime.sleep();

    }
}

