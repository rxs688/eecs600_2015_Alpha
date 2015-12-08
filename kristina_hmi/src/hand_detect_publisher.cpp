
#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Bool.h>
#include "cwru_pcl_utils.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_detect_publisher");
  ROS_INFO("New node based on minimal_publisher node.");


ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    CwruPclUtils cwru_pcl_utils(&n); //Subscribe to Kinect cloud
    double hand_height, htOfTable;

    tf::StampedTransform tf_kpc_to_torso_frame; //transform sensor frame to torso frame
    tf::TransformListener tf_listener;          //start a transform listener
    bool tferr = true;

        //------Subscribe to Tranform Listener-------------
    ROS_INFO("waiting for tf between torso and kinect PC frame...");
    while (tferr)
    {
        tferr = false;
        try
        {
            tf_listener.lookupTransform("torso","camera_rgb_optical_frame", 
                                        ros::Time(0), 
                                        tf_kpc_to_torso_frame);
        }
        catch (tf::TransformException &exception)
        {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    //tf-listener found a complete chain from kinect pc frame to Torso;
    ROS_INFO("tf is good"); 
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_kpc_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_kpc_to_torso_frame);
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Bool>("hand_detected", 1);
    std_msgs::Bool hand_detected; //create a variable  
   
   
   ros::Rate naptime(50.0); //create a ros object from the ros “Rate” class; 
   //set the sleep timer for 100Hz repetition rate (arg is in units of Hz)

    hand_detected.data = false;
    //Based on the minimal_publisher node provided. 

    //TODO: Set value for height at which hand is considered detected. 
    htOfTable = -0.12;
    hand_height= htOfTable + 0.12; 
    while (ros::ok())
    {
		while (!cwru_pcl_utils.got_kinect_cloud())
                {
			//ROS_INFO("did not receive pointcloud");
 			ros::spinOnce();
			ros::Duration(1).sleep();
		}
		ROS_INFO("got a kinect pointcloud");
		int point_count=0;
		cwru_pcl_utils.transform_kinect_cloud(A_kpc_wrt_torso);
            
		// Tranform wrt to torso
		pcl::PointCloud<pcl::PointXYZ> transformed_kinect_points;

		// Get the Transfromed Kinect point Cloud
		cwru_pcl_utils.get_transformed_kinect_points(transformed_kinect_points); //Point cloud
		// this loop has no sleep timer, and thus it will consume excessive CPU time
                // expect one core to be 100% dedicated (wastefully) to this small task
		int npts = transformed_kinect_points.width * transformed_kinect_points.height;
		int bcount = 0;
                 
      
		for(int i = 0; i < npts; i++)
                {
			if((transformed_kinect_points.points[i].z - hand_height) < 0.10)
                        {
          				point_count++;
                        }		
          	}
		//ROS_INFO("%i points at hand height detected.", point_count);
                if(point_count>100)
                { // will set number based on RVIZ output  
                   hand_detected.data=true;
                   //ROS_INFO("SELECTED POINT %f", seedpoint.z);
                }
                else
                {
                     hand_detected.data=false;
                }
                ROS_INFO("Hand detected is %i", hand_detected.data);
                my_publisher_object.publish(hand_detected); // publish the value--of type bool
                //to the topic "topic1"
                //the next line will cause the loop to sleep for the balance of the desired period 
                // to achieve the specified loop frequency 
                ros::Duration(3).sleep(); // sleep for half a second
                ros::spinOnce();
                cwru_pcl_utils.reset_got_kinect_cloud();
	}
        ROS_INFO("Hand detected is Done");
}
