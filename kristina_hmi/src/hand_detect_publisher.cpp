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
    double hand_height;

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
            tf_listener.lookupTransform("torso","kinect_pc_frame", 
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
    // as defined in: /opt/ros/indigo/share/std_msgs
    // any message published on a ROS topic must have a pre-defined format, 
    // so subscribers know how to interpret the serialized data transmission
   
   ros::Rate naptime(100.0); //create a ros object from the ros “Rate” class; 
   //set the sleep timer for 100Hz repetition rate (arg is in units of Hz)

    hand_detected.data = false;
    //Based on the minimal_publisher node provided. 

    hand_height=.25; //TODO: Set value for height at which hand is considered detected. 

    while (ros::ok())
    {
        
                 while (!cwru_pcl_utils.got_kinect_cloud())
                 {
                    ROS_INFO("did not receive pointcloud");
                    ros::spinOnce();
                    ros::Duration(1.0).sleep();
                 }
                 ROS_INFO("got a kinect pointcloud");
                 cwru_pcl_utils.transform_kinect_cloud(A_kpc_wrt_torso);
                 //cwru_pcl_utils.save_kinect_snapshot();     not needed for now
                 //cwru_pcl_utils.save_kinect_clr_snapshot(); not needed for now
                 //cwru_pcl_utils.save_transformed_kinect_snapshot();
                 
            
                 // Tranform wrt to torso
                 pcl::PointCloud<pcl::PointXYZ> transformed_kinect_points;

                 // Get the Transfromed Kinect point Cloud
                 cwru_pcl_utils.get_transformed_kinect_points(transformed_kinect_points); //Point cloud
                   // this loop has no sleep timer, and thus it will consume excessive CPU time
        // expect one core to be 100% dedicated (wastefully) to this small task
      float max_z = -DBL_MAX;
      int npts = transformed_kinect_points.width * transformed_kinect_points.height;
      pcl::PointXYZ seedpoint;
      int bcount = 0;
      
      for(int i = 0; i < npts; i++)
            {
          if(transformed_kinect_points.points[i].z > max_z)
                {
                seedpoint = transformed_kinect_points.points[i];
                max_z = seedpoint.z;
          }
      }
      if(max_z>hand_height){
        hand_detected.data=true;
        ROS_INFO("SELECTED POINT %f", seedpoint.z);
      } else {hand_detected.data=false;}
        

        my_publisher_object.publish(hand_detected); // publish the value--of type bool
        //to the topic "topic1"
  //the next line will cause the loop to sleep for the balance of the desired period 
        // to achieve the specified loop frequency 
  naptime.sleep();
  ros::spinOnce();
}



}