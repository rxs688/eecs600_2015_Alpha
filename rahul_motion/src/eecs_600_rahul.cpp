
#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <baxter_core_msgs/JointCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cwru_action/trajAction.h>
#include <eecs600_alpha_pcl_utils/baxter_cart_move_lib.h>
#include <eecs600_alpha_pcl_utils/eecs_600_rahul.h>
#include <blockfinder.h>

using namespace std;
int g_my_states = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PS8_eecs_example"); //node name
    ros::NodeHandle nh;
    cwru_action::trajGoal goal;    
    CwruPclUtils cwru_pcl_utils(&nh);
    ArmMotionCommander arm_motion_commander(&nh);
    
    arm_motion_commander.initalize();
 
    tf::StampedTransform tf_kpc_to_torso_frame; //use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener; //start a transform listener
    bool tferr = true;
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
    //tf-listener found a complete chain from kpc to world;
    ROS_INFO("tf is good"); 
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_kpc_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_kpc_to_torso_frame);
    Eigen::Vector3f plane_normal, slectedCentroid;
    Eigen::Vector3d dp_displacement;
    geometry_msgs::PoseStamped rt_tool_pose;
    double plane_dist, dpp= .01;
    int rtn_val;
    std::vector<int> myindices;
    block_data myBlockData;

    while (ros::ok())
    {
        switch (g_my_states)
        {
            case TAKE_SNAPSHOT:
            {
                 while (!cwru_pcl_utils.got_kinect_cloud())
                 {
                    ROS_INFO("did not receive pointcloud");
                    ros::spinOnce();
                    ros::Duration(1.0).sleep();
                 }
                 ROS_INFO("got a pointcloud");
                 cwru_pcl_utils.save_kinect_snapshot();
                 cwru_pcl_utils.save_kinect_clr_snapshot();
                 g_my_states = COMPUTE_CENTROID;
                 break;
            }
                 
                 // we need to compute the centroid
            case COMPUTE_CENTROID:
            {
                 // Tranform wrt to torso
                 pcl::PointCloud<pcl::PointXYZ> transformed_kinect_points;
                 pcl::PointCloud<pcl::PointXYZRGB> raw_kinect_clr_points;
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_kinect_clr_points_ptr (new pcl::PointCloud<pcl::PointXYZRGB>); ;
                 
                 // Get the Transfromed Kinect point Cloud
                 cwru_pcl_utils.get_transformed_kinect_points(transformed_kinect_points);
                 //Get the Raw Clr Points
                 cwru_pcl_utils.get_kinect_clr_pts(raw_kinect_clr_points);
                 
                 //Copy the Clr data into the transformed Kinect Data
                 int npts = transformed_kinect_points.points.size();
                 transformed_kinect_clr_points_ptr->points.resize(npts);
                 transformed_kinect_clr_points_ptr->header = transformed_kinect_points.header;
                 transformed_kinect_clr_points_ptr->is_dense = transformed_kinect_points.is_dense;
                 transformed_kinect_clr_points_ptr->width = npts;
                 transformed_kinect_clr_points_ptr->height = 1;

                 cout << "copying color data npts =" << npts << endl;
                 for (int i = 0; i < npts; ++i)
                 {
                    transformed_kinect_clr_points_ptr->points[i].getVector3fMap() = 
                                       transformed_kinect_points.points[i].getVector3fMap();
                    transformed_kinect_clr_points_ptr->points[i].r = raw_kinect_clr_points.points[i].r;
                    transformed_kinect_clr_points_ptr->points[i].g = raw_kinect_clr_points.points[i].g;
                    transformed_kinect_clr_points_ptr->points[i].b = raw_kinect_clr_points.points[i].b;
                 }
                 
                 myBlockData = find_the_block(transformed_kinect_clr_points_ptr);
                 // Check if there is valid Data in myBlockData
                 //TODO
                 g_my_states = GOTO_CENTROID;
                 break;
            }
            case GOTO_CENTROID:
            { 
                rtn_val = arm_motion_commander.rt_arm_request_tool_pose_wrt_torso();
                rt_tool_pose = arm_motion_commander.get_rt_tool_pose_stamped();
                //alter the tool pose:
                // Be a little above the Centroid
                rt_tool_pose.pose.position.z = myBlockData.centroid[2]-0.10; 
                rt_tool_pose.pose.position.x = myBlockData.centroid[0];
                rt_tool_pose.pose.position.y = myBlockData.centroid[1];
                // send move plan request:
                rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
                //send command to execute planned motion
                rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
                g_my_states = PICKUP_BLOCK;
                break;
            }
            case PICKUP_BLOCK:
            {
                // Open the Gripper 

               //go down by dp value
               dp_displacement<< 0,0,0.25;
               rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
               if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
               { 
                  //send command to execute planned motion
                  rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
               }
                // Close the Gripper
               g_my_states = MOVE_BLOCK; 
               break;
            }
  
            case MOVE_BLOCK:
            {
                 //go down by dp value
               dp_displacement<< .5,0,-0.25;
               rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
               if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
               { 
                  //send command to execute planned motion
                  rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
               }
                 g_my_states = DROP_BLOCK; 
                 break;
             }

            case DROP_BLOCK:
                 // Open the Gripper
                 g_my_states = GOTO_PREPOSE_FINAL; 
                 break;

            case GOTO_PREPOSE_INIT:
                 rtn_val=arm_motion_commander.plan_move_to_pre_pose(); 
                 //send command to execute planned motion
                 rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
                 g_my_states = GOTO_PREPOSE_FINAL; 
                 break;

            case GOTO_PREPOSE_FINAL:
                 rtn_val=arm_motion_commander.plan_move_to_pre_pose();   
                 //send command to execute planned motion
                 rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
                 g_my_states = TAKE_SNAPSHOT;
                 break;

        } // End of Case statement
        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
    }
    ROS_INFO("my work here is done!");

}

