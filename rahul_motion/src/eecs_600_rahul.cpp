
#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <baxter_core_msgs/JointCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cwru_action/trajAction.h>
#include <eecs600_alpha_pcl_utils/baxter_cart_move_lib.h>
#include <blockfinder.h>

using namespace std;


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

    //let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between torso and kinect PC frame...");
    while (tferr)
    {
        tferr = false;
        try
        {
            tf_listener.lookupTransform("torso","kinect_pc_frame", ros::Time(0), tf_kpc_to_torso_frame);
        }
        catch (tf::TransformException &exception)
        {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from kpc to world;
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_kpc_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_kpc_to_torso_frame);
    Eigen::Vector3f plane_normal, slectedCentroid;
    Eigen::Vector3d dp_displacement;
    geometry_msgs::PoseStamped rt_tool_pose;
    double plane_dist, dpp= .01;
    int rtn_val;
    std::vector<int> myindices;
    while (ros::ok())
    {
        if (cwru_pcl_utils.got_selected_points())
        {
            // go to Pre pose, hhere the Robots tool is facing down
            rtn_val=arm_motion_commander.plan_move_to_pre_pose();
            rtn_val=arm_motion_commander.rt_arm_execute_planned_path();

            ROS_INFO("transforming selected points");
            cwru_pcl_utils.transform_selected_points_cloud(A_kpc_wrt_torso);
            cwru_pcl_utils.reset_got_selected_points();
            cwru_pcl_utils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist);
            // you get the centroid only after plane normal and plane distance is computed
            slectedCentroid = cwru_pcl_utils.get_centroid();
            ROS_INFO(" Centroid Point : %f , %f, %f ", slectedCentroid[0], slectedCentroid[1], slectedCentroid[2]);
            rtn_val = arm_motion_commander.rt_arm_request_tool_pose_wrt_torso();
            rt_tool_pose = arm_motion_commander.get_rt_tool_pose_stamped();
            //alter the tool pose:
            rt_tool_pose.pose.position.z = slectedCentroid[2]; 
            rt_tool_pose.pose.position.x = slectedCentroid[0];
            rt_tool_pose.pose.position.y = slectedCentroid[1];
            // send move plan request:
            rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
            //send command to execute planned motion
            rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
            for (int i =0; i< 5; i++)
            {
               // Now execute the sweeping path
               dp_displacement<< -.25,0,0.0;
               rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
               if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
               { 
                  //send command to execute planned motion
                  rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
               }
               // Now execute the sweeping path
               dp_displacement<< 0,0.01,0.0;
               rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
               if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
               { 
                   //send command to execute planned motion
                   rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
                }
               // Now execute the sweeping path
               dp_displacement<< .25,0.01,0.0;
               rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
               if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
               { 
                  //send command to execute planned motion
                  rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
               }
            }
            rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    
            //send command to execute planned motion
            rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    
        }
        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
    }
    ROS_INFO("my work here is done!");

}

