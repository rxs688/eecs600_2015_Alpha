#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <baxter_core_msgs/JointCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cwru_action/trajAction.h>
#include <eecs600_alpha_pcl_utils/baxter_cart_move_lib.h>
#include <eecs600_alpha_pcl_utils/eecs_600_rahul.h>
#include <blockfinder.h>

using namespace std;
int g_my_states = GOTO_PREPOSE_INIT;
int g_my_prevState = g_my_states;
bool hand_state_saved = 0;
int g_count =0;

void HandDetectioncb(const std_msgs::Bool& result)
{
  ROS_INFO("HandDetectioncb: server responded " );
  if ((result.data !=  hand_state_saved) && 
     (g_my_states == GOTO_CENTROID) &&
     (g_my_states == PICKUP_BLOCK)&&
     (g_my_states == DROP_BLOCK) &&
     (g_my_states == MOVE_BLOCK))
  {
      hand_state_saved = result.data;
      if (result.data) // hand is there
      {
         //g_my_prevState = g_my_states;
         //g_my_states = GOTO_IDLE_WAIT;
         system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/share/images/baxter_hold.png");
         ROS_INFO("Hand detection triggered.");
      }
      else 
      {
          //g_my_states = g_my_prevState;
          //g_my_prevState = g_my_states;
          ROS_INFO("Hand is not detected.");
      }
      
  }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "eecs600_Alpha"); //node name
    ros::NodeHandle nh;
    tf::StampedTransform tf_kpc_to_torso_frame; //transform sensor frame to torso frame
    tf::StampedTransform tf_right_hand_wrt_torso;
    tf::TransformListener tf_listener;          //start a transform listener
    bool tferr = true;
    Eigen::Vector3f plane_normal, slectedCentroid;
    Eigen::Vector3d dp_displacement;
    geometry_msgs::PoseStamped rt_tool_pose;
    double plane_dist, dpp= .01;
    int rtn_val;
    std::vector<int> myindices;
    block_data myBlockData;
        system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/share/images/baxter_orange.png");

    // subscribe to Kinect point cloud data  
    CwruPclUtils cwru_pcl_utils(&nh);
    ArmMotionCommander arm_motion_commander(&nh);

    //------Publish to the Gripper listner-----------------
    ros::Publisher gripper_cmd = nh.advertise< std_msgs::Bool>("gripper_cmd", 1);
    // Listen to Hand detection Messages
    ros::Subscriber my_subscriber_object= nh.subscribe("hand_detected",1,HandDetectioncb); 

    // Wait for cart move action client to initialize
    arm_motion_commander.initalize();     
    // attempt to connect to the server:
    
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
    
    
    tferr = true;
    ROS_INFO("waiting for tf between yale gripper frame and torso...");
    while (tferr)
    {
        tferr = false;
        try
        {
            tf_listener.lookupTransform("torso", "yale_gripper_frame", ros::Time(0), tf_right_hand_wrt_torso);

        } catch (tf::TransformException &exception) {
            ROS_WARN("%s", exception.what());
            tferr = true;
            ros::Duration(0.1).sleep(); // sleep briefly
            ros::spinOnce();
        }
    }
    ROS_INFO("got gripper tf");
    Eigen::Affine3f A_yale_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_right_hand_wrt_torso);

    while (ros::ok())
    {
        switch (g_my_states)
        {
            case GOTO_PREPOSE_INIT:
            {
                 ROS_INFO("State : GOTO_PREPOSE_INIT:"); 
                 rtn_val=arm_motion_commander.plan_move_to_pre_pose(); 
                 //send command to execute planned motion
                 rtn_val=arm_motion_commander.rt_arm_execute_planned_path();

                 // Go to state of take snapshot
                 g_my_states = TAKE_SNAPSHOT; 
                 break;
            }

            case TAKE_SNAPSHOT:
            {
                 ROS_INFO("State : TAKE_SNAPSHOT:"); 
                 while (!cwru_pcl_utils.got_kinect_cloud())
                 {
                    ROS_INFO("did not receive pointcloud");
                    ros::spinOnce();
                    ros::Duration(1.0).sleep();
                 }
                 ROS_INFO("got a kinect pointcloud");
                 cwru_pcl_utils.transform_kinect_cloud(A_kpc_wrt_torso);

                 cwru_pcl_utils.save_transformed_kinect_snapshot();
                 g_my_states = COMPUTE_CENTROID;
                 break;
            }
                 
            // we need to compute the centroid
            case COMPUTE_CENTROID:
            {
                 ROS_INFO("State : COMPUTE_CENTROID"); 
                 std::vector< int > my_index ;
                 // Tranform wrt to torso
                 pcl::PointCloud<pcl::PointXYZ> transformed_kinect_points;
                 pcl::PointCloud<pcl::PointXYZRGB> raw_kinect_clr_points;
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_kinect_clr_points_ptr (
                                                     new pcl::PointCloud<pcl::PointXYZRGB>); ;
                 
                 // Get the Transfromed Kinect point Cloud
                 cwru_pcl_utils.get_transformed_kinect_points(transformed_kinect_points);
                 //Get the Raw Color Points
                 cwru_pcl_utils.get_kinect_clr_pts(raw_kinect_clr_points);
                 // Remove NAN points  
                 pcl::removeNaNFromPointCloud 	( transformed_kinect_points,my_index ) ;

                 //Copy the Clr data into the transformed Kinect Datacd 
                 int npts = transformed_kinect_points.points.size();
                 int npts1 = my_index.size();
                 ROS_INFO(" Points befor NAN removal %d, After NAN Removal: %d",npts, npts1);

                 transformed_kinect_clr_points_ptr->points.resize(npts1);
                 transformed_kinect_clr_points_ptr->header = transformed_kinect_points.header;
                 transformed_kinect_clr_points_ptr->is_dense = transformed_kinect_points.is_dense;
                 transformed_kinect_clr_points_ptr->width = npts1;
                 transformed_kinect_clr_points_ptr->height = 1;

                 cout << "copying color data npts =" << npts << endl;
                 for (int i = 0; i < npts1; ++i)
                 {
                    transformed_kinect_clr_points_ptr->points[i].getVector3fMap() = 
                                       transformed_kinect_points.points[my_index[i]].getVector3fMap();
                    transformed_kinect_clr_points_ptr->points[i].r = raw_kinect_clr_points.points[my_index[i]].r;
                    transformed_kinect_clr_points_ptr->points[i].g = raw_kinect_clr_points.points[my_index[i]].g;
                    transformed_kinect_clr_points_ptr->points[i].b = raw_kinect_clr_points.points[my_index[i]].b;
                 }
                 
                 myBlockData = find_the_block(transformed_kinect_clr_points_ptr,nh);
                 // Check if there is valid Data in myBlockData
                 //TODO
                 g_my_states = GOTO_CENTROID;
                 break;
            }
            case GOTO_CENTROID:
            { 
                ROS_INFO("State : GOTO_CENTROID"); 
                Eigen::Vector3f yalePosition;
                rtn_val = arm_motion_commander.rt_arm_request_tool_pose_wrt_torso();
                rt_tool_pose = arm_motion_commander.get_rt_tool_pose_stamped();

                yalePosition = A_yale_wrt_torso * myBlockData.centroid;
                ROS_INFO ("Centroid : %f, %f, %f ",myBlockData.centroid[0], myBlockData.centroid[1], myBlockData.centroid[2]); 
                ROS_INFO ("Gripper : %f, %f, %f ",yalePosition [0], yalePosition [1],yalePosition [2]);
                 //Adding Offsets
                /*myBlockData.centroid[0] = myBlockData.centroid[0]+.0605;
                myBlockData.centroid[1] = myBlockData.centroid[1]-.079;*/
                myBlockData.centroid[2] = myBlockData.centroid[2]+.10;
                ROS_INFO ("Centroid_offsets : %f, %f, %f ",myBlockData.centroid[0], myBlockData.centroid[1], myBlockData.centroid[2]); 
                //alter the tool pose:
                // Be a little above the Centroid 
                 
                rt_tool_pose.pose.position.x = myBlockData.centroid[0];//0.573503;
                rt_tool_pose.pose.position.y = myBlockData.centroid[1];//-0.181500;
                rt_tool_pose.pose.position.z = myBlockData.centroid[2]; //-0.015773;
				rt_tool_pose.pose.orientation.x = 0;
				rt_tool_pose.pose.orientation.y = 1;
				rt_tool_pose.pose.orientation.y = 0;
				rt_tool_pose.pose.orientation.w = 0;
				rt_tool_pose.header.frame_id = "yale_gripper_frame";
				
                // send move plan request:
                rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
                //send command to execute planned motion
                rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
                g_my_states = PICKUP_BLOCK;
                break;
            }
            case PICKUP_BLOCK:
            {
               ROS_INFO("State : PICKUP_BLOCK"); 
               std_msgs::Bool close_grip;
               close_grip.data = 1;
               gripper_cmd.publish(close_grip); 
               ros::Duration(2.1).sleep(); // sleep for half a second
               
               g_my_states = MOVE_BLOCK; 
               break;
            }
  
            case MOVE_BLOCK:
            {
               ROS_INFO("State : MOVE_BLOCK"); 
                rtn_val = arm_motion_commander.rt_arm_request_tool_pose_wrt_torso();
                rt_tool_pose = arm_motion_commander.get_rt_tool_pose_stamped();

               if(myBlockData.color_name == BLOCK_RED)
               {
                   ROS_INFO("PICKUP_BLOCK RED ");
                   rt_tool_pose.pose.position.x = rt_tool_pose.pose.position.x + .25;
                   rt_tool_pose.pose.position.z = rt_tool_pose.pose.position.z + 0.25;
               }
               else if(myBlockData.color_name == BLOCK_GREEN)
               {
                   ROS_INFO("PICKUP_BLOCK GREEN ");
                   rt_tool_pose.pose.position.y = rt_tool_pose.pose.position.y- .25;
                   rt_tool_pose.pose.position.z = rt_tool_pose.pose.position.z + 0.5;
               }
               else if( myBlockData.color_name == BLOCK_BLUE)
               {
                   ROS_INFO("PICKUP_BLOCK BLUE ");
                   rt_tool_pose.pose.position.x = rt_tool_pose.pose.position.x + .25;
                   rt_tool_pose.pose.position.y = rt_tool_pose.pose.position.y + .25;
                   rt_tool_pose.pose.position.z = rt_tool_pose.pose.position.z + 0.35;
               }  
                rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
                //send command to execute planned motion
                rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
                 g_my_states = DROP_BLOCK; 
                 break;
             }

            case DROP_BLOCK:
             {
                ROS_INFO("State : DROP_BLOCK"); 
                 // Open the Gripper
                 std_msgs::Bool close_grip;
                  close_grip.data = 0;
                  gripper_cmd.publish(close_grip); 
                 ros::Duration(2.0).sleep(); // sleep for half a second
                 g_my_states = GOTO_PREPOSE_FINAL; 
                 break;
             }

            case GOTO_PREPOSE_FINAL:
                 ROS_INFO("State :GOTO_PREPOSE_FINAL"); 
                 rtn_val=arm_motion_commander.plan_move_to_pre_pose();   
                 //send command to execute planned motion
                 rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
                 g_my_states = IAM_DONE_HERE; 
                 break;
            case IAM_DONE_HERE:

            case GOTO_IDLE_WAIT:
                ROS_INFO("State :GOTO_IDLE_WAIT"); 
                // Wait here till hand is removed
                break;

        } // End of Case statement
        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
        
        // we are done here, exit the code
        if(g_my_states == IAM_DONE_HERE) break;
    }
    ROS_INFO("my work here is done!");

}

