#include "blockfinder.cpp"
#include <cwru_pcl_utils/cwru_pcl_utils.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_two");
	ros::NodeHandle nh;

	tf::StampedTransform tf_sensor_frame_to_torso_frame; //need objects of this type to hold tf's
	tf::TransformListener tf_listener;
	tf::TransformListener * g_tfListenerPtr = &tf_listener;
	bool tferr = true;
	ROS_INFO("waiting for tf between kinect_pc_frame and world...");
	while (tferr) {
		tferr = false;
		try {
			//The direction of the transform returned will be from the target_frame to the source_frame. 
			//Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
			tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
		}
		catch (tf::TransformException &exception) {
			ROS_ERROR("%s", exception.what());
			tferr = true;
			ros::Duration(0.5).sleep(); // sleep for half a second
			ros::spinOnce();
		}
	}
	Eigen::Affine3f A= cwru_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
	ROS_INFO("tf is good");
}