#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>


#ifndef PCL_BLOCKFIND_H_
#define PCL_BLOCKFIND_H_

enum block_color{
	BLOCK_RED = 0,
	BLOCK_GREEN,
	BLOCK_BLUE,
	BLOCK_WHITE,
	BLOCK_WOOD,
	BLOCK_BLACK,
	BLOCK_CONFUSED
};

const float PERFECT_RED[3] =	{255, 0.0, 0.0};
const float PERFECT_GREEN[3] =	{0.0, 255, 0.0};
const float PERFECT_BLUE[3] =	{0.0, 0.0, 255};
const float PERFECT_WHITE[3] =	{255, 255, 255};
const float PERFECT_WOOD[3] =	{1.0, 1.0, 0.0};
const float PERFECT_BLACK[3] =	{0.0, 0.0, 0.0};

const float PERFECT_COLORS[6][3] = {{255, 0.0, 0.0}, {0.0, 255, 0.0}, {0.0, 0.0, 255}, {255, 255, 255}, {1.0, 1.0, 0.0}, {0.0, 0.0, 0.0}};

ros::Publisher xdisplay_pub;
sensor_msgs::ImagePtr baxter_red, baxter_yellow, baxter_blue;

struct block_data
{
	Eigen::Vector3f centroid;

        Eigen::Vector3f norm_color;
	float r;
	float g;
	float b;

	Eigen::Vector3f color_avg;
	block_color color_name;

	Eigen::Vector3f major_axis;
	float top_plane_z;
};

const double B_EPS = 0.1;
const double CONFUSION_TOLERANCE = 0.5;//Will need to be experimentally tuned.

block_data find_the_block(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud);

//const double AOI_X = 0.7;
//const double AOI_Y = 0.0;
//const double AOI_Z = 



block_data find_the_block(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, ros::NodeHandle n);

#endif
