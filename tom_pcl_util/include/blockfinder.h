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

struct block_data{
	Eigen::Vector3f centroid;
	float r;
	float g;
	float b;
	Eigen::Vector3f major_axis;
	float top_plane_z;
};

const double B_EPS = 0.1;

block_data find_the_block(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud);
Eigen::Vector3f computeCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud);
void normalizeColors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ccloud, int size);

#endif