#include "../include/blockfinder.h"
#include <vector>
#include <stdlib.h>
using namespace std;

Eigen::Vector3f computeCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, vector<int> &indices)
 {
    Eigen::Vector3f centroid;
    centroid << 0, 0, 0;

    int size = indices.size();
    std::cout << "frame: " << pcl_cloud->header.frame_id << "size : " << size << std::endl;
    for (size_t i = 0; i < size; ++i)
    {
        centroid += pcl_cloud->points[indices[i]].getVector3fMap();
    }
    if (size > 0)
    {
        centroid /= ((float) size);
    }
    return centroid;
}

void normalizeColors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr subject_cloud, vector<int> &indices)
{
    int size = indices.size();
    for(int i = 0; i < size; i++)
    {
	pcl::PointXYZRGB * p = &(subject_cloud->points[indices[i]]);
	float n = sqrt(pow(p->r, 2) + pow(p->g, 2) + pow(p->b, 2));
	if(n != 0.0)
        {
	    p->r = p->r / n;
	    p->g = p->g / n;
	    p->b = p->b / n;
        }
    }
}

block_data find_the_block(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud)
 {
	//I would recommend prforming blockfinding AFTER we have already checked for a hand stopsignal or other Very High Points.
	//You will be checking for errors in there first,
	//so IN THEORY we shouldn't have to worry about them causing false positives here.
	//However, I may come back and implement that functionality here as well.
	//Also, since it is very much a point-cloud-related problem, feel free to email/text/whatever
	//me if you want me to do the outlier checking or anything else for the HMI.
	
	float max_z = -DBL_MAX;
	int npts = inputCloud->width * inputCloud->height;
	pcl::PointXYZRGB seedpoint;
	int bcount = 0;
	
	for(int i = 0; i < npts; i++)
        {
	    if(inputCloud->points[i].z > max_z)
            {
		seedpoint = inputCloud->points[i];
		max_z = seedpoint.z;
	    }
	}
	
	ROS_INFO("LOCATED BLOCK OF HEIGHT %f.", max_z);
	
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr block_candidate_points(
        //                                new pcl::PointCloud<pcl::PointXYZRGB>);
        vector<int> indices;
        indices.clear();
	for(int i = 0; i < npts; i++)
        {
           if(abs(inputCloud->points[i].z - max_z) < B_EPS)
           {
               indices.push_back(i);
	   }
	}
        int npts1 = indices.size();
	ROS_INFO("No of points saved in Block Count %d ",npts1 );
	Eigen::Vector3f center = computeCentroid(inputCloud, indices);
	
	normalizeColors(inputCloud, indices);
	
	float c_avg_r = 0.0;
	float c_avg_g = 0.0;
	float c_avg_b = 0.0;
	float xx = 0;
	float yy = 0;
	float xy = 0;
	for(int i = 0; i < npts1; i++)
        {
		pcl::PointXYZRGB * p = &(inputCloud->points[indices[i]]);
		c_avg_r = c_avg_r + p->r;
		c_avg_g = c_avg_g + p->g;
		c_avg_b = c_avg_b + p->b;
		xx = xx + (p->x - center[0]) * (p->x - center[0]);
		yy = yy + (p->y - center[1]) * (p->y - center[1]);
		xy = xy + (p->x - center[0]) * (p->y - center[1]);
	}

	c_avg_r = c_avg_r / bcount;
	c_avg_g = c_avg_g / bcount;
	c_avg_b = c_avg_b / bcount;
	
	float mindist = DBL_MAX;
	float dist;
	//int 
        block_color probable_col;
	
	dist = sqrt(pow(c_avg_r - PERFECT_RED[0], 2) + pow(c_avg_g - PERFECT_RED[1], 2) + pow(c_avg_b - PERFECT_RED[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_RED;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/images/baxter_red.png");
	}
	dist = sqrt(pow(c_avg_r - PERFECT_GREEN[0], 2) + pow(c_avg_g - PERFECT_GREEN[1], 2) + pow(c_avg_b - PERFECT_GREEN[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_GREEN;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/images/baxter_green.png");
	}
	dist = sqrt(pow(c_avg_r - PERFECT_BLUE[0], 2) + pow(c_avg_g - PERFECT_BLUE[1], 2) + pow(c_avg_b - PERFECT_BLUE[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_BLUE;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/images/baxter_blue.png");
	}
	dist = sqrt(pow(c_avg_r - PERFECT_WHITE[0], 2) + pow(c_avg_g - PERFECT_WHITE[1], 2) + pow(c_avg_b - PERFECT_WHITE[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_WHITE;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/images/baxter_white.png");
	}
	dist = sqrt(pow(c_avg_r - PERFECT_WOOD[0], 2) + pow(c_avg_g - PERFECT_WOOD[1], 2) + pow(c_avg_b - PERFECT_WOOD[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_WOOD;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/images/baxter_wood.png");
	}
	dist = sqrt(pow(c_avg_r - PERFECT_BLACK[0], 2) + pow(c_avg_g - PERFECT_BLACK[1], 2) + pow(c_avg_b - PERFECT_BLACK[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_BLACK;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/images/baxter_black.png");
	}
	
	
	float b = -(xx - yy) / (2 * xy);
	
	float c1_x = b + sqrt((b * b) + 1);
	float c1_y = 1.0;
	float c2_x = b - sqrt((b * b) + 1);
	float c2_y = 1.0;
	float ax_x;
	float ax_y;
	
	if(abs(c1_x) > abs(c2_x)){
		ax_x = c1_x / sqrt(c1_x * c1_x + 1);
		ax_y = c1_y / sqrt(c1_x * c1_x + 1);
	}
	else{
		ax_x = c2_x / sqrt(c2_x * c2_x + 1);
		ax_y = c2_y / sqrt(c2_x * c2_x + 1);
	}
	
	Eigen::Vector3f maxis;
	maxis << ax_x, ax_y, 0.0;
	
	Eigen::Vector3f avg_col;
	avg_col << c_avg_r, c_avg_g, c_avg_b;

	ROS_INFO ( " Center :  %f, %f, %f ", center[0], center[1], center[2]);
	block_data out;
		out.centroid  = center;
		out.color_avg = avg_col;
		out.major_axis = maxis;
		out.top_plane_z = max_z;
		out.color_name = probable_col;
	
	return out;
}


