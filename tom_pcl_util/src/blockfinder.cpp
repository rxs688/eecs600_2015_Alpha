#include "../include/blockfinder.h"
#include <vector>
using namespace std; 

void rgbTOhsv(double (&col)[3]){//Wonky syntax...
	double r = col[0];
	double g = col[1];
	double b = col[2];
	
	double cmax = max(r, max(g, b));
	double cmin = min(r, min(g, b));
	double delta = cmax - cmin;
	
	double val = cmax;
	
	double sat = 0.0;
	if(cmax != 0.0){
		sat = delta / cmax;
	}
	
	double hue = 0.0;
	if(delta != 0.0){
		if(cmax == r){
			hue = fmod(((g - b) / delta), 6.0) / 6.0;
		}
		else if(cmax == g){
			hue = (((b - r) / delta) + 2.0) / 6.0;
		}
		else if(cmax == b){
			hue = (((r - g) / delta) + 4.0) / 6.0;
		}
	}
	
	col[0] = hue;
	col[1] = sat;
	col[2] = val;
}

void findBlocks(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_pcl_cloud,
                double ht_ofTable, double ht_ofBlock, const float selectColor[3],
                vector<int> &output_indices)
{
    int size = in_pcl_cloud->width * in_pcl_cloud->height;
    output_indices.clear();
	
    for (size_t i = 0; i != size; ++i)
    {
	if ( fabs((ht_ofTable /*This is -ve*/ + ht_ofBlock) - in_pcl_cloud->points[i].z /* This is -ve*/ ) < .01 )
	{
	   output_indices.push_back(i);
	}
    }
}



Eigen::Vector3f computeCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                                vector<int> &selected_indices)
{
    Eigen::Vector3f centroid;
    centroid << 0, 0, 0;

    int size = selected_indices.size();
    std::cout << "frame: " << pcl_cloud->header.frame_id << std::endl;
    for (size_t i = 0; i != size; ++i)
    {
        centroid += pcl_cloud->points[selected_indices[i]].getVector3fMap();
    }
    if (size > 0)
    {
        centroid /= ((float) size);
    }
    return centroid;
}

void normalizeColors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr subject_cloud, 
                     vector<int> &selected_indices)
{
	int size = selected_indices.size();
	for(int i = 0; i < size; i++)
    {
		pcl::PointXYZRGB * p = &(subject_cloud->points[selected_indices[i]]);
		float n = sqrt(pow(p->r, 2) + pow(p->g, 2) + pow(p->b, 2));
		if(n != 0.0)
        {
			p->r = p->r / n;
			p->g = p->g / n;
			p->b = p->b / n;
		}
	}
}

block_data find_the_block(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, ros::NodeHandle n){
	
	vector<int> my_indices;
	ros::Publisher xdisplay_pub = n.advertise<sensor_msgs::Image>("/robot/xdisplay", 1000);
	
	int npts = inputCloud->width * inputCloud->height;

	int n_block_points = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr chosen_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	for(int i = 0 ; i < npts; i++){
		pcl::PointXYZRGB p = inputCloud->points[i];
		if(AOI_X_MIN < p.x && p.x < AOI_X_MAX){
			if(AOI_Y_MIN < p.y && p.y < AOI_Y_MAX){
				if(AOI_Z_MIN < p.z && p.z < AOI_Z_MAX){
					if(true){//TODO: Disregardment of stool points will go here!
						chosen_cloud->push_back(p);
						n_block_points++;
					}
				}
			}
		}
	}


	sensor_msgs::PointCloud2::Ptr g_pcl2_display_cloud (new sensor_msgs::PointCloud2);//Output for points of appropriate height.
	ros::Publisher pubCloud = n.advertise<sensor_msgs::PointCloud2> ("/pcl_cloud_display", 1);

	chosen_cloud->header.frame_id = "base"; //base"; //vs /base_link?
	chosen_cloud->is_dense = true;
	chosen_cloud->width = n_block_points;
	chosen_cloud->height = 1;

	pcl::toROSMsg(*chosen_cloud,*g_pcl2_display_cloud);
	g_pcl2_display_cloud->header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain
	pubCloud.publish(g_pcl2_display_cloud);
	
	//findBlocks(inputCloud, 0.511359 /* Ht of Table */, .02 /* Ht of Block */, PERFECT_RED , my_indices);
    /*int npts1 = my_indices.size();	
	ROS_INFO("Number of Points %d, LOCATED BLOCK(s) OF HEIGHT .075.",npts1);	
	Eigen::Vector3f center = computeCentroid(inputCloud, my_indices );
	//normalizeColors(inputCloud, indices);
	//for(int i = 0; i < npts; i++){
		
	
	float c_avg_r = 0.0;
	float c_avg_g = 0.0;
	float c_avg_b = 0.0;
	float xx = 0;
	float yy = 0;
	float xy = 0;
	for(int i = 0; i < npts1; i++)
        {
		pcl::PointXYZRGB * p = &(inputCloud->points[my_indices[i]]);
		c_avg_r = c_avg_r + p->r;
		c_avg_g = c_avg_g + p->g;
		c_avg_b = c_avg_b + p->b;
		xx = xx + (p->x - center[0]) * (p->x - center[0]);
		yy = yy + (p->y - center[1]) * (p->y - center[1]);
		xy = xy + (p->x - center[0]) * (p->y - center[1]);
	}

	c_avg_r = c_avg_r / npts1;
	c_avg_g = c_avg_g / npts1;
	c_avg_b = c_avg_b / npts1;
	
	float mindist = CONFUSION_TOLERANCE;
	float dist;
	//int 
        block_color probable_col = BLOCK_CONFUSED;
	
	dist = sqrt(pow(c_avg_r - PERFECT_RED[0], 2) + pow(c_avg_g - PERFECT_RED[1], 2) + pow(c_avg_b - PERFECT_RED[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_RED;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/share/images/baxter_red.png");
	}
	dist = sqrt(pow(c_avg_r - PERFECT_GREEN[0], 2) + pow(c_avg_g - PERFECT_GREEN[1], 2) + pow(c_avg_b - PERFECT_GREEN[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_GREEN;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/share/images/baxter_green.png");
	}
	dist = sqrt(pow(c_avg_r - PERFECT_BLUE[0], 2) + pow(c_avg_g - PERFECT_BLUE[1], 2) + pow(c_avg_b - PERFECT_BLUE[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_BLUE;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/share/images/baxter_blue.png");
	}
	dist = sqrt(pow(c_avg_r - PERFECT_WHITE[0], 2) + pow(c_avg_g - PERFECT_WHITE[1], 2) + pow(c_avg_b - PERFECT_WHITE[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_WHITE;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/share/images/baxter_white.png");
	}
	dist = sqrt(pow(c_avg_r - PERFECT_WOOD[0], 2) + pow(c_avg_g - PERFECT_WOOD[1], 2) + pow(c_avg_b - PERFECT_WOOD[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_WOOD;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/share/images/baxter_wood.png");
	}
	dist = sqrt(pow(c_avg_r - PERFECT_BLACK[0], 2) + pow(c_avg_g - PERFECT_BLACK[1], 2) + pow(c_avg_b - PERFECT_BLACK[2], 2));
	if(dist < mindist){
		mindist = dist;
		probable_col = BLOCK_BLACK;
		system("rosrun baxter_examples xdisplay_image.py --file=`rospack find kristina_hmi`/share/images/baxter_black.png");
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
		out.top_plane_z = center[2];
		out.color_name = probable_col;
	
	return out;*/
	block_data b;
	return b;
}

block_color singlepoint_id_color(double pointcols[3]){
	block_color retcol = BLOCK_CONFUSED;
	double test_dist = CONFUSION_TOLERANCE;
	double test_hsv[3] = {pointcols[0], pointcols[1], pointcols[2]};
	rgbTOhsv(test_hsv);
	
	for(int i = 0; i < BLOCK_CONFUSED; i++){
		double color_under_test[3] = {PERFECT_COLORS[i][0], PERFECT_COLORS[i][1], PERFECT_COLORS[i][2]};
		rgbTOhsv(color_under_test);
		double cdist = sqrt(pow(acos(sin(color_under_test[0]) * sin(test_hsv[0])), 2) + pow(color_under_test[1] - test_hsv[1], 2) + pow(color_under_test[2] - test_hsv[2], 2));
		if(cdist < test_dist){
			test_dist = cdist;
			retcol = (block_color)i;
		}
	}
	return retcol;
}

block_color multipoint_id_color(vector<pcl::PointXYZRGB> points){
	double r_sum = 0.0;
	double g_sum = 0.0;
	double b_sum = 0.0;
	
	for(int i = 0; i < points.size(); i++){
		r_sum = r_sum + points[i].r;
		g_sum = g_sum + points[i].g;
		b_sum = b_sum + points[i].b;
	}
	
	r_sum = r_sum / points.size();
	g_sum = g_sum / points.size();
	b_sum = b_sum / points.size();

	double x[3] = {r_sum, g_sum, b_sum};
	
	return singlepoint_id_color(x);
}
