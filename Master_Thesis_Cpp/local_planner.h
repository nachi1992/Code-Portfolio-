

#ifndef ROTORS_CONTROL_GLOBAL_PLANNER_H
#define ROTORS_CONTROL_GLOBAL_PLANNER_H

#include <iostream>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/TransformStamped.h>

 #include <std_msgs/Bool.h>

 #include <geometry_msgs/Point.h>
#include <nav_msgs/GridCells.h>

 #include <geometry_msgs/Vector3Stamped.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>

#include <math.h> 


namespace rotors_gazebo {

#define PI 3.14159265
#define alpha_res 10

float distance2d(geometry_msgs::Point a, geometry_msgs::Point b);

class Histogram 
{ 
	int* array; 
	int m_width; 
	int h_width;

	public: 

		Histogram( int w, int h ) : m_width(w),h_width(h),array( new int[ w * h ] ) {} 
		
		~Histogram() 
		{ 
			delete[] array; 
		} 
		
		int get( int x, int y ) const 
		{
		 return array[ index( x, y ) ]; 
		} 

		void set( int x, int y , int value)  
		{
		 array[ index( x, y ) ] = value; 
		}

		int setZero() 
		{
		  memset(array,0,m_width*h_width*sizeof(int)); 
		} 
		
		protected: 
			int index( int x, int y ) const 
			{ 
				return x + m_width * y; 
			} 
};

class LocalPlanner{

public:

int grid_length;

pcl::PointCloud<pcl::PointXYZ> final_cloud;
octomap::Pointcloud octomapCloud;
bool obstacle;
Histogram polar_histogram; 
nav_msgs::GridCells path_candidates;
geometry_msgs::Point p1;
geometry_msgs::Vector3Stamped waypt;


geometry_msgs::Point pose, min, max, min_cache, max_cache, front, back, goal;
float min_x= 1.5 ,max_x= 1.5,min_y= 1.5,max_y= 1.5,min_z= 1.5,max_z=1.5; 
float back_x= 0 ,front_x= 2.5,back_y= 0.5,front_y= 0.5,back_z= 0.5,front_z=0.5;
float min_cache_x= 2 ,max_cache_x= 2,min_cache_y= 2,max_cache_y= 2,min_cache_z= 2,max_cache_z=2; 
int init =0;
float rad = 1;

LocalPlanner();
~LocalPlanner();

void setPose(const geometry_msgs::PoseStamped);
void setGoal();
void setLimits();
//void transformPointCloudToGlobalCoordinates(const sensor_msgs::PointCloud2&);
void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>& );
void cropPointCloud();
void convertPointcloudToOctomap();
bool obstacleAhead();
void createPolarHistogram();
void thresholdPolarHistogram();
void findFreeDirections();
bool checkForCollision();
//void costFunction();
void calculateCostMap();
void getNextWaypoint();
void goFast();

};

}




#endif // ROTORS_CONTROL_GLOBAL_PLANNER_H
