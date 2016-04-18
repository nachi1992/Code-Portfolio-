

#ifndef ROTORS_GAZEBO_LOCAL_PLANNER_NODE_H
#define ROTORS_GAZEBO_LOCAL_PLANNER_NODE_H


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

#include "library/local_planner.h"


namespace rotors_gazebo {



class LocalPlannerNode {

public:
	LocalPlannerNode();
	~LocalPlannerNode();

private:
	LocalPlanner local_planner;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    tf::TransformListener *tf_listener; 
    
	//Subricbers
	ros::Subscriber pose_stamped_sub_ ; 
	ros::Subscriber vi_sensor_pointcloud_sub_ ;
	
	//Publisher
    ros::Publisher path_pub_ ; 
    ros::Publisher local_pointcloud_pub_ ;
    ros::Publisher local_octomap_pub_ ;
    ros::Publisher front_pointcloud_pub_ ;
    ros::Publisher cached_pointcloud_pub_ ;
    ros::Publisher multi_dof_joint_trajectory_pub_ ;
    ros::Publisher waypoint_pub_;
    ros::Publisher path_candidates_pub_;

    void pointCloudCallback(
    	const sensor_msgs::PointCloud2 input);

    void poseStampedCallback(
    	const geometry_msgs::PoseStamped input);

    void publishAll();

};





}
























#endif // ROTORS_GAZEBO_LOCAL_PLANNER_NODE_H
