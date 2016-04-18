

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

#include "local_planner_node.h"



namespace rotors_gazebo {

    

    LocalPlannerNode::LocalPlannerNode()
    {   

        ros::NodeHandle nh;

        vi_sensor_pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            "vi_sensor/camera_depth/depth/points",1,&LocalPlannerNode::pointCloudCallback,this); 

        pose_stamped_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
            "ground_truth/pose",1,&LocalPlannerNode::poseStampedCallback,this);

        multi_dof_joint_trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
             mav_msgs::default_topics::COMMAND_TRAJECTORY, 1); 

        local_pointcloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            "local_pointcloud", 1);

        waypoint_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(
            "new_waypoint", 1);

        path_candidates_pub_ = nh.advertise<nav_msgs::GridCells>(
            "path_candidates", 1); 

        tf_listener = new tf::TransformListener();

    }

    LocalPlannerNode::~LocalPlannerNode(){}

     void LocalPlannerNode::publishAll()
    {
        trajectory_msg.header.stamp = ros::Time::now();

        Eigen::Vector3d desired_position(local_planner.waypt.vector.x, local_planner.waypt.vector.y, local_planner.waypt.vector.z);
   
        const float DEG_2_RAD = PI / 180.0;
        double desired_yaw = 0 * DEG_2_RAD;

        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
     
        ROS_INFO("Publishing waypoint: [%f, %f, %f].",
           desired_position.x(),
           desired_position.y(),
           desired_position.z());
           
        multi_dof_joint_trajectory_pub_.publish(trajectory_msg);
        local_pointcloud_pub_.publish(local_planner.final_cloud);
        path_candidates_pub_.publish(local_planner.path_candidates);
        waypoint_pub_.publish(local_planner.waypt);
    }

	

    void LocalPlannerNode::poseStampedCallback(const geometry_msgs::PoseStamped input)
    {
        local_planner.setPose(input);
    }

    void LocalPlannerNode::pointCloudCallback(const sensor_msgs::PointCloud2 input)
    {
      pcl::PointCloud<pcl::PointXYZ> complete_cloud;
      sensor_msgs::PointCloud2 pc2cloud_world;
      //local_planner.complete_cloud.points.clear();
      
      tf_listener->waitForTransform("world", input.header.frame_id, input.header.stamp, ros::Duration(1));
      pcl_ros::transformPointCloud("world",input,pc2cloud_world,*tf_listener);
      //pcl::fromROSMsg(pc2cloud_world, local_planner.complete_cloud);
      
      pcl::fromROSMsg(pc2cloud_world, complete_cloud);

      local_planner.filterPointCloud(complete_cloud);

    if(local_planner.obstacleAhead() && local_planner.init!=0)
    {
        local_planner.createPolarHistogram();
        //local_planner.thresholdPolarHistogram();
        local_planner.findFreeDirections();
        
        local_planner.calculateCostMap();
        
        local_planner.getNextWaypoint();

    }
    else
    {
        local_planner.goFast();
    }
     
     publishAll();
    
    local_planner.cropPointCloud(); 

     if(local_planner.init == 0)
    { 
      ros::Duration(2).sleep();
      local_planner.init =1;
    }


    }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planner_node");
  rotors_gazebo::LocalPlannerNode local_planner_node;
  ros::Duration(5).sleep(); 
  ros::spin();
  
  return 0;
}







