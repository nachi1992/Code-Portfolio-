
#include "local_planner.h"

namespace rotors_gazebo {


float distance2d(geometry_msgs::Point a, geometry_msgs::Point b)
{
   return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) +(a.z-b.z)*(a.z-b.z) );
}

LocalPlanner::LocalPlanner(): grid_length(360/alpha_res),polar_histogram(grid_length,grid_length){}

LocalPlanner::~LocalPlanner(){}

void LocalPlanner::setPose(const geometry_msgs::PoseStamped input)
{
   pose.x = input.pose.position.x ;
   pose.y = input.pose.position.y ;
   pose.z = input.pose.position.z ; 
   setGoal();
   setLimits();

}

void LocalPlanner::setLimits()
{
   min.x = pose.x- min_x;
   min.y = pose.y- min_y;
   min.z = pose.z- min_z;
   max.x = pose.x + max_x;
   max.y = pose.y + max_y;
   max.z= pose.z + max_z;
   front.x = pose.x + front_x;
   front.y = pose.y + front_y;
   front.z = pose.z + front_z;
   back.x = pose.x - back_x;
   back.y = pose.y - back_y;
   back.z = pose.z - back_z;
   min_cache.x = pose.x - min_cache_x;
   min_cache.y = pose.y - min_cache_y;
   min_cache.z = pose.z - min_cache_z;
   max_cache.x = pose.x + max_cache_x;
   max_cache.y = pose.y + max_cache_y;
   max_cache.z = pose.z + max_cache_z;
}

void LocalPlanner::setGoal()
{
   goal.x = pose.x + 1;
   goal.y = 0;//input.pose.position.y ;
   goal.z = 1.5; 
}
/*
void LocalPlanner::transformPointCloudToGlobalCoordinates(const sensor_msgs::PointCloud2& input)
{
 	sensor_msgs::PointCloud2 pc2cloud_world;
 	complete_cloud.clear();
 	tf_listener->waitForTransform("world", input.header.frame_id, input.header.stamp, ros::Duration(1));
 	pcl_ros::transformPointCloud("world",input,pc2cloud_world,*tf_listener);
 	pcl::fromROSMsg(pc2cloud_world, complete_cloud);
}
*/
void LocalPlanner::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>& complete_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it;
  pcl::PointCloud<pcl::PointXYZ> front_cloud;
  final_cloud.points.clear();

	for (pcl_it = complete_cloud.begin(); pcl_it != complete_cloud.end(); ++pcl_it)
	{
      // Check if the point is invalid
    if (!std::isnan (pcl_it->x) && !std::isnan (pcl_it->y) && !std::isnan (pcl_it->z))
    {
   		if((pcl_it->x)<max_cache.x&&(pcl_it->x)>min_cache.x&&(pcl_it->y)<max_cache.y&&(pcl_it->y)>min_cache.y&&(pcl_it->z)<max_cache.z&&(pcl_it->z)>min_cache.z) //    if((pcl_it->x)<max_cache_x&&(pcl_it->x)>-min_cache_x&&(pcl_it->y)<max_cache_y&&(pcl_it->y)>-min_cache_y&&(pcl_it->z)<max_cache_z&&(pcl_it->z)>-min_cache_z)
      {
        octomapCloud.push_back(pcl_it->x, pcl_it->y, pcl_it->z);
      }

      if((pcl_it->x)<front.x&&(pcl_it->x)>back.x&&(pcl_it->y)<front.y&&(pcl_it->y)>back.y&&(pcl_it->z)<front.z&&(pcl_it->z)>back.z)// if((data.x)<max_x&&(data.x)>-min_x&&(data.y)<max_y&&(data.y)>-min_y&&(data.z)<max_z&&(data.z)>-min_z)
      {
        front_cloud.points.push_back(pcl::PointXYZ(pcl_it->x,pcl_it->y,pcl_it->z));
      }    

    }
  }

  if(front_cloud.points.size()>1)
  {
    octomap::Pointcloud::iterator oc_it;
		for (oc_it = octomapCloud.begin(); oc_it != octomapCloud.end(); ++oc_it)
		{
      if((oc_it->x())<max.x&&(oc_it->x())>min.x&&(oc_it->y())<max.y&&(oc_it->y())>min.y&&(oc_it->z())<max.z&&(oc_it->z())>min.z)// if((data.x)<max_x&&(data.x)>-min_x&&(data.y)<max_y&&(data.y)>-min_y&&(data.z)<max_z&&(data.z)>-min_z)
      {
        final_cloud.points.push_back(pcl::PointXYZ(oc_it->x(),oc_it->y(),oc_it->z()));
      }
    }
      
    obstacle = true;
  }
  else
    obstacle = false;
   
    ROS_INFO(" Cloud transformed");

    final_cloud.header.stamp =  complete_cloud.header.stamp;
    final_cloud.header.frame_id = complete_cloud.header.frame_id;
    final_cloud.width = final_cloud.points.size();
    final_cloud.height = 1; 
}

bool LocalPlanner::obstacleAhead()
{ 
  float dist_y =  abs(pose.y - goal.y);
  float dist_z =  abs(pose.z - goal.z);
	if(obstacle || dist_y>0.2 ||  dist_z > 0.7)
		return true;
	else
		return false;
}

void LocalPlanner::createPolarHistogram()
{
	float bbx_rad = (max.x-min.x)*sqrt(2)/2; 
  float dist;
  
 
  polar_histogram.setZero();

  pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
    
  for( it = final_cloud.begin(); it != final_cloud.end(); ++it)
  {   
    geometry_msgs::Point temp; 
    temp.x= it->x;
    temp.y = it->y;
    temp.z = it->z;
    dist = distance2d(pose,temp);
   
    if(dist < bbx_rad)
    { 
      int beta_z = floor((atan2(temp.x-pose.x,temp.y-pose.y)*180.0/PI)); //azimuthal angle
      int beta_e = floor((atan2(temp.z-pose.z,sqrt((temp.x-pose.x)*(temp.x-pose.x)+(temp.y-pose.y)*(temp.y-pose.y)))*180.0/PI));//elevation angle
    
      beta_z = beta_z + (alpha_res - beta_z%alpha_res);
      beta_e = beta_e + (alpha_res - beta_e%alpha_res); 
      
      int e = (180+beta_e)/alpha_res - 1 ;
      int z = (180+beta_z)/alpha_res - 1 ;

      polar_histogram.set(e,z,polar_histogram.get(e,z)+1);
    }
 }

}

void LocalPlanner::findFreeDirections()
{


  int n = floor(40/alpha_res); //n+1 - angular window size
  int a=0,b=0;
  int sum = 0, pathID = 1;
  bool free = true;
  bool corner = false;
  geometry_msgs::Point p;

  path_candidates.cells.clear();
  path_candidates.header.stamp = ros::Time::now();
  path_candidates.header.frame_id = "/world";
  path_candidates.cell_width = alpha_res;
  path_candidates.cell_height = alpha_res;


  for(int e= 0;e<grid_length; e++)
        {  
          for(int z= 0;z<grid_length; z++)
            {  

              for(int i=e-n;i<=e+n;i++)
              {
                for(int j=z-n;j<=z+n;j++)
                {
                  free = true;
                  corner = false;

                   //Case 1 - i < 0
                  if(i<0 && j>=0 && j<grid_length)
                  {
                    a = -i;
                    b = grid_length-j-1;
                  }
                   //Case 2 - j < 0
                  else if(j<0 && i>=0 && i<grid_length)
                  {
                    b = j+grid_length;
                  } 
                   //Case 3 - i >= grid_length
                  else if(i>=grid_length && j>=0 && j<grid_length)
                  {
                    a = grid_length-(i%(grid_length-1));
                    b = grid_length-j-1;
                  }
                   //Case 4 - j >= grid_length
                  else if(j>=grid_length && i>=0 && i<grid_length)
                  {
                    b = j%grid_length;
                  }
                  else if( i>=0 && i<grid_length && j>=0 && j<grid_length)
                  {
                    a = i;
                    b = j;
                  }
                  else
                  {
                    corner = true;
                  }

                  if(!corner)
                  {
                    if(polar_histogram.get(a,b)!=0)
                    {
                     free = false;
                    // ROS_INFO(" in the path loop %f %d %d", Hist_polar_binary[a][b] , a, b ) ;
                     break;
                    } 
                  }
                }
                  if(!free)
                    break;
              }
               
           if(free)
           {
            p.x = e*alpha_res+alpha_res-180;  
            p.y = z*alpha_res+alpha_res-180;
            p.z = 0;
            path_candidates.cells.push_back(p);   
           }
      } 
    }

   ROS_INFO(" Path_candidates calculated");

}

//void LocalPlanner::costFunction()
//{

//}

void LocalPlanner::calculateCostMap()
{

     int goal_z = floor(atan2(goal.x-pose.x,goal.y-pose.y)*180.0/PI); //azimuthal angle
     int goal_e = floor(atan2(goal.z-pose.z,sqrt((goal.x-pose.x)*(goal.x-pose.x)+(goal.y-pose.y)*(goal.y-pose.y)))*180.0/PI);//elevation angle
  

     int e = 0,z =0 ;
     
     float cost_path; //[path_candidates.cells.size()] = {0};
     float small ; int small_i;

      

    for(int i=0;i<path_candidates.cells.size();i++)
    {  
       e = path_candidates.cells[i].x;
       z = path_candidates.cells[i].y;
       if(init==0)
      {
        p1.x = e;
        p1.y = z;
      }
      if(e>-50)
       cost_path = 2*sqrt((goal_e-e)*(goal_e-e)+(goal_z-z)*(goal_z-z)) + 1.5*sqrt((p1.x-e)*(p1.x-e)+(p1.y-z)*(p1.y-z)) ; // 10*abs(sin(e*(PI/180))) ;//1*(abs(p1.x-e)+abs(p1.y-z));  
     else
       cost_path = 10000;
      if(i == 0)
      {
        small = cost_path;
        small_i = i;
      }

       if(cost_path<small)
      {
        small = cost_path;
        small_i = i ;
      }  

    }
    
    p1.x = path_candidates.cells[small_i].x;
    p1.y = path_candidates.cells[small_i].y;
    p1.z = path_candidates.cells[small_i].z;
  //  path_candidates.cells.clear();
  //  path_candidates.cells.push_back(p1);

   ROS_INFO(" min_e - %f min_z- %f min_cost %f", p1.x,p1.y, small) ;


}

void LocalPlanner::getNextWaypoint()
{
  waypt.header.stamp = ros::Time::now();
  waypt.header.frame_id = "/world";
   
  waypt.vector.x = pose.x+ rad*cos(p1.x*(PI/180))*sin(p1.y*(PI/180));
  waypt.vector.y = pose.y+ rad*cos(p1.x*(PI/180))*cos(p1.y*(PI/180));
  waypt.vector.z = pose.z+ rad*sin(p1.x*(PI/180));

  if((waypt.vector.z<0.5) && checkForCollision())
  {
    waypt.vector.x = pose.x-0.1;
    waypt.vector.y = pose.y;
    waypt.vector.z = pose.z;
    p1.x = 0;
    p1.y = 0;
    ROS_INFO(" Will hit the obstacle. Going back");
  }
}

bool LocalPlanner::checkForCollision()
{
  bool avoid = false;
  geometry_msgs::Point temp;
  geometry_msgs::Point p;
  p.x = waypt.vector.x;
  p.y = waypt.vector.y;
  p.z = waypt.vector.z;

  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  for( it = final_cloud.begin(); it != final_cloud.end(); ++it)
  {
    temp.x = it->x;
    temp.y = it->y;
    temp.z = it->z;
     
    if(distance2d(p,temp)<0.5 && init != 0)
    {
     avoid = true;
     break;
    }
  }  
  return avoid;
}

void LocalPlanner::goFast()
{
   waypt.vector.x = waypt.vector.x+0.3;
   waypt.vector.y = goal.y;
   waypt.vector.z = goal.z;

}

void LocalPlanner::cropPointCloud()
{  
  octomap::point3d half_min_cache;
  octomap::point3d half_max_cache;
  half_min_cache.x() = waypt.vector.x - min_x - 1;
  half_min_cache.y() = waypt.vector.y - min_y - 1;
  half_min_cache.z() = waypt.vector.z - min_z - 1;
  half_max_cache.x() = waypt.vector.x;
  half_max_cache.y() = waypt.vector.y + max_y + 1;
  half_max_cache.z() = waypt.vector.z + max_z + 1;

  octomapCloud.crop(half_min_cache, half_max_cache); 
}

}
