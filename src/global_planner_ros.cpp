/***********************************************************
 *
 * @file: global_planner_ros.cpp
 * @breif: ROS related framework for global path planner
 * @author: Jing Zongxin
 * @update: 2023-12-13
 * @version: 1.0
 *
 * Copyright (c) 2023，Jing Zongxin
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/

#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <tf/tf.h>

#include "global_planner_ros.h"

PLUGINLIB_EXPORT_CLASS(global_motion_planner::globalMotionPlannerROS, nav_core::BaseGlobalPlanner)

namespace global_motion_planner
{

  globalMotionPlannerROS::globalMotionPlannerROS() : costmap_(nullptr), initialized_(false){}

  globalMotionPlannerROS::globalMotionPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :costmap_ros_(costmap_ros),initialized_(false)
  {
    initialize(name, costmap_ros);
  }

  globalMotionPlannerROS::~globalMotionPlannerROS()
  {
    if (g_planner_)
    {
      delete g_planner_;
      g_planner_ = NULL;
    }
  }

  // Variable initialization
  void globalMotionPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if (!initialized_)
    {
      // Initialize map
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();
      frame_id_ = costmap_ros->getGlobalFrameID();
      // get costmap properties
      nx_ = costmap_->getSizeInCellsX(), ny_ = costmap_->getSizeInCellsY();
      origin_x_ = costmap_->getOriginX(), origin_y_ = costmap_->getOriginY();
      resolution_ = costmap_->getResolution();
  
      ros::NodeHandle private_nh("~/" + name);
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1); //发布全局计算
     
      bool pub_particles;
      int n_particles,n_inherited,pointNum,max_speed,initposmode,pso_max_iter;
      double obs_factor, w_inertial, w_social, w_cognitive;
      private_nh.param("n_particles", n_particles, 50);        // number of particles
      private_nh.param("n_inherited", n_inherited, 20);        // number of inherited particles
      private_nh.param("pointNum", pointNum, 5);               // number of position points contained in each particle
      private_nh.param("obs_factor", obs_factor, 0.39);        // obstacle factor(greater means obstacles)
      private_nh.param("max_speed", max_speed,40);             // The maximum velocity of particle motion
      private_nh.param("w_inertial", w_inertial,1.0);          // inertia weight
      private_nh.param("w_social", w_social, 2.0);             // social weight
      private_nh.param("w_cognitive", w_cognitive, 1.2);       // cognitive weight
      private_nh.param("initposmode", initposmode, 2);         // Set the generation mode for the initial position points of the particle swarm
      private_nh.param("pub_particles", pub_particles, false); // Whether to publish particles
      private_nh.param("pso_max_iter", pso_max_iter, 5);       // maximum iterations

      g_planner_ = new PSO(nx_, ny_, resolution_ ,origin_x_,origin_y_, n_particles,n_inherited, pointNum, w_inertial, w_social, w_cognitive, obs_factor, max_speed ,initposmode ,pub_particles,pso_max_iter);

      ROS_INFO("PSO planner initialized successfully");
      initialized_ = true;
    }
    else
    {
      ROS_WARN("This planner has already been initialized... doing nothing");
    }
  }

  // Path planning
  bool globalMotionPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan)
  {
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    std::pair<int, int> start_node, goal_node;
    unsigned int mx = 0,my = 0;

    if(this->collision(start.pose.position.x, start.pose.position.y))
    {
      ROS_WARN("failed to get a path.start point is obstacle.");
      return false;
    }
    else
    {
      this->costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mx,my);
      start_node.first =static_cast<int>(mx);
      start_node.second=static_cast<int>(my);
    }

    if(this->collision(goal.pose.position.x, goal.pose.position.y))
    {
      ROS_WARN("failed to get a path.goal point is obstacle.");
      return false;
    }
    else
    {
      this->costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,mx,my);
      goal_node.first =static_cast<int>(mx);
      goal_node.second=static_cast<int>(my);
    }

    std::vector< std::pair<int, int>> path;
    plan.clear();

    double start_time = ros::Time::now().toSec();

    bool path_found = g_planner_->plan(costmap_->getCharMap(), start_node, goal_node, path);

    if (path_found)
    {
      if (getPlanFromPath(path, plan))
      {
        geometry_msgs::PoseStamped goal_copy = goal;
        goal_copy.header.stamp = ros::Time::now();
        plan.push_back(goal_copy);
      }
      else
      {
        ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
      }
    }
    else
    {
      ROS_ERROR("Failed to get a path.");
    }

    // path point attitude correction
    optimizationOrientation(plan);

    // publish visulization plan
    nav_msgs::Path path_pose;
    path_pose.header.frame_id = this->frame_id_;
    path_pose.header.stamp = ros::Time::now();
    path_pose.poses = plan;
    plan_pub_.publish(path_pose);

    return !plan.empty();
  }

  bool globalMotionPlannerROS::getPlanFromPath(std::vector< std::pair<int, int> >& path, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    plan.clear();
    unsigned int mx = 0,my = 0;
    double wx, wy;

    for (int i = 0; i < path.size(); i++)
    {
      mx=static_cast<unsigned int>(path[i].first);
      my=static_cast<unsigned int>(path[i].second);
      this->costmap_->mapToWorld(mx,my,wx,wy);
      // coding as message type
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = frame_id_;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }

    return !plan.empty();
  }

  // Optimizes the orientation of poses in a given plan.
  void globalMotionPlannerROS::optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan)
  {
    size_t num = plan.size()-1;
    if(num < 1)
      return;
    for(size_t i=0;i<num;i++)
    {
      plan[i].pose.orientation = tf::createQuaternionMsgFromYaw( atan2( plan[i+1].pose.position.y - plan[i].pose.position.y,
                                                                 plan[i+1].pose.position.x - plan[i].pose.position.x ) );
    }
  }

  // Calculate whether the distance between two points is less than the set error
  bool globalMotionPlannerROS::pointCircleCollision(double x1, double y1, double x2, double y2, double goal_radius)
  {
    double dist = distance(x1, y1, x2, y2);
    if (dist < goal_radius)
      return true;
    else
      return false;
  }

  bool globalMotionPlannerROS::isLineFree(const std::pair<double, double> p1, const std::pair<double, double> p2)
  {
   std::pair<double, double> ptmp;
    ptmp.first = 0.0;
    ptmp.second = 0.0;

    double dist = sqrt( (p2.second-p1.second) * (p2.second-p1.second) +
                        (p2.first-p1.first) * (p2.first-p1.first) );
    if (dist < this->resolution_)
    {
        return true;
    }
    else
    {
      int value = int(floor(dist/this->resolution_));
      double theta = atan2(p2.second - p1.second,
                           p2.first - p1.first);
      int n = 1;
      for (int i = 0;i < value; i++)
      {
        ptmp.first = p1.first + this->resolution_*cos(theta) * n;
        ptmp.second = p1.second + this->resolution_*sin(theta) * n;
        if (collision(ptmp.first, ptmp.second))
          return false;
        n++;
      }
      return true;
    }
  }

  // Calculate the distance between two points
  double globalMotionPlannerROS::distance(double px1, double py1, double px2, double py2)
  {
    return sqrt((px1 - px2)*(px1 - px2) + (py1 - py2)*(py1 - py2));
  }

  // Check if the checkpoint is an obstacle
  bool globalMotionPlannerROS::collision(double x, double y)
  {
    unsigned int mx,my;
    if(!this->costmap_->worldToMap(x,y,mx,my))
      return true;
    if ((mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
      return true;
    if (costmap_->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return true;
    return false;
  }
  
  // Check if there are any obstacles around the checkpoint
  bool globalMotionPlannerROS::isAroundFree(double wx, double wy)
  {
    unsigned int mx, my;
    if(!this->costmap_->worldToMap(wx,wy,mx,my))
      return false;
    if(mx <= 1 || my <= 1 || mx >= this->costmap_->getSizeInCellsX()-1 || my >= this->costmap_->getSizeInCellsY()-1)
      return false;
    int x,y;
    for(int i=-1;i<=1;i++)
    {
      for(int j=-1;j<=1;j++)
      {
        x = static_cast<int>(mx) + i;
        y = static_cast<int>(my) + j;
        if(this->costmap_->getCost(static_cast<unsigned int>(x),static_cast<unsigned int>(y)) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          return false;
      }
    }
    return true;
  }


}; // global_motion_planner
