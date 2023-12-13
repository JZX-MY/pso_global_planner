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

#ifndef GLOBAL_PLANNER_ROS_H
#define GLOBAL_PLANNER_ROS_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <vector>
#include <visualization_msgs/Marker.h>

#include "pso.h"

 
namespace global_motion_planner 
{

  class globalMotionPlannerROS : public nav_core::BaseGlobalPlanner 
  {

    public:
      /**
      * @brief Default constructor of the plugin
      */
      globalMotionPlannerROS();

      globalMotionPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      ~globalMotionPlannerROS();

      /**
      * @brief  Initialization function for the PlannerCore object
      * @param  name The name of this planner
      * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
      */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief Generates a plan from a given path.
       * @param path    Vector of pairs representing the path in (x, y) coordinates.
       * @param plan    Vector of PoseStamped messages representing the generated plan.
       * @return bool   True if the plan is successfully generated, false otherwise.
       */
      bool getPlanFromPath(std::vector< std::pair<int, int> >& path, std::vector<geometry_msgs::PoseStamped>& plan);


      /**
      * @brief Compute the euclidean distance (straight-line distance) between two points
      * @param px1 point 1 x
      * @param py1 point 1 y
      * @param px2 point 2 x
      * @param py2 point 2 y
      * @return the distance computed
      */
      double distance(double px1, double py1, double px2, double py2);

      /**
       * @brief Check if there is a collision.
       * @param x coordinate (cartesian system)
       * @param y coordinate (cartesian system)
       * @return True is the point collides and false otherwise
      */
      bool collision(double x, double y); //是否为障碍物

      /**
       * @brief Checks if the specified world coordinates are in the vicinity of a free space.
       * @param wx   The world x-coordinate to be checked.
       * @param wy   The world y-coordinate to be checked.
       * @return bool True if the specified coordinates are around a free space, false otherwise.
       */
      bool isAroundFree(double wx, double wy);

      /**
       * @brief Checks for collision between two points in a circular region.
       * @param x1     The x-coordinate of the first point.
       * @param y1     The y-coordinate of the first point.
       * @param x2     The x-coordinate of the second point.
       * @param y2     The y-coordinate of the second point.
       * @param radius The radius of the circular region for collision checking.
       * @return bool  True if there is a collision, false otherwise.
       */
      bool pointCircleCollision(double x1, double y1, double x2, double y2, double radius);
      
      /**
       * @brief Optimizes the orientation of poses in a given plan.
       * @param plan   Vector of PoseStamped messages representing the plan to be optimized.
       */
      void optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan);

      /**
       * @brief Checks if the line segment between two points is free from obstacles.
       * @param p1   Pair representing the coordinates (x, y) of the first point.
       * @param p2   Pair representing the coordinates (x, y) of the second point.
       * @return bool True if the line segment is free from obstacles, false otherwise.
       */
      bool isLineFree(const std::pair<double, double> p1,const std::pair<double, double> p2);

    protected:

      costmap_2d::Costmap2D* costmap_;                 // costmap
      costmap_2d::Costmap2DROS* costmap_ros_;          // costmap ros
      global_motion_planner::PSO* g_planner_;          // planner
      unsigned int nx_, ny_;                           // costmap size
      double origin_x_, origin_y_;                     // costmap origin
      double resolution_;                              // costmap resolution
      std::string frame_id_;
      ros::Publisher plan_pub_;

    private:

      bool initialized_;

  };
} // global_motion_planner namespace
#endif
