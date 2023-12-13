/***********************************************************
 *
 * @file: pso_plan_node.cpp
 * @breif: PSO Global Path Planner ROS Node
 * @author: Jing Zongxin
 * @update: 2023-12-13
 * @version: 1.0
 *
 * Copyright (c) 2023，Jing Zongxin
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/


#include <global_planner_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace global_motion_planner
{

class globalMotionPlannerWithCostmap : public globalMotionPlannerROS
{
    public:
        globalMotionPlannerWithCostmap(std::string name, costmap_2d::Costmap2DROS* cmap);
        ~globalMotionPlannerWithCostmap();

    private:
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
        costmap_2d::Costmap2DROS* cmap_;
        ros::Subscriber pose_sub_;
};


void globalMotionPlannerWithCostmap::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    geometry_msgs::PoseStamped robot_pose;
    cmap_->getRobotPose(robot_pose);
    std::vector<geometry_msgs::PoseStamped> path;
    unsigned int mx = 0,my = 0;
    if(!this->costmap_->worldToMap(goal->pose.position.x,goal->pose.position.y,mx,my))
    {
      std::cout << "worldToMap error" << std::endl;
      return;
    }
    if(this->costmap_->getCost(mx,my) != costmap_2d::FREE_SPACE)
    {
      std::cout << "The target point is unreachable." << std::endl;
      return;
    }
    makePlan(robot_pose, *goal, path);
}

globalMotionPlannerWithCostmap::globalMotionPlannerWithCostmap(std::string name, costmap_2d::Costmap2DROS* cmap) :
        globalMotionPlannerROS(name, cmap)
{
    ros::NodeHandle private_nh("move_base_simple");
    cmap_ = cmap;
    pose_sub_ = private_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &globalMotionPlannerWithCostmap::poseCallback, this);
}

globalMotionPlannerWithCostmap::~globalMotionPlannerWithCostmap()
{}

} // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pso_planner");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    costmap_2d::Costmap2DROS gcm("global_costmap", buffer);
    global_motion_planner::globalMotionPlannerWithCostmap pppp("globalMotionPlannerROS", &gcm);
    ros::spin();
    return 0;
}

