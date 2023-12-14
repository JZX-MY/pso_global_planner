# PSO Global Path Planner for ROS

![PSO_Planner](assets/PSO_Planner.png)

<p align="center">
    <img width="100px" height="20px" src="https://img.shields.io/badge/Ubuntu-20.04-orange?logo=Ubuntu&Ubuntu-20.04"
        alt="ubuntu" />
    <img width="100px" height="20px" src="https://img.shields.io/badge/ROS-noetic-blue?logo=ROS&ROS=noetic" alt="ROS" />
</p>

## 1. Introduction

This ROS Global Planner Plugin implements the PSO (Particle Swarm Optimization) path planning algorithm. The examples below demonstrate its usage in both static and dynamic environments:

### (1) Static Environment:

<div align="center">
  <img src="assets/pso_demo1.gif" alt="demo1.gif">
</div>

### (2) Dynamic Environment:

<div align="center">
  <img src="assets/pso_demo2.gif" alt="demo2.gif">
</div>

## 2. How to Use

### (1) Method One: Utilized as a ROS Global Planner Plugin.

You can employ this plugin within the ROS navigation package by configuring the global path planner plugin to `pso_planner/globalMotionPlannerROS` in the launch file where the 'move_base' node is situated. Additionally, load the parameter configuration file `pso_planner.yaml`. An example is provided below:

```bash
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">

    <!-- Load other parameter configuration files -->
    ...
    ...
    ...

    <!-- Load PSO Global Planner Parameters -->
    <rosparam file="$(find pso_global_planner)/params/pso_planner.yaml" command="load" />

    <!-- Set Global Path Planner Plugin -->
    <param name="base_global_planner" value="pso_planner/globalMotionPlannerROS" />

    <!-- Set other parameters such as local path planner plugin -->
    ...
    ...
    ...

</node>
```

### (2) Method Two: Employing through the ros_motion_planning library.

In addition to the method described above, which involves using it as an independent ROS global path planner plugin, we also offer an alternative approach. We have integrated the PSO global path planner into the ROS-based motion planning library, [ros_motion_planning](https://github.com/ai-winter/ros_motion_planning). You can easily utilize it by setting the 'robot1_global_planner' parameter in the ['user_config.yaml'](https://github.com/ai-winter/ros_motion_planning/blob/master/src/user_config/user_config.yaml) file of this motion planning library to 'pso'. An example is provided below:

```bash
   map: "warehouse"
   world: "warehouse"
   rviz_file: "sim_env.rviz"

   robots_config:
     - robot1_type: "turtlebot3_waffle"
       robot1_global_planner: "pso"      <!-- Set the global path planning algorithm to PSO -->
       robot1_local_planner: "dwa"

       <!-- Set other parameters -->
       ...
       ...
       ...

   plugins:

        <!-- Set other parameters -->
        ...
        ...
        ...
    
```

The demonstration of the results is as follows:

<div align="center">
  <img src="assets/pso_ros_1.gif" alt="demo3.gif" width="280"/> 
  <img src="assets/pso_fitness.png" alt="pso_fitness.png" width="520"/> 
</div>