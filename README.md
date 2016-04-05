# robot_pick_place
The robot_pick_place provides a cross-robot pick and place ROS (robot operating system) solution.

It contains the classes, interfaces, topics and messages required to perform a successful pick an place task with any ROS compatible multi-jointed robot.

It is implemented for two different ROS compatible robots (Baxter and UR5) while documenting the implementation of the classes and interfaces needed.

## Motivation

This project is developed to help integrate computer vision solutions with multi-jointed robots using ROS communication protocols and availabe packages 
for performing a picking and placing task.

This package provides to users the needed interfaces to integrate their computer vision solution with their multi-jointed robots (ROS compatible) in an easy and reusable manner.
The robot_pick_place package can be implemented with different ROS compatible robots that can perform picking and placing tasks with personalized and existing computer vision 
solutions, reducing the time and complexities presented when configuring the solution for an expecific pick and place task.

## Installing and Running the Package

Please, follow the Getting Started wiki page for instructions on installing the robot_pick_place package.

Please, follow the Running the robot_pick_place Package wiki on running and implementing the robot_pick_place package with a multi-jointed ROS compatible robot
and an existing computer vision solution.

## Installing Dependent packages

To be able to use robot_pick_place successfully, please install the package MoveIt (http://wiki.ros.org/moveit).

Check if MoveIt is installed.

```
$ rospack find moveit_core 
```

If you need to install MoveIt, please follow the instructions for your ROS version, see: http://moveit.ros.org/install/.

Install the industrial_robot_simulator package (http://wiki.ros.org/industrial_robot_simulator) for simulation purposes.
The simulation is performed and shown in Rviz, create a simulated world (when possible) that recreates the robot pick and place scene, 
then execute the robot_pick_place package and visualize its performance in Rviz. Simulating pick and place tasks, avoids setting up the
robots while saving energy and avoiding any possible malfunction.

The TF package is used for communicating through the different nodes running, while keeping track of the different coordinate frames over time.

Check if TF is installed.

```
$ rospack find tf
```

If you need to install TF, please follow the instructions for your ROS version, see: http://wiki.ros.org/tf.
