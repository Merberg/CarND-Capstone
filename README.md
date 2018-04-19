# Self-Driving Car Nanodegree - Capstone

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).


## Team Members

| Name  | Email | Responsibilities & Contributions |
| ----- | ----- | ---------------- | 
| [Ramesh Misra](http://github.com/ramesh)       | ramesh.misra@gmail.com     | Team Lead |
| [Alan Gordon](http://github.com/alangordon258) | alangordon258@gmail.com    | Trajectory planner and control |
| [Andrea Dranberg](http://github.com/merberg)   | merrick.dranberg@gmail.com | Traffic Light Classification |
| [Felipe Reis](http://github.com/falreis)       | falreis@gmail.com          | Trajectory planner and control |


## Contents

* [Installation](#installation)
* [System Details](#system-details)
  * [Software Architecture](#software-architecture)
  * [Subsystems](#subsystems)
  * [Perception Subsystem](#perception-subsystem)
    * [Traffic Light Detection](#traffic-light-detection)
    * [Traffic Light Classification](#traffic-light-classification)
    * [Obstacle Detection](#traffic-light-classification)
  * [Planning Subsystem](#perception-subsystem)
    * [Waypoint Loader](#waypoint-loader)
    * [Waypoint Updater](#waypoint-updater)
  * [Control Subsystem](#control-subsystem)
    * [Twist Controller](#twist-controller)
    * [Drive-By-Wire Controller](#dbw-controller)
* [Test and Running](#test-and-running)
  * [Testing in Simulator](#testing-in-simulator)
  * [Testing with Real Data](#testing-with-real-data)
  * [Running in Real World](#running-in-real-world)
* [Next Steps](#next-steps)
  * [Goals](#goals)
  * [Known Issues](#known-issues)

## Installation

Please follow detailed information about how to install and configure the project [here](https://github.com/Merberg/CarND-Capstone/blob/master/installation.md).

## System Details

This section contains information about software architecture and some details of the implementation.

The code is written in Pyhton and C++. Also, it's use some libraries like [Tensorflow](https://www.tensorflow.org/) and [ROS](http://www.ros.org/). 

### Software Architecture

The system architecture diagram shows ROS nodes and topics used in the project 

![software_architecture](imgs/readme_files/final-project-ros-graph-v2.png)
*(Software Architecture - diagram was provided by Udacity)*

### Subsystems

#### Perception Subsystem

##### Traffic Light Detection

##### Traffic Light Classification

##### Obstacle Detection

#### Planning Subsystem

##### Waypoint Loader

##### Waypoint Updater

#### Control Subsystem

##### Twist Controller

##### DBW Controller

## Test and Running

### Testing in Simulator

### Testing with Real Data

### Running in Real World

Coming soon..

## Next Steps

In this section shows the next steps for the project, our goals and known bugs.

### Goals

1. Implement Object detection
2. Improve car behaviour in different scenarios

### Known Issues
