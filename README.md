## Table of Contents
1. [General Info](#general-info)
2. [Technologies](#technologies)
3. [Installation](#installation)
4. [FAQs](#faqs)
<!-- 4. [Collaboration](#collaboration) -->

### General Info
***
The purpose of this projethe aim of this project is to make a node ros2.
This node can receive information from the GPS quectel runner.
It will disperse it to different ros topics for use in ros.
The topics are as follows:

* pose
* gps_fix
* fix
* imu
* visualization_marker

## Technologies
***
A list of technologies used within the project:
* [Quectel Runner](https://s3.amazonaws.com/files.pointonenav.com/quectel/lg69t/quectel-lg69t-am-evb.0.6.8.zip)
* [Fusion Engine Client](https://github.com/PointOneNav/fusion-engine-client)
* [ROS 2](https://docs.ros.org/en/humble/Installation.html)

## Installation
***
In order to install the project you will need to install others technologies.
You will first need to install [Quectel Runner](https://s3.amazonaws.com/files.pointonenav.com/quectel/lg69t/quectel-lg69t-am-evb.0.6.8.zip).
In a second step you should also install [Fusion Engine Client](https://github.com/PointOneNav/fusion-engine-client).
Finally, you will also need [ROS 2](https://docs.ros.org/en/humble/Installation.html).
```
$ git clone https://github.com/PointOneNav/ros2-fusion-engine-driver.git
$ colcon build --packages-select fusion-engine-driver                                                            
$ . install/local_setup.zsh                                                                                      
$ ros2 run fusion-engine-driver gps
```
Side information: To use the application in a special environment use ```lorem ipsum``` to start
<!-- ## Collaboration
***
Give instructions on how to collaborate with your project.
> Maybe you want to write a quote in this part. 
> It should go over several rows?
> This is how you do it. -->

## FAQs
***
A list of frequently asked questions
1. **This is a question in bold**
Answer of the first question with _italic words_. 
2. __Second question in bold__ 
To answer this question we use an unordered list:
* First point
* Second Point
* Third point
3. **Third question in bold**
Answer of the third question with *italic words*.
4. **Fourth question in bold**
| Headline 1 in the tablehead | Headline 2 in the tablehead | Headline 3 in the tablehead |
|:--------------|:-------------:|--------------:|
| text-align left | text-align center | text-align right |