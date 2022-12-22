## Table of Contents
1. [Fusion engine driver ros2](#fusion-engine-driver-ros2)
2. [Technologies](#technologies)
3. [Installation](#installation)
<!-- 4. [FAQs](#faqs) -->
<!-- 4. [Collaboration](#collaboration) -->

### Fusion engine driver ros2
***
The purpose of this project is to make a node ros2.
This node can receive information from the GPS quectel runner.
It will disperse it to different ros topics for use in ros.
The topics are as follows:

* "pose" send [PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)
* gps_fix
* fix
* imu
* visualization_marker

## Technologies
***
A list of technologies used within the project:
* [Quectel GPS](https://cdn.sanity.io/files/2p5fn5cz/production/5fd38edae48d577105acd1393bf918b81c9837e1.pdf)
* [ROS 2](https://docs.ros.org/en/humble/Installation.html)

## Installation
***
In order to install the project you will need to install others technologies.
You will first need to install [Quectel GPS](https://cdn.sanity.io/files/2p5fn5cz/production/5fd38edae48d577105acd1393bf918b81c9837e1.pdf).
Finally, you will also need [ROS 2](https://docs.ros.org/en/humble/Installation.html).

Once these installations are done, you will have to do these actions on 3 different terminals.

* Terminal 1.

On this first terminal you must launch the gps on port 12345.

* Terminal 2.

```
$ mkdir ros-fusion-engine
$ git clone https://github.com/PointOneNav/ros2-fusion-engine-driver.git ros-fusion-engine
$ cd ros-fusion-engine
$ colcon build --packages-select fusion-engine-driver                                                            
$ . install/local_setup.bash                                                                                   
$ ros2 run fusion-engine-driver gps
```

* Terminal 3.

```
$ rviz2
```

Once on rviz you will need to go to the File > Open config tab at the top of the page.
And you will need to go to the rviz2_config folder in the repository and select the file from there.


<!-- Side information: To use the application in a special environment use ```lorem ipsum``` to start -->
<!-- ## Collaboration
***
Give instructions on how to collaborate with your project.
> Maybe you want to write a quote in this part. 
> It should go over several rows?
> This is how you do it. -->
<!-- 
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
| text-align left | text-align center | text-align right | -->