## Table of Contents
1. [Fusion engine driver ros2](#fusion-engine-driver-ros2)
2. [Technologies](#requirements)
3. [Installation](#installation)
3. [Usage](#usage)

### Fusion engine driver ros2
***
This project enables developers to use the fusion engine output data in a ROS2 node.
The node will receive from the quectel_runner the the data on the following topics that you can susbcribe to:

* [pose](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)
* [gps_fix](http://docs.ros.org/en/hydro/api/gps_common/html/msg/GPSFix.html)
* [fix](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html)
* [imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)
* [visualization_marker](http://wiki.ros.org/rviz/DisplayTypes/Marker)

## Requirements
***
A list of technologies used within the project:
* [Quectel](https://cdn.sanity.io/files/2p5fn5cz/production/5fd38edae48d577105acd1393bf918b81c9837e1.pdf)
* [ROS 2](https://docs.ros.org/en/humble/Installation.html)

## Installation
***

The first time you use ros2 with the gps you may not get any messages you need to set the gps for specific message types. To do this after launching the virtual environment you will need to go in the folder p1_runner/bin to make the following control. This step is necessary for enabled ROS messages in FusionEngine.

```
$ python3 -m venv venv
$ source venv/bin/activate
$ pip install -r requirements.txt
```

```
$ config_tool.py apply uart2_message_rate fe ROSPoseMessage 100ms
$ config_tool.py apply uart2_message_rate fe ROSGPSFixMessage 100ms
$ config_tool.py apply uart2_message_rate fe ROSIMUMessage 100ms
$ config_tool.py save
```
Once you have activated the ros messages. You will install the node.

```
$ mkdir ros-fusion-engine
$ git clone https://github.com/PointOneNav/ros2-fusion-engine-driver.git ros-fusion-engine
$ cd ros-fusion-engine
$ sudo apt-get install ros-humble-gps-msgs
$ rosdep install -i --from-path ./ --rosdistro foxy -y
$ colcon build --packages-select fusion-engine-driver                                                            
$ . install/local_setup.bash                                                                                   
```

## Usage
***

Now that you have installed the node and it is ready for use here is how to use it.
You have 3 different modes to use it. At first you will have to fill in the type of connection to launch the program.
The three types are as follows (tcp, udp, serial). These three types have different parameters, they are the following.

> **Warning**
> To use the tcp and udp modes, you need to run the p1-runner in the mode of your choice next to it.

* TCP

To launch in tcp you will need its port and id. 
Here is an example to launch in tcp.

```
$ colcon build --packages-select fusion-engine-driver                                                            
$ . install/local_setup.bash                                                                                   
$ ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args -p connection_type:=tcp -p tcp_ip:=localhost -p tcp_port:=12345
```

In this case, these arguments are optional, in fact there are default values (localhost and 12345).

* UDP

To launch in udp you will need its port. 
Here is an example to launch in udp.

```
$ colcon build --packages-select fusion-engine-driver                                                            
$ . install/local_setup.bash                                                                                   
$ ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args -p connection_type:=udp -p udp_port:=12345
```

In this case, these arguments are optional, in fact there are default values (12345).

* serial

To launch in serial you will need its port. 
Here is an example to launch in serial.

```
$ colcon build --packages-select fusion-engine-driver                                                            
$ . install/local_setup.bash                                                                                   
$ ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args -p connection_type:=tty -p tty_port:=/dev/ttyUSB1
```

In this case, these arguments are optional, in fact there are default values (/dev/ttyUSB1).


## Annexes

Once these installations are done, you will have to do these actions on 3 different terminals.

Tcp mode require more than just turning on the card.
For using tcp, you should go to the p1_runner folder, and doing this actions:

```
$ pip install -r requirements.txt
$ python3 -m quectel_runner --device-id [YOUR ID] --polaris [YOUR KEY]  -v  --tcp 12345 --output-type=all
```

Now that you can retrieve the fusion engine information in ros, you have the possibility to display it in rviz by performing these actions:

```
$ rviz2
```

Initially when you open rviz for the first time you will see this screen:

![Screenshot](./docs/images/basic_rviz_without_config.png)

When you're on rviz you will need to go to the File > Open config tab at the top of the page.
And you will need to go to the rviz2_config folder in the repository and select the file from there.
Once you have added the configuration you will have this screen:

![Screenshot](./docs/images/rviz_window_with_config.png)
