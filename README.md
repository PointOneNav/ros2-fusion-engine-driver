# Point One FusionEngine ROS 2 Driver

This project contains support for interacting with the Point One navigation engine in ROS 2 using the FusionEngine
protocol. The driver can be configured to talk to a FusionEngine device over TCP, UDP, or serial connection. The ROS
driver will receive data from the FusionEngine device, and will publish the following ROS topics:

* [pose](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)
* [gps_fix](http://docs.ros.org/en/hydro/api/gps_common/html/msg/GPSFix.html)
* [fix](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html)
* [imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)
* [visualization_marker](http://wiki.ros.org/rviz/DisplayTypes/Marker)

See https://github.com/PointOneNav/fusion-engine-client for the latest details and support code for the Point One
FusionEngine protocol.

This library is released under the [MIT license agreement](LICENSE). We welcome code and documentation contributions
from users. See [Contributing](CONTRIBUTING.md) for instructions on how to report issues and develop, test, and submit
code changes.

If you encounter an issue, please [submit a ticket](CONTRIBUTING.md#reporting-issues). If you need support with Point
One FusionEngine or a Point One device (Atlas, Quectel LG69T, etc.), please contact support@pointonenav.com.

### Table Of Contents
* [Requirements](#requirements)
* [Installation](#installation)
* [Usage](#usage)
  * [Connecting Over TCP](#connecting-over-tcp)
  * [Connecting Over Serial](#connecting-over-serial)
* [Visualizing Output With `rviz2`](#visualizing-output-with-rviz2)

### Requirements

- C++11 or later
- CMake 3.x
- GCC, Clang, or Microsoft Visual Studio
- [ROS 2](https://docs.ros.org/en/humble/Installation.html)
- The ROS `gps-msgs` package for the version of ROS 2 that you installed above. For example, if you installed ROS 2
  Humble:
  ```
  sudo apt-get install ros-humble-gps-msgs
  ```
- The ROS `nmea-msgs` package for the version of ROS 2 that you installed above. For example, if you installed ROS 2
  Humble:
  ```
  sudo apt install ros-humble-nmea-msgs
  ```
- The ROS `mavros-msgs` package for the version of ROS 2 that you installed above. For example, if you installed ROS 2
  Humble:
  ```
   sudo apt install ros-humble-mavros
  ```

- A Point One FusionEngine device (https://pointonenav.com/product/)

## Installation

Before using the ROS 2 driver, you must configure your FusionEngine device to output the following messages:
- `ROSPoseMessage`
- `ROSGPSFixMessage`
- `ROSIMUMessage` (optional)

You may configure your Point One device using the Point One Desktop Application or Python Configuration Tool. Both
tools are available at https://pointonenav.com/docs/#standard-dev-kit. See the sections below for instructions on using
the Configuration Tool (`config_tool`).

With your device configured, you may now compile and install the ROS 2 driver:

```
git clone https://github.com/PointOneNav/ros2-fusion-engine-driver.git
cd ros2-fusion-engine-driver
rosdep install -i --from-path ./ --rosdistro humble -y
colcon build --packages-select fusion-engine-driver
source install/local_setup.bash
```

_Note that you should replace `humble` with the appropriate version of ROS 2 above._

### Configuring A FusionEngine Device Using `config_tool.exe` (Windows)

The following example configures serial UART2 on a Quectel LG69T device to output the required FusionEngine ROS
messages:

```
config_tool.exe apply uart2_message_rate fe ROSPoseMessage 100ms
config_tool.exe apply uart2_message_rate fe ROSGPSFixMessage 100ms
config_tool.exe apply uart2_message_rate fe ROSIMUMessage on
config_tool.exe save
```

### Configuring A FusionEngine Device Using `config_tool.py` (Linux/Mac)

_Note: For Python, we strongly recommend using a virtual environment to avoid conflicts between the application's
requirements and other Python applications on your computer:_

```
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

The following example configures serial UART2 on a Quectel LG69T device to output the required FusionEngine ROS
messages:

```
config_tool.py apply uart2_message_rate fe ROSPoseMessage 100ms
config_tool.py apply uart2_message_rate fe ROSGPSFixMessage 100ms
config_tool.py apply uart2_message_rate fe ROSIMUMessage 100ms
config_tool.py save
```

## Usage

Before running the ROS 2 driver, you must first source the ROS setup script as follows:
```
source install/local_setup.bash
```

Next, run the ROS 2 driver and configure it to connect to the FusionEngine device. You may connect over TCP, UDP, or
serial, depending on which transports your device supports.

### Connecting Over TCP

To connect to a device over TCP, specify the device hostname/IP address and port as shown below:

```
ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args -p connection_type:=tcp -p tcp_ip:=192.168.1.3 -p tcp_port:=12345
```

#### Connecting Over TCP Through `p1_runner`

For some devices, the `p1_runner` Python application can be used to log data and supply GNSS corrections from an NTRIP
server. If you are using the `p1_runner` to communicate with a device over serial, you can configure `p1_runner` to
relay the data to the ROS 2 driver over TCP as follows:

```
p1_runner/bin/runner.py --tcp 12345
```

Then you can configure the driver to connect to `p1_runner` on `localhost`:

```
ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args -p connection_type:=tcp -p tcp_ip:=localhost -p tcp_port:=12345
```

### Connecting Over Serial

In serial mode, it is possible to access the rtk correction. To do this you need to run the ntrip client node of ros2 in parallel.
To connect to a ntrip server, you have to follow the instructions as follows:

```
git clone https://github.com/LORD-MicroStrain/ntrip_client
cd ntrip_client
git checkout ros2
cd ..
colcon build --packages-select ntrip_client
source install/local_setup.bash
```
Once these commands are done you can launch the node, here is the list of parameters and an example of execution.

Optional launch parameters:
  - host: Hostname or IP address of the NTRIP server to connect to and receive corrections from 
  - port: Port to connect to on the server. Default: 2101 - mountpoint: Mountpoint to connect to on the NTRIP server
  - authenticate: Whether or not to authenticate with the server, or send an unauthenticated request. If set to true, username, and password must be supplied.
  - username: Username to use when authenticating with the NTRIP server. Only used if authenticate is true
  - password: Password to use when authenticating with the NTRIP server. Only used if authenticate is true

```
ros2 launch ntrip_client ntrip_client_launch.py host:=[HOST] mountpoint:=[MOUNTPOINT]  username:=[YOUR USERNAME] password:=[YOUR PASSWORD]
```

[Here is the complete documentation for ntrip client node.](https://index.ros.org/r/ntrip_client/)

Once the node is connected to the server you can connect the node in serial.
To connect to a device over serial, specify the serial port device name as follows:

```
ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args -p connection_type:=tty -p tty_port:=/dev/ttyUSB1
```

## Visualizing Output With `rviz2`

Once your ROS 2 driver is running and communicating with the device, you should be able to display its output using the
ROS 2 `rviz2` tool.

1. Open `rviz2`:
   ```
   rviz2
   ```
2. Go to File -> Open Config and select
   [rviz2_config/fusion_engine_driver.rviz](rviz2_config/fusion_engine_driver.rviz).
3. You should see the following display, and the output from the device will be shown as it arrives:
   ![rviz2 Screenshot](./docs/images/rviz_window_with_config.png)
