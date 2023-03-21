#ifndef ATLAS_MESSAGE_EVENT_HPP
#define ATLAS_MESSAGE_EVENT_HPP

#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <point_one/fusion_engine/messages/ros.h>

using namespace point_one::fusion_engine::messages;
using namespace point_one::fusion_engine::messages::ros;

/**
 * Data class that wraps message data in a generic object.
 */
class FusionEngineMessageEvent {
public:
  gps_msgs::msg::GPSFix gps_fix_;
  sensor_msgs::msg::Imu imu_;
  geometry_msgs::msg::PoseStamped pose_;
  MessageType type_;

  FusionEngineMessageEvent(geometry_msgs::msg::PoseStamped pose, const MessageType &type)
      : pose_(pose), type_(type) {}

  FusionEngineMessageEvent(gps_msgs::msg::GPSFix gps_fix, const MessageType &type) 
      : gps_fix_(gps_fix), type_(type) {}

  FusionEngineMessageEvent(sensor_msgs::msg::Imu imu, const MessageType &type)
      : imu_(imu), type_(type) {}

};

#endif