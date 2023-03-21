#ifndef ATLAS_MESSAGE_EVENT_HPP
#define ATLAS_MESSAGE_EVENT_HPP

#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <point_one/fusion_engine/messages/ros.h>

/**
 * Data class that wraps message data in a generic object.
 */
class FusionEngineMessageEvent {
public:
  gps_msgs::msg::GPSFix gps_fix;
  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::PoseStamped pose;
  size_t typeId;

  FusionEngineMessageEvent(geometry_msgs::msg::PoseStamped pose_)
      : pose(pose_), typeId(typeid(pose_).hash_code()) {}

  FusionEngineMessageEvent(gps_msgs::msg::GPSFix gps_fix_) 
      : gps_fix(gps_fix_), typeId(typeid(gps_fix_).hash_code()) {}

  FusionEngineMessageEvent(sensor_msgs::msg::Imu imu_)     
      : imu(imu_), typeId(typeid(imu_).hash_code()) {}

};

#endif