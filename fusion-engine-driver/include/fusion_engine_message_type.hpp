/*
** EPITECH PROJECT, 2023
** ros2-fusion-engine-driver
** File description:
** fusion_message_type
*/

#ifndef FUSION_MESSAGE_TYPE_HPP_
#define FUSION_MESSAGE_TYPE_HPP_

/**
 * @breif Atlas message type definitions.
 */
enum class FusionEngineMessageType {
  /** Standard ROS GPSFix message. 
   *  @see http://docs.ros.org/api/gps_common/html/msg/GPSFix.html
   * */
  GPS_FIX = 0,
  /** Standard ROS IMU message. 
   *  @see http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
   * */
  IMU = 1,
  /** Standard ROS POSE message. 
   *  @see https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
   * */
  POSE = 2,
};

#endif
