#pragma once

#include <point_one/fusion_engine/messages/core.h>
#include <point_one/fusion_engine/messages/ros.h>
#include <point_one/fusion_engine/parsers/fusion_engine_framer.h>

#include <cstdio>
#include <vector>

#include "conversion_utils.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "tcp_listener.hpp"
#include "tty_listener.hpp"
#include "udp_listener.hpp"

using namespace point_one::fusion_engine::messages;
using namespace point_one::fusion_engine::messages::ros;

/*
 * Reads bit stream from the Point One Nav Atlas and notifies all event
 * listeners attached to this singelton object once a complete message has
 * been received.
 */
class FusionEngineInterface {
 public:
  /**
   * Initialize needed to set a ros envoronment for logging output.
   * @param node Link to ROS environment.
   * @return Nothing.
   */
  FusionEngineInterface(
      std::function<void(const MessageHeader& header, const void* payload_in)>
          funcPublisher);

  void initialize(rclcpp::Node* node, const std::string &tcp_ip, int tcp_port);

  void initialize(rclcpp::Node* node, int udp_port);

  void initialize(rclcpp::Node* node, const std::string &tty_port);

  /**
   * Callback function for every new parsed message received from Atlas.
   * @param header Metadata on payload.
   * @param payload_in Message received.
   * @return Nothing.
   */
  void messageReceived(const MessageHeader& header, const void* payload_in);

  /**
   * Notifies all AtlasByteFrameListeners of a newly recieved byte frame.
   * @param frame Raw byte frame received.
   * @param bytes_read Size of byte frame.
   * @param frame_ip Frame source ip.
   * @return Nothing.
   */
  void DecodeFusionEngineMessage(uint8_t* frame, size_t bytes_read);

  /**
   * Main service to receive gps data from Atlas.
   * @return Nothing.
   */
  void service();

 private:
  point_one::fusion_engine::parsers::FusionEngineFramer framer;
  std::function<void(const MessageHeader& header, const void* payload_in)>
      publisher;
  rclcpp::Node* node_;
  std::shared_ptr<DataListener> data_listener_;
};
