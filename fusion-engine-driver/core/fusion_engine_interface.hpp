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
   * @brief Construct a new Fusion Engine Interface object
   *
   * @param funcPublisher
   */
  FusionEngineInterface(
      std::function<void(const MessageHeader& header, const void* payload_in)>
          funcPublisher);

  void initialize(rclcpp::Node* node, const std::string& tcp_ip, int tcp_port);

  void initialize(rclcpp::Node* node, int udp_port);

  void initialize(rclcpp::Node* node, const std::string& tty_port);

  /**
   * Callback function for every new parsed message received from Atlas.
   * @param header Metadata on payload.
   * @param payload_in Message received.
   */
  void messageReceived(const MessageHeader& header, const void* payload_in);

  /**
   * @brief Call fusion engine decoder.
   *
   * @param frame Message content.
   * @param bytes_read Message size.
   */
  void DecodeFusionEngineMessage(uint8_t* frame, size_t bytes_read);

  /**
   * Main service to receive gps data from Atlas.
   */
  void service();

  void write(uint8_t* data, size_t size);

 private:
  point_one::fusion_engine::parsers::FusionEngineFramer framer;
  std::function<void(const MessageHeader& header, const void* payload_in)>
      publisher;
  rclcpp::Node* node_;
  std::shared_ptr<DataListener> data_listener_;
};
