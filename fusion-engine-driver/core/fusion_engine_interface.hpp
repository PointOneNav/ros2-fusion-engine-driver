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

using namespace point_one::fusion_engine::messages;
using namespace point_one::fusion_engine::messages::ros;

/**
 * @brief Reads bit stream from the Point One Nav Atlas and notifies all event
 * listeners attached to this object once a complete message has
 * been received according to the good type of reveiver manage in this
 * interface.
 *
 */
class FusionEngineInterface {
 public:
  /**
   * @brief Construct a new Fusion Engine Interface object
   *
   * @param funcPublisher Receiver for Fusion Engine Message.
   */
  FusionEngineInterface(
      std::function<void(const MessageHeader& header, const void* payload_in)>
          funcPublisher);

  /**
   * @brief Initialize the Fusion Engine interface with the correct type of data
   * listener, in this case TCP.
   *
   * @param node The address of the node to be able to use the logging system
   * provided by ROS.
   * @param tcp_ip The IP address of the TCP server.
   * @param tcp_port The port number used by the server.
   */
  void initialize(rclcpp::Node* node, const std::string& tcp_ip, int tcp_port);

  /**
   * @brief Initialize the Fusion Engine interface with the correct type of data
   * listener, in this case Serial.
   *
   * @param node The address of the node to be able to use the logging system
   * provided by ROS.
   * @param tty_port The targeted serial port.
   */
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
  void decodeFusionEngineMessage(uint8_t* frame, size_t bytes_read);

  /**
   * Main service to receive gps data from Atlas.
   */
  void dataListenerService();

  /**
   * @brief Write data to a communication interface.
   *
   * @param data Pointer to an array of `uint8_t` type representing the data to
   * write.
   * @param size Size of the `data` array.
   */
  void write(uint8_t* data, size_t size);

  /**
   * @brief Send a signal to stop the listener.
   *
   */
  void stop();

 private:
  /**
   * @brief Provide a framer for obtaining messages from the data read by
   * a device using Fusion Engine.
   *
   * This framer is used to extract messages from the raw data read.
   * It formats the data into messages that can be parsed and used by other
   * parts of the system.
   *
   * @note This function is specific to the FusionEngine system and may not be
   * applicable in other contexts.
   */
  point_one::fusion_engine::parsers::FusionEngineFramer framer;

  /**
   * @brief Function object for publishing data received by the framer.
   *
   * This function object takes in a `MessageHeader` object and a payload input,
   * processes the data, and publishes it to some external system or service.
   *
   * @param header A constant reference to a `MessageHeader` object.
   * @param payload_in A constant void pointer to the input payload.
   *
   */
  std::function<void(const MessageHeader& header, const void* payload_in)>
      publisher;

  /**
   * @brief Pointer to a ROS 2 node for accessing the logging system.
   *
   * This pointer is used to create a ROS 2 node and access its logging system,
   * provided by the `rclcpp` library. The logging system allows for the output
   * of messages to the console and other loggers, with various levels of
   * severity.
   *
   */
  rclcpp::Node* node_;

  /**
   * @brief Smart pointer to a data listener interfaces.
   *
   * This smart pointer provides access to a data listener interface, which is
   * created in the `initialize` functions. The data listener interface is used
   * to receive data from a specific source, such as a TCP or serial port.
   *
   */
  std::shared_ptr<DataListener> data_listener_;
};
