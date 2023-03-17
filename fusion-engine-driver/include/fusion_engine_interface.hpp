#ifndef POINT_ONE_NAV_ATLAS_HPP
#define POINT_ONE_NAV_ATLAS_HPP

#include <vector>
#include <cstdio>

#include <point_one/fusion_engine/messages/core.h>
#include <point_one/fusion_engine/parsers/fusion_engine_framer.h>
#include <point_one/fusion_engine/messages/core.h>
#include <point_one/fusion_engine/messages/ros.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "tcp_listener.hpp"
#include "udp_listener.hpp"
#include "tty_listner.hpp"
#include "fusion_engine_utils.hpp"
#include "fusion_engine_message_event.hpp"
#include "fusion_engine_receiver.hpp"

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
  FusionEngineInterface(std::function<void(FusionEngineMessageEvent & evt)> funcPublisher) :
    framer(1024),
    publisher(funcPublisher)
  {
    framer.SetMessageCallback(std::bind(&FusionEngineInterface::messageReceived, this, std::placeholders::_1, std::placeholders::_2));
  }

  void initialize(rclcpp::Node * node, std::string tcp_ip, int tcp_port) {
    this->node_ = node;
    data_listener_ = std::make_shared<TcpListener>(node_, tcp_ip, tcp_port);
    data_listener_->setCallback(std::bind(&FusionEngineInterface::DecodeFusionEngineMessage, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(node_->get_logger(), "Initialize connection_type tcp in port %d", tcp_port);
  }

  void initialize(rclcpp::Node * node, int udp_port) {
    this->node_ = node;
    data_listener_ = std::make_shared<UdpListener>(node_, udp_port);
    data_listener_->setCallback(std::bind(&FusionEngineInterface::DecodeFusionEngineMessage, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(node_->get_logger(), "Initialize connection_type udp in port %d", udp_port);
  }

  void initialize(rclcpp::Node * node) {
    this->node_ = node;
    data_listener_ = std::make_shared<TtyListener>(node_);
    data_listener_->setCallback(std::bind(&FusionEngineInterface::DecodeFusionEngineMessage, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(node_->get_logger(), "Initialize connection_type tty");
  }

  /**
   * Callback function for every new parsed message received from Atlas.
   * @param header Metadata on payload.
   * @param payload_in Message received.
   * @return Nothing.
   */
  void messageReceived(const MessageHeader & header, const void * payload_in) {
    auto payload = static_cast<const uint8_t*>(payload_in);

    if(header.message_type == MessageType::ROS_GPS_FIX) {
      auto & contents = *reinterpret_cast<const GPSFixMessage*>(payload); 
      gps_msgs::msg::GPSFix gps_fix = AtlasUtils::toGPSFix(contents);
      FusionEngineMessageEvent evt(gps_fix);
      publisher(evt);
    } 
    else if(header.message_type == MessageType::ROS_IMU) {
      auto & contents = *reinterpret_cast<const IMUMessage*>(payload);
      FusionEngineMessageEvent evt( AtlasUtils::toImu(contents) );
      publisher(evt);
    }
    else if (header.message_type == MessageType::ROS_POSE) {
      auto & contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::PoseMessage*>(payload);
      
      geometry_msgs::msg::PoseStamped pos =  AtlasUtils::toPose(contents);
      FusionEngineMessageEvent evt(pos);
      publisher(evt);
    }
  }

  /**
   * Notifies all AtlasByteFrameListeners of a newly recieved byte frame.
   * @param frame Raw byte frame received.
   * @param bytes_read Size of byte frame.
   * @param frame_ip Frame source ip.
   * @return Nothing.
   */
  void DecodeFusionEngineMessage(uint8_t * frame, size_t bytes_read) {
    framer.OnData(frame, bytes_read);
  }

  /**
   * Main service to receive gps data from Atlas.
   * @return Nothing.
   */
  void service() {
    RCLCPP_INFO(node_->get_logger(), "Start listening using connection_type");
    data_listener_->listen();
  }

private:
  point_one::fusion_engine::parsers::FusionEngineFramer framer;
  std::function<void(FusionEngineMessageEvent & evt)> publisher;
  rclcpp::Node * node_;
  std::shared_ptr<DataListener> data_listener_;
};

#endif