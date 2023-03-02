#ifndef POINT_ONE_NAV_ATLAS_HPP
#define POINT_ONE_NAV_ATLAS_HPP

#include <vector>
#include <cstdio>

#include "fusion_engine_framer.h"
#include "core.h"
#include "ros.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gps_msgs/msg/gps_fix.hpp"

#include "tcp_listener.hpp"
#include "udp_listener.hpp"

#include "atlas_message_listener.hpp"
#include "atlas_message_event.hpp"
#include "atlas_byte_frame_listener.hpp"
#include "atlas_byte_frame_event.hpp"
#include "fusion_engine_receiver.hpp"
#include "utils.hpp"

using namespace point_one::fusion_engine::messages;
using namespace point_one::fusion_engine::messages::ros;

namespace point_one {
  namespace fusion_engine {
    void messageReceived(const messages::MessageHeader&, const void*);
  }
}

/*
 * Reads bit stream from the Point One Nav Atlas and notifies all event
 * listeners attached to this singelton object once a complete message has
 * been received.
 */
class FusionEngineInterface : public AtlasByteFrameListener {

public:
  /**
   * Singleton object. Only one message parser is necessary.
   */
  static FusionEngineInterface & getInstance() {
    static FusionEngineInterface instance; // static method fields are instatiated once
    return instance;
  }

  /**
   * Initialize needed to set a ros envoronment for logging output.
   * @param node Link to ROS environment.
   * @return Nothing.
   */
  // void initialize(rclcpp::Node * node, int udp_port, std::string connection_type, std::string tcp_ip, int tcp_port) {
  //   recv.initialize(node, udp_port, connection_type, tcp_ip, tcp_port);
  //   this->node_ = node;
  // }

  void initialize(rclcpp::Node * node, std::string tcp_ip, int tcp_port) {
    this->node_ = node;
    data_listener_ = std::make_shared<TcpListener>(node_, tcp_ip, tcp_port);
    RCLCPP_INFO(node_->get_logger(), "Initialize connection_type tcp in port %d", tcp_port);
  }

  void initialize(rclcpp::Node * node, int udp_port) {
    this->node_ = node;
    data_listener_ = std::make_shared<UdpListener>(node_, udp_port);
    RCLCPP_INFO(node_->get_logger(), "Initialize connection_type udp in port %d", udp_port);
  }

  /**
   * Adds an event listener to be notified for every gps message received.
   * @param listener object to be notified for gps message received.
   * @return Nothing.
   */
  void addAtlasMessageListener(AtlasMessageListener & listener) {
    listenerList.push_back(&listener);
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
      AtlasMessageEvent evt(gps_fix);
      fireAtlasMessageEvent(evt);
    } 
    else if(header.message_type == MessageType::ROS_IMU) {
      auto & contents = *reinterpret_cast<const IMUMessage*>(payload);
      AtlasMessageEvent evt( AtlasUtils::toImu(contents) );
      fireAtlasMessageEvent(evt);
    }
    else if (header.message_type == MessageType::ROS_POSE) {
      auto & contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::PoseMessage*>(payload);
      
      geometry_msgs::msg::PoseStamped pos =  AtlasUtils::toPose(contents);
      AtlasMessageEvent evt(pos);
      fireAtlasMessageEvent(evt);
    }
  }

  /**
   * Callback function for every new byte frame received from Atlas.
   * @note Inherited from AtlasByteFrameListener interface.
   * @param evt Wrapper that holds the byte frame data recieved.
   * @return Nothing.
   */
  void receivedAtlasByteFrame(AtlasByteFrameEvent & evt) {
    framer.OnData(evt.frame, evt.bytes_read);
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
  std::vector<AtlasMessageListener *> listenerList;
  FusionEngineReceiver & recv;
  rclcpp::Node * node_;
  std::shared_ptr<DataListener> data_listener_;

  /* only one instance will exist - singleton object. */
  FusionEngineInterface() : framer(1024), recv(FusionEngineReceiver::getInstance()) {
    recv.addByteFrameListener(*this);
    framer.SetMessageCallback(point_one::fusion_engine::messageReceived);
  }

  /**
   * Notifies all AtlasMessageListeners of a newly recieved gps message.
   * @param evt data sent to listeners.
   * @return Nothing.
   */
  void fireAtlasMessageEvent(AtlasMessageEvent evt) {
    for(AtlasMessageListener * listener : listenerList) {
      listener->receivedAtlasMessage(evt);
    }
  }
};

// annoying
namespace point_one {
  namespace fusion_engine {
    void messageReceived(const messages::MessageHeader& header, const void* payload_in) {
      FusionEngineInterface::getInstance().messageReceived(header, payload_in);
    }
  }
}

#endif