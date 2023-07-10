#include "fusion_engine_interface.hpp"

/******************************************************************************/
FusionEngineInterface::FusionEngineInterface(
    std::function<void(const MessageHeader& header, const void* payload_in)>
        funcPublisher)
    : framer(2048), publisher(funcPublisher) {
  framer.SetMessageCallback(std::bind(&FusionEngineInterface::messageReceived,
                                      this, std::placeholders::_1,
                                      std::placeholders::_2));
}

/******************************************************************************/
void FusionEngineInterface::initialize(rclcpp::Node* node,
                                       const std::string& tcp_ip,
                                       int tcp_port) {
  this->node_ = node;
  data_listener_ = std::make_shared<TcpListener>(node_, tcp_ip, tcp_port);
  data_listener_->setCallback(
      std::bind(&FusionEngineInterface::decodeFusionEngineMessage, this,
                std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(node_->get_logger(), "Initialize connection_type tcp in port %d",
              tcp_port);
}

/******************************************************************************/
void FusionEngineInterface::initialize(rclcpp::Node* node,
                                       const std::string& tty_port) {
  this->node_ = node;
  data_listener_ = std::make_shared<TtyListener>(node_, tty_port);
  data_listener_->setCallback(
      std::bind(&FusionEngineInterface::decodeFusionEngineMessage, this,
                std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(node_->get_logger(), "Initialize connection_type tty on port %s",
              tty_port.c_str());
}

/******************************************************************************/
void FusionEngineInterface::messageReceived(const MessageHeader& header,
                                            const void* payload_in) {
  auto payload = static_cast<const uint8_t*>(payload_in);

  publisher(header, payload);
}

/******************************************************************************/
void FusionEngineInterface::decodeFusionEngineMessage(uint8_t* frame,
                                                      size_t bytes_read) {
  framer.OnData(frame, bytes_read);
}

/******************************************************************************/
void FusionEngineInterface::dataListenerService() {
  RCLCPP_INFO(node_->get_logger(), "Start listening using connection_type");
  data_listener_->listen();
}

/******************************************************************************/
void FusionEngineInterface::write(uint8_t* data, size_t size) {
  data_listener_->write(data, size);
}

/******************************************************************************/
void FusionEngineInterface::stop() { data_listener_->stop(); }
