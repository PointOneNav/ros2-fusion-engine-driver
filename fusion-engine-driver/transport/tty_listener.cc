#include "tty_listener.hpp"

/******************************************************************************/
TtyListener::TtyListener(rclcpp::Node* node, const std::string& port)
    : node_(node), port_(port) {}

/******************************************************************************/
void TtyListener::setCallback(const std::function<void(uint8_t*, size_t)>& func) {
  callback_function_ = func;
}

/******************************************************************************/
void TtyListener::listen() {
  uint8_t buffer[1024];
  size_t total_bytes_read = 0;
  SerialPortReader p(port_);

  while (rclcpp::ok()) {
    ssize_t bytes_read = p.portRead(1024, &buffer[0]);
    if (bytes_read < 0) {
      RCLCPP_INFO(node_->get_logger(), "Error reading from socket: %s (%d)",
                  std::strerror(errno), errno);
      break;
    } else if (bytes_read == 0) {
      // RCLCPP_INFO(node_->get_logger(), "Socket closed remotely.");
    }
    total_bytes_read += bytes_read;
    callback_function_(buffer, bytes_read);
  }
  p.~SerialPortReader();
}