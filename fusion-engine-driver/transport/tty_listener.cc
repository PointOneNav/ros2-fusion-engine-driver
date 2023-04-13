#include "tty_listener.hpp"

/******************************************************************************/
TtyListener::TtyListener(rclcpp::Node* node, const std::string& port)
    : node_(node), port_(port) {
  p.Open(port_.c_str(), 460800);
}

/******************************************************************************/
void TtyListener::setCallback(
    const std::function<void(uint8_t*, size_t)>& func) {
  callback_function_ = func;
}

/******************************************************************************/
void TtyListener::listen() {
  size_t total_bytes_read = 0;
  uint8_t buffer[1024];

  running_ = true;

  while (running_) {
    ssize_t bytes_read = p.Read(&buffer[0], 1024);
  
    if (bytes_read < 0) {
      RCLCPP_INFO(node_->get_logger(), "Error reading from socket: %s (%d)",
                  std::strerror(errno), errno);
      break;
    } else if (bytes_read == 0) {
      continue;
    }
    total_bytes_read += bytes_read;
    callback_function_(buffer, bytes_read);
  }
}

/******************************************************************************/
void TtyListener::write(uint8_t* data, size_t size) { p.Write(data, size); }