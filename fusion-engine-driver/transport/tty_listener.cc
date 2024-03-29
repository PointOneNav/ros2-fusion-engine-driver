#include "tty_listener.hpp"

/******************************************************************************/
TtyListener::TtyListener(rclcpp::Node* node, const std::string& port)
    : node_(node), port_(port) {
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

  serial_port_.Open(port_.c_str(), 460800);
  running_ = true;
  while (running_) {
    ssize_t bytes_read = serial_port_.Read(&buffer[0], 1024);
  
    if (bytes_read < 0) {
      RCLCPP_INFO(node_->get_logger(), "Error reading from socket: %s (%d)",
                  std::strerror(errno), errno);
      break;
    } else if (bytes_read == 0) {
      // Since we read from the serial socket in a non blocking way,
      // it happens that not data is read from the socket,
      // hence returning 0 bytes and here skipping.
      continue;
    }
    total_bytes_read += bytes_read;
    callback_function_(buffer, bytes_read);
  }
}

/******************************************************************************/
void TtyListener::write(uint8_t* data, size_t size) { serial_port_.Write(data, size); }
