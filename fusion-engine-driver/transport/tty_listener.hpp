#pragma once

#include "data_listener.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_port.hpp"

class TtyListener : public DataListener {
 public:
  TtyListener(rclcpp::Node* node, const std::string& port);

  ~TtyListener() = default;

  void setCallback(const std::function<void(uint8_t*, size_t)>& func);
  void listen();
  void write(uint8_t* data, size_t size);

 private:
  rclcpp::Node* node_;
  std::string port_;
  std::function<void(uint8_t*, size_t)> callback_function_;
  SerialPort serial_port_;
};
