#pragma once

#include "data_listener.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_port_reader.hpp"

class TtyListener : public DataListener {
 public:
  TtyListener(rclcpp::Node* node, const std::string& port);

  ~TtyListener() = default;

  void setCallback(const std::function<void(uint8_t*, size_t)>& func);

  void listen();

 private:
  rclcpp::Node* node_;
  std::string port_;
  std::function<void(uint8_t*, size_t)> callback_function_;
};
