#pragma once

#include <netdb.h>       // For gethostbyname() and hostent
#include <netinet/in.h>  // For IPPROTO_* macros and htons()
#include <sys/socket.h>  // For socket support.
#include <unistd.h>      // For close()

#include <csignal>  // For signal()
#include <cstdio>   // For fprintf()
#include <cstring>  // For memcpy()
#include <iostream>
#include <string>  // For stoi() and strerror()

#include "data_listener.hpp"
#include "rclcpp/rclcpp.hpp"

class TcpListener : public DataListener {
 public:
  TcpListener(rclcpp::Node *node, const std::string &ip, const int &port);
  ~TcpListener() = default;

  void setCallback(const std::function<void(uint8_t *, size_t)> &func);
  void listen();
  void write(uint8_t *data, size_t size);

 private:
  int open();

  rclcpp::Node *node_;
  std::string _ip;
  int _port;
  int sock_ = 0;
  std::function<void(uint8_t *, size_t)> callback_function_;
};
