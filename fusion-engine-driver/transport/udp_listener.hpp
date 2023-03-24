#pragma once

#include <arpa/inet.h>
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

class UdpListener : public DataListener {
 public:
  UdpListener(rclcpp::Node *node, const int &port);
  ~UdpListener() = default;

  void setCallback(const std::function<void(uint8_t *, size_t)> &func);
  void listen();

 private:
  int open();
  void *getInAddr(struct sockaddr *sa);

  std::function<void(uint8_t *, size_t)> callback_function_;
  rclcpp::Node *node_;
  int port_;
  int sock_ = 0;
};
