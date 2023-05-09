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

/**
 * @brief A class for listening to data over TCP/IP.
 */
class TcpListener : public DataListener {
 public:
  /**
   * @brief Constructs a TcpListener object.
   *
   * @param node A pointer to the ROS2 node to use for logging.
   * @param ip The IP address to listen on.
   * @param port The port number to listen on.
   */
  TcpListener(rclcpp::Node *node, const std::string &ip, const int &port);

  /**
   * @brief Destroys the TcpListener object.
   */
  ~TcpListener() = default;

  /**
   * @brief Sets a callback function to be executed when data is received.
   *
   * @param func The callback function to set.
   */
  void setCallback(const std::function<void(uint8_t *, size_t)> &func);

  /**
   * @brief Listens for incoming data.
   */
  void listen();

  /**
   * @brief Writes data to the TCP connection.
   *
   * @param data A pointer to the data to write.
   * @param size The size of the data to write.
   */
  void write(uint8_t *data, size_t size);

 private:
  /**
   * @brief Opens a TCP connection.
   */
  int open();

  /**
   * @brief A pointer to the ROS2 node for logging.
   */
  rclcpp::Node *node_;

  /**
   * @brief The IP address to listen on.
   */
  std::string _ip;

  /**
   * @brief The port number to listen on.
   */
  int _port;

  /**
   * @brief The file descriptor of the TCP connection.
   */
  int sock_ = 0;

  /**
   * @brief The callback function to execute on incoming data.
   */
  std::function<void(uint8_t *, size_t)> callback_function_;
};
