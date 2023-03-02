#ifndef FUSION_ENGINE_RECEIVER_HPP
#define FUSION_ENGINE_RECEIVER_HPP

#include <cerrno>
#include <cmath> // For lround()
#include <csignal> // For signal()
#include <cstdio> // For fprintf()
#include <cstring> // For memcpy()
#include <string> // For stoi() and strerror()
#include <netdb.h> // For gethostbyname() and hostent
#include <netinet/in.h> // For IPPROTO_* macros and htons()
#include <sys/socket.h> // For socket support.
#include <unistd.h> // For close()
#include <vector>
#include <iostream>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "atlas_byte_frame_listener.hpp"
#include "atlas_byte_frame_event.hpp"

/*
 * Reads bit stream from the Point One Nav Atlas and notifies all event 
 * listeners attached to this singleton object once a raw data packet 
 * been receieved.
 */
class FusionEngineReceiver {
public:

  /**
   * Singleton object. Only one message parser is necessary.
   * @return static reference to single FusionEngineReceiver instance.
   */
  static FusionEngineReceiver & getInstance() {
    static FusionEngineReceiver instance; // static method field instatiated once
    return instance;
  }

  /**
   * Initialize needed to set a ros envoronment for logging output.
   * @param node link to ros environment.
   * @return Nothing.
   */
  void initialize(rclcpp::Node * node, int udp_port, std::string connection_type, std::string tcp_ip, int tcp_port) {
    node_ = node;
    udp_port_ = udp_port;
    connection_type_ = connection_type;
    tcp_ip_ = tcp_ip;
    tcp_port_ = tcp_port;
  }

  /**
   * Adds an event listener to be notified for every byte frames received.
   * @param listener listener to be notified for byte frames received.
   * @return Nothing.
   */
  void addByteFrameListener(AtlasByteFrameListener & listener) {
    listenerList.push_back(&listener);
  }

  /**
   * Getter for type of network transport layer protocol (udp/tcp).
   * @return String identification of connection type protocol.
   */
  std::string get_connection_type() {
    return connection_type_;
  }

private:
  int udp_port_ = 0;
  int tcp_port_ = 0;
  std::string connection_type_ = "";
  std::string tcp_ip_ = "";
  int sock_ = 0;
  std::vector<AtlasByteFrameListener *> listenerList;
  rclcpp::Node * node_;

  /* private constructor for singleton design */
  FusionEngineReceiver() {}

  /**
   * Notifies all AtlasByteFrameListeners of a newly recieved byte frame.
   * @param frame Raw byte frame received.
   * @param bytes_read Size of byte frame.
   * @param frame_ip Frame source ip.
   * @return Nothing.
   */
  void DecodeFusionEngineMessage(uint8_t * frame, size_t bytes_read) {
    AtlasByteFrameEvent evt(frame, bytes_read);
    for(AtlasByteFrameListener * listener : listenerList) {
      listener->receivedAtlasByteFrame(evt);
    }
  }

  /**
   * Creates and binds to a UDP network socket.
   * @return status code 0 on success, non-zero otherwise.
   */
  int open_udp_socket() {
    // create UDP socket.
    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sock_ < 0) {
      RCLCPP_INFO(node_->get_logger(), "Error creating UDP socket.");
      return 2;
    }   
    // bind socket to port.
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(udp_port_);
    addr.sin_addr.s_addr = INADDR_ANY; // any local address
    int ret = bind(sock_, (struct sockaddr *) &addr, sizeof(addr));
    if(ret < 0) {
      close(sock_);
      RCLCPP_INFO(node_->get_logger(), "Error binding UDP");
      return 3;
    }
    RCLCPP_INFO(node_->get_logger(), "Opened port %d", udp_port_);
    return 0;
  }

  /**
   * Creates and connects to a tcp network socket.
   * @return status code 0 on success, non-zero otherwise.
   */
  int open_tcp_socket() {
    // Connect the socket.
    sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock_ < 0) {
      RCLCPP_INFO(node_->get_logger(), "Error creating socket");
      return 2;
    }
    const char* hostname = tcp_ip_.c_str();
    hostent* host_info = gethostbyname(hostname);
    if (host_info == NULL) {
      RCLCPP_INFO(node_->get_logger(), "Error: IP address lookup failed for hostname '%s'.\n", hostname);
      return 1;
    }
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(tcp_port_);
    memcpy(&addr.sin_addr, host_info->h_addr_list[0], host_info->h_length);
    int ret = connect(sock_, (sockaddr*)&addr, sizeof(addr));
    if (ret < 0) {
      close(sock_);
      RCLCPP_INFO(node_->get_logger(), "Error connecting to target device: %s (%d)\n", std::strerror(errno),
            errno);
      return 3;
    }
    RCLCPP_INFO(node_->get_logger(), "Opened port %d at ip %s", tcp_port_, hostname);
    return 0;
  }

  /**
   * IPv4 and IPv5 address resolution.
   * @param sa socket address.
   * @return Resolved ip address.
   */
  void * get_in_addr(struct sockaddr * sa) {
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in *)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6 *)sa)->sin6_addr);
  }
};

#endif