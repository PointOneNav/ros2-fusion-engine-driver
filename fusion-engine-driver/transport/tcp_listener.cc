#include "tcp_listener.hpp"

/******************************************************************************/
TcpListener::TcpListener(rclcpp::Node* node, const std::string& ip,
                         const int& port)
    : node_(node), _ip(ip), _port(port) {}

/******************************************************************************/
void TcpListener::setCallback(
    const std::function<void(uint8_t*, size_t)>& func) {
  callback_function_ = func;
}

/******************************************************************************/
void TcpListener::listen() {
  uint8_t buffer[1024];
  size_t total_bytes_read = 0;

  open();
  try {
    while (rclcpp::ok()) {
      ssize_t bytes_read = recv(sock_, buffer, sizeof(buffer), 0);
      if (bytes_read < 0) {
        RCLCPP_INFO(node_->get_logger(), "Error reading from socket: %s (%d)",
                    std::strerror(errno), errno);
        break;
      } else if (bytes_read == 0) {
        RCLCPP_INFO(node_->get_logger(), "Socket closed remotely.");
        break;
      }
      total_bytes_read += bytes_read;
      callback_function_(buffer, bytes_read);
    }
  } catch (std::exception const& ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "Decoder exception: " << ex.what());
  }
}

/******************************************************************************/
int TcpListener::open() {
  // Connect the socket.
  sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock_ < 0) {
    RCLCPP_INFO(node_->get_logger(), "Error creating socket");
    return 2;
  }
  hostent* host_info = gethostbyname(_ip.c_str());
  if (host_info == NULL) {
    RCLCPP_INFO(node_->get_logger(),
                "Error: IP address lookup failed for hostname '%s'.\n",
                _ip.c_str());
    return 1;
  }
  sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(_port);
  memcpy(&addr.sin_addr, host_info->h_addr_list[0], host_info->h_length);
  int ret = connect(sock_, (sockaddr*)&addr, sizeof(addr));
  if (ret < 0) {
    close(sock_);
    RCLCPP_INFO(node_->get_logger(),
                "Error connecting to target device: %s (%d)\n",
                std::strerror(errno), errno);
    return 3;
  }
  RCLCPP_INFO(node_->get_logger(), "Opened port %d at ip %s", _port,
              _ip.c_str());
  return 0;
}
