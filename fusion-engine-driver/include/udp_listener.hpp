#ifndef UDP_LISTENER_HPP_
#define UDP_LISTENER_HPP_

#include <csignal> // For signal()
#include <cstdio> // For fprintf()
#include <cstring> // For memcpy()
#include <string> // For stoi() and strerror()
#include <netdb.h> // For gethostbyname() and hostent
#include <netinet/in.h> // For IPPROTO_* macros and htons()
#include <sys/socket.h> // For socket support.
#include <unistd.h> // For close()
#include <iostream>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "data_listener.hpp"

class UdpListener : public DataListener
{
    public:
        UdpListener(rclcpp::Node* node, const int &port) :
            node_(node),
            port_(port)
        {

        }
        ~UdpListener() = default;

        void setCallback(const std::function<void(uint8_t*, size_t)>&func)
        {
            callback_function_ = func;
        }

        void listen()
        {
            uint8_t buffer[1024];
            size_t total_bytes_read = 0;
            struct sockaddr_storage their_addr;
            socklen_t addr_len = sizeof(their_addr);
            char their_ip[INET6_ADDRSTRLEN];

            open();

            try {
                while(rclcpp::ok()) {
                    ssize_t bytes_read = recvfrom(sock_, buffer, sizeof(buffer), 0, (struct sockaddr *)&their_addr, &addr_len);
                    if(bytes_read < 0) {
                        RCLCPP_INFO(node_->get_logger(), "Error reading from socket: %s (%d)", std::strerror(errno), errno);
                        break;
                    }   
                        else if (bytes_read == 0) {
                        RCLCPP_INFO(node_->get_logger(), "Socket closed remotely.");
                        break;
                    }
                    inet_ntop(their_addr.ss_family, get_in_addr((struct sockaddr *)&their_addr), their_ip, sizeof(their_ip));
                    total_bytes_read += bytes_read;
                    callback_function_(buffer, bytes_read);
                }
                close(sock_);
                RCLCPP_INFO(node_->get_logger(), "Finished. %zu bytes read.", total_bytes_read);
            } catch(std::exception const & ex) {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "Decoder exception: " << ex.what());
            }
        }

    private:

        int open()
        {
            sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if(sock_ < 0) {
                RCLCPP_INFO(node_->get_logger(), "Error creating UDP socket.");
                return 2;
            }   
            sockaddr_in addr;
            addr.sin_family = AF_INET;
            addr.sin_port = htons(port_);
            addr.sin_addr.s_addr = INADDR_ANY;
            int ret = bind(sock_, (struct sockaddr *) &addr, sizeof(addr));
            if(ret < 0) {
                close(sock_);
                RCLCPP_INFO(node_->get_logger(), "Error binding UDP");
                return 3;
            }
            RCLCPP_INFO(node_->get_logger(), "Opened port %d", port_);
            return 0;
        }

        void * get_in_addr(struct sockaddr * sa)
        {
            if (sa->sa_family == AF_INET) {
                return &(((struct sockaddr_in *)sa)->sin_addr);
            }
            return &(((struct sockaddr_in6 *)sa)->sin6_addr);
        }

        std::function<void(uint8_t*, size_t)> callback_function_;
        rclcpp::Node * node_;
        int port_;
        int sock_ = 0;
};

#endif /* !UDP_LISTENER_HPP_ */
