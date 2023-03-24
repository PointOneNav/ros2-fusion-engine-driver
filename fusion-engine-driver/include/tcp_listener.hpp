#ifndef TCP_LISTENER_HPP_
#define TCP_LISTENER_HPP_

#include <csignal> // For signal()
#include <cstdio> // For fprintf()
#include <cstring> // For memcpy()
#include <string> // For stoi() and strerror()
#include <netdb.h> // For gethostbyname() and hostent
#include <netinet/in.h> // For IPPROTO_* macros and htons()
#include <sys/socket.h> // For socket support.
#include <unistd.h> // For close()
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "data_listener.hpp"

class TcpListener : public DataListener
{
    public:
        TcpListener(rclcpp::Node* node, const std::string &ip, const int &port);
        ~TcpListener() = default;

        void setCallback(const std::function<void(uint8_t*, size_t)>&func);
        void listen();

    private:

        int open();

        rclcpp::Node * node_;
        std::string _ip;
        int _port;
        int sock_ = 0;
        std::function<void(uint8_t*, size_t)> callback_function_;
};

#endif /* !TCP_LISTENER_HPP_ */
