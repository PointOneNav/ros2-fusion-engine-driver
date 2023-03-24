#ifndef TTY_LISTENER_HPP_
#define TTY_LISTENER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "data_listener.hpp"
#include "serial_port_reader.hpp"

class TtyListener : public DataListener
{
    public:
        TtyListener(rclcpp::Node* node, const std::string &port) :
            node_(node),
            port_(port)
        {
        }

        ~TtyListener() = default;

        void setCallback(const std::function<void(uint8_t*, size_t)>&func)
        {
            callback_function_ = func;
        }

        void listen()
        {
            uint8_t buffer[1024];
            size_t total_bytes_read = 0;
            SerialPortReader p(port_);

            while(rclcpp::ok()) {
                ssize_t bytes_read = p.portRead(1024, &buffer[0]);
                if(bytes_read < 0) {
                    RCLCPP_INFO(node_->get_logger(), "Error reading from socket: %s (%d)", std::strerror(errno), errno);
                    break;
                }   
                else if (bytes_read == 0) {
                    // RCLCPP_INFO(node_->get_logger(), "Socket closed remotely.");
                }
                total_bytes_read += bytes_read;
                callback_function_(buffer, bytes_read);
            }
            p.~SerialPortReader();
        }

    private:

        int open()
        {
            return 0;
        }

        rclcpp::Node * node_;
        std::string port_;
        std::function<void(uint8_t*, size_t)> callback_function_;
};


#endif /* !TTY_LISTENER_HPP_ */
