// #ifndef FUSION_ENGINE_RECEIVER_HPP
// #define FUSION_ENGINE_RECEIVER_HPP

// #include <cerrno>
// #include <cmath> // For lround()
// #include <csignal> // For signal()
// #include <cstdio> // For fprintf()
// #include <cstring> // For memcpy()
// #include <string> // For stoi() and strerror()
// #include <netdb.h> // For gethostbyname() and hostent
// #include <netinet/in.h> // For IPPROTO_* macros and htons()
// #include <sys/socket.h> // For socket support.
// #include <unistd.h> // For close()
// #include <vector>
// #include <iostream>
// #include <arpa/inet.h>

// #include "rclcpp/rclcpp.hpp"
// #include "atlas_byte_frame_listener.hpp"
// #include "atlas_byte_frame_event.hpp"

// /*
//  * Reads bit stream from the Point One Nav Atlas and notifies all event 
//  * listeners attached to this singleton object once a raw data packet 
//  * been receieved.
//  */
// class FusionEngineReceiver {
// public:


//   FusionEngineReceiver() = default;
//   /**
//    * Singleton object. Only one message parser is necessary.
//    * @return static reference to single FusionEngineReceiver instance.
//    */
//   static FusionEngineReceiver & getInstance() {
//     static FusionEngineReceiver instance; // static method field instatiated once
//     return instance;
//   }

//   /**
//    * Initialize needed to set a ros envoronment for logging output.
//    * @param node link to ros environment.
//    * @return Nothing.
//    */
//   void initialize(rclcpp::Node * node, int udp_port, std::string connection_type, std::string tcp_ip, int tcp_port) {
//     node_ = node;
//     udp_port_ = udp_port;
//     connection_type_ = connection_type;
//     tcp_ip_ = tcp_ip;
//     tcp_port_ = tcp_port;
//   }

//   /**
//    * Adds an event listener to be notified for every byte frames received.
//    * @param listener listener to be notified for byte frames received.
//    * @return Nothing.
//    */
//   void addByteFrameListener(AtlasByteFrameListener & listener) {
//     listenerList.push_back(&listener);
//   }

//   /**
//    * Getter for type of network transport layer protocol (udp/tcp).
//    * @return String identification of connection type protocol.
//    */
//   std::string get_connection_type() {
//     return connection_type_;
//   }

// private:
//   int udp_port_ = 0;
//   int tcp_port_ = 0;
//   std::string connection_type_ = "";
//   std::string tcp_ip_ = "";
//   int sock_ = 0;
//   std::vector<AtlasByteFrameListener *> listenerList;
//   rclcpp::Node * node_;
// };

// #endif