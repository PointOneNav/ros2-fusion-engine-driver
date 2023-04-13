#include <chrono>
#include <functional>
#include <memory>

#include "fusion_engine_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "gps_msgs/msg/gps_status.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

/*
 * Point One Nav Atlas Node publishes realtime GPSFix/IMU messages.
 */
class FusionEngineNode : public rclcpp::Node {
 public:
  FusionEngineNode();

  /**
   * @brief Receive Fusion Message, build and post them in ros system.
   * 
   * @param header Message header with type of message.
   * @param payload Message content
   */
  void receivedFusionEngineMessage(const MessageHeader &header,
                                   const void *payload);
  /**
   * Translate GPSFix to NavFixMsg
   * @param gps_fix Atlas gps data point.
   */
  void publishNavFixMsg(const gps_msgs::msg::GPSFix &gps_fix);

 private:
  FusionEngineInterface fe_interface_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_fix_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_publisher_;
  rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr ntrip_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  double prev_time_;
  uint16_t satellite_nb_;
  std::string frame_id_;
  int id = 0;

  void receiveCorrection(const mavros_msgs::msg::RTCM::SharedPtr msg);

  /**
   * Initiate gps unit to read data.
   */
  void serviceLoopCb();
};
