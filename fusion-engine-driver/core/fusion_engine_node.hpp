#include <chrono>
#include <functional>
#include <memory>

#include "fusion_engine_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "gps_msgs/msg/gps_status.hpp"
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
   * Callback function triggered by atlas receiving a complete message.
   * @param evt GPS/IMU data.
   * @return Nothing.
   */
  void receivedFusionEngineMessage(const MessageHeader &header,
                                   const void *payload);
  /**
   * Translate GPSFix to NavFixMsg
   * @param gps_fix Atlas gps data point.
   * @return Nothing.
   */
  void publishNavFixMsg(const gps_msgs::msg::GPSFix &gps_fix);

 private:
  FusionEngineInterface gps;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_fix_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;
  int id = 0;

  /**
   * Initiate gps unit to read data.
   */
  void serviceLoopCb();
};
