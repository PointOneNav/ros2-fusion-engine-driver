#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" 
#include "gps_msgs/msg/gps_fix.hpp"
#include "gps_msgs/msg/gps_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "atlas_message_listener.hpp"
#include "atlas_message_event.hpp"
#include "atlas_message_type.hpp"
#include "atlas.hpp"

/*
 * Point One Nav Atlas Node publishes realtime GPSFix/IMU messages.
 */ 
class PointOneNavAtlasNode : public AtlasMessageListener, public rclcpp::Node {
public:
  PointOneNavAtlasNode() : Node("atlas_node"), gps(PointOneNavAtlas::getInstance()) {
    this->declare_parameter("atlas_udp_port", 23456);
    this->declare_parameter("atlas_connection_type", "tcp");
    this->declare_parameter("atlas_tcp_ip", "localhost");
    this->declare_parameter("atlas_tcp_port", 12346);
    this->declare_parameter("frame_id", "");
    frame_id_ = this->get_parameter("frame_id").as_string();
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", rclcpp::SensorDataQoS());
    gps_fix_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("gps_fix", rclcpp::SensorDataQoS());
    nav_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", rclcpp::SensorDataQoS());
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&PointOneNavAtlasNode::serviceLoopCb, this));
    std::cout << std::to_string(this->get_parameter("atlas_udp_port").as_int()) << std::endl;
    std::cout << this->get_parameter("atlas_connection_type").as_string() << std::endl;
    std::cout << this->get_parameter("atlas_tcp_ip").as_string() << std::endl;
    std::cout << std::to_string(this->get_parameter("atlas_tcp_port").as_int()) << std::endl;
    gps.initialize(
      this,
      this->get_parameter("atlas_udp_port").as_int(),
      this->get_parameter("atlas_connection_type").as_string(),
      this->get_parameter("atlas_tcp_ip").as_string(),
      this->get_parameter("atlas_tcp_port").as_int()
    );
    gps.addAtlasMessageListener(*this);
  }

  /**
   * Callback function triggered by atlas receiving a complete message.
   * @param evt GPS/IMU data.
   * @return Nothing.
   */
  void receivedAtlasMessage(AtlasMessageEvent & evt) {
    auto time = now();

    if(evt.message_type == AtlasMessageType::GPS_FIX) {
      evt.gps_fix.header.frame_id = frame_id_;
      evt.gps_fix.header.stamp = time;
      gps_fix_publisher_->publish(evt.gps_fix);
      publishNavFixMsg(evt.gps_fix);
    }
    else if(evt.message_type == AtlasMessageType::IMU) {
      evt.imu.header.frame_id = frame_id_;
      evt.imu.header.stamp = time;
      imu_publisher_->publish(evt.imu);
    }
    else if (evt.message_type == AtlasMessageType::POSE) {
      evt.pose.header.frame_id = frame_id_;
      evt.pose.header.stamp = time;
      pose_publisher_->publish(evt.pose);
    }
  }

  /**
   * Translate GPSFix to NavFixMsg
   * @param gps_fix Atlas gps data point.
   * @return Nothing.
   */
  void publishNavFixMsg(const gps_msgs::msg::GPSFix &gps_fix)
  {
    sensor_msgs::msg::NavSatFix fix;
    fix.header = gps_fix.header;
    fix.status.status = gps_fix.status.status;
    fix.status.service = 0;
    if (gps_fix.status.position_source & gps_msgs::msg::GPSStatus::SOURCE_GPS)
    {
      fix.status.service = fix.status.service | sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    }
    if (gps_fix.status.orientation_source & gps_msgs::msg::GPSStatus::SOURCE_MAGNETIC)
    {
      fix.status.service = fix.status.service | sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
    }
    fix.latitude = gps_fix.latitude;
    fix.longitude = gps_fix.longitude;
    fix.altitude = gps_fix.altitude;
    fix.position_covariance = gps_fix.position_covariance;
    fix.position_covariance_type = gps_fix.position_covariance_type;
    nav_fix_publisher_->publish(fix);
  }

private:
  PointOneNavAtlas & gps;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_fix_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;

  /**
   * Initiate gps unit to read data.
   */
  void serviceLoopCb() {
    RCLCPP_INFO(this->get_logger(), "Service");
    timer_->cancel(); // one-time enrty into service loop
    gps.service();
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointOneNavAtlasNode>());
  rclcpp::shutdown();
  return 0;
}