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
#include "visualization_msgs/msg/marker.hpp"

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
    this->declare_parameter("atlas_connection_type", "tty");
    this->declare_parameter("atlas_tcp_ip", "localhost");
    this->declare_parameter("atlas_tcp_port", 12345);
    this->declare_parameter("frame_id", "");
    this->declare_parameter("tty_port", "/dev/ttyUSB1");
    frame_id_ = this->get_parameter("frame_id").as_string();
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", rclcpp::SensorDataQoS());
    gps_fix_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("gps_fix", rclcpp::SensorDataQoS());
    nav_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", rclcpp::SensorDataQoS());
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&PointOneNavAtlasNode::serviceLoopCb, this));
    gps.initialize(
      this,
      this->get_parameter("atlas_udp_port").as_int(),
      this->get_parameter("atlas_connection_type").as_string(),
      this->get_parameter("atlas_tcp_ip").as_string(),
      this->get_parameter("atlas_tcp_port").as_int(),
      this->get_parameter("tty_port").as_string()
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
      visualization_msgs::msg::Marker points;
      evt.pose.header.frame_id = frame_id_;
      evt.pose.header.stamp = time;
      pose_publisher_->publish(evt.pose);
      points.header.frame_id = "/my_frame";
      points.header.stamp = this->now();
      points.ns = "basic_shapes";
      points.action = visualization_msgs::msg::Marker::ADD;
      points.pose.orientation.w = 1.0;
      points.id = id;
      points.type = visualization_msgs::msg::Marker::POINTS;
      points.scale.x = 0.1;
      points.scale.y = 0.1;
      points.color.g = 1.0f;
      points.color.a = 1.0;
      geometry_msgs::msg::Point p;
      p.x = evt.pose.pose.position.x;
      p.y = evt.pose.pose.position.y;
      p.z = evt.pose.pose.position.z;
      points.points.push_back(p);
      while (publisher_->get_subscription_count() < 1)
      {
          if (!rclcpp::ok())
          {
            return;
          }
          RCLCPP_WARN_ONCE(this->get_logger(), "Please create a subscriber to the marker");
          sleep(1);
      }
      publisher_->publish(points);
      id++;
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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;
  int id = 0;

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