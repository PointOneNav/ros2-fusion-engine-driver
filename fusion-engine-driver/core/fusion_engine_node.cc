#include "fusion_engine_node.hpp"

/******************************************************************************/
FusionEngineNode::FusionEngineNode()
    : Node("fusion_engine_node"),
      gps(std::bind(&FusionEngineNode::receivedFusionEngineMessage, this,
                    std::placeholders::_1, std::placeholders::_2)) {
  this->declare_parameter("udp_port", 12345);
  this->declare_parameter("connection_type", "tty");
  this->declare_parameter("tcp_ip", "localhost");
  this->declare_parameter("tty_port", "/dev/ttyUSB1");
  this->declare_parameter("tcp_port", 12345);
  frame_id_ = "";
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "pose", rclcpp::SensorDataQoS());
  gps_fix_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>(
      "gps_fix", rclcpp::SensorDataQoS());
  nav_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "fix", rclcpp::SensorDataQoS());
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS());
  publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", 1);
  timer_ = create_wall_timer(std::chrono::milliseconds(1),
                             std::bind(&FusionEngineNode::serviceLoopCb, this));

  if (this->has_parameter("connection_type")) {
    std::string argValue(this->get_parameter("connection_type").as_string());
    if (argValue == "tcp") {
      gps.initialize(this, this->get_parameter("tcp_ip").as_string(),
                     this->get_parameter("tcp_port").as_int());
    } else if (argValue == "udp") {
      gps.initialize(this, this->get_parameter("udp_port").as_int());
    } else if (argValue == "tty") {
      gps.initialize(this, this->get_parameter("tty_port").as_int());
    } else {
      std::cout << "Invalid args" << std::endl;
      rclcpp::shutdown();
    }
  } else {
    std::cout << "Invalid args" << std::endl;
    rclcpp::shutdown();
  }
}

/******************************************************************************/
void FusionEngineNode::receivedFusionEngineMessage(const MessageHeader &header,
                                                   const void *payload) {
  auto time = now();

  if (header.message_type == MessageType::ROS_GPS_FIX) {
    auto &contents = *reinterpret_cast<const GPSFixMessage *>(payload);
    gps_msgs::msg::GPSFix gps_fix = ConversionUtils::toGPSFix(contents);
    gps_fix.header.frame_id = frame_id_;
    gps_fix.header.stamp = time;
    gps_fix_publisher_->publish(gps_fix);
    publishNavFixMsg(gps_fix);
  } else if (header.message_type == MessageType::ROS_IMU) {
    auto &contents = *reinterpret_cast<const IMUMessage *>(payload);
    sensor_msgs::msg::Imu imu = ConversionUtils::toImu(contents);
    imu.header.frame_id = frame_id_;
    imu.header.stamp = time;
    imu_publisher_->publish(imu);
  } else if (header.message_type == MessageType::ROS_POSE) {
    auto &contents = *reinterpret_cast<
        const point_one::fusion_engine::messages::ros::PoseMessage *>(payload);

    geometry_msgs::msg::PoseStamped pos = ConversionUtils::toPose(contents);
    visualization_msgs::msg::Marker points;
    pos.header.frame_id = frame_id_;
    pos.header.stamp = time;
    pose_publisher_->publish(pos);
    points.header.frame_id = "/p1_frame";
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
    p.x = pos.pose.position.x;
    p.y = pos.pose.position.y;
    p.z = pos.pose.position.z;

    points.points.push_back(p);
    while (publisher_->get_subscription_count() < 1) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_WARN_ONCE(this->get_logger(),
                       "Please create a subscriber to the marker");
      sleep(1);
    }
    publisher_->publish(points);
    id++;
  }
}

/******************************************************************************/
void FusionEngineNode::publishNavFixMsg(const gps_msgs::msg::GPSFix &gps_fix) {
  sensor_msgs::msg::NavSatFix fix;
  fix.header = gps_fix.header;
  fix.status.status = gps_fix.status.status;
  fix.status.service = 0;
  if (gps_fix.status.position_source & gps_msgs::msg::GPSStatus::SOURCE_GPS) {
    fix.status.service =
        fix.status.service | sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  }
  if (gps_fix.status.orientation_source &
      gps_msgs::msg::GPSStatus::SOURCE_MAGNETIC) {
    fix.status.service =
        fix.status.service | sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
  }
  fix.latitude = gps_fix.latitude;
  fix.longitude = gps_fix.longitude;
  fix.altitude = gps_fix.altitude;
  fix.position_covariance = gps_fix.position_covariance;
  fix.position_covariance_type = gps_fix.position_covariance_type;
  nav_fix_publisher_->publish(fix);
}

/******************************************************************************/
void FusionEngineNode::serviceLoopCb() {
  RCLCPP_INFO(this->get_logger(), "Service");
  timer_->cancel();
  gps.service();
}
