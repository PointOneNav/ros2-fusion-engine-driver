#include "fusion_engine_node.hpp"

/******************************************************************************/
FusionEngineNode::FusionEngineNode()
    : Node("fusion_engine_node"),
      fe_interface_(std::bind(&FusionEngineNode::receivedFusionEngineMessage, this,
                    std::placeholders::_1, std::placeholders::_2)) {
  this->declare_parameter("udp_port", 12345);
  this->declare_parameter("connection_type", "tcp");
  this->declare_parameter("tcp_ip", "localhost");
  this->declare_parameter("tty_port", "/dev/ttyUSB0");
  this->declare_parameter("tcp_port", 12345);
  this->declare_parameter("debug", false);
  frame_id_ = "";
  timer_ = create_wall_timer(std::chrono::milliseconds(1),
                             std::bind(&FusionEngineNode::rosServiceLoop, this));
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
                             std::bind(&FusionEngineNode::rosServiceLoop, this));

  if (this->has_parameter("connection_type")) {
    std::string argValue(this->get_parameter("connection_type").as_string());
    if (argValue == "tcp") {
      fe_interface_.initialize(this, this->get_parameter("tcp_ip").as_string(),
                     this->get_parameter("tcp_port").as_int());
    } else if (argValue == "udp") {
      fe_interface_.initialize(this, this->get_parameter("udp_port").as_int());
    } else if (argValue == "tty") {
      nmea_publisher_ = this->create_publisher<nmea_msgs::msg::Sentence>(
          "ntrip_client/nmea", 10);
      subscription_ = this->create_subscription<mavros_msgs::msg::RTCM>(
          "ntrip_client/rtcm", 10,
          [this](const mavros_msgs::msg::RTCM::SharedPtr msg) {
            if (this->get_parameter("debug").as_bool())
              RCLCPP_INFO(this->get_logger(), "RTCM message received.");
            fe_interface_.write(msg->data.data(), msg->data.size());
          });
      fe_interface_.initialize(this, this->get_parameter("tty_port").as_string());
      listener_thread_ =
          std::thread(std::bind(&FusionEngineNode::dataListenerService, this));
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
FusionEngineNode::~FusionEngineNode() {
  if (listener_thread_.joinable()) {
    fe_interface_.stop();
    listener_thread_.join();
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

    if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
      if (this->get_parameter("debug").as_bool())
        RCLCPP_INFO(this->get_logger(), "Point published = [LLA=%f, %f, %f]",
                    p.x, p.y, p.z);
      points.points.push_back(p);
      publisher_->publish(points);
      id++;

    } else {
      if (this->get_parameter("debug").as_bool())
        RCLCPP_INFO(this->get_logger(), "Point dropped = [LLA=%f, %f, %f]", p.x,
                    p.y, p.z);
    }
  } else if (header.message_type == MessageType::POSE &&
             this->get_parameter("connection_type").as_string() == "tty") {
    auto &contents = *reinterpret_cast<
        const point_one::fusion_engine::messages::PoseMessage *>(payload);
    double gps_time_sec =
        contents.gps_time.seconds + contents.gps_time.fraction_ns * 1e-9;
    if (gps_time_sec - previous_gps_time_sec_ >
        TIME_BETWEEN_NMEA_UPDATES_SEC_) {
      nmea_msgs::msg::Sentence nmea =
          ConversionUtils::toNMEA(contents, satellite_nb_);
      nmea.header.stamp = this->now();
      nmea.header.frame_id = "gps";
      previous_gps_time_sec_ = gps_time_sec;
      nmea_publisher_->publish(nmea);
    }
  } else if (header.message_type == MessageType::GNSS_SATELLITE) {
    auto &contents = *reinterpret_cast<
        const point_one::fusion_engine::messages::GNSSSatelliteMessage *>(
        payload);
    satellite_nb_ = contents.num_satellites;
  } else if (header.message_type == MessageType::GNSS_INFO) {
    auto &contents = *reinterpret_cast<
        const point_one::fusion_engine::messages::GNSSInfoMessage *>(payload);
    if (this->get_parameter("debug").as_bool())
      RCLCPP_INFO(this->get_logger(), "Corrections age received (%d)",
                  contents.corrections_age);
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
void FusionEngineNode::rosServiceLoop() {
  RCLCPP_INFO(this->get_logger(), "Service");
  timer_->cancel();
}

/******************************************************************************/
void FusionEngineNode::dataListenerService() { fe_interface_.dataListenerService(); }
