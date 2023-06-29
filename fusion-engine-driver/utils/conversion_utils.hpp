#pragma once

#include <point_one/fusion_engine/messages/ros.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <numeric>
#include <sstream>
#include <map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ConversionUtils {
  public:
  /**
   * Helper method to translate atlas GPSFixMessage to ROS standard GPSFix.
   * @param contents Culprit gps data to be translated.
   * @return ROS standard message - GPSFix;
   */
  static gps_msgs::msg::GPSFix toGPSFix(
      const point_one::fusion_engine::messages::ros::GPSFixMessage& contents) {
    gps_msgs::msg::GPSFix gps_fix;
    gps_fix.latitude = contents.latitude_deg;
    gps_fix.longitude = contents.longitude_deg;
    gps_fix.altitude = contents.altitude_m;
    gps_fix.track = contents.track_deg;
    gps_fix.speed = contents.speed_mps;
    gps_fix.climb = contents.climb_mps;
    gps_fix.pitch = contents.pitch_deg;
    gps_fix.roll = contents.roll_deg;
    gps_fix.dip = contents.dip_deg;
    gps_fix.time =
        0;  // contents.p1_time.seconds + (contents.p1_time.fraction_ns * 1e-9);
            // // time since power-on
    gps_fix.gdop = contents.gdop;
    gps_fix.hdop = contents.hdop;
    gps_fix.vdop = contents.vdop;
    gps_fix.tdop = contents.tdop;
    gps_fix.err = contents.err_3d_m;
    gps_fix.err_horz = contents.err_horiz_m;
    gps_fix.err_vert = contents.err_vert_m;
    gps_fix.err_speed = contents.err_speed_mps;
    gps_fix.err_climb = contents.err_climb_mps;
    gps_fix.err_time = contents.err_time_sec;
    gps_fix.err_pitch = contents.err_pitch_deg;
    gps_fix.err_roll = contents.err_roll_deg;
    gps_fix.err_dip = contents.err_dip_deg;
    std::copy(std::begin(contents.position_covariance_m2),
              std::end(contents.position_covariance_m2),
              std::begin(gps_fix.position_covariance));
    gps_fix.position_covariance_type = contents.position_covariance_type;
    return gps_fix;
  }

  /**
   * Helper method to translate atlas IMUMessage to ROS standard Imu.
   * @param contents Culprit gps data to be translated.
   * @return ROS standard message - Imu;
   */
  static sensor_msgs::msg::Imu toImu(
      const point_one::fusion_engine::messages::ros::IMUMessage& contents) {
    sensor_msgs::msg::Imu imu;
    imu.orientation.x = contents.orientation[0];
    imu.orientation.y = contents.orientation[1];
    imu.orientation.z = contents.orientation[2];
    imu.orientation.w = contents.orientation[3];
    std::copy(std::begin(contents.orientation_covariance),
              std::end(contents.orientation_covariance),
              std::begin(imu.orientation_covariance));
    imu.angular_velocity.x = contents.angular_velocity_rps[0];
    imu.angular_velocity.y = contents.angular_velocity_rps[1];
    imu.angular_velocity.z = contents.angular_velocity_rps[2];
    std::copy(std::begin(contents.angular_velocity_rps),
              std::end(contents.angular_velocity_rps),
              std::begin(imu.angular_velocity_covariance));
    imu.linear_acceleration.x = contents.acceleration_mps2[0];
    imu.linear_acceleration.y = contents.acceleration_mps2[1];
    imu.linear_acceleration.z = contents.acceleration_mps2[2];
    std::copy(std::begin(contents.acceleration_covariance),
              std::end(contents.acceleration_covariance),
              std::begin(imu.linear_acceleration_covariance));
    return imu;
  }

  /**
   * Helper method to translate atlas PoseMessage to ROS standard PoseStamped.
   * @param contents Culprit pose data to be translated.
   * @return ROS standard message - PoseStamped;
   */
  static geometry_msgs::msg::PoseStamped toPose(
      const point_one::fusion_engine::messages::ros::PoseMessage& contents) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = contents.position_rel_m[0];
    pose_stamped.pose.position.y = contents.position_rel_m[1];
    pose_stamped.pose.position.z = contents.position_rel_m[2];
    pose_stamped.pose.orientation.x = contents.orientation[0];
    pose_stamped.pose.orientation.y = contents.orientation[1];
    pose_stamped.pose.orientation.z = contents.orientation[2];
    pose_stamped.pose.orientation.w = contents.orientation[3];
    return pose_stamped;
  }

  /**
   * @brief Calculate the XOR checksum of a string message.
   *
   * @param message The string message to calculate the checksum for.
   * @return The calculated checksum as a single char.
   */
  static char calculateChecksum(const std::string& message) {
    char checksum = 0;
    for (auto c : message) {
      checksum ^= c;
    }
    return checksum;
  }

  static int convertNtpToUnix(int current_timestamp) {
    return current_timestamp - 2208988800;
  }

  static int64_t convertUnixToNtp(int64_t current_timestamp) {
    return current_timestamp + 2208988800;
  }

  static int getLastLeapSecondDate(int64_t current_timestamp) {
      int leap_second_value = 0;
      std::map<int64_t, int> leap_second = getLeapSeconds();

      for (const auto &ele : leap_second) {
          if (ele.first <= convertUnixToNtp(current_timestamp)) {
            leap_second_value = ele.second;
          }
      }
      return leap_second_value;
  }

  /**
   * @brief Convert GPS time to UTC time in string format.
   *
   * @param gps_time The GPS time to convert, in seconds.
   * @return The UTC time in string format (hhmmss.ssssss).
   */
  static std::string convertGpsToUtc(double gps_time) {
    const int kSecondsInDay = 86400;
    auto currentTime = std::chrono::system_clock::now();
    std::time_t currentTimeT = std::chrono::system_clock::to_time_t(currentTime);
    int64_t currentTimestamp = static_cast<int64_t>(currentTimeT);
    const int kLeapSecondsOffset = getLastLeapSecondDate(currentTimestamp);

    // Extract GPS time components
    int gps_hours = static_cast<int>((gps_time / 3600.0));
    int gps_minutes = static_cast<int>((gps_time - gps_hours * 3600.0) / 60.0);
    double gps_seconds = gps_time - gps_hours * 3600.0 - gps_minutes * 60.0;

    // Convert GPS time to seconds
    int gps_seconds_total =
        gps_hours * 3600 + gps_minutes * 60 + static_cast<int>(gps_seconds);

    // Subtract leap seconds offset
    int utc_seconds_total = gps_seconds_total - kLeapSecondsOffset;

    // Handle cases where the UTC time is negative or greater than 24 hours
    if (utc_seconds_total < 0) {
      utc_seconds_total += kSecondsInDay;
    } else if (utc_seconds_total >= kSecondsInDay) {
      utc_seconds_total -= kSecondsInDay;
    }

    // Convert UTC time to hours, minutes, and seconds
    int utc_hours = int(utc_seconds_total / 3600) % 24;
    int utc_minutes = (utc_seconds_total % 3600) / 60;
    double utc_seconds = static_cast<double>(utc_seconds_total % 60) +
                         gps_seconds - static_cast<int>(gps_seconds);

    // Create output string with the desired format
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(2) << utc_hours << std::setw(2)
       << utc_minutes << std::fixed << std::setprecision(3) << std::setw(6)
       << utc_seconds;
    return ss.str();
  }

  /**
   * @brief Converts a degree-based latitude or longitude to a NMEA DDMM format
   * @param angle_deg The angle to be converted in degrees
   * @param is_longitude A flag indicating whether the angle represents a
   * longitude
   * @return A string in NMEA DDMM format
   */
  static std::string nmeaDegToDdmm(double angle_deg,
                                      bool is_longitude = false) {
    std::string direction;
    if (is_longitude) {
      direction = angle_deg >= 0.0 ? "E" : "W";
    } else {
      direction = angle_deg >= 0.0 ? "N" : "S";
    }

    double abs_angle_deg = std::fabs(angle_deg);
    int degree = static_cast<int>(abs_angle_deg);
    double minute = (abs_angle_deg - degree) * 60.0;

    std::ostringstream oss;
    oss << degree << std::fixed << std::setprecision(2) << minute << ","
        << direction;
    return oss.str();
  }

  /**
   * @brief Converts the given PoseMessage to a NMEA GPGGA sentence.
   *
   * @param contents The PoseMessage to convert.
   * @param nb_satellite The number of GPS satellites used to generate the pose.
   * @return A NMEA GPGGA sentence as a nmea_msgs::msg::Sentence.
   */
  static nmea_msgs::msg::Sentence toNMEA(
      const point_one::fusion_engine::messages::PoseMessage& contents,
      uint16_t& nb_satellite) {
    double gps_time_sec =
        contents.gps_time.seconds + contents.gps_time.fraction_ns * 1e-9;

    std::stringstream nmea_stream;
    nmea_msgs::msg::Sentence nmea;
    nmea_stream << "$GPGGA,";
    nmea_stream << convertGpsToUtc(gps_time_sec) << ",";
    nmea_stream << nmeaDegToDdmm(contents.lla_deg[0], false) << ",";
    nmea_stream << nmeaDegToDdmm(contents.lla_deg[1], true) << ",";
    // nmea_stream << static_cast<uint8_t>(contents.solution_type) << ",";
    nmea_stream << std::setprecision(1) << 1 << ",";    // hdop
    nmea_stream << nb_satellite << ",";                 // nb sattelite
    nmea_stream << std::setprecision(1) << 1.6 << ",";  // antenna alt
    nmea_stream << std::setprecision(1) << 1.6 << ",";
    nmea_stream << "M,";  // unit alt M or F
    nmea_stream << std::setprecision(4) << contents.lla_deg[2] << ",";
    nmea_stream << "M,-20.7,M,,";  // unit alt M or F
    unsigned char checksum = calculateChecksum(nmea_stream.str().substr(1));

    nmea_stream << "*" << std::hex << static_cast<int>(checksum) << std::endl;
    nmea.sentence = nmea_stream.str();
    return nmea;
  }

  // static std::map<int, int> leap_seconds_;

  static const std::map<int64_t, int> &getLeapSeconds() {
    static const std::map<int64_t, int> leap_seconds = {
      {2272060800, 10},    // 1 Jan 1972
      {2287785600, 11},    // 1 Jul 1972
      {2303683200, 12},    // 1 Jan 1973
      {2335219200, 13},    // 1 Jan 1974
      {2366755200, 14},    // 1 Jan 1975
      {2398291200, 15},    // 1 Jan 1976
      {2429913600, 16},    // 1 Jan 1977
      {2461449600, 17},    // 1 Jan 1978
      {2492985600, 18},    // 1 Jan 1979
      {2524521600, 19},    // 1 Jan 1980
      {2571782400, 20},    // 1 Jul 1981
      {2603318400, 21},    // 1 Jul 1982
      {2634854400, 22},    // 1 Jul 1983
      {2698012800, 23},    // 1 Jul 1985
      {2776982400, 24},    // 1 Jan 1988
      {2840140800, 25},    // 1 Jan 1990
      {2871676800, 26},    // 1 Jan 1991
      {2918937600, 27},    // 1 Jul 1992
      {2950473600, 28},    // 1 Jul 1993
      {2982009600, 29},    // 1 Jul 1994
      {3029443200, 30},    // 1 Jan 1996
      {3076704000, 31},    // 1 Jul 1997
      {3124137600, 32},    // 1 Jan 1999
      {3345062400, 33},    // 1 Jan 2006
      {3439756800, 34},    // 1 Jan 2009
      {3550089600, 35},    // 1 Jul 2012
      {3644697600, 36},    // 1 Jul 2015
      {3692217600, 37}     // 1 Jan 2017
    };
    return leap_seconds;
  }
};
