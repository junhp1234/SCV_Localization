/*
 * Copyright (c) 2024 Robot Localization Contributors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <robot_localization/map_origin_transform.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace robot_localization
{

MapOriginTransform::MapOriginTransform(const rclcpp::NodeOptions & options)
: Node("map_origin_transform_node", options),
  broadcast_transform_(true),
  static_transform_(true),
  frequency_(10.0),
  map_frame_("map"),
  odom_frame_("odom"),
  initialized_(false)
{
  RCLCPP_INFO(this->get_logger(), "Map Origin Transform node starting...");
  
  // Load parameters from YAML configuration
  loadParameters();
  
  // Initialize transform broadcasters
  initializeTransformBroadcaster();
  
  // Compute and set up the transform
  computeTransform();
  
  if (static_transform_) {
    // Broadcast static transform once
    broadcastStaticTransform();
  } else {
    // Set up timer for periodic transform broadcasting
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / frequency_));
    timer_ = this->create_wall_timer(
      period, std::bind(&MapOriginTransform::transformCallback, this));
  }
  
  initialized_ = true;
  
  RCLCPP_INFO(this->get_logger(), 
    "Map Origin Transform node initialized successfully");
  RCLCPP_INFO(this->get_logger(),
    "Map origin: GPS(%.6f, %.6f, %.2f) -> UTM(%.2f, %.2f) Zone %s",
    map_origin_.gps_latitude, map_origin_.gps_longitude, map_origin_.gps_altitude,
    map_origin_.utm_easting, map_origin_.utm_northing, map_origin_.utm_zone.c_str());
  RCLCPP_INFO(this->get_logger(),
    "Transform mode: %s, Frame: %s -> %s",
    static_transform_ ? "static" : "dynamic", 
    map_frame_.c_str(), odom_frame_.c_str());
}

MapOriginTransform::~MapOriginTransform()
{
  if (timer_) {
    timer_->cancel();
  }
}

void MapOriginTransform::loadParameters()
{
  // Map origin GPS coordinates
  map_origin_.gps_latitude = this->declare_parameter("map_origin.gps.latitude", 37.5665);
  map_origin_.gps_longitude = this->declare_parameter("map_origin.gps.longitude", 126.9780);
  map_origin_.gps_altitude = this->declare_parameter("map_origin.gps.altitude", 0.0);
  
  // Map origin UTM coordinates
  map_origin_.utm_easting = this->declare_parameter("map_origin.utm.easting", 323000.0);
  map_origin_.utm_northing = this->declare_parameter("map_origin.utm.northing", 4157000.0);
  map_origin_.utm_zone = this->declare_parameter("map_origin.utm.zone", std::string("52N"));
  
  // Transform broadcasting parameters
  broadcast_transform_ = this->declare_parameter("broadcast_transform", true);
  static_transform_ = this->declare_parameter("static_transform", true);
  frequency_ = this->declare_parameter("frequency", 10.0);
  
  // Frame IDs
  map_frame_ = this->declare_parameter("map_frame", std::string("map"));
  odom_frame_ = this->declare_parameter("odom_frame", std::string("odom"));
  
  // Transform offset parameters
  transform_offset_.translation_x = this->declare_parameter("transform_offset.translation.x", 0.0);
  transform_offset_.translation_y = this->declare_parameter("transform_offset.translation.y", 0.0);
  transform_offset_.translation_z = this->declare_parameter("transform_offset.translation.z", 0.0);
  transform_offset_.rotation_roll = this->declare_parameter("transform_offset.rotation.roll", 0.0);
  transform_offset_.rotation_pitch = this->declare_parameter("transform_offset.rotation.pitch", 0.0);
  transform_offset_.rotation_yaw = this->declare_parameter("transform_offset.rotation.yaw", 0.0);
  
  RCLCPP_INFO(this->get_logger(), "Parameters loaded successfully");
}

void MapOriginTransform::initializeTransformBroadcaster()
{
  if (static_transform_) {
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    RCLCPP_INFO(this->get_logger(), "Static transform broadcaster initialized");
  } else {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(this->get_logger(), "Dynamic transform broadcaster initialized");
  }
}

void MapOriginTransform::transformCallback()
{
  if (!broadcast_transform_ || !initialized_) {
    return;
  }
  
  // Update timestamp for dynamic transform
  transform_stamped_.header.stamp = this->now();
  
  // Broadcast the transform
  tf_broadcaster_->sendTransform(transform_stamped_);
}

void MapOriginTransform::computeTransform()
{
  // Set up the transform message
  transform_stamped_.header.frame_id = map_frame_;
  transform_stamped_.child_frame_id = odom_frame_;
  transform_stamped_.header.stamp = this->now();
  
  // Calculate translation based on UTM coordinates
  // The map frame origin is at the specified UTM coordinates
  // The odom frame is typically centered at (0,0) in the local coordinate system
  // So we need to translate by the negative of the UTM coordinates to center odom at the map origin
  transform_stamped_.transform.translation.x = -map_origin_.utm_easting + transform_offset_.translation_x;
  transform_stamped_.transform.translation.y = -map_origin_.utm_northing + transform_offset_.translation_y;
  transform_stamped_.transform.translation.z = -map_origin_.gps_altitude + transform_offset_.translation_z;
  
  // Set rotation based on offset parameters
  tf2::Quaternion quaternion;
  quaternion.setRPY(
    transform_offset_.rotation_roll,
    transform_offset_.rotation_pitch, 
    transform_offset_.rotation_yaw);
  
  transform_stamped_.transform.rotation.x = quaternion.x();
  transform_stamped_.transform.rotation.y = quaternion.y();
  transform_stamped_.transform.rotation.z = quaternion.z();
  transform_stamped_.transform.rotation.w = quaternion.w();
  
  RCLCPP_DEBUG(this->get_logger(),
    "Transform computed: translation(%.2f, %.2f, %.2f), rotation(%.3f, %.3f, %.3f, %.3f)",
    transform_stamped_.transform.translation.x,
    transform_stamped_.transform.translation.y,
    transform_stamped_.transform.translation.z,
    transform_stamped_.transform.rotation.x,
    transform_stamped_.transform.rotation.y,
    transform_stamped_.transform.rotation.z,
    transform_stamped_.transform.rotation.w);
}

void MapOriginTransform::broadcastStaticTransform()
{
  if (!broadcast_transform_ || !static_tf_broadcaster_) {
    return;
  }
  
  // Broadcast the static transform
  static_tf_broadcaster_->sendTransform(transform_stamped_);
  
  RCLCPP_INFO(this->get_logger(),
    "Static transform broadcasted: %s -> %s", 
    map_frame_.c_str(), odom_frame_.c_str());
  RCLCPP_INFO(this->get_logger(),
    "Translation: (%.2f, %.2f, %.2f)",
    transform_stamped_.transform.translation.x,
    transform_stamped_.transform.translation.y,
    transform_stamped_.transform.translation.z);
}

void MapOriginTransform::gpsToUTM(double lat, double lon, double& easting, double& northing)
{
  // WGS84 ellipsoid parameters
  const double a = 6378137.0;           // Semi-major axis
  const double f = 1.0 / 298.257223563; // Flattening
  const double k0 = 0.9996;            // UTM scale factor

  // Determine the UTM zone
  int zone = static_cast<int>((lon + 180.0) / 6.0) + 1;

  // Calculate the central meridian for the zone
  double lon0_deg = (zone - 1) * 6 - 180 + 3;
  double lon0_rad = lon0_deg * M_PI / 180.0;

  // False easting and northing
  double false_easting = 500000.0;
  double false_northing = (lat < 0) ? 10000000.0 : 0.0; // Southern hemisphere

  // Convert lat/lon to radians
  double lat_rad = lat * M_PI / 180.0;
  double lon_rad = lon * M_PI / 180.0;

  // Equations for conversion
  double e2 = 2 * f - f * f;
  double e_prime_sq = e2 / (1.0 - e2);

  double N = a / std::sqrt(1.0 - e2 * std::sin(lat_rad) * std::sin(lat_rad));
  double T = std::tan(lat_rad) * std::tan(lat_rad);
  double C = e_prime_sq * std::cos(lat_rad) * std::cos(lat_rad);
  double A = std::cos(lat_rad) * (lon_rad - lon0_rad);

  double M = a * ((1.0 - e2/4.0 - 3.0*e2*e2/64.0 - 5.0*e2*e2*e2/256.0) * lat_rad
                 - (3.0*e2/8.0 + 3.0*e2*e2/32.0 + 45.0*e2*e2*e2/1024.0) * std::sin(2.0*lat_rad)
                 + (15.0*e2*e2/256.0 + 45.0*e2*e2*e2/1024.0) * std::sin(4.0*lat_rad)
                 - (35.0*e2*e2*e2/3072.0) * std::sin(6.0*lat_rad));

  easting = false_easting + k0 * N * (A + (1.0 - T + C) * std::pow(A, 3) / 6.0
                                     + (5.0 - 18.0*T + T*T + 72.0*C - 58.0*e_prime_sq) * std::pow(A, 5) / 120.0);

  northing = false_northing + k0 * (M + N * std::tan(lat_rad) * (std::pow(A, 2) / 2.0
                                                                + (5.0 - T + 9.0*C + 4.0*C*C) * std::pow(A, 4) / 24.0
                                                                + (61.0 - 58.0*T + T*T + 600.0*C - 330.0*e_prime_sq) * std::pow(A, 6) / 720.0));
}

}  // namespace robot_localization