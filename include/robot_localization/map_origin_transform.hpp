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

#ifndef ROBOT_LOCALIZATION__MAP_ORIGIN_TRANSFORM_HPP_
#define ROBOT_LOCALIZATION__MAP_ORIGIN_TRANSFORM_HPP_

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace robot_localization
{

/**
 * @brief Map Origin Transform Node
 * 
 * This node broadcasts the map -> odom transform based on YAML configuration.
 * It reads map origin parameters from YAML and publishes a static or dynamic
 * transform to establish the relationship between map and odom frames.
 */
class MapOriginTransform : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  explicit MapOriginTransform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~MapOriginTransform();

private:
  /**
   * @brief Load parameters from YAML configuration
   */
  void loadParameters();

  /**
   * @brief Initialize the transform broadcaster
   */
  void initializeTransformBroadcaster();

  /**
   * @brief Callback for computing and publishing the transform
   */
  void transformCallback();

  /**
   * @brief Compute the transform from map to odom frame
   */
  void computeTransform();

  /**
   * @brief Broadcast the static transform (called once)
   */
  void broadcastStaticTransform();

  /**
   * @brief Convert GPS coordinates to UTM
   * @param lat Latitude in decimal degrees
   * @param lon Longitude in decimal degrees
   * @param easting UTM easting output
   * @param northing UTM northing output
   */
  void gpsToUTM(double lat, double lon, double& easting, double& northing);

  // Map origin parameters
  struct MapOrigin {
    // GPS coordinates
    double gps_latitude;
    double gps_longitude; 
    double gps_altitude;
    
    // UTM coordinates
    double utm_easting;
    double utm_northing;
    std::string utm_zone;
  } map_origin_;

  // Transform offset parameters
  struct TransformOffset {
    double translation_x;
    double translation_y;
    double translation_z;
    double rotation_roll;
    double rotation_pitch;
    double rotation_yaw;
  } transform_offset_;

  // Configuration parameters
  bool broadcast_transform_;
  bool static_transform_;
  double frequency_;
  std::string map_frame_;
  std::string odom_frame_;

  // Transform broadcasters
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer for periodic transform broadcasting
  rclcpp::TimerBase::SharedPtr timer_;

  // Transform message
  geometry_msgs::msg::TransformStamped transform_stamped_;
  
  // Initialization flag
  bool initialized_;
};

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__MAP_ORIGIN_TRANSFORM_HPP_