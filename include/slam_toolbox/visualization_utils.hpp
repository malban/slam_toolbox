/*
 * visualization_utils
 * Copyright (c) 2019, Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#ifndef SLAM_TOOLBOX__VISUALIZATION_UTILS_HPP_
#define SLAM_TOOLBOX__VISUALIZATION_UTILS_HPP_

#include <string>

#include "std_msgs/msg/color_rgba.hpp"
#include "Eigen/Core"
#include "karto_sdk/Karto.h"
#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/toolbox_types.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

namespace vis_utils
{

inline visualization_msgs::msg::Marker toMarker(
  const std::string & frame,
  const std::string & ns,
  const double & scale,
  rclcpp::Node::SharedPtr node)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame;
  marker.header.stamp = node->now();
  marker.ns = ns;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 1.0;
  marker.color.g = 0;
  marker.color.b = 0.0;
  marker.color.a = 1.;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0);

  return marker;
}

inline visualization_msgs::msg::InteractiveMarker toInteractiveMarker(
  visualization_msgs::msg::Marker & marker,
  const double & scale,
  rclcpp::Node::SharedPtr node)
{
  // marker basics
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker.header.frame_id;
  int_marker.header.stamp = node->now();
  int_marker.name = std::to_string(marker.id);
  int_marker.pose.orientation.w = 1.;
  int_marker.pose.position.x = marker.pose.position.x;
  int_marker.pose.position.y = marker.pose.position.y;
  int_marker.scale = scale;

  // translate control
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.orientation_mode =
    visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  control.always_visible = true;
  control.orientation.w = 0;
  control.orientation.x = 0.7071;
  control.orientation.y = 0;
  control.orientation.z = 0.7071;
  control.interaction_mode =
    visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  control.markers.push_back( marker );
  int_marker.controls.push_back( control );

  // rotate control
  visualization_msgs::msg::InteractiveMarkerControl control_rot;
  control_rot.orientation_mode =
    visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  control_rot.always_visible = true;
  control_rot.orientation.w = 0;
  control_rot.orientation.x = 0.7071;
  control_rot.orientation.y = 0;
  control_rot.orientation.z = 0.7071;
  control_rot.interaction_mode =
    visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control_rot);

  return int_marker;
}

inline void toNavMap(
  const karto::OccupancyGrid * occ_grid,
  nav_msgs::msg::OccupancyGrid & map)
{
  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset =
    occ_grid->GetCoordinateConverter()->GetOffset();

  if (map.info.width != (unsigned int) width ||
    map.info.height != (unsigned int) height ||
    map.info.origin.position.x != offset.GetX() ||
    map.info.origin.position.y != offset.GetY())
  {
    map.info.origin.position.x = offset.GetX();
    map.info.origin.position.y = offset.GetY();
    map.info.width = width;
    map.info.height = height;
    map.data.resize(map.info.width * map.info.height);
  }

  for (kt_int32s y = 0; y < height; y++) {
    for (kt_int32s x = 0; x < width; x++) {
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));
      switch (value) {
        case karto::GridStates_Unknown:
          map.data[MAP_IDX(map.info.width, x, y)] = -1;
          break;
        case karto::GridStates_Occupied:
          map.data[MAP_IDX(map.info.width, x, y)] = 100;
          break;
        case karto::GridStates_Free:
          map.data[MAP_IDX(map.info.width, x, y)] = 0;
          break;
      }
    }
  }
}

// More portable rviz colormap
// source: http://docs.ros.org/en/melodic/api/rviz_visual_tools/html/rviz__visual__tools_8cpp_source.html
enum colors
{
  BLUE = 0,
  GREEN = 1,
  RED = 2,
  YELLOW = 3,
  DEFAULT = 4
};
 
inline std_msgs::msg::ColorRGBA getColor(colors color)
{
  std_msgs::msg::ColorRGBA result;

  switch (color)
  {
    case RED:
      result.r = 0.8;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 1.0;
      break;
    case GREEN:
      result.r = 0.1;
      result.g = 0.8;
      result.b = 0.1;
      result.a = 1.0;
      break;
    case YELLOW:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case BLUE:
    default:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.8;
      result.a = 1.0;
      break;
  }

  return result;
}

visualization_msgs::msg::Marker toArrowMarker(const karto::Pose2& pose, const double & scale);

visualization_msgs::msg::Marker toSphereMarker(const karto::Pose2& pose, const double & scale);

visualization_msgs::msg::Marker toCovarianceMarker(const karto::Edge<karto::LocalizedRangeScan>& edge);

inline double slerp(double start, double end, double range, double value)
{
  return start + (((end - start) / range) * value);
}

inline std_msgs::msg::ColorRGBA getColorScale(const double input_value)
{
  double value = input_value;
   // User warning
   if (value < 0.0)
   {
     std::cout << "Intensity value for color scale is below range [0,1], value: " << value << std::endl;
     value = 0.0;
   }
   else if (value > 1.0)
   {
     std::cout << "Intensity value for color scale is above range [0,1], value: " << value << std::endl;
     value = 1.0;
   }
 
   std_msgs::msg::ColorRGBA start;
   std_msgs::msg::ColorRGBA end;
 
   // For second half of color range move towards RED
   if (value == 0.0)
   {
     return getColor(RED);
   }
   else if (value == 1.0)
   {
     return getColor(GREEN);
   }
   else if (value <= 0.5)
   {
     start = getColor(RED);
     end = getColor(YELLOW);
   }
   else
   {
     start = getColor(YELLOW);
     end = getColor(GREEN);
     value = fmod(value, 0.5);
   }
 
   std_msgs::msg::ColorRGBA result;
   result.r = slerp(start.r, end.r, 0.5, value);
   result.g = slerp(start.g, end.g, 0.5, value);
   result.b = slerp(start.b, end.b, 0.5, value);
   result.a = 1.0;
 
   return result;
}

}  // namespace vis_utils

#endif  // SLAM_TOOLBOX__VISUALIZATION_UTILS_HPP_
