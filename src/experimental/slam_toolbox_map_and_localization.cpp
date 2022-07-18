
/**
 * Copyright 2022 Hatchbed L.L.C.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Marc Alban */

#include <memory>
#include <string>
#include "slam_toolbox/experimental/slam_toolbox_map_and_localization.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
MapAndLocalizationSlamToolbox::MapAndLocalizationSlamToolbox(rclcpp::NodeOptions options)
: LocalizationSlamToolbox(options)
/*****************************************************************************/
{
  // disable interactive mode
  enable_interactive_mode_ = false;

  ssSetLocalizationMode_ = create_service<std_srvs::srv::SetBool>(
    "slam_toolbox/set_localization_mode",
    std::bind(&MapAndLocalizationSlamToolbox::setLocalizationModeCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

/*****************************************************************************/
void MapAndLocalizationSlamToolbox::configure()
/*****************************************************************************/
{
  SlamToolbox::configure();
  toggleMode(false);
  RCLCPP_INFO(get_logger(), "toggled");
}

/*****************************************************************************/
bool MapAndLocalizationSlamToolbox::setLocalizationModeCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
/*****************************************************************************/
{
  toggleMode(req->data);

  resp->success = true;
  return true;
}

void MapAndLocalizationSlamToolbox::toggleMode(bool enable_localization) {

  RCLCPP_INFO(get_logger(), "toggleMode %s", (enable_localization ? "true" : "false"));

  bool in_localization_mode = processor_type_ == PROCESS_LOCALIZATION;
  if (in_localization_mode == enable_localization) {
    return;
  }

  if (enable_localization) {
    RCLCPP_INFO(get_logger(), "Enabling localization ...");
    processor_type_ = PROCESS_LOCALIZATION;

    localization_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1, std::bind(&MapAndLocalizationSlamToolbox::localizePoseCallback, this, std::placeholders::_1));
    clear_localization_ = create_service<std_srvs::srv::Empty>(
      "slam_toolbox/clear_localization_buffer",
      std::bind(&MapAndLocalizationSlamToolbox::clearLocalizationBuffer, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // in localization mode, disable map saver
    map_saver_.reset();
  }
  else {
    RCLCPP_INFO(get_logger(), "Enabling mapping ...");
    processor_type_ = PROCESS;
    RCLCPP_INFO(get_logger(), "localization_pose_sub_.reset()");
    localization_pose_sub_.reset();
    RCLCPP_INFO(get_logger(), "clear_localization_.reset()");
    clear_localization_.reset();
    RCLCPP_INFO(get_logger(), "map_saver_ = std::make_unique<map_saver::MapSaver>(node_, map_name_)");
    map_saver_ = std::make_unique<map_saver::MapSaver>(shared_from_this(), map_name_);

    boost::mutex::scoped_lock lock(smapper_mutex_);
    RCLCPP_INFO(get_logger(), "Clearing localization buffer.");
    if (smapper_ && !smapper_->getMapper()->GetLocalizationVertices().empty()) {
      smapper_->clearLocalizationBuffer();
    }
  }
}

/*****************************************************************************/
void MapAndLocalizationSlamToolbox::loadPoseGraphByParams()
/*****************************************************************************/
{
  if (processor_type_ == PROCESS_LOCALIZATION) {
    LocalizationSlamToolbox::loadPoseGraphByParams();
  }
  else {
    SlamToolbox::loadPoseGraphByParams();
  }
}

/*****************************************************************************/
bool MapAndLocalizationSlamToolbox::serializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (processor_type_ == PROCESS_LOCALIZATION) {
    return LocalizationSlamToolbox::serializePoseGraphCallback(request_header, req, resp);
  }
  else {
    return SlamToolbox::serializePoseGraphCallback(request_header, req, resp);
  }
}

/*****************************************************************************/
bool MapAndLocalizationSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (processor_type_ == PROCESS_LOCALIZATION) {
    return LocalizationSlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
  }
  else {
    return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
  }
}

/*****************************************************************************/
void MapAndLocalizationSlamToolbox::laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
  //                                                                                      100 milliseconds
  if (!tf_->canTransform(odom_frame_, scan->header.frame_id, scan->header.stamp, rclcpp::Duration(100000000))) {
    RCLCPP_WARN(get_logger(), "Failed to get transform %s -> %s.", scan->header.frame_id.c_str(), odom_frame_.c_str());
    return;
  }


  // store scan header
  scan_header = scan->header;
  // no odom info
  Pose2 pose;
  if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder * laser = getLaser(scan);

  if (!laser) {
    RCLCPP_WARN(get_logger(), "Failed to create laser device for"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  addScan(laser, scan, pose);
}

/*****************************************************************************/
LocalizedRangeScan * MapAndLocalizationSlamToolbox::addScan(
  LaserRangeFinder * laser,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  Pose2 & odom_pose)
/*****************************************************************************/
{
  if (processor_type_ == PROCESS_LOCALIZATION) {
    return LocalizationSlamToolbox::addScan(laser, scan, odom_pose);
  }
  else {
    return SlamToolbox::addScan(laser, scan, odom_pose);
  }
}

}  // namespace slam_toolbox
