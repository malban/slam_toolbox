
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

#ifndef SLAM_TOOLBOX__SLAM_TOOLBOX_MAP_AND_LOCALIZATION_HPP_
#define SLAM_TOOLBOX__SLAM_TOOLBOX_MAP_AND_LOCALIZATION_HPP_

#include <memory>
#include "slam_toolbox/slam_toolbox_localization.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace slam_toolbox
{

class MapAndLocalizationSlamToolbox : public LocalizationSlamToolbox
{
public:
  explicit MapAndLocalizationSlamToolbox(rclcpp::NodeOptions options);
  virtual ~MapAndLocalizationSlamToolbox() {}
  void loadPoseGraphByParams() override;
  void configure() override;

protected:
  void laserCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override;
  bool serializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> resp) override;
  bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp) override;
  bool setLocalizationModeCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
  LocalizedRangeScan * addScan(
    LaserRangeFinder * laser,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
    Pose2 & pose) override;
  void toggleMode(bool enable_localization);

  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> ssSetLocalizationMode_;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX__SLAM_TOOLBOX_MAP_AND_LOCALIZATION_HPP_
