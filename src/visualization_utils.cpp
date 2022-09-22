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

#include <cmath>

#include "Eigen/SVD"
#include "slam_toolbox/visualization_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace vis_utils
{

visualization_msgs::msg::Marker toCovarianceMarker(const karto::Edge<karto::LocalizedRangeScan>& edge)
{
  visualization_msgs::msg::Marker marker;

  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.scale.z = 0.01;
  marker.color.r = 1;
  marker.color.b = 1;
  marker.color.a = 0.5;

  const auto & pose0 = edge.GetSource()->GetObject()->GetCorrectedPose();
  const auto & pose1 = edge.GetTarget()->GetObject()->GetCorrectedPose();
  const auto & cov = ((karto::LinkInfo *)(edge.GetLabel()))->GetCovariance();

  marker.pose.position.x = (pose0.GetX() + pose1.GetX()) * 0.5;
  marker.pose.position.y = (pose0.GetY() + pose1.GetY()) * 0.5;

  Eigen::Matrix2d A;
  A << cov(0, 0), cov(0, 1), cov(1, 0), cov(1, 1);

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(A, Eigen::ComputeFullV);

  Eigen::Vector2d Sigma = svd.singularValues();
  Eigen::Matrix2d Vt = svd.matrixV().transpose();

  marker.scale.x = std::sqrt(Sigma(0));
  marker.scale.y = std::sqrt(Sigma(1));

  double angle = std::atan2(Vt(1, 0), Vt(0, 0));
  tf2::Quaternion rotation;
  rotation.setRPY(0, 0, pose0.GetHeading() - angle);
  marker.pose.orientation =  tf2::toMsg(rotation);

  return marker;
}

}  // namespace vis_utils
