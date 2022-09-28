/*
 * loop_closure_assistant
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

#include <unordered_map>
#include <memory>

#include "slam_toolbox/loop_closure_assistant.hpp"
#include "solvers/ceres_utils.h"

namespace loop_closure_assistant
{

/*****************************************************************************/
LoopClosureAssistant::LoopClosureAssistant(
  rclcpp::Node::SharedPtr node,
  karto::Mapper * mapper,
  laser_utils::ScanHolder * scan_holder,
  PausedState & state, ProcessType & processor_type)
: mapper_(mapper), scan_holder_(scan_holder),
  node_(node), state_(state),
  processor_type_(processor_type)
/*****************************************************************************/
{
  node_->declare_parameter("paused_processing", false);
  node_->set_parameter(rclcpp::Parameter("paused_processing", false));

  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  solver_ = mapper_->getScanSolver();

  scan_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "slam_toolbox/scan_visualization",10);

  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "slam_toolbox/graph_visualization", rclcpp::QoS(1));
  map_frame_ = node->get_parameter("map_frame").as_string();
}

/*****************************************************************************/
void LoopClosureAssistant::publishGraph()
/*****************************************************************************/
{
  if(marker_publisher_->get_subscription_count() == 0){
    return;
  }
  
  auto graph = solver_->getGraph();

  if (graph->size() == 0) {
    return;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Graph size: %zu", graph->size());

  const auto & vertices = mapper_->GetGraph()->GetVertices();
  const auto & edges = mapper_->GetGraph()->GetEdges();
  const auto & localization_vertices = mapper_->GetLocalizationVertices();

  int first_localization_id = std::numeric_limits<int>::max();
  if (!localization_vertices.empty()) {
    first_localization_id = localization_vertices.front().vertex->GetObject()->GetUniqueId();
  }

  visualization_msgs::msg::MarkerArray marray;

  // clear existing markers to account for any removed nodes
  visualization_msgs::msg::Marker clear;
  clear.header.stamp = node_->now();
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marray.markers.push_back(clear);

  visualization_msgs::msg::Marker m = vis_utils::toMarker(map_frame_, "slam_toolbox", 0.1, node_);

  // add map nodes
  for (const auto & sensor_name : vertices) {
    for (const auto & vertex : sensor_name.second) {
      m.color.g = vertex.first < first_localization_id ? 0.0 : 1.0;
      const auto & pose = vertex.second->GetObject()->GetCorrectedPose();
      m.id = vertex.first;
      m.pose.position.x = pose.GetX();
      m.pose.position.y = pose.GetY();

      marray.markers.push_back(m);

    }
  }

  // add line markers for graph edges
  visualization_msgs::msg::Marker edges_marker;
  edges_marker.header.frame_id = map_frame_;
  edges_marker.header.stamp = node_->now();
  edges_marker.id = 0;
  edges_marker.ns = "slam_toolbox_edges";
  edges_marker.action = visualization_msgs::msg::Marker::ADD;
  edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker.pose.orientation.w = 1;
  edges_marker.scale.x = 0.05;
  // edges_marker.color.b = 1;
  // edges_marker.color.a = 1;
  edges_marker.lifetime = rclcpp::Duration::from_seconds(0);
  edges_marker.points.reserve(edges.size() * 2);
  edges_marker.colors.reserve(edges.size() * 2);

  visualization_msgs::msg::Marker localization_edges_marker;
  localization_edges_marker.header.frame_id = map_frame_;
  localization_edges_marker.header.stamp = node_->now();
  localization_edges_marker.id = 1;
  localization_edges_marker.ns = "slam_toolbox_edges";
  localization_edges_marker.action = visualization_msgs::msg::Marker::ADD;
  localization_edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  localization_edges_marker.pose.orientation.w = 1;
  localization_edges_marker.scale.x = 0.05;
  localization_edges_marker.color.g = 1;
  localization_edges_marker.color.b = 1;
  localization_edges_marker.color.a = 1;
  localization_edges_marker.lifetime = rclcpp::Duration::from_seconds(0);
  localization_edges_marker.points.reserve(localization_vertices.size() * 3);

  for (const auto & edge : edges) {
    int source_id = edge->GetSource()->GetObject()->GetUniqueId();
    const auto & pose0 = edge->GetSource()->GetObject()->GetCorrectedPose();
    geometry_msgs::msg::Point p0;
    p0.x = pose0.GetX();
    p0.y = pose0.GetY();
    double p0_yaw = pose0.GetHeading();

    int target_id = edge->GetTarget()->GetObject()->GetUniqueId();
    const auto & pose1 = edge->GetTarget()->GetObject()->GetCorrectedPose();
    geometry_msgs::msg::Point p1;
    p1.x = pose1.GetX();
    p1.y = pose1.GetY();
    double p1_yaw = pose1.GetHeading();

    if (source_id >= first_localization_id || target_id >= first_localization_id) {
      localization_edges_marker.points.push_back(p0);
      localization_edges_marker.points.push_back(p1);
    } else {
      edges_marker.points.push_back(p0);
      edges_marker.points.push_back(p1);

      // update the edge marker color based on the current residual value
      karto::LinkInfo * pLinkInfo = (karto::LinkInfo *)(edge->GetLabel());
      karto::Pose2 diff = pLinkInfo->GetPoseDifference();
      Eigen::Vector3d pose2d(diff.GetX(), diff.GetY(), diff.GetHeading());

      karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
      Eigen::Matrix3d information;
      information(0, 0) = precisionMatrix(0, 0);
      information(0, 1) = information(1, 0) = precisionMatrix(0, 1);
      information(0, 2) = information(2, 0) = precisionMatrix(0, 2);
      information(1, 1) = precisionMatrix(1, 1);
      information(1, 2) = information(2, 1) = precisionMatrix(1, 2);
      information(2, 2) = precisionMatrix(2, 2);
      Eigen::Matrix3d sqrt_information = information.llt().matrixU();

      auto error_function = PoseGraph2dErrorTerm(pose2d(0), pose2d(1), pose2d(2), sqrt_information);
      double residuals[3];
      error_function(&p0.x, &p0.y, &p0_yaw, &p1.x, &p1.y, &p1_yaw, &residuals[0]);
      // TODO: The residual as a vector but we need a scalar to visualize, so using arbitrary weights atm.
      double residual_value = std::abs(residuals[0]) + std::abs(residuals[1]) + std::abs(residuals[2]);
      const double min_residual = 0.0;
      const double max_residual = 10.0;
      residual_value = std::min(max_residual, std::max(min_residual, residual_value));
      // convert the residual to a color (need one per point)
      auto residual_color = vis_utils::getColorScale(residual_value / max_residual);
      edges_marker.colors.push_back(residual_color);
      edges_marker.colors.push_back(residual_color);
    }
  }

  marray.markers.push_back(edges_marker);
  marray.markers.push_back(localization_edges_marker);

  // if disabled, clears out old markers
  marker_publisher_->publish(marray);
}


/*****************************************************************************/
void LoopClosureAssistant::moveNode(
  const int & id, const Eigen::Vector3d & pose)
/*****************************************************************************/
{
  solver_->ModifyNode(id, pose);
}


/*****************************************************************************/
void  LoopClosureAssistant::clearMovedNodes()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_.clear();
}

/*****************************************************************************/
void LoopClosureAssistant::addMovedNodes(const int & id, Eigen::Vector3d vec)
/*****************************************************************************/
{
  RCLCPP_INFO(
    node_->get_logger(),
    "LoopClosureAssistant: Node %i new manual loop closure "
    "pose has been recorded.",id);
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_[id] = vec;
}

/*****************************************************************************/
void LoopClosureAssistant::setMapper(karto::Mapper * mapper)
/*****************************************************************************/
{
  mapper_ = mapper;
}

}  // namespace loop_closure_assistant
