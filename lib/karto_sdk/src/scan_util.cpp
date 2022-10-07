
#include "Eigen/SVD"
#include "karto_sdk/scan_util.h"

namespace karto
{

std::vector<Eigen::Vector2d> calculatePointNormals(const LocalizedRangeScan& scan, float baseline, bool unfiltered)
{
  // get filtered scan points
  const auto& readings = scan.GetRangeReadingsVector();
  const auto& raw_points = scan.GetPointReadings();
  std::vector<Vector2<kt_double>> points;
  points.reserve(raw_points.size());
  for (size_t i = 0; i < readings.size(); i++)
  {
    if (std::isfinite(readings[i]))
    {
      points.push_back(raw_points[i]);
    }
  }

  // compute sequential distance of each valid point in the scan from the first point
  std::vector<double> distances(points.size(), 0.0f);
  for (int i = 1; i < points.size(); i++)
  {
    distances[i] = distances[i - 1] + (points[i] - points[i - 1]).Length();
  }

  double half_baseline = 0.5f * baseline;

  // calculate the normal vector of each valid point that has a valid correspondence
  int start_idx = 0;
  int end_idx   = 0;
  std::vector<Eigen::Vector2d> normals(points.size());
  for (int i = 0; i < points.size(); i++)
  {
    double current_dist = distances[i];

    // find the interpolated start point to measure the normal of the scan point
    auto start_pt = points[start_idx];
    if (current_dist > half_baseline)
    {
      double target_dist = current_dist - half_baseline;
      while (distances[start_idx] < target_dist)
      {
        start_idx++;
      }

      double r  = half_baseline - (current_dist - distances[start_idx]);
      double l  = distances[start_idx] - distances[start_idx - 1];
      double w  = r / l;
      start_pt = points[start_idx - 1] * w + points[start_idx] * (1.0 - w);
    }

    // find the interpolated end point to measure the normal of the scan point
    auto end_pt = points[end_idx];
    if (distances.back() - current_dist > half_baseline)
    {
      double target_dist = current_dist + half_baseline;
      while (distances[end_idx] < target_dist)
      {
        end_idx++;
      }

      double r = half_baseline - (distances[end_idx - 1] - current_dist);
      double l = distances[end_idx] - distances[end_idx - 1];
      double w = r / l;
      end_pt  = points[end_idx] * w + points[end_idx - 1] * (1.0 - w);
    }

    Eigen::Vector2d n(start_pt.GetY() - end_pt.GetY(), end_pt.GetX() - start_pt.GetX());
    normals[i] = n.normalized();
  }

  if (!unfiltered)
  {
    return normals;
  }

  std::vector<Eigen::Vector2d> unfiltered_normals(readings.size(), {0.0, 0.0});
  size_t curr_idx = 0;
  for (size_t i = 0; i < unfiltered_normals.size() && curr_idx < normals.size(); i++)
  {
    if (!std::isfinite(readings[i]))
    {
      continue;
    }
    unfiltered_normals[i] = normals[curr_idx++];
  }

  return unfiltered_normals;
}

}  // namespace karto
