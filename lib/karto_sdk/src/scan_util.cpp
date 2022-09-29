
#include "Eigen/SVD"
#include "karto_sdk/scan_util.h"


namespace karto
{

Eigen::Vector2d checkAxisDegeneracy(const LocalizedRangeScan& scan, float baseline)
{
  // get filtered scan points
  const auto& points = scan.GetPointReadings(true);

  if (points.size() < 2)
  {
    return {0, 0};
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

  Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>> N(normals.data()->data(), 2, normals.size());

  // compute the SVD of the normal vector matrix. if sigma_min << sigma_max,
  // the pointcloud is degenerate (i.e. uninformative in a given direction)
  auto svd             = N.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  auto singular_values = svd.singularValues();
  int min_idx          = 0;
  int max_idx          = 1;
  float min_val        = singular_values(0);
  float max_val        = singular_values(1);
  if (min_val > max_val)
  {
    std::swap(min_idx, max_idx);
    std::swap(min_val, max_val);
  }

  if (max_val == 0)
  {
    return {0, 0};
  }

  float degeneracy             = std::max(0.0001, 1.0 - std::min(1.0f, min_val / max_val));
  Eigen::Vector2d primary_axis = svd.matrixU().col(max_idx);
  primary_axis.normalize();

  Eigen::Vector2d secondary_axis = {-primary_axis[1], primary_axis[0]};

  return secondary_axis * degeneracy;
}

}  // namespace karto
