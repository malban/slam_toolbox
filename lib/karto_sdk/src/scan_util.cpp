
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

/*
  // get the scan data
  const auto& range_readings = scan.GetRangeReadingsVector();
  const auto& points = scan.GetPointReadings();

  std::vector<Vector2<kt_double>> valid_points;
  valid_points.reserve(points.size());
  for (size_t i = 0; i < points.size(); i++)
  {
    if (!std::isfinite(range_readings[i]))
    {
      continue;
    }
    valid_points.push_back(points[i]);
  }

  // compute sequential distance of each valid point in the scan from the first point
  std::vector<double> distances(1, 0.0f);
  distances.reserve(valid_points.size());
  int last_point_idx = -1;
  for (int i = 1; i < valid_points.size(); i++)
  {
    distances[i] = distances[i - 1] + (valid_points[i] - valid_points[i - 1]).Length();
  }

  double half_baseline = 0.5f * baseline;

  // calculate the normal vector of each valid point that has a valid correspondence
  int curr_idx = 0;
  int start_idx = 0;
  int end_idx   = 0;


  std::vector<Eigen::Vector2d> normals(points.size(), {0, 0});
  for (int i = 0; i < normals.size(); i++)
  {
    if (!std::isfinite(range_readings[i]))
    {
      continue;
    }

    double current_dist = distances[curr_idx++];

    // find the interpolated start point to measure the normal of the scan point
    auto start_pt = valid_points[start_idx];
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
      start_pt = valid_points[start_idx - 1] * w + valid_points[start_idx] * (1.0 - w);
    }

    // find the interpolated end point to measure the normal of the scan point
    auto end_pt = valid_points[end_idx];
    if (distances.back() - current_dist > half_baseline)
    {
      double target_dist = current_dist + half_baseline;
      while (distances[end_idx] < target_dist && end_idx < valid_points.size())
      {
        end_idx++;
      }

      double r = half_baseline - (distances[end_idx - 1] - current_dist);
      double l = distances[end_idx] - distances[end_idx - 1];
      double w = r / l;
      end_pt  = valid_points[end_idx] * w + valid_points[end_idx - 1] * (1.0 - w);
    }

    Eigen::Vector2d n(start_pt.GetY() - end_pt.GetY(), end_pt.GetX() - start_pt.GetX());
    normals[i] = n.normalized();
  }

  return normals;
}

*/

LocalizedRangeScan::Ptr aggregateScanPoints(const LocalizedRangeScanVector& scans)
{
  if (scans.empty()) {
    return {};
  }

  // TODO aggregate normals

  size_t total_points = scans[0]->GetNumberOfRangeReadings() * scans.size();

  auto aggregated = std::make_shared<LocalizedRangeScan>(scans[0]->GetSensorName(), std::vector<double>());
  aggregated->SetUniqueId(scans[0]->GetUniqueId());
  aggregated->SetTime(scans[0]->GetTime());
  aggregated->SetStateId(scans[0]->GetStateId());
  aggregated->SetCorrectedPose(scans[0]->GetCorrectedPose());

  auto point_readings = scans[0]->GetPointReadings(true);
  point_readings.reserve(point_readings.size() * scans.size());

  for (size_t i = 1; i < scans.size(); i++) {
    const auto& points = scans[i]->GetPointReadings(true);
    point_readings.insert(point_readings.end(), points.begin(), points.end());
  }

  // fill in range readings with arbitrary 'valid' ranges
  std::vector<double> range_readings(point_readings.size(), 1.0);
  range_readings.resize(total_points, std::numeric_limits<double>::quiet_NaN());
  aggregated->SetRangeReadings(range_readings);

  point_readings.resize(total_points);
  aggregated->SetPointReadings(point_readings);
  aggregated->SetIsDirty(false);


  return aggregated;
}

Eigen::Vector2d checkAxisDegeneracy(const LocalizedRangeScan& scan, float baseline)
{
  /*
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
  */


  const auto& range_readings = scan.GetRangeReadingsVector();

  double range_threshold = scan.GetLaserRangeFinder()->GetRangeThreshold();
  double minimum_range = scan.GetLaserRangeFinder()->GetMinimumRange();

  std::vector<Eigen::Vector2d> normals = calculatePointNormals(scan, 0.1, true);
  std::vector<Eigen::Vector2d> valid_normals;
  valid_normals.reserve(normals.size());
  for (size_t i = 0; i < range_readings.size(); i++)
  {
    if (math::InRange(range_readings[i], minimum_range, range_threshold))
    {
      valid_normals.push_back(normals[i]);
    }
  }

  Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>> N(valid_normals.data()->data(), 2, valid_normals.size());


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
