#pragma once

#include "Eigen/Core"
#include "Karto.h"  // NOLINT

namespace karto
{

std::vector<Eigen::Vector2d> calculatePointNormals(const LocalizedRangeScan& scan, float baseline, bool unfiltered=false);

LocalizedRangeScan::Ptr aggregateScanPoints(const LocalizedRangeScanVector& scan);

Eigen::Vector2d checkAxisDegeneracy(const LocalizedRangeScan& scan, float baseline);

}  // namespace karto
