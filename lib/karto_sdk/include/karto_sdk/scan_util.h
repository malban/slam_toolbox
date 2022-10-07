#pragma once

#include "Eigen/Core"
#include "Karto.h"  // NOLINT

namespace karto
{

std::vector<Eigen::Vector2d> calculatePointNormals(const LocalizedRangeScan& scan, float baseline, bool unfiltered=false);

}  // namespace karto
