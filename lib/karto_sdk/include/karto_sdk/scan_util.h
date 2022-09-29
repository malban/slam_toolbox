#pragma once

#include "Eigen/Core"
#include "Karto.h"  // NOLINT

namespace karto
{

Eigen::Vector2d checkAxisDegeneracy(const LocalizedRangeScan& scan, float baseline);

}  // namespace karto
