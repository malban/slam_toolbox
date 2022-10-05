#include "karto_sdk/Math.h"

#include "Eigen/Eigenvalues"
#include "Eigen/SVD"

namespace karto
{

Eigen::Matrix2d GetPrincipalComponents(const Eigen::Matrix2d& matrix)
{
  Eigen::EigenSolver<Eigen::Matrix2d> solver(matrix);

  auto eigen_values = solver.eigenvalues();
  auto eigen_vectors = solver.eigenvectors();

  double primary_val = std::fabs(eigen_values[0].real());
  double secondary_val = std::fabs(eigen_values[1].real());

  Eigen::Vector2d primary({ eigen_vectors.col(0)[0].real(), eigen_vectors.col(0)[1].real() });
  primary *= primary_val;

  Eigen::Vector2d secondary({ eigen_vectors.col(1)[0].real(), eigen_vectors.col(1)[1].real() });
  secondary *= secondary_val;

  if (primary_val < secondary_val)
  {
    std::swap(primary, secondary);
  }

  Eigen::Matrix2d components;
  components.row(0) = primary;
  components.row(1) = secondary;

  return components;
}

Eigen::Matrix2d GetPrincipalComponents(const std::vector<Eigen::Vector2d>& data)
{
  Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> A(data.data()->data(), 2, data.size());

  // compute the SVD of the data
  auto svd  = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  auto singular_values = svd.singularValues();
  int min_idx = 0;
  int max_idx = 1;
  float min_val = singular_values(0);
  float max_val = singular_values(1);
  if (min_val > max_val)
  {
    std::swap(min_idx, max_idx);
    std::swap(min_val, max_val);
  }

  Eigen::Matrix2d components;
  components.row(0) = svd.matrixU().col(max_idx).normalized() * max_val;
  components.row(1) = svd.matrixU().col(min_idx).normalized() * min_val;

  return components;
}


}
