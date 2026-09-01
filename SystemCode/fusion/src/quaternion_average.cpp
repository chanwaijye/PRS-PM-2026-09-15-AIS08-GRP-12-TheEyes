#include "the_eyes/fusion/quaternion_average.hpp"

#include <stdexcept>

#include <Eigen/Eigenvalues>

#include "the_eyes/fusion/so3.hpp"

namespace the_eyes::fusion {

Eigen::Quaterniond WeightedQuaternionAverage(const std::vector<Eigen::Quaterniond>& quats,
                                             const std::vector<double>& weights) {
  if (quats.empty()) {
    throw std::invalid_argument("WeightedQuaternionAverage: no quaternions");
  }
  if (quats.size() != weights.size()) {
    throw std::invalid_argument("WeightedQuaternionAverage: size mismatch between "
                                "quaternions and weights");
  }
  if (quats.size() == 1) return quats.front().normalized();

  double weight_sum = 0.0;
  for (const double w : weights) {
    if (w < 0.0) throw std::invalid_argument("WeightedQuaternionAverage: negative weight");
    weight_sum += w;
  }
  if (weight_sum <= 0.0) {
    throw std::invalid_argument("WeightedQuaternionAverage: weights sum to zero");
  }

  Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
  for (size_t i = 0; i < quats.size(); ++i) {
    const Eigen::Vector4d q = quats[i].normalized().coeffs();  // (x, y, z, w)
    m.noalias() += weights[i] * q * q.transpose();
  }
  m /= weight_sum;

  // M is symmetric PSD by construction, so the self-adjoint solver applies and
  // gives real eigenvalues in ascending order.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(m);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("WeightedQuaternionAverage: eigendecomposition failed");
  }
  const Eigen::Vector4d principal = solver.eigenvectors().col(3);

  Eigen::Quaterniond avg;
  avg.coeffs() = principal;
  avg.normalize();

  // The eigenvector's sign is arbitrary. Fix it against the heaviest input so
  // repeated calls on the same data return the same quaternion rather than
  // alternating between q and -q -- fusion output must be deterministic.
  size_t heaviest = 0;
  for (size_t i = 1; i < weights.size(); ++i) {
    if (weights[i] > weights[heaviest]) heaviest = i;
  }
  return so3::CanonicalizeSign(avg, quats[heaviest].normalized());
}

}  // namespace the_eyes::fusion
