// Weighted quaternion averaging by the Markley maximum-eigenvalue method.
//
//   Markley, Cheng, Crassidis & Oshman (2007), "Averaging Quaternions",
//   Journal of Guidance, Control, and Dynamics 30(4).
//
// Implemented from the paper rather than taken from a library: CLAUDE.md
// constraint 2 makes pose averaging part of the graded contribution.
//
// Method: build M = sum_i w_i q_i q_i^T (4x4, symmetric positive semi-definite).
// The average is the eigenvector of M for the largest eigenvalue. The outer
// product q q^T is invariant to the sign of q, which is exactly why this method
// suits multi-view fusion -- two views reporting q and -q describe the same
// rotation, and naive componentwise averaging would cancel them to zero.
#ifndef THE_EYES_FUSION_QUATERNION_AVERAGE_HPP
#define THE_EYES_FUSION_QUATERNION_AVERAGE_HPP

#include <vector>

#include <Eigen/Geometry>

namespace the_eyes::fusion {

// Throws std::invalid_argument on empty input, mismatched sizes, or all-zero /
// negative weights. Weights need not sum to 1.
Eigen::Quaterniond WeightedQuaternionAverage(const std::vector<Eigen::Quaterniond>& quats,
                                             const std::vector<double>& weights);

}  // namespace the_eyes::fusion

#endif  // THE_EYES_FUSION_QUATERNION_AVERAGE_HPP
