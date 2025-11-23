/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "UpdaterPose.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

UpdaterPose::UpdaterPose(UpdaterOptions &options, bool rotate_180) : _options(options), _rotate_180(rotate_180) {
  // Initialize chi squared test table with confidence level 0.95
  for (int i = 1; i < 100; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
  
  PRINT_DEBUG("[POSE]: UpdaterPose initialized\n");
  PRINT_DEBUG("[POSE]:   chi2_multipler = %.2f\n", _options.chi2_multipler);
  PRINT_DEBUG("[POSE]:   rotate_180 = %s\n", _rotate_180 ? "true" : "false");
}

void UpdaterPose::feed_pose(const ov_core::PoseData &message, double oldest_time) {
  // Add the pose measurement to our buffer
  ov_core::PoseData pose_to_add = message;
  
  // If rotation is enabled, rotate the pose by 180 degrees in yaw
  if (_rotate_180) {
    // Create a 180-degree rotation around Z-axis using rotation matrix method
    // Step 1: Convert input quaternion to rotation matrix
    Eigen::Matrix3d R_orig = quat_2_Rot(message.quat);
    
    // Step 2: Create 180° rotation around Z-axis matrix
    Eigen::Matrix3d R_z_180;
    R_z_180 << -1.0,  0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0,  0.0, 1.0;
    
    // Step 3: Apply rotation: R_new = R_z_180 * R_orig
    Eigen::Matrix3d R_rotated = R_z_180 * R_orig;
    
    // Step 4: Convert back to quaternion
    pose_to_add.quat = rot_2_quat(R_rotated);
    
    // Extract yaw for debug output
    double yaw_orig = std::atan2(R_orig(1, 0), R_orig(0, 0));
    double yaw_rotated = std::atan2(R_rotated(1, 0), R_rotated(0, 0));
  
    
    // For position: Rotate by 180° around Z-axis
    // This flips X and Y, keeps Z the same
    Eigen::Matrix3d R_rot_180;
    R_rot_180 << -1.0,  0.0, 0.0,
                  0.0, -1.0, 0.0,
                  0.0,  0.0, 1.0;
    pose_to_add.pos = R_rot_180 * message.pos;
    
    // Covariance doesn't change for a 180° rotation around Z
    // (it's symmetric around the rotation axis)
    pose_to_add.covariance = message.covariance;
    
    PRINT_DEBUG("[POSE]: Rotated input pose by 180 degrees in yaw\n");
    PRINT_DEBUG("[POSE]:   Original quat: [%.3f, %.3f, %.3f, %.3f]\n", 
                message.quat(0), message.quat(1), message.quat(2), message.quat(3));
    PRINT_DEBUG("[POSE]:   Rotated quat:  [%.3f, %.3f, %.3f, %.3f]\n", 
                pose_to_add.quat(0), pose_to_add.quat(1), pose_to_add.quat(2), pose_to_add.quat(3));
    PRINT_DEBUG("[POSE]:   Original yaw: %.2f deg, Rotated yaw: %.2f deg\n", 
                yaw_orig * 180.0 / M_PI, yaw_rotated * 180.0 / M_PI);
  }
  
  pose_data.push_back(pose_to_add);
  
  // Clean old measurements
  clean_old_pose_measurements(oldest_time - 0.10);
}

void UpdaterPose::clean_old_pose_measurements(double oldest_time) {
  if (oldest_time < 0)
    return;
    
  auto it = pose_data.begin();
  while (it != pose_data.end()) {
    if (it->timestamp < oldest_time) {
      it = pose_data.erase(it);
    } else {
      it++;
    }
  }
}

bool UpdaterPose::try_update(std::shared_ptr<State> state, double timestamp) {
  
  // Return if we don't have any pose data
  if (pose_data.empty()) {
    return false;
  }

  // Find the closest pose measurement to the requested timestamp
  ov_core::PoseData *closest_pose = nullptr;
  double min_time_diff = std::numeric_limits<double>::max();
  
  for (auto &pose : pose_data) {
    double time_diff = std::abs(pose.timestamp - timestamp);
    if (time_diff < min_time_diff) {
      min_time_diff = time_diff;
      closest_pose = &pose;
    }
  }

  // If no measurement within 0.1 seconds, skip update
  if (closest_pose == nullptr || min_time_diff > 0.1) {
    PRINT_DEBUG("[POSE]: No measurement within 0.1s of timestamp %.3f (closest: %.3f)\n", 
                timestamp, min_time_diff);
    return false;
  }

  //==========================================================
  //==========================================================
  // Build the measurement update
  // Measurement dimension: 6 (3 position + 3 orientation)
  // State ordering: [q_GtoI, p_IinG, v_IinG, bg, ba]
  // We measure position and orientation directly
  
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 6);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd R = closest_pose->covariance;

  // Debug: Print the measurement covariance
  PRINT_DEBUG("[POSE-DEBUG]: Measurement covariance diagonal: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
              R(0,0), R(1,1), R(2,2), R(3,3), R(4,4), R(5,5));

  // Get current state estimate
  Eigen::Matrix<double, 4, 1> q_GtoI = state->_imu->quat();
  Eigen::Matrix<double, 3, 1> p_IinG = state->_imu->pos();

  // Position residual: measured - estimated
  res.block<3, 1>(0, 0) = closest_pose->pos - p_IinG;

  // Orientation residual (as small angle error)
  // We compute the error quaternion: q_err = q_meas * q_est^-1
  // Then convert to small angle approximation
  Eigen::Matrix<double, 4, 1> q_meas = closest_pose->quat;
  Eigen::Matrix<double, 4, 1> q_err = quat_multiply(q_meas, Inv(q_GtoI));
  
  // Small angle approximation: theta ≈ 2 * [q_x, q_y, q_z]
  // This is valid for small rotations (which should be the case after VIO convergence)
  res.block<3, 1>(3, 0) = 2.0 * q_err.block<3, 1>(0, 0);

  // Debug: Print the actual values for diagnosis
  PRINT_DEBUG("[POSE-DEBUG]: Measured pos: [%.3f, %.3f, %.3f]\n", 
              closest_pose->pos(0), closest_pose->pos(1), closest_pose->pos(2));
  PRINT_DEBUG("[POSE-DEBUG]: Estimated pos: [%.3f, %.3f, %.3f]\n", 
              p_IinG(0), p_IinG(1), p_IinG(2));
  PRINT_DEBUG("[POSE-DEBUG]: Position residual: [%.3f, %.3f, %.3f] (norm: %.3f)\n", 
              res(0), res(1), res(2), res.block<3,1>(0,0).norm());
  PRINT_DEBUG("[POSE-DEBUG]: Measured quat: [%.3f, %.3f, %.3f, %.3f]\n", 
              q_meas(0), q_meas(1), q_meas(2), q_meas(3));
  PRINT_DEBUG("[POSE-DEBUG]: Estimated quat: [%.3f, %.3f, %.3f, %.3f]\n", 
              q_GtoI(0), q_GtoI(1), q_GtoI(2), q_GtoI(3));
  PRINT_DEBUG("[POSE-DEBUG]: Orientation residual: [%.3f, %.3f, %.3f] (norm: %.3f rad = %.1f deg)\n", 
              res(3), res(4), res(5), res.block<3,1>(3,0).norm(), 
              res.block<3,1>(3,0).norm() * 180.0 / M_PI);

  //==========================================================
  //==========================================================
  // Sanity check: Reject measurements with unreasonably large residuals
  // This prevents corrupting the filter with bad measurements
  
  double pos_residual_norm = res.block<3,1>(0,0).norm();
  double rot_residual_norm = res.block<3,1>(3,0).norm();
  
  // Reject if position error > 10 meters
  if (pos_residual_norm > 10.0) {
    PRINT_WARNING(YELLOW "[POSE]: Position residual too large (%.3fm > 10m), rejecting measurement\n" RESET,
                  pos_residual_norm);
    PRINT_WARNING(YELLOW "[POSE]: This likely means frame mismatch between external pose and VIO estimate\n" RESET);
    return false;
  }
  
  // Reject if orientation error > 90 degrees (pi/2 radians)
  if (rot_residual_norm > M_PI/2.0) {
    PRINT_WARNING(YELLOW "[POSE]: Orientation residual too large (%.1f deg > 90 deg), rejecting measurement\n" RESET,
                  rot_residual_norm * 180.0 / M_PI);
    PRINT_WARNING(YELLOW "[POSE]: This likely means quaternion convention mismatch\n" RESET);
    return false;
  }

  //==========================================================
  //==========================================================
  // Jacobian construction
  // H = [dh/dq, dh/dp, 0, 0, 0]
  // For the condensed Jacobian, we only include states that affect the measurement
  
  // Orientation measurement is affected by orientation state (first 3 DOF)
  H.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
  
  // Position measurement is affected by position state (DOF 3-5 in the condensed form)
  H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

  //==========================================================
  //==========================================================
  // Perform chi-squared test to reject outliers
  
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->p());

  // Get marginal covariance for the states we're measuring
  Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
  
  // Innovation covariance: S = H * P * H^T + R
  Eigen::MatrixXd S = H * P_marg * H.transpose() + R;
  
  // Check that S is positive definite before inverting
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(S);
  if (eigensolver.info() != Eigen::Success) {
    PRINT_WARNING(YELLOW "[POSE]: Failed to compute eigenvalues of innovation covariance S\n" RESET);
    return false;
  }
  double min_eigenvalue = eigensolver.eigenvalues().minCoeff();
  if (min_eigenvalue <= 0) {
    PRINT_WARNING(YELLOW "[POSE]: Innovation covariance S is not positive definite (min eigenvalue: %.6f)\n" RESET,
                  min_eigenvalue);
    return false;
  }
  
  // Mahalanobis distance: chi2 = res^T * S^-1 * res
  double chi2 = res.transpose() * S.inverse() * res;
  
  // Check against chi-squared threshold
  int dof = res.rows();
  double chi2_thresh = chi_squared_table[dof] * _options.chi2_multipler;
  
  PRINT_WARNING("[POSE-DEBUG]: Chi-squared calculation:\n");
  PRINT_WARNING("[POSE-DEBUG]:   DOF = %d\n", dof);
  PRINT_WARNING("[POSE-DEBUG]:   chi2_table[%d] = %.2f\n", dof, chi_squared_table[dof]);
  PRINT_WARNING("[POSE-DEBUG]:   chi2_multipler = %.2f\n", _options.chi2_multipler);
  PRINT_WARNING("[POSE-DEBUG]:   threshold = %.2f * %.2f = %.2f\n", 
              chi_squared_table[dof], _options.chi2_multipler, chi2_thresh);
  PRINT_WARNING("[POSE-DEBUG]:   actual chi2 = %.2f\n", chi2);
  
  if (chi2 > chi2_thresh) {
    PRINT_WARNING(YELLOW "[POSE]: Chi-squared test failed (%.2f > %.2f), rejecting measurement\n" RESET,
                  chi2, chi2_thresh);
    return false;
  }

  //==========================================================
  //==========================================================
  // Perform the EKF update
  
  StateHelper::EKFUpdate(state, Hx_order, H, res, R);

  // Validate that the covariance is still valid after update
  Eigen::MatrixXd P_updated = StateHelper::get_marginal_covariance(state, Hx_order);
  for (int i = 0; i < P_updated.rows(); i++) {
    if (P_updated(i, i) <= 0) {
      PRINT_ERROR(RED "[POSE]: ERROR - Covariance diagonal %d became negative/zero (%.6f) after update!\n" RESET,
                  i, P_updated(i, i));
      PRINT_ERROR(RED "[POSE]: This indicates numerical instability. Update corrupted the filter.\n" RESET);
      // Note: State is already corrupted at this point, but at least we can see what happened
      return true; // Return true since update was applied (even though it broke things)
    }
  }

  // Print debug information
  PRINT_INFO(GREEN "[POSE]: Successful update at time %.3f (chi2 = %.2f < %.2f)\n" RESET,
             timestamp, chi2, chi2_thresh);
  PRINT_DEBUG("[POSE]: Position residual: [%.3f, %.3f, %.3f]\n", 
              res(0), res(1), res(2));
  PRINT_DEBUG("[POSE]: Orientation residual: [%.3f, %.3f, %.3f] deg\n", 
              res(3)*180.0/M_PI, res(4)*180.0/M_PI, res(5)*180.0/M_PI);
  
  return true;
}
