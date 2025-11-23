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

#ifndef OV_MSCKF_UPDATER_POSE_H
#define OV_MSCKF_UPDATER_POSE_H

#include <deque>
#include <map>
#include <memory>

#include "UpdaterOptions.h"
#include "utils/sensor_data.h"

namespace ov_msckf {

class State;

/**
 * @brief Updates the state using external pose measurements with covariance
 *
 * This updater takes pose measurements from an external source (e.g., motion capture,
 * GPS+compass, or another localization system) and fuses them with the VIO estimate.
 * The pose measurements include both position and orientation along with their
 * associated covariance matrices.
 *
 * The measurement model is:
 * - Position: z_p = p_IinG + n_p
 * - Orientation: z_q = q_GtoI * exp(n_q)
 *
 * where n_p and n_q are zero-mean Gaussian noise terms.
 */
class UpdaterPose {

public:
  /**
   * @brief Constructor
   * @param options Updater options (chi2 multiplier, etc.)
   * @param rotate_180 If true, rotate input pose by 180 degrees in yaw
   */
  UpdaterPose(UpdaterOptions &options, bool rotate_180 = false);

  /**
   * @brief Feed function for pose data
   * @param message Contains timestamp, pose, and covariance
   * @param oldest_time Time before which measurements can be discarded
   */
  void feed_pose(const ov_core::PoseData &message, double oldest_time = -1);

  /**
   * @brief Try to perform an update with a pose measurement
   * @param state State of the filter
   * @param timestamp Timestamp to update at
   * @return True if update was performed successfully
   */
  bool try_update(std::shared_ptr<State> state, double timestamp);

protected:
  /// Options used during update (chi2 multiplier)
  UpdaterOptions _options;

  /// If we should rotate input pose by 180 degrees in yaw
  bool _rotate_180;

  /// Chi squared 95th percentile table (lookup by DOF)
  std::map<int, double> chi_squared_table;

  /// Buffer of pose measurements (timestamp, pose, covariance)
  std::deque<ov_core::PoseData> pose_data;

  /**
   * @brief Remove measurements older than the specified time
   * @param oldest_time Timestamp before which measurements should be removed
   */
  void clean_old_pose_measurements(double oldest_time);
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_POSE_H
