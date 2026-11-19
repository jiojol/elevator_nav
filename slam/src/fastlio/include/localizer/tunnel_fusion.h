#ifndef FASTLIO_LOCALIZER_TUNNEL_FUSION_H_
#define FASTLIO_LOCALIZER_TUNNEL_FUSION_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace fastlio {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

enum class FusionMode : std::uint8_t {
  NORMAL = 0,
  DEGRADED = 1,
  RECOVERING = 2,
  LOST = 3,
  RELOCALIZATION_REQUIRED = 4,
};

struct PoseResidual {
  double translation = std::numeric_limits<double>::infinity();
  double yaw = std::numeric_limits<double>::infinity();
};

struct TranslationObservability {
  bool available = false;
  double ratio = 0.0;
  double strong_value = 0.0;
  double weak_value = 0.0;
  Eigen::Vector2d strong_direction = Eigen::Vector2d::UnitX();
  Eigen::Vector2d weak_direction = Eigen::Vector2d::UnitY();
};

struct DirectionalMotionConsistency {
  bool available = false;
  bool inconsistent = false;
  Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
  double odom_progress = 0.0;
  double lidar_progress = 0.0;
  double motion_ratio = std::numeric_limits<double>::quiet_NaN();
  double confidence = 0.0;
  double window_duration = 0.0;
  std::size_t sample_count = 0;
};

inline double wrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double yawFromMatrix(const Eigen::Matrix4d& pose) {
  return std::atan2(pose(1, 0), pose(0, 0));
}

inline Eigen::Matrix4d makePlanarPose(double x, double y, double z,
                                      double yaw) {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  pose(0, 0) = c;
  pose(0, 1) = -s;
  pose(1, 0) = s;
  pose(1, 1) = c;
  pose(0, 3) = x;
  pose(1, 3) = y;
  pose(2, 3) = z;
  return pose;
}

inline PoseResidual poseResidual(const Eigen::Matrix4d& predicted,
                                 const Eigen::Matrix4d& observed) {
  PoseResidual residual;
  if (!predicted.allFinite() || !observed.allFinite()) {
    return residual;
  }

  // Fusion thresholds are defined in map XY/yaw. Computing a full relative
  // transform first would project horizontal displacement into local Z and
  // couple yaw with roll/pitch on sloped terrain.
  residual.translation =
      (observed.block<2, 1>(0, 3) - predicted.block<2, 1>(0, 3)).norm();
  residual.yaw = std::abs(
      wrapAngle(yawFromMatrix(observed) - yawFromMatrix(predicted)));
  return residual;
}

// Compare map->odom candidates at the current robot pose. Comparing the
// transforms at their origins creates a false translation residual when a
// small yaw correction is applied far away from the odometry origin.
inline PoseResidual mapToOdomCandidateResidualAtCurrentBase(
    const Eigen::Matrix4d& first_map_to_odom,
    const Eigen::Matrix4d& second_map_to_odom,
    const Eigen::Matrix4d& odom_to_base) {
  if (!first_map_to_odom.allFinite() || !second_map_to_odom.allFinite() ||
      !odom_to_base.allFinite()) {
    return PoseResidual();
  }
  return poseResidual(first_map_to_odom * odom_to_base,
                      second_map_to_odom * odom_to_base);
}

// The translation columns of the scan-matching normal matrix are expressed in
// the map frame. SVD of the planar block therefore gives directly usable strong
// and weak correction directions in map XY.
inline TranslationObservability planarTranslationObservability(
    const Matrix6d& hessian) {
  TranslationObservability result;
  if (!hessian.allFinite()) {
    return result;
  }

  const Eigen::Matrix2d planar =
      0.5 * (hessian.topLeftCorner<2, 2>() +
             hessian.topLeftCorner<2, 2>().transpose());
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(
      planar, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Vector2d singular = svd.singularValues().cwiseAbs();
  if (!singular.allFinite() || singular.x() <= 1e-12) {
    return result;
  }

  result.available = true;
  result.strong_value = singular.x();
  result.weak_value = singular.y();
  result.ratio = result.weak_value / result.strong_value;
  result.strong_direction = svd.matrixV().col(0).normalized();
  result.weak_direction = svd.matrixV().col(1).normalized();
  return result;
}

inline bool updateDegeneracyLatch(bool currently_degenerate, double ratio,
                                  double enter_threshold,
                                  double recover_threshold) {
  if (!std::isfinite(ratio)) {
    return currently_degenerate;
  }
  recover_threshold = std::max(enter_threshold, recover_threshold);
  return currently_degenerate ? ratio < recover_threshold
                              : ratio < enter_threshold;
}

inline double motionRatioConfidence(double ratio, double hard_min,
                                    double full_min, double full_max,
                                    double hard_max) {
  if (!std::isfinite(ratio)) {
    return 0.0;
  }
  hard_min = std::max(0.0, hard_min);
  full_min = std::max(hard_min, full_min);
  full_max = std::max(full_min, full_max);
  hard_max = std::max(full_max, hard_max);
  if (ratio <= hard_min || ratio >= hard_max) {
    return 0.0;
  }
  if (ratio < full_min) {
    return (ratio - hard_min) / std::max(1e-9, full_min - hard_min);
  }
  if (ratio <= full_max) {
    return 1.0;
  }
  return (hard_max - ratio) / std::max(1e-9, hard_max - full_max);
}

inline double medianFinite(std::vector<double> values) {
  values.erase(
      std::remove_if(values.begin(), values.end(),
                     [](double value) { return !std::isfinite(value); }),
      values.end());
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2 != 0) {
    return upper;
  }
  std::nth_element(values.begin(), values.begin() + middle - 1,
                   values.begin() + middle);
  return 0.5 * (values[middle - 1] + upper);
}

// Theil-Sen median slope over a motion window. Isolated frozen results and
// neighbouring-voxel aliases do not become the next baseline as they do in an
// endpoint-only detector.
inline DirectionalMotionConsistency evaluateRobustDirectionalMotionConsistency(
    const std::vector<Eigen::Vector2d>& odom_positions,
    const std::vector<Eigen::Vector2d>& lidar_positions,
    Eigen::Vector2d direction, double min_odom_progress,
    double min_lidar_progress, double min_pair_progress,
    double hard_min_ratio, double full_min_ratio, double full_max_ratio,
    double hard_max_ratio) {
  DirectionalMotionConsistency result;
  if (odom_positions.size() != lidar_positions.size() ||
      odom_positions.size() < 2) {
    return result;
  }
  result.sample_count = odom_positions.size();
  for (std::size_t i = 0; i < odom_positions.size(); ++i) {
    if (!odom_positions[i].allFinite() || !lidar_positions[i].allFinite()) {
      result.inconsistent = true;
      return result;
    }
  }

  const Eigen::Vector2d odom_delta =
      odom_positions.back() - odom_positions.front();
  if (!direction.allFinite() || direction.norm() < 1e-9) {
    direction = odom_delta;
  }
  if (direction.norm() < 1e-9) {
    return result;
  }
  direction.normalize();
  if (direction.dot(odom_delta) < 0.0) {
    direction = -direction;
  }

  result.direction = direction;
  result.odom_progress = direction.dot(odom_delta);
  if (result.odom_progress < std::max(0.0, min_odom_progress)) {
    return result;
  }

  const double pair_progress = std::max(1e-6, min_pair_progress);
  std::vector<double> slopes;
  slopes.reserve(odom_positions.size() * (odom_positions.size() - 1) / 2);
  for (std::size_t i = 0; i + 1 < odom_positions.size(); ++i) {
    const double odom_i = direction.dot(odom_positions[i]);
    const double lidar_i = direction.dot(lidar_positions[i]);
    for (std::size_t j = i + 1; j < odom_positions.size(); ++j) {
      const double odom_step = direction.dot(odom_positions[j]) - odom_i;
      if (odom_step < pair_progress) {
        continue;
      }
      const double lidar_step = direction.dot(lidar_positions[j]) - lidar_i;
      slopes.push_back(lidar_step / odom_step);
    }
  }

  const double robust_ratio = medianFinite(std::move(slopes));
  if (!std::isfinite(robust_ratio)) {
    return result;
  }
  result.available = true;
  result.motion_ratio = robust_ratio;
  result.lidar_progress = robust_ratio * result.odom_progress;
  result.confidence = motionRatioConfidence(
      robust_ratio, hard_min_ratio, full_min_ratio, full_max_ratio,
      hard_max_ratio);
  result.inconsistent =
      result.lidar_progress < std::max(0.0, min_lidar_progress) ||
      robust_ratio <= hard_min_ratio || robust_ratio >= hard_max_ratio;
  return result;
}

inline Eigen::Matrix4d fusePlanarPoseDirectional(
    const Eigen::Matrix4d& predicted, const Eigen::Matrix4d& lidar,
    const Eigen::Vector2d& weak_direction, double weak_weight,
    double yaw_weight) {
  if (!predicted.allFinite() || !lidar.allFinite() ||
      !weak_direction.allFinite()) {
    return predicted;
  }

  Eigen::Vector2d weak = weak_direction;
  if (weak.norm() < 1e-9) {
    weak = Eigen::Vector2d::UnitX();
  } else {
    weak.normalize();
  }
  weak_weight = std::clamp(weak_weight, 0.0, 1.0);
  yaw_weight = std::clamp(yaw_weight, 0.0, 1.0);

  const Eigen::Vector2d predicted_xy(predicted(0, 3), predicted(1, 3));
  const Eigen::Vector2d lidar_xy(lidar(0, 3), lidar(1, 3));
  const Eigen::Vector2d residual = lidar_xy - predicted_xy;
  const Eigen::Vector2d corrected_residual =
      residual - (1.0 - weak_weight) * weak * weak.dot(residual);
  const double corrected_yaw =
      yawFromMatrix(predicted) +
      yaw_weight * wrapAngle(yawFromMatrix(lidar) - yawFromMatrix(predicted));
  return makePlanarPose(predicted_xy.x() + corrected_residual.x(),
                        predicted_xy.y() + corrected_residual.y(),
                        predicted(2, 3), corrected_yaw);
}

// Apply the absolute lidar residual only along the strong direction. Any weak
// correction is a separately gated relative-motion increment, never the weak
// component of an absolute scan-matching pose.
inline Eigen::Matrix4d fusePlanarPoseDirectionalRelative(
    const Eigen::Matrix4d& predicted, const Eigen::Matrix4d& lidar,
    const Eigen::Vector2d& weak_direction, double weak_relative_correction,
    double yaw_weight) {
  if (!predicted.allFinite() || !lidar.allFinite() ||
      !weak_direction.allFinite() ||
      !std::isfinite(weak_relative_correction)) {
    return predicted;
  }

  Eigen::Vector2d weak = weak_direction;
  if (weak.norm() < 1e-9) {
    weak = Eigen::Vector2d::UnitX();
  } else {
    weak.normalize();
  }
  yaw_weight = std::clamp(yaw_weight, 0.0, 1.0);

  const Eigen::Vector2d predicted_xy(predicted(0, 3), predicted(1, 3));
  const Eigen::Vector2d lidar_xy(lidar(0, 3), lidar(1, 3));
  const Eigen::Vector2d residual = lidar_xy - predicted_xy;
  const Eigen::Vector2d strong_correction =
      residual - weak * weak.dot(residual);
  const Eigen::Vector2d corrected = predicted_xy + strong_correction +
                                    weak * weak_relative_correction;
  const double corrected_yaw =
      yawFromMatrix(predicted) +
      yaw_weight * wrapAngle(yawFromMatrix(lidar) - yawFromMatrix(predicted));
  return makePlanarPose(corrected.x(), corrected.y(), predicted(2, 3),
                        corrected_yaw);
}

inline double observableTranslationResidual(
    const Eigen::Matrix4d& predicted, const Eigen::Matrix4d& lidar,
    const Eigen::Vector2d& weak_direction, bool direction_is_degenerate) {
  if (!predicted.allFinite() || !lidar.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }
  const Eigen::Vector2d residual(lidar(0, 3) - predicted(0, 3),
                                 lidar(1, 3) - predicted(1, 3));
  if (!direction_is_degenerate || !weak_direction.allFinite() ||
      weak_direction.norm() < 1e-9) {
    return residual.norm();
  }
  const Eigen::Vector2d weak = weak_direction.normalized();
  return (residual - weak * weak.dot(residual)).norm();
}

inline Eigen::Matrix4d limitPlanarCorrection(const Eigen::Matrix4d& current,
                                             const Eigen::Matrix4d& target,
                                             double max_translation,
                                             double max_yaw) {
  if (!current.allFinite() || !target.allFinite()) {
    return current;
  }
  max_translation = std::max(0.0, max_translation);
  max_yaw = std::max(0.0, max_yaw);

  const Eigen::Matrix4d error = current.inverse() * target;
  Eigen::Vector3d translation = error.block<3, 1>(0, 3);
  const double translation_norm = translation.norm();
  if (translation_norm > max_translation && translation_norm > 1e-9) {
    translation *= max_translation / translation_norm;
  }
  const double yaw =
      std::clamp(wrapAngle(yawFromMatrix(error)), -max_yaw, max_yaw);
  return current * makePlanarPose(translation.x(), translation.y(),
                                  translation.z(), yaw);
}

inline Eigen::Matrix4d limitMapToOdomCorrectionAtCurrentBase(
    const Eigen::Matrix4d& current_map_to_odom,
    const Eigen::Matrix4d& odom_to_base,
    const Eigen::Matrix4d& target_map_to_base, double max_translation,
    double max_yaw) {
  if (!current_map_to_odom.allFinite() || !odom_to_base.allFinite() ||
      !target_map_to_base.allFinite()) {
    return current_map_to_odom;
  }
  const Eigen::Matrix4d current_map_to_base =
      current_map_to_odom * odom_to_base;
  const Eigen::Matrix4d limited_map_to_base = limitPlanarCorrection(
      current_map_to_base, target_map_to_base, max_translation, max_yaw);
  return limited_map_to_base * odom_to_base.inverse();
}

class FusionStateMachine {
 public:
  FusionStateMachine(int bad_frames_to_degraded = 2,
                     int good_frames_to_recover = 10)
      : bad_frames_to_degraded_(std::max(1, bad_frames_to_degraded)),
        good_frames_to_recover_(std::max(1, good_frames_to_recover)) {}

  void reset() {
    mode_ = FusionMode::NORMAL;
    bad_frames_ = 0;
    good_frames_ = 0;
  }

  bool rejectFrame() {
    good_frames_ = 0;
    if (mode_ == FusionMode::RECOVERING) {
      ++bad_frames_;
      if (bad_frames_ >= bad_frames_to_degraded_) {
        mode_ = FusionMode::DEGRADED;
      }
      return mode_ == FusionMode::DEGRADED;
    }
    if (mode_ == FusionMode::LOST ||
        mode_ == FusionMode::RELOCALIZATION_REQUIRED) {
      return true;
    }
    ++bad_frames_;
    if (bad_frames_ >= bad_frames_to_degraded_) {
      mode_ = FusionMode::DEGRADED;
    }
    return mode_ == FusionMode::DEGRADED;
  }

  bool acceptRecoveryCandidate() {
    bad_frames_ = 0;
    if (mode_ == FusionMode::NORMAL) {
      good_frames_ = 0;
      return true;
    }
    if (mode_ == FusionMode::LOST) {
      mode_ = FusionMode::DEGRADED;
      good_frames_ = 0;
    }
    if (mode_ == FusionMode::RELOCALIZATION_REQUIRED) {
      ++good_frames_;
      if (good_frames_ >= good_frames_to_recover_) {
        mode_ = FusionMode::RECOVERING;
        return true;
      }
      return false;
    }
    if (mode_ == FusionMode::DEGRADED) {
      ++good_frames_;
      if (good_frames_ >= good_frames_to_recover_) {
        mode_ = FusionMode::RECOVERING;
        return true;
      }
      return false;
    }
    return mode_ == FusionMode::RECOVERING;
  }

  void correctionComplete() {
    mode_ = FusionMode::NORMAL;
    bad_frames_ = 0;
    good_frames_ = 0;
  }

  void setLost() {
    mode_ = FusionMode::LOST;
    bad_frames_ = 0;
    good_frames_ = 0;
  }

  void setDegraded() {
    mode_ = FusionMode::DEGRADED;
    bad_frames_ = bad_frames_to_degraded_;
    good_frames_ = 0;
  }

  void setRelocalizationRequired() {
    mode_ = FusionMode::RELOCALIZATION_REQUIRED;
    bad_frames_ = 0;
    good_frames_ = 0;
  }

  FusionMode mode() const { return mode_; }
  int badFrames() const { return bad_frames_; }
  int goodFrames() const { return good_frames_; }

 private:
  FusionMode mode_ = FusionMode::NORMAL;
  int bad_frames_ = 0;
  int good_frames_ = 0;
  int bad_frames_to_degraded_;
  int good_frames_to_recover_;
};

}  // namespace fastlio

#endif  // FASTLIO_LOCALIZER_TUNNEL_FUSION_H_
