#ifndef VIO_H_
#define VIO_H_

#include <Eigen/Core>
#include <memory>
#include <sophus/se3.hpp>
#include <vikit/math_utils.h>
#include <vikit/pinhole_camera.h>

namespace vk {

using namespace Sophus;

/// Base class for visual-inertial odometry.
class VIOManager {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VIOManager(const PinholeCamera &cam);
  virtual ~VIOManager();

  /// Initialize the VIO system.
  virtual bool initialize() = 0;

  /// Process a new image.
  virtual void processImage(const cv::Mat &img, const double timestamp) = 0;

  /// Process IMU measurements.
  virtual void processIMU(const Eigen::Vector3d &acc,
                          const Eigen::Vector3d &gyr,
                          const double timestamp) = 0;

  /// Get the current pose estimate.
  virtual SE3<double> getCurrentPose() const = 0;

  /// Get the current velocity estimate.
  virtual Eigen::Vector3d getCurrentVelocity() const = 0;

  /// Get the current bias estimates.
  virtual void getCurrentBiases(Eigen::Vector3d &acc_bias,
                                Eigen::Vector3d &gyr_bias) const = 0;

  /// Reset the VIO system.
  virtual void reset() = 0;

protected:
  const PinholeCamera &cam_; //!< Camera model.
  SE3<double> T_w_i_;        //!< Transform from IMU to world frame.
  Eigen::Vector3d v_w_i_;    //!< Velocity of IMU in world frame.
  Eigen::Vector3d acc_bias_; //!< Accelerometer bias.
  Eigen::Vector3d gyr_bias_; //!< Gyroscope bias.
  bool initialized_;         //!< Whether the system is initialized.
};

} // namespace vk

#endif // VIO_H_