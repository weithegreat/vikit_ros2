#ifndef FEATURE_H_
#define FEATURE_H_

#include <Eigen/Core>
#include <boost/noncopyable.hpp>
#include <sophus/se3.hpp>

namespace vk {

using namespace Sophus;

/// A feature that has been tracked over multiple frames.
class Feature : boost::noncopyable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Feature(const Eigen::Vector2d &px);
  ~Feature();

  /// Get the 3D position of the feature in world coordinates.
  Eigen::Vector3d getWorldPos();

  /// Set the 3D position of the feature in world coordinates.
  void setWorldPos(const Eigen::Vector3d &pos);

  /// Get the pose of the feature in world coordinates.
  SE3<double> getFeaturePose();

  /// Set the pose of the feature in world coordinates.
  void setFeaturePose(const SE3<double> &T_f_w);

  /// The unique id of the feature.
  static int feature_counter_; //!< Counts the number of created features. Used
                               //!< to set the unique id.
  const int id_;               //!< Unique id of the feature.
  const Eigen::Vector2d px_;   //!< Pixel coordinates in the first frame.

protected:
  SE3<double> T_f_w_;     //!< Transform from world to feature.
  Eigen::Vector3d pos_w_; //!< 3D position in world coordinates.
};

} // namespace vk

#endif // FEATURE_H_