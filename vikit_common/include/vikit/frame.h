#ifndef FRAME_H_
#define FRAME_H_

#include <boost/noncopyable.hpp>
#include <sophus/se3.hpp>
#include <vikit/math_utils.h>

namespace vk {

using namespace Sophus;

class Frame : boost::noncopyable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame(const cv::Mat &img, const double timestamp);
  ~Frame();

  /// Initialize a frame and its features from an image.
  void initFrame(const cv::Mat &img);

  /// Get the pose of the frame in world coordinates.
  SE3<double> getFramePose();

  /// Set the pose of the frame in world coordinates.
  void setFramePose(const SE3<double> &T_f_w);

  /// The unique id of the frame.
  static int frame_counter_; //!< Counts the number of created frames. Used to
                             //!< set the unique id.
  const int id_;             //!< Unique id of the frame.
  const cv::Mat img_;        //!< Image data.
  const double timestamp_;   //!< Timestamp of when the image was recorded.

protected:
  SE3<double> T_f_w_; //!< Transform from world to frame.
};

} // namespace vk

#endif // FRAME_H_