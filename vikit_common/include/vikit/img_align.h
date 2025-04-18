/*
 * img_align.h
 *
 *  Created on: Aug 22, 2012
 *      Author: cforster
 */

#ifndef IMG_ALIGN_H_
#define IMG_ALIGN_H_

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/StdVector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <stdint.h>
#include <vector>
#include <vikit/math_utils.h>
#include <vikit/nlls_solver.h>
#include <vikit/performance_monitor.h>
#include <vikit/pinhole_camera.h>

namespace vk {

using namespace std;
// Remove the using namespace Eigen to avoid ambiguity
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using namespace vk;
using namespace Sophus;

// Define Vector6d as it's not provided by Eigen
typedef Eigen::Matrix<double, 6, 1> Vector6d;

//! Forward Compositional Image Alignment
class ForwardCompositionalSE3 : public NLLSSolver<6, SE3<double>> {

protected:
  vector<vk::PinholeCamera> &cam_pyr_;
  vector<cv::Mat> &depth_pyr_;
  vector<cv::Mat> &img_pyr_;
  vector<cv::Mat> &tpl_pyr_;
  vector<cv::Mat> &img_pyr_dx_;
  vector<cv::Mat> &img_pyr_dy_;
  int level_;
  int n_levels_;
  PerformanceMonitor permon_;
  bool display_;
  bool log_;
  double res_thresh_;

  virtual double computeResiduals(const SE3<double> &model,
                                  bool linearize_system,
                                  bool compute_weight_scale = false);

  virtual int solve();

  virtual void update(const ModelType &old_model, ModelType &new_model);

  virtual void startIteration();

  virtual void finishIteration();

public:
  cv::Mat resimg_;

  ForwardCompositionalSE3(vector<PinholeCamera> &cam_pyr,
                          vector<cv::Mat> &depth_pyr, vector<cv::Mat> &img_pyr,
                          vector<cv::Mat> &tpl_pyr, vector<cv::Mat> &img_pyr_dx,
                          vector<cv::Mat> &img_pyr_dy, SE3<double> &init_model,
                          int n_levels, int n_iter = 50, float res_thresh = 0.2,
                          bool display = true,
                          Method method = LevenbergMarquardt, int test_id = 0);

  ForwardCompositionalSE3(vector<PinholeCamera> &cam_pyr,
                          vector<cv::Mat> &depth_pyr, vector<cv::Mat> &img_pyr,
                          vector<cv::Mat> &tpl_pyr, vector<cv::Mat> &img_pyr_dx,
                          vector<cv::Mat> &img_pyr_dy, int n_levels,
                          int n_iter = 50, float res_thresh = 0.2,
                          bool display = true,
                          Method method = LevenbergMarquardt, int test_id = 0);

  void runOptimization(SE3<double> &model, int levelBegin = -1,
                       int levelEnd = -1);
};

//! Efficient Second Order Minimization (ESM)
class SecondOrderMinimisationSE3 : public NLLSSolver<6, SE3<double>> {

protected:
  vector<vk::PinholeCamera> &cam_pyr_;
  vector<cv::Mat> &depth_pyr_;
  vector<cv::Mat> &img_pyr_;
  vector<cv::Mat> &tpl_pyr_;
  vector<cv::Mat> &img_pyr_dx_;
  vector<cv::Mat> &img_pyr_dy_;
  vector<cv::Mat> &tpl_pyr_dx_;
  vector<cv::Mat> &tpl_pyr_dy_;
  int level_;
  PerformanceMonitor permon_;
  bool display_;
  bool log_;
  float res_thresh_;

  virtual double computeResiduals(const SE3<double> &model,
                                  bool linearize_system,
                                  bool compute_weight_scale = false);

  virtual int solve();

  virtual void update(const ModelType &old_model, ModelType &new_model);

  virtual void startIteration();

  virtual void finishIteration();

public:
  cv::Mat resimg_;

  SecondOrderMinimisationSE3(
      vector<PinholeCamera> &cam_pyr, vector<cv::Mat> &depth_pyr,
      vector<cv::Mat> &img_pyr, vector<cv::Mat> &tpl_pyr,
      vector<cv::Mat> &img_pyr_dx, vector<cv::Mat> &img_pyr_dy,
      vector<cv::Mat> &tpl_pyr_dx, vector<cv::Mat> &tpl_pyr_dy,
      SE3<double> &init_model, int n_levels, int n_iter = 50,
      float res_thresh = 0.2, bool display = true,
      Method method = LevenbergMarquardt, int test_id = -1);
};

} // namespace vk

#endif /* IMG_ALIGN_H_ */
