/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CameraResectioning.cpp( Camera Resectioning和camera calibration一个意思)
 * @brief   An example of gtsam for solving the camera resectioning problem
 * @author  Duy-Nguyen Ta
 * @date    Aug 23, 2011
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <boost/make_shared.hpp>
#include <utility>

using namespace gtsam;
using namespace gtsam::noiseModel;
using symbol_shorthand::X;

/**
 * 通过测量图像中已知3D点的投影得到的未知位姿的一元因子
 */
class ResectioningFactor: public NoiseModelFactor1<Pose3> {    // 标定因子

    typedef NoiseModelFactor1<Pose3> Base;

    Cal3_S2::shared_ptr K_; ///< 相机内参
    Point3 P_;              ///< 标定板3D点 (标定板？)
    Point2 p_;              ///< 3D点投影到2D

public:

  /// 构建已知点P及其投影p的因子
  ResectioningFactor(const SharedNoiseModel& model, const Key& key,
                     Cal3_S2::shared_ptr calib, const Point2& p, const Point3& P) :
                     Base(model, key), K_(std::move(calib)), P_(P), p_(p) {}

  /// 评价误差
  Vector evaluateError(const Pose3& pose, boost::optional<Matrix&> H = boost::none) const override
  {
    SimpleCamera camera(pose, *K_);
    return camera.project(P_, H, boost::none, boost::none) - p_;
  }
};

/*******************************************************************************
 *
 * 相机焦距：f=1，图像大小：100×100，中心坐标：50×50
 * 真实位姿：(Xw, -Yw, -Zw, [0,0,2.0]')
 * 已知路标点：3D Points (10,10,0) (-10,10,0) (-10,-10,0) (10,-10,0)
 * 测量值（根据针孔模型计算得到）：2D Point  (55,45)   (45,45)    (45,55)     (55,55)
 *
 *******************************************************************************/
int main(int argc, char* argv[]) {
  /*  读入相机内参矩阵(五个参数) */
  Cal3_S2::shared_ptr calib(new Cal3_S2(1, 1, 0, 50, 50));

  /* 1. 创建图 */
  NonlinearFactorGraph graph;

  /* 2. 把因子加入到图中 */
  // 加入测量因子

  SharedDiagonal measurementNoise = Diagonal::Sigmas(Vector2(0.5, 0.5));
  boost::shared_ptr<ResectioningFactor> factor;
  graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
          Point2(55, 45), Point3(10, 10, 0));
  graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
          Point2(45, 45), Point3(-10, 10, 0));
  graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
          Point2(45, 55), Point3(-10, -10, 0));
  graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
          Point2(55, 55), Point3(10, -10, 0));

  /* ***************
   * 3. 创建相机位姿初始估计
   * T =
   * |1  0  0  0|
   * |0 -1  0  0|
   * |0  0 -1  2|
   * |0  0  0  1|
   *
   ************** */
  Values initial;
  initial.insert(X(1),
      Pose3(Rot3(1, 0,  0,
                       0, -1, 0,
                       0, 0,  -1),
                Point3(0,   0,    2)));

  /**
   * 4. 使用LM算法优化图
   **/
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("Final result:\n");

  return 0;
}
