/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExample.cpp
 * @brief   A structure-from-motion problem on a simulated dataset
 * @author  Duy-Nguyen Ta
 */

// 有关加载数据的信息，请参阅其中的注释（相机围绕立方体旋转）
#include "SFMdata.h"

// 路标的相机观测（即像素坐标）将被存储为Point2（x，y）。
#include <gtsam/geometry/Point2.h>

// 必须使用唯一键标识系统中的每个变量（位姿和路标）。
// 我们可以使用简单的整数key（1,2,3，...）或符号（X1，X2，L1）。 这里我们将使用Symbols
#include <gtsam/inference/Symbol.h>

// 在GTSAM中，测量函数表示为“因子”。 库已经提供了几个常见因子用于解决机器人/ SLAM /BA问题。
// 在这里，我们将使用投影因子来模拟相机的地标观测。 此外，我们将使用Prior因子在某个位置初始化机器人。
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

// 创建因子后，我们将它们添加到因子图。 由于我们使用的因素是非线性因素，我们需要一个非线性因子图。
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// 最后，一旦将所有因子添加到我们的因子图中，我们将需要对图进行求解/优化以找到最佳（最大后验）变量值集。
// GTSAM包括几个非线性优化器来执行此步骤。 在这里，我们将使用称为Powell的Degleg的信任区域方法.
#include <gtsam/nonlinear/DoglegOptimizer.h>

// GTSAM中的非线性求解器是迭代求解器，意味着它们围绕初始线性化点线性化非线性函数，然后求解线性系统以更新线性化点。
// 这种情况会重复发生，直到求解器收敛到一组一致的变量值。 这要求我们为每个变量指定一个初始猜测，保存在Values容器中。
#include <gtsam/nonlinear/Values.h>

#include <vector>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // 定义相机标定参数
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // 定义相机观测噪声模型
  noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // 创建一系列路标真值
  vector<Point3> points = createPoints();

  // 创建一系列位姿真值
  vector<Pose3> poses = createPoses();

  // 创建因子图
  NonlinearFactorGraph graph;

  // 在位姿x1上添加先验。 这间接指定了原点的位置。
  noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', 0), poses[0], poseNoise); // 直接加到图中


  // 每个摄像机的模拟测量结果，将它们添加到因子图中
  for (size_t i = 0; i < poses.size(); ++i) {
    SimpleCamera camera(poses[i], *K);
    for (size_t j = 0; j < points.size(); ++j) {
      Point2 measurement = camera.project(points[j]);
      graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
    }
  }

  // 因为运动结构问题具有尺度模糊性，所以问题仍然不足。 在这里，我们在第一个地标的位置上添加了先验。
  // 这通过指示第一相机和第一地标之间的距离来固定比例。 所有其他地标位置都使用此比例进行解释。
  noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
  graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', 0), points[0], pointNoise); // add directly to graph
  graph.print("Factor Graph:\n");

  // 创建Values结构以保存解决方案的初始估算值。 有目的的初始化变量
  Values initialEstimate;
  for (size_t i = 0; i < poses.size(); ++i)
    initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
  for (size_t j = 0; j < points.size(); ++j)
    initialEstimate.insert<Point3>(Symbol('l', j), points[j] + Point3(-0.25, 0.20, 0.15));
  initialEstimate.print("Initial Estimates:\n");

  /* 优化图，输出结果 */
  Values result = DoglegOptimizer(graph, initialEstimate).optimize();
  result.print("Final results:\n");
  cout << "initial error = " << graph.error(initialEstimate) << endl;
  cout << "final error = " << graph.error(result) << endl;

  return 0;
}