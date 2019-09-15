/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample.cpp
 * @brief A 2D Pose SLAM example
 * @date Oct 21, 2010
 * @author Yong Dian Jian
 */

/**
 * A simple 2D pose slam example
 *  - The robot moves in a 2 meter square
 *  - The robot moves 2 meters each step, turning 90 degrees after each step
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have a loop closure constraint when the robot returns to the first position
 */

// 在平面SLAM示例中，我们使用Pose2变量（x，y，theta）来表示机器人位姿
#include <gtsam/geometry/Pose2.h>

// 我们将使用简单的整数Key来推测机器人位姿                  。
#include <gtsam/inference/Key.h>

// 在GTSAM中，测量函数表示为“因子”。 库中已经提供了几个常见的因子来解决机器人/SLAM/BA问题。
// 在这里，我们将使用BetweenFactor里程计测量信息表示相对运动。
// 我们还将使用BetweenFactor来编码循环闭约束。另外，我们将使用Prior因子在原点初始化机器人。
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// 创建因子后，我们将它们添加到因子图。由于我们使用的因子是非线性因子，我们需要一个非线性因子图。
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// 最后，一旦将所有因子添加到我们的因子图中，我们将需要求解/优化以绘制图以找到最佳（最大后验）变量值集。
// GTSAM包括几个非线性优化器来执行此步骤。这里我们将使用Gauss-Newton求解器
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// 一旦计算出优化值，我们还可以计算所需变量的边缘协方差
#include <gtsam/nonlinear/Marginals.h>

// GTSAM中的非线性求解器是迭代求解器，这意味着它们将非线性函数线性化为初始线性化点，
// 然后求解线性系统以更新线性化点。 这种情况会重复发生，直到求解器收敛到一组一致的变量值。
// 这要求我们为每个变量指定一个初始猜测，保存在Values容器中。
#include <gtsam/nonlinear/Values.h>


using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

  // 1. 创建因子图容器并向其添加因子
  NonlinearFactorGraph graph;

  // 2a. 在第一个位姿上添加先验，将其设置为原点
  // 先验因素包括均值和噪声模型（协方差矩阵）
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.emplace_shared<PriorFactor<Pose2> >(1, Pose2(0, 0, 0), priorNoise);

  // 为简单起见，我们将使用相同的噪声模型进行测距和闭环
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  // 2b. 添加里程计因子
  // 在连续位姿之间创建里程计（两者之间）因子
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2, 0, 0     ), model);
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<BetweenFactor<Pose2> >(3, 4, Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<BetweenFactor<Pose2> >(4, 5, Pose2(2, 0, M_PI_2), model);

  // 2c. 添加闭环约束
  // 这个因子编码了回到同一位姿的情况。 在实际系统中，可以以许多方式识别这些约束，
  // 例如具有相机图像的基于外观的技术。 我们将使用另一个Between Factor来强制执行此约束：
  graph.emplace_shared<BetweenFactor<Pose2> >(5, 2, Pose2(2, 0, M_PI_2), model);
  graph.print("\nFactor Graph:\n");

  // 3. 创建数据结构以将initialEstimate估计值保存到解决方案中
  // 出于说明的目的，这些被故意设置为不正确的值
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0,  0.2   ));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2   ));
  initialEstimate.insert(3, Pose2(4.1, 0.1,  M_PI_2));
  initialEstimate.insert(4, Pose2(4.0, 2.0,  M_PI  ));
  initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));
  initialEstimate.print("\nInitial Estimate:\n"); // print

  // 4. 使用Gauss-Newton非线性优化器优化初始值
  // 优化器接受一组可选的配置参数，控制收敛标准，
  // 要使用的线性系统求解器的类型以及优化期间显示的信息量。我们将设置一些参数作为演示。
  GaussNewtonParams parameters;
  // 一旦步骤之间的误差变化小于此值，就停止迭代
  parameters.relativeErrorTol = 1e-5;
  // 执行不超过N次迭代步骤
  parameters.maxIterations = 100;
  // 创建优化器......
  GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
  // 优化
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. 计算并打印所有变量的边缘协方差
  cout.precision(3);
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
  cout << "x4 covariance:\n" << marginals.marginalCovariance(4) << endl;
  cout << "x5 covariance:\n" << marginals.marginalCovariance(5) << endl;

  return 0;
}
