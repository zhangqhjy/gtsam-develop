/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LocalizationExample.cpp
 * @brief Simple robot localization example, with three "GPS-like" measurements
 * @author Frank Dellaert
 */

/**
 * A simple 2D pose slam example with "GPS" measurements
 *  - The robot moves forward 2 meter each iteration
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have "GPS-like" measurements implemented with a custom factor
 */

// 我们将使用Pose2变量（x，y，theta）来表示机器人位置
#include <gtsam/geometry/Pose2.h>

// 我们将使用简单的整数Key来推断机器人位姿。
#include <gtsam/inference/Key.h>

// 与在OdometryExample.cpp中一样，我们使用BetweenFactor来模拟测距测量。
#include <gtsam/slam/BetweenFactor.h>

// 我们将所有因子添加到非线性因子图中，因为我们的因子是非线性的。
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// GTSAM中的非线性求解器是迭代求解器，这意味着它们将非线性函数线性化为初始线性化点，
// 然后求解线性系统以更新线性化点。 这种情况会重复发生，直到求解器收敛到一组一致的变量值。
// 这要求我们为每个变量指定一个初始估计值，保存在Values容器中。
#include <gtsam/nonlinear/Values.h>

// 最后，一旦将所有因子添加到我们的因子图中，我们需要求解/优化图以找到最佳（最大后验）变量值集。
// GTSAM包括几个非线性优化器来执行此步骤。 在这里，我们将使用标准的Levenberg-Marquardt求解器
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// 一旦计算出优化值，我们还可以计算所需变量的边缘协方差
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

#include <gtsam/nonlinear/NonlinearFactor.h>

// 在我们开始这个例子之前，我们必须创建一个自定义的一元因子来实现“类似GPS”的功能。
// 由于标准GPS测量仅提供有关位置的信息，而不提供方向信息，因此我们无法使用简单的方法对此测量进行正确建模。
// 该因子将是一元因子，仅影响单个系统变量。它还将使用标准的高斯噪声模型。因此，我们将从NoiseModelFactor1中推导出我们的新因子。
class UnaryFactor: public NoiseModelFactor1<Pose2> {

  // 该因子将包含一个由（X，Y）位置组成的测量。 我们可以使用Point2，但在这里我们只使用two doubles
  double mx_, my_;

public:
  // 指向因子的智能指针的简写
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  // 构造函数需要变量Key、（X，Y）测量值和噪声模型
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

  virtual ~UnaryFactor() {}

  // 使用NoiseModelFactor1基类有两个必须重写的函数。
  // 第一个是'evaluateError'函数：
  // 此函数实现所需的测量函数，在提供的变量值处评估时返回错误向量。 如果需要，它还必须计算该测量函数的雅可比行列式。
  Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const
  {
    // GPS类测量的测量功能很简单：
    // error_x = pose.x - measurement.x
    // error_y = pose.y - measurement.y
    // 因此，雅各比是：
    // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
    // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
    if (H) (*H) = (Matrix(2,3) << 1.0,0.0,0.0, 0.0,1.0,0.0).finished();
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }

  // 第二个是'克隆'功能，允许复制因子。 在大多数情况下，使用默认复制构造函数的以下代码应该可以正常工作。
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

  // 此外，我们鼓励您使用单元测试您的自定义因子（如所有GTSAM因子），您需要equals和print,
  // 以满足Testable.h中定义的GTSAM_CONCEPT_TESTABLE_INST（T），但下面不需要这些。

}; // UnaryFactor


int main(int argc, char** argv) {

  // 1. 创建因子图容器并向其添加因子
  NonlinearFactorGraph graph;

  // 2a. 添加里程计因子
  // 为简单起见，我们将为每个里程计因子使用相同的噪声模型
  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  // 在连续位姿之间创建里程计因子（BetweenFactor）
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2.0, 0.0, 0.0), odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2.0, 0.0, 0.0), odometryNoise);

  // 2b. 添加“类GPS”测量
  // 我们将使用我们的自定义UnaryFactor。
  noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y
  graph.emplace_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise);
  graph.emplace_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise);
  graph.emplace_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise);
  graph.print("\nFactor Graph:\n"); // print

  // 3. 创建数据结构以将initialEstimate估计值保存到解决方案中
  // 出于讲解的目的，这些被故意设置为不正确的值
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
  initialEstimate.print("\nInitial Estimate:\n"); // print

  // 4. 使用Levenberg-MarquardtOptimizer进行优化。 优化一组可选的配置参数，控制收敛标准，
  // 要使用的线性系统求解器的类型以及优化期间显示的信息量。 这里我们将使用默认的参数集。有关完整参数集，请参阅文档。
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. 计算并打印所有变量的边缘协方差
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  return 0;
}
