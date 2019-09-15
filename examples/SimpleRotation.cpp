/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SimpleRotation.cpp
 * @brief This is a super-simple example of optimizing a single rotation according to a single prior
 * @date Jul 1, 2010
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

  /**
   * This example will perform a relatively trivial(不重要的) optimization on
   * a single variable with a single factor.
   */

// In this example, a 2D rotation will be used as the variable of interest
// 在这个例子当中，2D旋转将用作待优化的变量
#include <gtsam/geometry/Rot2.h>

// 系统中的每个变量（位姿）需要使用唯一key标识。
// 我们可以使用简单的整数key（1,2,3，...）或符号（X1，X2，L1）。
// 这里我们将使用符号
#include <gtsam/inference/Symbol.h>

// 在GTSAM中，measurement functions表示为“因子”。 库中提供了几个常见factors用于解决机器人/ SLAM /BA问题。
// 我们将在旋转时应用简单的先验
#include <gtsam/slam/PriorFactor.h>

// 创建因子后，我们将它们添加到因子图。 由于我们使用的因子是非线性因子，我们需要一个非线性因子图。
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// GTSAM中的非线性求解器是迭代求解器，意味着它们围绕初始线性化点线性化非线性函数，
// 然后求解线性系统以更新线性化点。 这种情况会重复发生，直到求解器收敛到一组一致的变量值。
// 这要求我们为每个变量指定一个初始猜测，保存在Values容器中。
#include <gtsam/nonlinear/Values.h>

// 最后，一旦将所有因子添加到我们的因子图中，我们将需要求解/优化图以找到最佳（最大后验）变量值集。
// GTSAM包括几个非线性优化器来执行此步骤。 在这里，我们将使用标准的Levenberg-Marquardt求解器。
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


using namespace std;
using namespace gtsam;

// 弧度制角度
const double degree = M_PI / 180;

int main() {

  /** 第1步：创建一个表达一元约束的因子。 在这种情况下，“先验”是来自传感器的测量，其具有测量噪声。
   *  这里创建的“key”是用于将状态的部分（存储在“RotValues”中）与特定因子相关联的标签。
   *  它们需要一个索引来允许查找，并且应该是唯一的。 通常，创建一个因子需要：
   *  - 标记所作用变量的一个或一组键
   *  - 测量值
   *  - 具有正确的因子维数的测量模型
   */

  Rot2 prior = Rot2::fromAngle(30 * degree);
  prior.print("goal angle");
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(1, 1 * degree);
  Symbol key('x',1);
  PriorFactor<Rot2> factor(key, prior, model);

  /**
   * 第2步：创建graph容器并将因子添加到其中。 在优化之前，需要将所有因子添加到graph容器中，
   * 该容器为定义约束系统提供必要的top-level functionality。 在这个例子中，只有一个因子，
   * 但在实际情况中，会有很多因子。
   */
  NonlinearFactorGraph graph;
  graph.push_back(factor);
  graph.print("full graph");

  /**
   * 第3步：创建初始估算
   * 系统的初始值对于开始优化是必要的。该系统状态是“RotValues”结构，其结构类似于STL映射，
   * 因为它将key（在步骤1中创建的标签）映射到特定值。
   *
   * 提供给优化的初始估计将用作优化的线性化点（泰勒展开），因此图中的所有变量在此结构中具有相应的值非常重要。
   *
   * 所有RotValues类型的接口都是相同的，它只取决于key的类型。
   */
  Values initial;
  initial.insert(key, Rot2::fromAngle(20 * degree));
  initial.print("initial estimate. ");

  /**
   * 第4步：优化
   * 在使用约束图和初始估计来构建问题之后，
   * 执行优化就非常简单（通过调用优化函数并以图和初始估计为参数）。
   * 这将产生具有最优化最终状态的新RotValues结构。
   */
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("final result");

  return 0;
}
