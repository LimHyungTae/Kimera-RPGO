#include "kimera_rpgo/solver/least_squares_solver.h"

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "kimera_rpgo/utils/utils.h"
namespace kimera_rpgo {
using gtsam::DoglegParams;
using gtsam::GaussNewtonParams;
using gtsam::LevenbergMarquardtParams;
using gtsam::NoiseModelFactor;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;

LeastSquaresSolver::LeastSquaresSolver(const SolverConfig& config)
    : Solver(config), config_(config) {}

LeastSquaresSolver::~LeastSquaresSolver() {}

gtsam::Values LeastSquaresSolver::optimize(const NonlinearFactorGraph& factors,
                                           const Values& initial,
                                           std::vector<double>& weights) {
  std::cout << "Least squares optimize\n";
  const auto& verbosity = static_cast<OptimizerParams::Verbosity>(config_.verbosity);
  switch (config_.least_squares_option) {
    case SolverConfig::LeastSquaresOption::GN: {
      auto gn_params =
          std::dynamic_pointer_cast<GaussNewtonParams>(config_.optimizer_params);
      if (!gn_params) {
        throw std::runtime_error("Optimizer option and param mismatch");
      }
      gn_params->verbosity = verbosity;
      gtsam::GaussNewtonOptimizer optimizer(factors, initial, *gn_params);
      auto result = optimizer.optimize();
      weights.resize(factors.size());
      for (size_t i = 0; i < factors.size(); i++) {
        // TODO(Yun) handle factors not derived from NoiseModelFactor
        auto factor = factor_pointer_cast<NoiseModelFactor>(factors[i]);
        weights[i] = factor->weight(result);
      }
      return result;
    }
    case SolverConfig::LeastSquaresOption::LM: {
      auto lm_params =
          std::dynamic_pointer_cast<LevenbergMarquardtParams>(config_.optimizer_params);
      if (!lm_params) {
        throw std::runtime_error("Optimizer option and param mismatch");
      }
      lm_params->verbosity = verbosity;
      lm_params->verbosityLM =
          static_cast<LevenbergMarquardtParams::VerbosityLM>(config_.verbosity);
      gtsam::LevenbergMarquardtOptimizer optimizer(factors, initial, *lm_params);
      auto result = optimizer.optimize();
      weights.resize(factors.size());
      for (size_t i = 0; i < factors.size(); i++) {
        // TODO(Yun) handle factors not derived from NoiseModelFactor
        auto factor = factor_pointer_cast<NoiseModelFactor>(factors[i]);
        weights[i] = factor->weight(result);
      }
      return result;
    }
    case SolverConfig::LeastSquaresOption::DOGLEG: {
      auto dogleg_params =
          std::dynamic_pointer_cast<DoglegParams>(config_.optimizer_params);
      if (!dogleg_params) {
        throw std::runtime_error("Optimizer option and param mismatch");
      }
      dogleg_params->verbosity = verbosity;
      dogleg_params->verbosityDL =
          static_cast<DoglegParams::VerbosityDL>(config_.verbosity);
      gtsam::DoglegOptimizer optimizer(factors, initial, *dogleg_params);
      auto result = optimizer.optimize();
      weights.resize(factors.size());
      for (size_t i = 0; i < factors.size(); i++) {
        // TODO(Yun) handle factors not derived from NoiseModelFactor
        auto factor = factor_pointer_cast<NoiseModelFactor>(factors[i]);
        weights[i] = factor->weight(result);
      }
      return result;
    }
    default:
      throw std::invalid_argument("Unexpected Least Squares option");
      return gtsam::Values();
  }
}
}  // namespace kimera_rpgo
