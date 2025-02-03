#pragma once

#include "kimera_rpgo/solver/solver.h"

namespace kimera_rpgo {

class LeastSquaresSolver : public Solver {
 public:
  LeastSquaresSolver(const SolverConfig& config);
  ~LeastSquaresSolver();

 private:
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& factors,
                         const gtsam::Values& initial,
                         std::vector<double>& weights) override;

 private:
  SolverConfig config_;
};
}  // namespace kimera_rpgo
