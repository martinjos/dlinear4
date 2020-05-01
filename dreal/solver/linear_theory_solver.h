#pragma once

#include <set>
#include <vector>

#include "dreal/solver/config.h"
#include "dreal/symbolic/symbolic.h"
#include "dreal/util/box.h"
#include "dreal/qsopt_ex.h"
#include "dreal/gmp.h"

namespace dreal {

/// Theory solver for linear theory over the Reals.
class LinearTheorySolver {
 public:
  LinearTheorySolver() = delete;
  explicit LinearTheorySolver(const Config& config);

  /// Checks consistency. Returns true if there is a satisfying
  /// assignment. Otherwise, return false.
  bool CheckSat(const Box& box, const std::vector<Formula>& assertions,
                const qsopt_ex::mpq_QSprob prob,
                const std::vector<Variable>& var_map);

  /// Gets a satisfying Model.
  const Box& GetModel() const;

  /// Gets a list of used constraints.
  const std::set<Formula>& GetExplanation() const;

 private:
  const Config& config_;
  Box model_;
  std::set<Formula> explanation_;
  mpq_class precision_;
};

}  // namespace dreal
