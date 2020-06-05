#pragma once

#include <set>
#include <vector>
#include <map>
#include <functional>
#include <utility>

#include "dreal/solver/config.h"
#include "dreal/symbolic/symbolic.h"
#include "dreal/util/box.h"
#include "dreal/util/literal.h"
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
  int CheckSat(const Box& box, const std::vector<Literal>& assertions,
               const qsopt_ex::mpq_QSprob prob,
               const std::map<int, Variable>& var_map);

  /// Gets a satisfying Model.
  const Box& GetModel() const;

  /// Gets a list of used constraints.
  const LiteralSet& GetExplanation() const;

 private:
  const Config& config_;
  Box model_;
  LiteralSet explanation_;
  mpq_class precision_;
};

}  // namespace dreal
