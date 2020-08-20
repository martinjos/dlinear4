#pragma once

#include "dreal/solver/context_impl.h"
#include "dreal/solver/soplex_sat_solver.h"
#include "dreal/solver/soplex_theory_solver.h"

namespace dreal {

// The actual implementation.
class Context::SoplexImpl : public Context::Impl {
 public:
  SoplexImpl();
  explicit SoplexImpl(Config config);

  void Assert(const Formula& f);
  void Pop();
  void Push();

 protected:
  // Returns the current box in the stack.
  optional<Box> CheckSatCore(const ScopedVector<Formula>& stack, Box box);

  void MinimizeCore(const Expression& obj_expr);

  SoplexSatSolver sat_solver_;
  SoplexTheorySolver theory_solver_;
};

}  // namespace dreal
