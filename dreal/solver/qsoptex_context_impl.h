#pragma once

#include "dreal/solver/context_impl.h"
#include "dreal/solver/qsoptex_sat_solver.h"
#include "dreal/solver/qsoptex_theory_solver.h"

namespace dreal {

// The actual implementation.
class Context::QsoptexImpl : public Context::Impl {
 public:
  QsoptexImpl();
  explicit QsoptexImpl(Config config);

  void Assert(const Formula& f);
  void Pop();
  void Push();

 protected:
  // Returns the current box in the stack.
  optional<Box> CheckSatCore(const ScopedVector<Formula>& stack, Box box);

  QsoptexSatSolver sat_solver_;
  QsoptexTheorySolver theory_solver_;
};

}  // namespace dreal
