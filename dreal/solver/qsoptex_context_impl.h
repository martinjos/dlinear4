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
  optional<Box> CheckSatCore(const ScopedVector<Formula>& stack, Box box, mpq_class* actual_precision);
  int CheckOptCore(const ScopedVector<Formula>& stack, mpq_class* obj_lo, mpq_class* obj_up, Box* box);

  void MinimizeCore(const Expression& obj_expr);

  QsoptexSatSolver sat_solver_;
  QsoptexTheorySolver theory_solver_;
  Expression obj_expr_;
};

}  // namespace dreal
