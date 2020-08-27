#include <algorithm>
#include <vector>
#include <stdexcept>
#include <cassert>

#include <gtest/gtest.h>

#include "dreal/symbolic/symbolic_expression.h"
#include "dreal/symbolic/symbolic_formula.h"
#include "dreal/symbolic/symbolic_variable.h"
#include "dreal/util/infty.h"  // For InftyStart()/InftyFinish()
#include "dreal/qsopt_ex.h"  // For QSXStart()/QSXFinish()

#if HAVE_SOPLEX
# include "dreal/soplex.h"  // For soplex::infinity
#endif

#define EXPECT_MPQ_EQ_DOUBLE(mpq, d) EXPECT_DOUBLE_EQ((mpq).get_d(), d)
#define EXPECT_MPQ_NEAR(mpq, d, tol) EXPECT_NEAR((mpq).get_d(), d, tol)

namespace dreal {
namespace drake {
namespace symbolic {
namespace test {

// Important: must be defined the same as dreal::Config::LPSolver in dreal/solver/config.h
// (We can't include that file here, as it leads to namespace clashes.)
enum LPSolver {
  SOPLEX = 0,
  QSOPTEX,
};

struct DrakeSymbolicGuard {
  int solver_;
  DrakeSymbolicGuard(int solver = QSOPTEX) : solver_{solver} {
    if (solver_ == QSOPTEX) {
      dreal::qsopt_ex::QSXStart();
      dreal::util::InftyStart(qsopt_ex::mpq_INFTY, qsopt_ex::mpq_NINFTY);
    } else {
      assert(solver_ == SOPLEX);
#if HAVE_SOPLEX
      dreal::util::InftyStart(soplex::infinity);
#else
      throw std::runtime_error("SoPlex not enabled at compile time");
#endif
    }
    Expression::InitConstants();
  }
  ~DrakeSymbolicGuard() {
    Expression::DeInitConstants();
    dreal::util::InftyFinish();
    if (solver_ == QSOPTEX) {
      dreal::qsopt_ex::QSXFinish();
    }
  }
};

inline bool VarEqual(const Variable& v1, const Variable& v2) {
  return v1.equal_to(v2);
}

inline bool VarNotEqual(const Variable& v1, const Variable& v2) {
  return !VarEqual(v1, v2);
}

inline bool VarLess(const Variable& v1, const Variable& v2) {
  return v1.less(v2);
}

inline bool VarNotLess(const Variable& v1, const Variable& v2) {
  return !VarLess(v1, v2);
}

inline bool ExprEqual(const Expression& e1, const Expression& e2) {
  return e1.EqualTo(e2);
}

inline bool ExprNotEqual(const Expression& e1, const Expression& e2) {
  return !ExprEqual(e1, e2);
}

inline bool ExprLess(const Expression& e1, const Expression& e2) {
  return e1.Less(e2);
}

inline bool ExprNotLess(const Expression& e1, const Expression& e2) {
  return !ExprLess(e1, e2);
}

template <typename F>
bool all_of(const std::vector<Formula>& formulas, const F& f) {
  return std::all_of(formulas.begin(), formulas.end(), f);
}

template <typename F>
bool any_of(const std::vector<Formula>& formulas, const F& f) {
  return std::any_of(formulas.begin(), formulas.end(), f);
}

inline bool FormulaEqual(const Formula& f1, const Formula& f2) {
  return f1.EqualTo(f2);
}

inline bool FormulaNotEqual(const Formula& f1, const Formula& f2) {
  return !FormulaEqual(f1, f2);
}

inline bool FormulaLess(const Formula& f1, const Formula& f2) {
  return f1.Less(f2);
}

inline bool FormulaNotLess(const Formula& f1, const Formula& f2) {
  return !FormulaLess(f1, f2);
}

}  // namespace test
}  // namespace symbolic
}  // namespace drake
}  // namespace dreal
