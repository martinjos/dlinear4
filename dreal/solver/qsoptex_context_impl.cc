#include "dreal/solver/qsoptex_context_impl.h"

#include <vector>
#include <utility>

#include <fmt/format.h>

//#include "dreal/solver/filter_assertion.h"
#include "dreal/util/assert.h"
#include "dreal/util/exception.h"
#include "dreal/util/if_then_else_eliminator.h"
#include "dreal/util/interrupt.h"
#include "dreal/util/logging.h"
#include "dreal/util/literal.h"

namespace dreal {

using std::pair;
using std::vector;

Context::QsoptexImpl::QsoptexImpl() : Context::QsoptexImpl{Config{}} {}

Context::QsoptexImpl::QsoptexImpl(Config config)
    : Context::Impl{config}, sat_solver_{config_}, theory_solver_{config_} {}

void Context::QsoptexImpl::Assert(const Formula& f) {
  if (is_true(f)) {
    return;
  }
  if (box().empty()) {
    return;
  }
  if (is_false(f)) {
    box().set_empty();
    return;
  }
  //if (FilterAssertion(f, &box()) == FilterAssertionResult::NotFiltered) {
  DREAL_LOG_DEBUG("Context::QsoptexImpl::Assert: {} is added.", f);
  IfThenElseEliminator ite_eliminator;
  const Formula no_ite{ite_eliminator.Process(f)};
  for (const Variable& ite_var : ite_eliminator.variables()) {
    // Note that the following does not mark `ite_var` as a model variable.
    AddToBox(ite_var);
  }
  stack_.push_back(no_ite);
  sat_solver_.AddFormula(no_ite);
  return;
#if 0
  } else {
    DREAL_LOG_DEBUG("Context::QsoptexImpl::Assert: {} is not added.", f);
    DREAL_LOG_DEBUG("Box=\n{}", box());
    return;
  }
#endif
}  // namespace dreal

optional<Box> Context::QsoptexImpl::CheckSatCore(const ScopedVector<Formula>& stack,
                                          Box box) {
  DREAL_LOG_DEBUG("Context::QsoptexImpl::CheckSatCore()");
  DREAL_LOG_TRACE("Context::QsoptexImpl::CheckSat: Box =\n{}", box);
  if (box.empty()) {
    return {};
  }
  // If false ∈ stack, it's UNSAT.
  for (const auto& f : stack.get_vector()) {
    if (is_false(f)) {
      return {};
    }
  }
  // If stack = ∅ or stack = {true}, it's trivially SAT.
  if (stack.empty() || (stack.size() == 1 && is_true(stack.first()))) {
    DREAL_LOG_DEBUG("Context::QsoptexImpl::CheckSatCore() - Found Model\n{}", box);
    return box;
  }
  bool have_unsolved = false;
  while (true) {
    // Note that 'DREAL_CHECK_INTERRUPT' is only defined in setup.py,
    // when we build dReal python package.
#ifdef DREAL_CHECK_INTERRUPT
    if (g_interrupted) {
      DREAL_LOG_DEBUG("KeyboardInterrupt(SIGINT) Detected.");
      throw std::runtime_error("KeyboardInterrupt(SIGINT) Detected.");
    }
#endif

    // The box is passed in to the SAT solver solely to provide the LP solver
    // with initial bounds on the numerical variables.
    const auto optional_model = sat_solver_.CheckSat(box);
    if (optional_model) {
      const vector<pair<Variable, bool>>& boolean_model{optional_model->first};
      for (const pair<Variable, bool>& p : boolean_model) {
        // Here, we modify Boolean variables only (not used by the LP solver).
        box[p.first] = p.second ? 1 : 0;  // true -> 1 and false -> 0
      }
      const vector<pair<Variable, bool>>& theory_model{optional_model->second};
      if (!theory_model.empty()) {
        // SAT from SATSolver.
        DREAL_LOG_DEBUG("Context::QsoptexImpl::CheckSatCore() - Sat Check = SAT");

        // The selected assertions have already been enabled in the LP solver
        int theory_result{
          theory_solver_.CheckSat(box, theory_model,
                                  sat_solver_.GetLinearSolver(),
                                  sat_solver_.GetLinearVarMap())};
        if (theory_result == QS_EXACT_DELTA_SAT) {
          // SAT from TheorySolver.
          DREAL_LOG_DEBUG(
              "Context::QsoptexImpl::CheckSatCore() - Theory Check = delta-SAT");
          Box model{theory_solver_.GetModel()};
          return model;
        } else {
          if (theory_result == QS_EXACT_UNSAT) {
            // UNSAT from TheorySolver.
            DREAL_LOG_DEBUG("Context::QsoptexImpl::CheckSatCore() - Theory Check = UNSAT");
          } else {
            DREAL_ASSERT(theory_result == QS_EXACT_UNKNOWN);
            DREAL_LOG_DEBUG("Context::QsoptexImpl::CheckSatCore() - Theory Check = UNKNOWN");
            have_unsolved = true;  // Will prevent return of UNSAT
          }
          const LiteralSet& explanation{theory_solver_.GetExplanation()};
          DREAL_LOG_DEBUG(
              "Context::QsoptexImpl::CheckSatCore() - size of explanation = {} - stack "
              "size = {}",
              explanation.size(), stack.get_vector().size());
          sat_solver_.AddLearnedClause(explanation);
        }
      } else {
        return box;
      }
    } else {
      if (have_unsolved) {
        // Can't assert UNSAT, because some branches were unsolved.
        DREAL_LOG_DEBUG("Context::QsoptexImpl::CheckSatCore() - Sat Check = UNKNOWN");
        throw DREAL_RUNTIME_ERROR("LP solver failed to solve some instances");
      }
      // UNSAT from SATSolver. Escape the loop.
      DREAL_LOG_DEBUG("Context::QsoptexImpl::CheckSatCore() - Sat Check = UNSAT");
      return {};
    }
  }
}

void Context::QsoptexImpl::Pop() {
  DREAL_LOG_DEBUG("Context::QsoptexImpl::Pop()");
  stack_.pop();
  boxes_.pop();
  sat_solver_.Pop();
}

void Context::QsoptexImpl::Push() {
  DREAL_LOG_DEBUG("Context::QsoptexImpl::Push()");
  sat_solver_.Push();
  boxes_.push();
  boxes_.push_back(boxes_.last());
  stack_.push();
}

}  // namespace dreal
