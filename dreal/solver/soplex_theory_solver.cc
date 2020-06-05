#include "dreal/solver/soplex_theory_solver.h"

#include <atomic>
#include <iostream>

#include "dreal/util/assert.h"
#include "dreal/util/exception.h"
#include "dreal/util/logging.h"
#include "dreal/util/stat.h"
#include "dreal/util/timer.h"

namespace dreal {

using std::cout;
using std::set;
using std::vector;
using std::pair;

using soplex::SoPlex;
using soplex::SPxSolver;
using soplex::VectorRational;
using soplex::LPColRational;
using soplex::Rational;

using dreal::gmp::to_mpq_t;
using dreal::gmp::to_mpq_class;

SoplexTheorySolver::SoplexTheorySolver(const Config& config)
    : config_{config} {
}

namespace {
class TheorySolverStat : public Stat {
 public:
  explicit TheorySolverStat(const bool enabled) : Stat{enabled} {}
  TheorySolverStat(const TheorySolverStat&) = delete;
  TheorySolverStat(TheorySolverStat&&) = delete;
  TheorySolverStat& operator=(const TheorySolverStat&) = delete;
  TheorySolverStat& operator=(TheorySolverStat&&) = delete;
  ~TheorySolverStat() override {
    if (enabled()) {
      using fmt::print;
      print(cout, "{:<45} @ {:<20} = {:>15}\n", "Total # of CheckSat",
            "Theory level", num_check_sat_);
      print(cout, "{:<45} @ {:<20} = {:>15f} sec\n",
            "Total time spent in CheckSat", "Theory level",
            timer_check_sat_.seconds());
    }
  }

  void increase_num_check_sat() { increase(&num_check_sat_); }

  Timer timer_check_sat_;

 private:
  std::atomic<int> num_check_sat_{0};
};

}  // namespace

int SoplexTheorySolver::CheckSat(const Box& box,
                                 const std::vector<Literal>& assertions,
                                 SoPlex& prob,
                                 const std::map<int, Variable>& var_map) {
  static TheorySolverStat stat{DREAL_LOG_INFO_ENABLED};
  stat.increase_num_check_sat();
  TimerGuard check_sat_timer_guard(&stat.timer_check_sat_, stat.enabled(),
                                   true /* start_timer */);

  DREAL_LOG_TRACE("SoplexTheorySolver::CheckSat: Box = \n{}", box);

  SPxSolver::Status status = SPxSolver::Status::UNKNOWN;
  int lp_status = -1;

  precision_ = config_.precision();

  int rowcount = prob.numRowsRational();
  int colcount = prob.numColsRational();
  VectorRational x;

  model_ = box;
  for (const pair<int, Variable>& kv : var_map) {
    if (!model_.has_variable(kv.second)) {
      // Variable should already be present
      DREAL_LOG_WARN("SoplexTheorySolver::CheckSat: Adding var {} to model from SAT", kv.second);
      model_.Add(kv.second);
    }
  }

  // The solver can't handle problems with inverted bounds, so we need to
  // handle that here.  Also, if there are no constraints, we can immediately
  // return SAT afterwards if the bounds are OK.
  // FIXME: check whether SoPlex can handle all this; if so, remove it.
  lp_status = QS_EXACT_DELTA_SAT;
  for (const pair<int, Variable>& kv : var_map) {
    LPColRational col;
    prob.getColRational(kv.first, col);
    const Rational& lb{col.lower()};
    const Rational& ub{col.upper()};
    if (lb > ub) {
      lp_status = QS_EXACT_UNSAT;
      // Prevent the exact same LP from coming up again
      explanation_.clear();
      explanation_.insert(assertions.begin(), assertions.end());
      break;
    }
    if (rowcount == 0) {
      Rational val;
      if (-soplex::infinity < lb) {
        val = lb;
      } else if (ub < soplex::infinity) {
        val = ub;
      } else {
        val = 0;
      }
      DREAL_ASSERT(to_mpq_t(model_[kv.second].lb()) <= val &&
                   val <= to_mpq_t(model_[kv.second].ub()));
      model_[kv.second] = val.getMpqRef();
    }
  }
  if (lp_status == QS_EXACT_UNSAT || rowcount == 0) {
    DREAL_LOG_DEBUG("SoplexTheorySolver::CheckSat: no need to call LP solver");
    return lp_status;
  }

  // Now we call the solver
  int sat_status = -1;
  DREAL_LOG_DEBUG("SoplexTheorySolver::CheckSat: calling SoPlex (phase {})",
                  config_.use_phase_one_simplex() ? "one" : "two");

  mpq_class actual_precision{precision_};
  status = prob.optimize();
  actual_precision = 0;  // Because we always solve exactly, at present

  if ((!config_.use_phase_one_simplex() && status != SPxSolver::Status::OPTIMAL) ||
      (status != SPxSolver::Status::OPTIMAL &&
       status != SPxSolver::Status::UNBOUNDED &&
       status != SPxSolver::Status::INFEASIBLE)) {
    throw DREAL_RUNTIME_ERROR("SoPlex returned {}", status);
  } else {
    DREAL_LOG_DEBUG("SoplexTheorySolver::CheckSat: SoPlex has returned with precision = {}",
                    actual_precision);
  }

  bool haveSoln = prob.getPrimalRational(x);
  DREAL_ASSERT(!haveSoln || x.dim() == colcount);
  DREAL_ASSERT(status != SPxSolver::Status::OPTIMAL || haveSoln);

  if (config_.use_phase_one_simplex()) {
    switch (status) {
    case SPxSolver::Status::OPTIMAL:
    case SPxSolver::Status::UNBOUNDED:
      sat_status = QS_EXACT_DELTA_SAT;
      break;
    case SPxSolver::Status::INFEASIBLE:
      sat_status = QS_EXACT_UNSAT;
      break;
    //case QS_LP_UNSOLVED:
    //  sat_status = QS_EXACT_UNKNOWN;
    //  break;
    default:
      DREAL_UNREACHABLE();
    }
  } else {
    // The feasibility LP should always be feasible & bounded
    DREAL_ASSERT(status == SPxSolver::Status::OPTIMAL);
    VectorRational obj;
    prob.getObjRational(obj);
    DREAL_ASSERT(obj.dim() == colcount);
    bool ok = true;
    // ok = std::ranges::all_of(0, colcount, [&] (int i) { return obj[i] == 0 || x[i] == 0; });
    for (int i = 0; i < colcount; ++i) {
      if (!(ok = (obj[i] == 0 || x[i] == 0))) {
        break;
      }
    }
    if (ok) {
      sat_status = QS_EXACT_DELTA_SAT;
    } else {
      sat_status = QS_EXACT_UNSAT;
    }
  }

  if (sat_status == QS_EXACT_UNKNOWN) {
    DREAL_LOG_DEBUG("SoplexTheorySolver::CheckSat: SoPlex failed to return a result");
  }

  switch (sat_status) {
  case QS_EXACT_DELTA_SAT:
    if (haveSoln) {
    // Copy delta-feasible point from x into model_
      for (const pair<int, Variable>& kv : var_map) {
        DREAL_ASSERT(model_[kv.second].lb() <= to_mpq_class(x[kv.first].getMpqRef()) &&
                     to_mpq_class(x[kv.first].getMpqRef()) <= model_[kv.second].ub());
        model_[kv.second] = x[kv.first].getMpqRef();
      }
    } else {
      throw DREAL_RUNTIME_ERROR("delta-sat but no solution available");
    }
    break;
  case QS_EXACT_UNSAT:
  case QS_EXACT_UNKNOWN:
    // Prevent the exact same LP from coming up again
    explanation_.clear();
    explanation_.insert(assertions.begin(), assertions.end());
    break;
  default:
    DREAL_UNREACHABLE();
  }

  return sat_status;
}

const Box& SoplexTheorySolver::GetModel() const {
  DREAL_LOG_DEBUG("SoplexTheorySolver::GetModel():\n{}", model_);
  return model_;
}

const LiteralSet& SoplexTheorySolver::GetExplanation() const {
  return explanation_;
}

}  // namespace dreal
