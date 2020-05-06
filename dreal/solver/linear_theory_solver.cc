#include "dreal/solver/linear_theory_solver.h"

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

using qsopt_ex::mpq_QSprob;
using qsopt_ex::MpqArray;
using qsopt_ex::mpq_infty;
using qsopt_ex::mpq_ninfty;

LinearTheorySolver::LinearTheorySolver(const Config& config)
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

bool LinearTheorySolver::LiteralComparator::operator()(const LinearTheorySolver::Literal& a,
                                                       const LinearTheorySolver::Literal& b) const {
  if (a.first.get_id() < b.first.get_id()) {
    return true;
  } else if (a.first.get_id() > b.first.get_id()) {
    return false;
  }
  return a.second < b.second;
}

int LinearTheorySolver::CheckSat(const Box& box,
                                 const std::vector<Literal>& assertions,
                                 const mpq_QSprob prob,
                                 const std::map<int, Variable>& var_map) {
  static TheorySolverStat stat{DREAL_LOG_INFO_ENABLED};
  stat.increase_num_check_sat();
  TimerGuard check_sat_timer_guard(&stat.timer_check_sat_, stat.enabled(),
                                   true /* start_timer */);

  DREAL_LOG_TRACE("LinearTheorySolver::CheckSat: Box = \n{}", box);

  int status = -1;
  int lp_status = -1;

  precision_ = config_.precision();

  int rowcount = mpq_QSget_rowcount(prob);
  int colcount = mpq_QSget_colcount(prob);
  // x: * must be allocated/deallocated using QSopt_ex.
  //    * should have room for the (rowcount) "logical" variables, which come
  //    after the (colcount) "structural" variables.
  MpqArray x{colcount + rowcount};

  model_ = Box();
  for (const pair<int, Variable>& kv : var_map) {
    model_.Add(kv.second);
  }
  DREAL_ASSERT(!config_.use_phase_one_simplex() || model_.size() == colcount);

  // The solver can't handle problems with inverted bounds, so we need to
  // handle that here.  Also, if there are no constraints, we can immediately
  // return SAT afterwards if the bounds are OK.
  lp_status = QS_EXACT_DELTA_SAT;
  mpq_t temp;
  mpq_init(temp);
  for (const pair<int, Variable>& kv : var_map) {
    int res;
    res = mpq_QSget_bound(prob, kv.first, 'L', &temp);
    DREAL_ASSERT(!res);
    mpq_class lb{temp};
    res = mpq_QSget_bound(prob, kv.first, 'U', &temp);
    DREAL_ASSERT(!res);
    mpq_class ub{temp};
    if (lb > ub) {
      lp_status = QS_EXACT_UNSAT;
      // Prevent the exact same LP from coming up again
      explanation_.clear();
      explanation_.insert(assertions.begin(), assertions.end());
      break;
    }
    if (rowcount == 0) {
      mpq_class val;
      if (mpq_ninfty() < lb) {
        val = lb;
      } else if (ub < mpq_infty()) {
        val = ub;
      } else {
        val = 0;
      }
      model_[kv.second] = val;
    }
  }
  mpq_clear(temp);
  if (lp_status == QS_EXACT_UNSAT || rowcount == 0) {
    DREAL_LOG_DEBUG("LinearTheorySolver::CheckSat: no need to call LP solver");
    return lp_status;
  }

  // Now we call the solver
  lp_status = -1;
  int sat_status = -1;
  DREAL_LOG_DEBUG("LinearTheorySolver::CheckSat: calling QSopt_ex (phase {})",
                  config_.use_phase_one_simplex() ? "one" : "two");

  mpq_class actual_precision{precision_};
  if (config_.use_phase_one_simplex()) {
    status = qsopt_ex::QSdelta_solver(prob, actual_precision.get_mpq_t(), x, NULL, NULL,
                                      DUAL_SIMPLEX, &lp_status);
  } else {
    status = qsopt_ex::QSexact_delta_solver(prob, x, NULL, NULL, DUAL_SIMPLEX,
                                            &sat_status, actual_precision.get_mpq_t());
  }

  if (status) {
    throw DREAL_RUNTIME_ERROR("QSopt_ex returned {}", status);
  } else {
    DREAL_LOG_DEBUG("LinearTheorySolver::CheckSat: QSopt_ex has returned with precision = {}",
                    actual_precision);
  }

  if (config_.use_phase_one_simplex()) {
    switch (lp_status) {
    case QS_LP_FEASIBLE:
    case QS_LP_DELTA_FEASIBLE:
      sat_status = QS_EXACT_DELTA_SAT;
      break;
    case QS_LP_INFEASIBLE:
      sat_status = QS_EXACT_UNSAT;
      break;
    case QS_LP_UNSOLVED:
      sat_status = QS_EXACT_UNKNOWN;
      break;
    default:
      DREAL_UNREACHABLE();
    }
  }

  switch (sat_status) {
  case QS_EXACT_SAT:
  case QS_EXACT_DELTA_SAT:
    // Copy delta-feasible point from x into model_
    for (const pair<int, Variable>& kv : var_map) {
      model_[kv.second] = x[kv.first];
    }
    sat_status = QS_EXACT_DELTA_SAT;
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

const Box& LinearTheorySolver::GetModel() const {
  DREAL_LOG_DEBUG("LinearTheorySolver::GetModel():\n{}", model_);
  return model_;
}

const LinearTheorySolver::LiteralSet& LinearTheorySolver::GetExplanation() const {
  return explanation_;
}

}  // namespace dreal
