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

using qsopt_ex::mpq_QSprob;

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

bool LinearTheorySolver::CheckSat(const Box& box, const mpq_QSprob prob) {
  static TheorySolverStat stat{DREAL_LOG_INFO_ENABLED};
  stat.increase_num_check_sat();
  TimerGuard check_sat_timer_guard(&stat.timer_check_sat_, stat.enabled(),
                                   true /* start_timer */);

  DREAL_LOG_DEBUG("LinearTheorySolver::CheckSat: Box = \n{}", box);

  int status = -1;
  int lp_status = -1;

  precision_ = config_.precision();

  status = qsopt_ex::QSdelta_solver(prob, precision_.get_mpq_t(), NULL, NULL, NULL,
                                    DUAL_SIMPLEX, &lp_status);

  if (status) {
    throw DREAL_RUNTIME_ERROR("QSdelta_solver() returned {}", status);
  }

  switch (lp_status) {
   case QS_LP_FEASIBLE:
   case QS_LP_DELTA_FEASIBLE:
    return true;
   case QS_LP_INFEASIBLE:
    return false;
   case QS_LP_UNSOLVED:
    throw DREAL_RUNTIME_ERROR("QSdelta_solver() failed to solve LP");
  }

  DREAL_UNREACHABLE();
}

const Box& LinearTheorySolver::GetModel() const {
  DREAL_LOG_DEBUG("LinearTheorySolver::GetModel():\n{}", model_);
  return model_;
}

const set<Formula>& LinearTheorySolver::GetExplanation() const {
  return explanation_;
}

}  // namespace dreal
