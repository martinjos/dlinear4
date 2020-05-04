#include "dreal/util/martin_cnfizer.h"

#include <algorithm>
#include <atomic>
#include <iostream>
#include <iterator>
#include <set>
#include <string>

#include "dreal/util/assert.h"
#include "dreal/util/exception.h"
#include "dreal/util/logging.h"
#include "dreal/util/stat.h"
#include "dreal/util/timer.h"

namespace dreal {

using std::cout;
using std::set;
using std::string;
using std::to_string;
using std::vector;

namespace {
// A class to show statistics information at destruction.
class MartinCnfizerStat : public Stat {
 public:
  explicit MartinCnfizerStat(const bool enabled) : Stat{enabled} {}
  MartinCnfizerStat(const MartinCnfizerStat&) = delete;
  MartinCnfizerStat(MartinCnfizerStat&&) = delete;
  MartinCnfizerStat& operator=(const MartinCnfizerStat&) = delete;
  MartinCnfizerStat& operator=(MartinCnfizerStat&&) = delete;
  ~MartinCnfizerStat() override {
    if (enabled()) {
      using fmt::print;
      print(cout, "{:<45} @ {:<20} = {:>15}\n", "Total # of Convert",
            "Martin Cnfizer", num_convert_);
      if (num_convert_ > 0) {
        print(cout, "{:<45} @ {:<20} = {:>15f} sec\n",
              "Total time spent in Converting", "Martin Cnfizer",
              timer_convert_.seconds());
      }
    }
  }

  void increase_num_convert() { increase(&num_convert_); }

  Timer timer_convert_;

 private:
  std::atomic<int> num_convert_{0};
};
}  // namespace

vector<Formula> MartinCnfizer::Convert(const Formula& f) {
  static MartinCnfizerStat stat{DREAL_LOG_INFO_ENABLED};
  TimerGuard timer_guard(&stat.timer_convert_, stat.enabled());
  stat.increase_num_convert();
  // Put the Formula into negation normal form
  const Formula& g{nnfizer_.Convert(f, true /* push_negation_into_relationals */)};
  aux_.clear();
  vars_.clear();
  vector<Formula> ret;
  const Formula head{Visit(f)};
  aux_.push_back(head);
  return aux_;
}

Formula MartinCnfizer::Visit(const Formula& f) {
  // TODO(soonho): use cache.
  return VisitFormula<Formula>(this, f);
}

Formula MartinCnfizer::VisitFalse(const Formula& f) { return f; }
Formula MartinCnfizer::VisitTrue(const Formula& f) { return f; }
Formula MartinCnfizer::VisitVariable(const Formula& f) { return f; }
Formula MartinCnfizer::VisitEqualTo(const Formula& f) { return f; }
Formula MartinCnfizer::VisitNotEqualTo(const Formula& f) { return f; }
Formula MartinCnfizer::VisitGreaterThan(const Formula& f) { return f; }
Formula MartinCnfizer::VisitGreaterThanOrEqualTo(const Formula& f) {
  return f;
}
Formula MartinCnfizer::VisitLessThan(const Formula& f) { return f; }
Formula MartinCnfizer::VisitLessThanOrEqualTo(const Formula& f) { return f; }

Formula MartinCnfizer::VisitForall(const Formula& f) {
  // We always need a variable
  static size_t id{0};
  const Variable bvar{string("forall") + to_string(id++),
                      Variable::Type::BOOLEAN};
  vars_.push_back(bvar);

  // Given: f := ∀y. φ(x, y), this process CNFizes φ(x, y), pushes the
  // universal quantifier over conjunctions, and inserts ¬b:
  //
  //     = ∀y. (clause₁(x, y) ∧ ... ∧ clauseₙ(x, y))
  //     = (∀y. ¬b v clause₁(x, y)) ∧ ... ∧ (∀y. ¬b v clauseₙ(x, y))
  const Variables& quantified_variables{get_quantified_variables(f)};  // y
  const Formula& quantified_formula{get_quantified_formula(f)};  // φ(x, y)
  // clause₁(x, y) ∧ ... ∧ clauseₙ(x, y)
  const set<Formula> clauses{
      get_clauses(naive_cnfizer_.Convert(quantified_formula))};
  for (const Formula& clause : clauses) {
    DREAL_ASSERT(is_clause(clause));
    set<Formula> new_clause_set{get_operands(clause)};
    new_clause_set.insert(!bvar);
    Formula new_clause{make_disjunction(new_clause_set)};
    // Only the old clause's variables can intersect
    if (HaveIntersection(clause.GetFreeVariables(), quantified_variables)) {
      aux_.emplace_back(forall(quantified_variables, new_clause));
    } else {
      aux_.emplace_back(new_clause);
    }
  }

  return Formula{bvar};
}

// TODO: flatten nested conjunctions and disjunctions?

Formula MartinCnfizer::VisitConjunction(const Formula& f) {
  static size_t id{0};
  // Introduce a new Boolean variable, `bvar` for `f`.
  const Variable bvar{string("conj") + to_string(id++),
                      Variable::Type::BOOLEAN};
  vars_.push_back(bvar);
  for (const Formula& op : get_operands(f)) {
    aux_.emplace_back(!bvar || this->Visit(op));
  }
  return Formula{bvar};
}

Formula MartinCnfizer::VisitDisjunction(const Formula& f) {
  static size_t id{0};
  // Introduce a new Boolean variable, `bvar` for `f`.
  const Variable bvar{string("disj") + to_string(id++),
                      Variable::Type::BOOLEAN};
  vars_.push_back(bvar);
  set<Formula> clause{::dreal::map(
      get_operands(f),
      [this](const Formula& formula) { return this->Visit(formula); })};
  clause.insert(!bvar);
  aux_.emplace_back(make_disjunction(clause));
  return Formula{bvar};
}

Formula MartinCnfizer::VisitNegation(const Formula& f) {
  DREAL_ASSERT(is_atomic(get_operand(f)));
  return f;
}
}  // namespace dreal
