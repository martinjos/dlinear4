#include "dreal/solver/soplex_sat_solver.h"

#include <ostream>
#include <utility>
#include <cmath>

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
using std::make_pair;
using std::abs;

using soplex::Rational;
using soplex::DSVectorRational;
using soplex::LPRowRational;
using soplex::LPColRational;

using dreal::gmp::to_mpq_t;

SoplexSatSolver::SoplexSatSolver(const Config& config) : sat_{picosat_init()},
    cur_clause_start_{0}, config_(config) {
  // Enable partial checks via picosat_deref_partial. See the call-site in
  // SoplexSatSolver::CheckSat().
  picosat_save_original_clauses(sat_);
  if (config.random_seed() != 0) {
    picosat_set_seed(sat_, config.random_seed());
    DREAL_LOG_DEBUG("SoplexSatSolver::Set Random Seed {}", config.random_seed());
  }
  picosat_set_global_default_phase(
      sat_, static_cast<int>(config.sat_default_phase()));
  DREAL_LOG_DEBUG("SoplexSatSolver::Set Default Phase {}",
                  config.sat_default_phase());
  spx_prob_.setRealParam(spx_prob_.FEASTOL, 0);
  spx_prob_.setRealParam(spx_prob_.OPTTOL, 0);
  spx_prob_.setBoolParam(spx_prob_.RATREC, false);
  spx_prob_.setIntParam(spx_prob_.READMODE, spx_prob_.READMODE_RATIONAL);
  spx_prob_.setIntParam(spx_prob_.SOLVEMODE, spx_prob_.SOLVEMODE_RATIONAL);
  spx_prob_.setIntParam(spx_prob_.CHECKMODE, spx_prob_.CHECKMODE_RATIONAL);
  spx_prob_.setIntParam(spx_prob_.SYNCMODE, spx_prob_.SYNCMODE_AUTO);
  spx_prob_.setIntParam(spx_prob_.VERBOSITY, config_.verbose_simplex());
  // Default is maximize.
  spx_prob_.setIntParam(spx_prob_.OBJSENSE, spx_prob_.OBJSENSE_MINIMIZE);
}

SoplexSatSolver::SoplexSatSolver(const Config& config, const vector<Formula>& clauses)
    : SoplexSatSolver{config} {
  AddClauses(clauses);
}

SoplexSatSolver::~SoplexSatSolver() {
  picosat_reset(sat_);
}

void SoplexSatSolver::AddFormula(const Formula& f) {
  DREAL_LOG_DEBUG("SoplexSatSolver::AddFormula({})", f);
  vector<Formula> clauses{cnfizer_.Convert(f)};
  // Collect CNF variables.
  for (const auto& p : cnfizer_.vars()) {
    cnf_variables_.insert(p.get_id());
  }
  for (Formula& clause : clauses) {
    clause = predicate_abstractor_.Convert(clause);
  }
  AddClauses(clauses);
}

void SoplexSatSolver::AddFormulas(const vector<Formula>& formulas) {
  for (const Formula& f : formulas) {
    AddFormula(f);
  }
}

void SoplexSatSolver::AddLearnedClause(const LiteralSet& literals) {
  for (const Literal& l : literals) {
      AddLiteral(make_pair(l.first, !(l.second)), true);
  }
  picosat_add(sat_, 0);
}

void SoplexSatSolver::AddClauses(const vector<Formula>& formulas) {
  for (const Formula& f : formulas) {
    AddClause(f);
  }
}

void SoplexSatSolver::AddClause(const Formula& f) {
  DREAL_LOG_DEBUG("SoplexSatSolver::AddClause({})", f);
  // Set up Variable ⇔ Literal (in SAT) map.
  for (const Variable& var : f.GetFreeVariables()) {
    MakeSatVar(var);
  }
  // Add clauses to SAT solver.
  DoAddClause(f);
}

namespace {
class SoplexSatSolverStat : public Stat {
 public:
  explicit SoplexSatSolverStat(const bool enabled) : Stat{enabled} {};
  SoplexSatSolverStat(const SoplexSatSolverStat&) = default;
  SoplexSatSolverStat(SoplexSatSolverStat&&) = default;
  SoplexSatSolverStat& operator=(const SoplexSatSolverStat&) = delete;
  SoplexSatSolverStat& operator=(SoplexSatSolverStat&&) = delete;
  ~SoplexSatSolverStat() override {
    if (enabled()) {
      using fmt::print;
      print(cout, "{:<45} @ {:<20} = {:>15}\n", "Total # of CheckSat",
            "SAT level", num_check_sat_);
      print(cout, "{:<45} @ {:<20} = {:>15f} sec\n",
            "Total time spent in SAT checks", "SAT level",
            timer_check_sat_.seconds());
    }
  }

  int num_check_sat_{0};
  Timer timer_check_sat_;
};
}  // namespace

// Collect active literals, removing those that are only required by learned
// clauses.
set<int> SoplexSatSolver::GetMainActiveLiterals() const {
    set<int> lits;
    for (int i = 1; i <= picosat_variables(sat_); ++i) {
      const int model_i{has_picosat_pop_used_ ? picosat_deref(sat_, i)
                                              : picosat_deref_partial(sat_, i)};
      if (model_i == 0) {
        continue;
      }
      lits.insert(model_i * i);
    }
    for (auto it = lits.begin(); it != lits.end(); ) {
      int i = *it;
      int required = false;
      // Determine whether literal `i' is required
      auto c_it = main_clause_lookup_.find(i);
      if (c_it != main_clause_lookup_.end()) {
        for (int c : c_it->second) {
          int count = 0;
          size_t j;
          for (j = c; j < main_clauses_copy_.size() &&
                          main_clauses_copy_[j]; ++j) {
            int k{main_clauses_copy_[j]};
            if (lits.find(k) != lits.end()) {
              ++count;
            }
          }
          DREAL_ASSERT(j < main_clauses_copy_.size());  // Detect buffer overrun
          DREAL_ASSERT(count > 0);  // Should contain at least `i'
          if (count == 1) {
            // `i' is the only active literal in clause `c'; hence, required.
            required = true;
            break;
          }
        }
      }
      if (!required) {
        // There is more than one literal in every main (non-learned) clause
        // containing literal `i'.  Hence, it is not required.
        it = lits.erase(it);
      } else {
        ++it;
      }
    }
    return lits;
}

optional<SoplexSatSolver::Model> SoplexSatSolver::CheckSat(const Box& box) {
  static SoplexSatSolverStat stat{DREAL_LOG_INFO_ENABLED};
  DREAL_LOG_DEBUG("SoplexSatSolver::CheckSat(#vars = {}, #clauses = {})",
                  picosat_variables(sat_),
                  picosat_added_original_clauses(sat_));
  stat.num_check_sat_++;
  // Call SAT solver.
  TimerGuard check_sat_timer_guard(&stat.timer_check_sat_,
                                   DREAL_LOG_INFO_ENABLED);
  const int ret{picosat_sat(sat_, -1)};
  check_sat_timer_guard.pause();

  Model model;
  if (ret == PICOSAT_SATISFIABLE) {
    // SAT Case.
    set<int> lits{GetMainActiveLiterals()};
    ResetLinearProblem(box);
    const auto& var_to_formula_map = predicate_abstractor_.var_to_formula_map();
    for (int i : lits) {
      const auto it_var = to_sym_var_.find(abs(i));
      if (it_var == to_sym_var_.end()) {
        // There is no symbolic::Variable corresponding to this
        // picosat variable (int). This could be because of
        // picosat_push/pop.
        continue;
      }
      const Variable& var{it_var->second};
      const auto it = var_to_formula_map.find(var);
      if (it != var_to_formula_map.end()) {
        DREAL_LOG_TRACE("SoplexSatSolver::CheckSat: Add theory literal {}{} to Model",
                        i > 0 ? "" : "¬", var);
        auto& theory_model = model.second;
        theory_model.emplace_back(var, i > 0);
        EnableLinearLiteral(var, i > 0);
      } else if (cnf_variables_.count(var.get_id()) == 0) {
        DREAL_LOG_TRACE(
            "SoplexSatSolver::CheckSat: Add Boolean literal {}{} to Model ",
            i > 0 ? "" : "¬", var);
        auto& boolean_model = model.first;
        boolean_model.emplace_back(var, i > 0);
      } else {
        DREAL_LOG_TRACE(
            "SoplexSatSolver::CheckSat: Skip {}{} which is a temporary variable.",
            i > 0 ? "" : "¬", var);
      }
    }
    DREAL_LOG_DEBUG("SoplexSatSolver::CheckSat() Found a model.");
    return model;
  } else if (ret == PICOSAT_UNSATISFIABLE) {
    DREAL_LOG_DEBUG("SoplexSatSolver::CheckSat() No solution.");
    // UNSAT Case.
    return {};
  } else {
    DREAL_ASSERT(ret == PICOSAT_UNKNOWN);
    DREAL_LOG_CRITICAL("PICOSAT returns PICOSAT_UNKNOWN.");
    throw DREAL_RUNTIME_ERROR("PICOSAT returns PICOSAT_UNKNOWN.");
  }
}

void SoplexSatSolver::Pop() {
  // FIXME: disabled for QSopt_ex changes
  throw DREAL_RUNTIME_ERROR("SoplexSatSolver::Pop() currently unsupported");
  DREAL_LOG_DEBUG("SoplexSatSolver::Pop()");
  cnf_variables_.pop();
  to_sym_var_.pop();
  to_sat_var_.pop();
  picosat_pop(sat_);
  has_picosat_pop_used_ = true;
}

void SoplexSatSolver::Push() {
  // FIXME: disabled for QSopt_ex changes
  throw DREAL_RUNTIME_ERROR("SoplexSatSolver::Push() currently unsupported");
  DREAL_LOG_DEBUG("SoplexSatSolver::Push()");
  picosat_push(sat_);
  to_sat_var_.push();
  to_sym_var_.push();
  cnf_variables_.push();
}

void SoplexSatSolver::SetSPXVarCoef(DSVectorRational* coeffs, const Variable& var,
                                    const mpq_class& value) {
  DREAL_ASSERT(coeffs != nullptr);
  const auto it = to_spx_col_.find(var.get_id());
  if (it == to_spx_col_.end()) {
    throw DREAL_RUNTIME_ERROR("Variable undefined: {}", var);
  }
  if (value <= -soplex::infinity || value >= soplex::infinity) {
    throw DREAL_RUNTIME_ERROR("LP coefficient too large: {}", value);
  }
  coeffs->add(it->second, to_mpq_t(value));
}

void SoplexSatSolver::SetSPXVarBound(const Variable& var, const char type,
                                     const mpq_class& value) {
  DREAL_ASSERT(type == 'L' || type == 'U' || type == 'B');
  const auto it = to_spx_col_.find(var.get_id());
  if (it == to_spx_col_.end()) {
    throw DREAL_RUNTIME_ERROR("Variable undefined: {}", var);
  }
  if (value <= -soplex::infinity || value >= soplex::infinity) {
    throw DREAL_RUNTIME_ERROR("Simple bound too large: {}", value);
  }
  if (type == 'L' || type == 'B') {
    if (to_mpq_t(value) > spx_lower_[it->second]) {
      spx_lower_[it->second] = to_mpq_t(value);
      DREAL_LOG_TRACE("SoplexSatSolver::SetSPXVarBound ('{}'): set lower bound of {} to {}",
                      type, var, spx_lower_[it->second]);
    }
  }
  if (type == 'U' || type == 'B') {
    if (to_mpq_t(value) < spx_upper_[it->second]) {
      spx_upper_[it->second] = to_mpq_t(value);
      DREAL_LOG_TRACE("SoplexSatSolver::SetSPXVarBound ('{}'): set upper bound of {} to {}",
                      type, var, spx_upper_[it->second]);
    }
  }
}

void SoplexSatSolver::ResetLinearProblem(const Box& box) {
  DREAL_LOG_TRACE("SoplexSatSolver::ResetLinearProblem(): Box =\n{}", box);
  // Omitting to do this seems to cause problems in soplex
  spx_prob_.clearBasis();
  // Clear constraint bounds
  const int spx_rows{spx_prob_.numRowsRational()};
  DREAL_ASSERT(static_cast<size_t>(spx_rows) == from_spx_row_.size());
  for (int i = 0; i < spx_rows; i++) {
    spx_prob_.changeRangeRational(i, -soplex::infinity, soplex::infinity);
  }
  // Clear variable bounds
  const int spx_cols{spx_prob_.numColsRational()};
  DREAL_ASSERT(!config_.use_phase_one_simplex() ||
               static_cast<size_t>(spx_cols) == from_spx_col_.size());
  for (const pair<int, Variable> kv : from_spx_col_) {
    DREAL_ASSERT(0 <= kv.first && kv.first < spx_cols);
    if (box.has_variable(kv.second)) {
      DREAL_ASSERT(-soplex::infinity <= box[kv.second].lb());
      DREAL_ASSERT(box[kv.second].lb() <= box[kv.second].ub());
      DREAL_ASSERT(box[kv.second].ub() <= soplex::infinity);
      spx_lower_[kv.first] = to_mpq_t(box[kv.second].lb());
      spx_upper_[kv.first] = to_mpq_t(box[kv.second].ub());
    } else {
      spx_lower_[kv.first] = -soplex::infinity;
      spx_upper_[kv.first] = soplex::infinity;
    }
    spx_prob_.changeBoundsRational(kv.first, -soplex::infinity, soplex::infinity);
  }
}

static bool is_simple_bound(const Formula& formula) {
  if (!is_relational(formula)) {
    return false;
  }
  const Expression& lhs{get_lhs_expression(formula)};
  const Expression& rhs{get_rhs_expression(formula)};
  return ((is_constant(lhs) && is_variable(rhs)) ||
          (is_variable(lhs) && is_constant(rhs)));
}

// Because the input precision > 0, and we have reduced this by a small amount,
// we can replace any strict inequalities with the equivalent non-strict
// inequalities, and ignore not-equal constraints altogether.

static bool is_equal_or_whatever(const Formula& formula, bool truth) {
  if (truth) {
    return is_equal_to(formula);
  } else {
    return is_not_equal_to(formula);
  }
}

static bool is_not_equal_or_whatever(const Formula& formula, bool truth) {
  return is_equal_or_whatever(formula, !truth);
}

static bool is_greater_or_whatever(const Formula& formula, bool truth) {
  if (truth) {
    return is_greater_than(formula) || is_greater_than_or_equal_to(formula);
  } else {
    return is_less_than(formula) || is_less_than_or_equal_to(formula);
  }
}

static bool is_less_or_whatever(const Formula& formula, bool truth) {
  return is_greater_or_whatever(formula, !truth);
}

void SoplexSatSolver::EnableLinearLiteral(const Variable& var, bool truth) {
    const auto it_row = to_spx_row_.find(make_pair(var.get_id(), truth));
    if (it_row != to_spx_row_.end()) {
      // A non-trivial linear literal from the input problem
      const int spx_row = it_row->second;
      const char sense{spx_sense_[spx_row]};
      const mpq_class& rhs{spx_rhs_[spx_row]};
      spx_prob_.changeRangeRational(spx_row,
        sense == 'G' || sense == 'E' ? Rational(to_mpq_t(rhs)) : Rational(-soplex::infinity),
        sense == 'L' || sense == 'E' ? Rational(to_mpq_t(rhs)) : Rational(soplex::infinity));
      DREAL_LOG_TRACE("SoplexSatSolver::EnableLinearLiteral({})", spx_row);
      return;
    }
    const auto& var_to_formula_map = predicate_abstractor_.var_to_formula_map();
    const auto it = var_to_formula_map.find(var);
    if (it != var_to_formula_map.end() && is_simple_bound(it->second)) {
      // A simple bound - set it directly
      const Formula& formula{it->second};
      const Expression& lhs{get_lhs_expression(formula)};
      const Expression& rhs{get_rhs_expression(formula)};
      DREAL_LOG_TRACE("SoplexSatSolver::EnableLinearLiteral({}{})",
                      truth ? "" : "¬", formula);
      if (is_equal_or_whatever(formula, truth)) {
        if (is_variable(lhs) && is_constant(rhs)) {
          SetSPXVarBound(get_variable(lhs), 'B', get_constant_value(rhs));
        } else if (is_constant(lhs) && is_variable(rhs)) {
          SetSPXVarBound(get_variable(rhs), 'B', get_constant_value(lhs));
        } else {
          DREAL_UNREACHABLE();
        }
      } else if (is_greater_or_whatever(formula, truth)) {
        if (is_variable(lhs) && is_constant(rhs)) {
          SetSPXVarBound(get_variable(lhs), 'L', get_constant_value(rhs));
        } else if (is_constant(lhs) && is_variable(rhs)) {
          SetSPXVarBound(get_variable(rhs), 'U', get_constant_value(lhs));
        } else {
          DREAL_UNREACHABLE();
        }
      } else if (is_less_or_whatever(formula, truth)) {
        if (is_variable(lhs) && is_constant(rhs)) {
          SetSPXVarBound(get_variable(lhs), 'U', get_constant_value(rhs));
        } else if (is_constant(lhs) && is_variable(rhs)) {
          SetSPXVarBound(get_variable(rhs), 'L', get_constant_value(lhs));
        } else {
          DREAL_UNREACHABLE();
        }
      } else if (is_not_equal_or_whatever(formula, truth)) {
        // Nothing to do, because this constraint is always delta-sat for
        // delta > 0.
      } else {
        throw DREAL_RUNTIME_ERROR("Formula {} not supported", formula);
      }
      return;
    }
    // Either a learned literal, or a not-equal literal from the input
    // problem.
    DREAL_LOG_TRACE("SoplexSatSolver::EnableLinearLiteral: ignoring ({}, {})",
                    var, truth);
}

void SoplexSatSolver::AddLinearLiteral(const Variable& formulaVar, bool truth) {
    const auto& var_to_formula_map = predicate_abstractor_.var_to_formula_map();
    const auto it = var_to_formula_map.find(formulaVar);
    if (it == var_to_formula_map.end()) {
      // Boolean variable - no need to involve theory solver
      return;
    }
    const auto it2 = to_spx_row_.find(make_pair(formulaVar.get_id(), truth));
    if (it2 != to_spx_row_.end()) {
      // Found.
      return;
    }
    // Theory formula
    const Formula& formula = it->second;
    // Create the LP solver variables
    for (const Variable& var : formula.GetFreeVariables()) {
      AddLinearVariable(var);
    }
    if (is_equal_or_whatever(formula, truth)) {
      if (is_simple_bound(formula)) {
        return;  // Just create simple bound in LP
      }
      spx_sense_.push_back('E');
    } else if (is_greater_or_whatever(formula, truth)) {
      if (is_simple_bound(formula)) {
        return;
      }
      spx_sense_.push_back('G');
    } else if (is_less_or_whatever(formula, truth)) {
      if (is_simple_bound(formula)) {
        return;
      }
      spx_sense_.push_back('L');
    } else if (is_not_equal_or_whatever(formula, truth)) {
      // Nothing to do, because this constraint is always delta-sat for
      // delta > 0.
      return;
    } else {
      throw DREAL_RUNTIME_ERROR("Formula {} not supported", formula);
    }
    Expression expr;
    expr = (get_lhs_expression(formula) - get_rhs_expression(formula)).Expand();
    const int spx_row{spx_prob_.numRowsRational()};
    DSVectorRational coeffs;
    DREAL_ASSERT(static_cast<size_t>(spx_row) == spx_sense_.size() - 1);
    DREAL_ASSERT(static_cast<size_t>(spx_row) == spx_rhs_.size());
    spx_rhs_.push_back(0);
    if (is_constant(expr)) {
      spx_rhs_.back() = -get_constant_value(expr);
    } else if (is_variable(expr)) {
      SetSPXVarCoef(&coeffs, get_variable(expr), 1);
    } else if (is_multiplication(expr)) {
      std::map<Expression,Expression> map = get_base_to_exponent_map_in_multiplication(expr);
      if (map.size() != 1
       || !is_variable(map.begin()->first)
       || !is_constant(map.begin()->second)
       || get_constant_value(map.begin()->second) != 1) {
        throw DREAL_RUNTIME_ERROR("Expression {} not supported", expr);
      }
      SetSPXVarCoef(&coeffs,
                    get_variable(map.begin()->first),
                    get_constant_in_multiplication(expr));
    } else if (is_addition(expr)) {
      const std::map<Expression,mpq_class>& map = get_expr_to_coeff_map_in_addition(expr);
      for (const pair<Expression,mpq_class>& pair : map) {
        if (!is_variable(pair.first)) {
          throw DREAL_RUNTIME_ERROR("Expression {} not supported", expr);
        }
        SetSPXVarCoef(&coeffs, get_variable(pair.first), pair.second);
      }
      spx_rhs_.back() = -get_constant_in_addition(expr);
    } else {
        throw DREAL_RUNTIME_ERROR("Expression {} not supported", expr);
    }
    if (spx_rhs_.back() <= -soplex::infinity || spx_rhs_.back() >= soplex::infinity) {
      throw DREAL_RUNTIME_ERROR("LP RHS value too large: {}", spx_rhs_.back());
    }
    // Inactive
    spx_prob_.addRowRational(LPRowRational(-soplex::infinity, coeffs, soplex::infinity));
    if (!config_.use_phase_one_simplex()) {
      CreateArtificials(spx_row);
    }
    // Update indexes
    to_spx_row_.emplace(make_pair(make_pair(formulaVar.get_id(), truth), spx_row));
    DREAL_ASSERT(static_cast<size_t>(spx_row) == from_spx_row_.size());
    from_spx_row_.push_back(make_pair(formulaVar, truth));
    DREAL_LOG_DEBUG("SoplexSatSolver::AddLinearLiteral({}{} ↦ {})",
                    truth ? "" : "¬", it->second, spx_row);
}

void SoplexSatSolver::CreateArtificials(const int spx_row) {
  DREAL_ASSERT(!config_.use_phase_one_simplex());
  const int spx_cols{spx_prob_.numColsRational()};
  spx_lower_.reDim(spx_cols + 2, true);  // Set lower bounds to zero
  spx_upper_.reDim(spx_cols + 2, false);
  spx_upper_[spx_cols] = soplex::infinity;  // Set upper bounds to infinity
  spx_upper_[spx_cols + 1] = soplex::infinity;
  DSVectorRational coeffsPos;
  coeffsPos.add(spx_row, 1);
  spx_prob_.addColRational(LPColRational(1, coeffsPos, soplex::infinity, 0));
  DSVectorRational coeffsNeg;
  coeffsNeg.add(spx_row, -1);
  spx_prob_.addColRational(LPColRational(1, coeffsNeg, soplex::infinity, 0));
  DREAL_LOG_DEBUG("SoplexSatSolver::CreateArtificials({} -> ({}, {}))",
                  spx_row, spx_cols, spx_cols + 1);
}

void SoplexSatSolver::UpdateLookup(int lit, int learned) {
  if (learned) {
    learned_clause_lits_.insert(lit);
  } else {
    main_clauses_copy_.push_back(lit);
    main_clause_lookup_[lit].insert(cur_clause_start_);
  }
}

void SoplexSatSolver::AddLiteral(const Literal& l, bool learned) {
  if (l.second) {
    // f = b
    const Variable& var{l.first};
    DREAL_ASSERT(var.get_type() == Variable::Type::BOOLEAN);
    // Add l = b
    int lit{to_sat_var_[var.get_id()]};
    picosat_add(sat_, lit);
    UpdateLookup(lit, learned);
    if (!learned) {
      AddLinearLiteral(var, true);
    }
  } else {
    // f = ¬b
    const Variable& var{l.first};
    DREAL_ASSERT(var.get_type() == Variable::Type::BOOLEAN);
    // Add l = ¬b
    int lit{-to_sat_var_[var.get_id()]};
    picosat_add(sat_, lit);
    UpdateLookup(lit, learned);
    if (!learned) {
      AddLinearLiteral(var, false);
    }
  }
}

void SoplexSatSolver::AddLiteral(const Formula& f) {
  DREAL_ASSERT(is_variable(f) ||
               (is_negation(f) && is_variable(get_operand(f))));
  if (is_variable(f)) {
    AddLiteral(make_pair(get_variable(f), true), false);
  } else {
    AddLiteral(make_pair(get_variable(get_operand(f)), false), false);
  }
}

void SoplexSatSolver::DoAddClause(const Formula& f) {
  cur_clause_start_ = main_clauses_copy_.size();
  if (is_disjunction(f)) {
    // f = l₁ ∨ ... ∨ lₙ
    for (const Formula& l : get_operands(f)) {
      AddLiteral(l);
    }
  } else {
    // f = b or f = ¬b.
    AddLiteral(f);
  }
  picosat_add(sat_, 0);
  main_clauses_copy_.push_back(0);
}

void SoplexSatSolver::MakeSatVar(const Variable& var) {
  auto it = to_sat_var_.find(var.get_id());
  if (it != to_sat_var_.end()) {
    // Found.
    return;
  }
  // It's not in the maps, let's make one and add it.
  const int sat_var{picosat_inc_max_var(sat_)};
  to_sat_var_.insert(var.get_id(), sat_var);
  to_sym_var_.insert(sat_var, var);
  DREAL_LOG_DEBUG("SoplexSatSolver::MakeSatVar({} ↦ {})", var, sat_var);
}

void SoplexSatSolver::AddLinearVariable(const Variable& var) {
  auto it = to_spx_col_.find(var.get_id());
  if (it != to_spx_col_.end()) {
    // Found.
    return;
  }
  const int spx_col{spx_prob_.numColsRational()};
  spx_lower_.reDim(spx_col + 1, false);
  spx_upper_.reDim(spx_col + 1, false);
  spx_lower_[spx_col] = -soplex::infinity;  // Set unbounded
  spx_upper_[spx_col] = soplex::infinity;
  // obj, coeffs, upper, lower
  spx_prob_.addColRational(LPColRational(0, DSVectorRational(),
                                         soplex::infinity, -soplex::infinity));
  to_spx_col_.emplace(make_pair(var.get_id(), spx_col));
  from_spx_col_[spx_col] = var;
  DREAL_LOG_DEBUG("SoplexSatSolver::AddLinearVariable({} ↦ {})", var, spx_col);
}

const std::map<int, Variable>& SoplexSatSolver::GetLinearVarMap() const {
  DREAL_LOG_TRACE("SoplexSatSolver::GetLinearVarMap(): from_spx_col_ =");
  if (log()->should_log(spdlog::level::trace)) {
    for (const pair<int, Variable> kv : from_spx_col_) {
      std::cerr << kv.first << ": " << kv.second << "\n";
    }
  }
  return from_spx_col_;
}
}  // namespace dreal
