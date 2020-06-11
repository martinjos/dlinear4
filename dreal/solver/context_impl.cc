#include "dreal/solver/context_impl.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <set>
#include <unordered_set>
#include <utility>

#include <fmt/format.h>

//#include "dreal/solver/filter_assertion.h"
#include "dreal/util/assert.h"
#include "dreal/util/exception.h"
#include "dreal/util/interrupt.h"
#include "dreal/util/logging.h"

namespace dreal {

using std::find_if;
using std::set;
using std::string;
using std::unordered_set;
using std::vector;

namespace {
// It is possible that a solution box has a dimension whose diameter
// is larger than delta when the given constraints are not tight.
// This function tighten the box @p box so that every dimension has a
// width smaller than delta.
#if 0
void Tighten(Box* box, const double delta) {
  for (int i = 0; i < box->size(); ++i) {
    auto& interval = (*box)[i];
    if (interval.diam() > delta) {
      const Variable& var{box->variable(i)};
      switch (var.get_type()) {
        case Variable::Type::BINARY:
        case Variable::Type::BOOLEAN:
          // Always pick True (1.0)
          interval = 1.0;
          break;
        case Variable::Type::CONTINUOUS: {
          const double mid{interval.mid()};
          const double half_delta{delta / 2.0};
          interval &= Box::Interval(mid - half_delta, mid + half_delta);
        } break;
        case Variable::Type::INTEGER: {
          const double mid{interval.mid()};
          interval = static_cast<int>(mid);
        } break;
      }
    }
  }
}
#endif

bool ParseBooleanOption(const string& key, const string& val) {
  if (val == "true") {
    return true;
  }
  if (val == "false") {
    return false;
  }
  throw DREAL_RUNTIME_ERROR("Unknown value {} is provided for option {}", val,
                            key);
}
}  // namespace

Context::Impl::Impl() : Impl{Config{}} {}

Context::Impl::Impl(Config config)
    : config_{config} {
  boxes_.push_back(Box{});
}

optional<Box> Context::Impl::CheckSat() {
  auto result = CheckSatCore(stack_, box());
  if (result) {
    // In case of delta-sat, do post-processing.
    //Tighten(&(*result), config_.precision());
    DREAL_LOG_DEBUG("ContextImpl::CheckSat() - Found Model\n{}", *result);
    model_ = ExtractModel(*result);
    return model_;
  } else {
    model_.set_empty();
    return result;
  }
}

void Context::Impl::AddToBox(const Variable& v) {
  DREAL_LOG_DEBUG("ContextImpl::AddToBox({})", v);
  const auto& variables = box().variables();
  if (find_if(variables.begin(), variables.end(), [&v](const Variable& v_) {
        return v.equal_to(v_);
      }) == variables.end()) {
    // v is not in box.
    box().Add(v);
  }
}

void Context::Impl::DeclareVariable(const Variable& v,
                                    const bool is_model_variable) {
  DREAL_LOG_DEBUG("ContextImpl::DeclareVariable({})", v);
  AddToBox(v);
  if (is_model_variable) {
    mark_model_variable(v);
  }
}

void Context::Impl::SetDomain(const Variable& v, const Expression& lb,
                              const Expression& ub) {
  const mpq_class& lb_fp = lb.Evaluate();
  const mpq_class& ub_fp = ub.Evaluate();
  SetInterval(v, lb_fp, ub_fp);
}

#if 0
void Context::Impl::Minimize(const vector<Expression>& functions) {
  // Given objective functions f₁(x), ... fₙ(x) and the current
  // constraints ϕᵢ which involves x. this method encodes them into a
  // universally quantified formula ψ:
  //
  //    ψ = ∀y. (⋀ᵢ ϕᵢ(y)) → (f₁(x) ≤ f₁(y) ∨ ... ∨ fₙ(x) ≤ fₙ(y))
  //      = ∀y. ¬(⋀ᵢ ϕᵢ(y)) ∨ (f₁(x) ≤ f₁(y) ∨ ... ∨ fₙ(x) ≤ fₙ(y))
  //      = ∀y. (∨ᵢ ¬ϕᵢ(y)) ∨ (f₁(x) ≤ f₁(y) ∨ ... ∨ fₙ(x) ≤ fₙ(y)).
  //
  // Here we introduce existential variables zᵢ for fᵢ(x) to have
  //
  //    ψ =  ∃z₁...zₙ. (z₁ = f₁(x) ∧ ... ∧ zₙ = fₙ(x)) ∧
  //              [∀y. (∨ᵢ ¬ϕᵢ(y)) ∨ (z₁ ≤ f₁(y) ∨ ... ∨ zₙ ≤ fₙ(y))].
  //
  // Note that when we have more than one objective function, this
  // encoding scheme denotes Pareto optimality.
  //
  // To construct ϕᵢ(y), we need to traverse both `box()` and
  // `stack_` because some of the asserted formulas are translated
  // and applied into `box()`.
  set<Formula> set_of_negated_phi;  // Collects ¬ϕᵢ(y).
  ExpressionSubstitution subst;  // Maps xᵢ ↦ yᵢ to build f(y₁, ..., yₙ).
  Variables quantified_variables;  // {y₁, ... yₙ}
  Variables x_vars;
  for (const Expression& f : functions) {
    x_vars += f.GetVariables();
  }

  // Collects side-constraints related to the cost functions.
  unordered_set<Formula> constraints;
  bool keep_going = true;
  while (keep_going) {
    keep_going = false;
    for (const Formula& constraint : stack_) {
      if (constraints.find(constraint) == constraints.end() &&
          HaveIntersection(x_vars, constraint.GetFreeVariables())) {
        x_vars += constraint.GetFreeVariables();
        constraints.insert(constraint);
        keep_going = true;
      }
    }
  }

  for (const Variable& x_i : x_vars) {
    // We add postfix '_' to name y_i
    const Variable y_i{x_i.get_name() + "_", x_i.get_type()};
    quantified_variables.insert(y_i);
    subst.emplace(x_i, y_i);
    // If `box()[x_i]` has a finite bound, let's add that information in
    // set_of_negated_phi as a constraint on y_i.
    const Box::Interval& bound_on_x_i{box()[x_i]};
    const mpq_class& lb_x_i{bound_on_x_i.lb()};
    const mpq_class& ub_x_i{bound_on_x_i.ub()};
    if (isfinite(lb_x_i)) {
      set_of_negated_phi.insert(!(lb_x_i <= y_i));
    }
    if (isfinite(ub_x_i)) {
      set_of_negated_phi.insert(!(y_i <= ub_x_i));
    }
  }

  for (const Formula& constraint : constraints) {
    set_of_negated_phi.insert(!constraint.Substitute(subst));
  }

  Formula quantified{make_disjunction(set_of_negated_phi)};  // ∨ᵢ ¬ϕᵢ(y)
  Formula new_z_block;  // This will have (z₁ = f₁(x) ∧ ... ∧ zₙ = fₙ(x)).
  static int counter{0};
  for (const Expression& f_i : functions) {
    const Variable z_i{fmt::format("Z{}", counter++),
                       Variable::Type::CONTINUOUS};
    AddToBox(z_i);
    new_z_block = new_z_block && (z_i == f_i);
    quantified = quantified || (z_i <= f_i.Substitute(subst));
  }
  const Formula psi{new_z_block && forall(quantified_variables, quantified)};
  return Assert(psi);
}
#endif

void Context::Impl::SetInfo(const string& key, const double val) {
  DREAL_LOG_DEBUG("ContextImpl::SetInfo({} ↦ {})", key, val);
  info_[key] = fmt::format("{}", val);
}

void Context::Impl::SetInfo(const string& key, const string& val) {
  DREAL_LOG_DEBUG("ContextImpl::SetInfo({} ↦ {})", key, val);
  info_[key] = val;
}

void Context::Impl::SetInterval(const Variable& v, const mpq_class& lb,
                                const mpq_class& ub) {
  DREAL_LOG_DEBUG("ContextImpl::SetInterval({} = [{}, {}])", v, lb, ub);
  box()[v] = Box::Interval{lb, ub};
}

void Context::Impl::SetLogic(const Logic& logic) {
  DREAL_LOG_DEBUG("ContextImpl::SetLogic({})", logic);
  logic_ = logic;
}

void Context::Impl::SetOption(const string& key, const double val) {
  DREAL_LOG_DEBUG("ContextImpl::SetOption({} ↦ {})", key, val);
  option_[key] = fmt::format("{}", val);

  if (key == ":precision") {
    if (val <= 0.0) {
      throw DREAL_RUNTIME_ERROR("Precision has to be positive (input = {}).",
                                val);
    }
    return config_.mutable_precision().set_from_file(val);
  }
}

void Context::Impl::SetOption(const string& key, const string& val) {
  DREAL_LOG_DEBUG("ContextImpl::SetOption({} ↦ {})", key, val);
  option_[key] = val;
  if (key == ":polytope") {
    return config_.mutable_use_polytope().set_from_file(
        ParseBooleanOption(key, val));
  }
  if (key == ":forall-polytope") {
    return config_.mutable_use_polytope_in_forall().set_from_file(
        ParseBooleanOption(key, val));
  }
  if (key == ":local-optimization") {
    return config_.mutable_use_local_optimization().set_from_file(
        ParseBooleanOption(key, val));
  }
  if (key == ":worklist-fixpoint") {
    return config_.mutable_use_worklist_fixpoint().set_from_file(
        ParseBooleanOption(key, val));
  }
  if (key == ":produce-models") {
    return config_.mutable_produce_models().set_from_file(
        ParseBooleanOption(key, val));
  }
}

Box Context::Impl::ExtractModel(const Box& box) const {
  if (static_cast<int>(model_variables_.size()) == box.size()) {
    // Every variable is a model variable. Simply return the @p box.
    return box;
  }
  Box new_box;
  for (const Variable& v : box.variables()) {
    if (is_model_variable(v)) {
      new_box.Add(v, box[v].lb(), box[v].ub());
    }
  }
  return new_box;
}

bool Context::Impl::is_model_variable(const Variable& v) const {
  return (model_variables_.find(v.get_id()) != model_variables_.end());
}

void Context::Impl::mark_model_variable(const Variable& v) {
  model_variables_.insert(v.get_id());
}

const ScopedVector<Formula>& Context::Impl::assertions() const {
  return stack_;
}

}  // namespace dreal
