#include "dreal/solver/context.h"

#include <utility>

#include "dreal/solver/qsoptex_context_impl.h"
#include "dreal/util/exception.h"
#include "dreal/util/logging.h"
#include "dreal/version.h"

#if HAVE_SOPLEX
# include "dreal/solver/soplex_context_impl.h"
#endif

using std::make_unique;
using std::unique_ptr;
using std::string;
using std::vector;

namespace dreal {

unique_ptr<Context::Impl> Context::make_impl(Config config) {
  if (config.lp_solver() == Config::QSOPTEX) {
    return make_unique<Context::QsoptexImpl>(config);
  } else {
    DREAL_ASSERT(config.lp_solver() == Config::SOPLEX);
#if HAVE_SOPLEX
    return make_unique<Context::SoplexImpl>(config);
#else
    throw DREAL_RUNTIME_ERROR("SoPlex not enabled at compile time");
#endif
  }
}

Context::Context() : Context{Config{}} {}

Context::Context(Context&& context) noexcept
    : impl_{std::move(context.impl_)} {}

Context::~Context() = default;

Context::Context(Config config) : impl_{make_impl(config)} {}

void Context::Assert(const Formula& f) { impl_->Assert(f); }

optional<Box> Context::CheckSat(mpq_class* actual_precision) {
  return impl_->CheckSat(actual_precision);
}

int Context::CheckOpt(mpq_class* obj_lo, mpq_class* obj_up, Box* model) {
  return impl_->CheckOpt(obj_lo, obj_up, model);
}

void Context::DeclareVariable(const Variable& v, const bool is_model_variable) {
  impl_->DeclareVariable(v, is_model_variable);
}

void Context::DeclareVariable(const Variable& v, const Expression& lb,
                              const Expression& ub,
                              const bool is_model_variable) {
  impl_->DeclareVariable(v, is_model_variable);
  impl_->SetDomain(v, lb, ub);
}

void Context::Exit() { DREAL_LOG_DEBUG("Context::Exit()"); }

void Context::Minimize(const Expression& f) { impl_->Minimize({f}); }

void Context::Minimize(const vector<Expression>& functions) {
  impl_->Minimize(functions);
}

void Context::Maximize(const Expression& f) { impl_->Maximize({f}); }

void Context::Pop(int n) {
  DREAL_LOG_DEBUG("Context::Pop({})", n);
  if (n <= 0) {
    throw DREAL_RUNTIME_ERROR(
        "Context::Pop(n) called with n = {} which is not positive.", n);
  }
  while (n-- > 0) {
    impl_->Pop();
  }
}

void Context::Push(int n) {
  DREAL_LOG_DEBUG("Context::Push({})", n);
  if (n <= 0) {
    throw DREAL_RUNTIME_ERROR(
        "Context::Push(n) called with n = {} which is not positive.", n);
  }
  while (n-- > 0) {
    impl_->Push();
  }
}

void Context::SetInfo(const string& key, const double val) {
  impl_->SetInfo(key, val);
}

void Context::SetInfo(const string& key, const string& val) {
  impl_->SetInfo(key, val);
}

void Context::SetInterval(const Variable& v, const mpq_class& lb, const mpq_class& ub) {
  impl_->SetInterval(v, lb, ub);
}

void Context::SetLogic(const Logic& logic) { impl_->SetLogic(logic); }

void Context::SetOption(const string& key, const double val) {
  impl_->SetOption(key, val);
}

void Context::SetOption(const string& key, const string& val) {
  impl_->SetOption(key, val);
}

const Config& Context::config() const { return impl_->config(); }
Config& Context::mutable_config() { return impl_->mutable_config(); }

string Context::version() { return DREAL_VERSION_STRING; }

string Context::repository_status() { return DREAL_VERSION_REPOSTAT; }

const Box& Context::box() const { return impl_->box(); }

const Box& Context::get_model() const { return impl_->get_model(); }

const ScopedVector<Formula>& Context::assertions() const {
  return impl_->assertions();
}

bool Context::have_objective() const { return impl_->have_objective(); }

bool Context::is_max() const { return impl_->is_max(); }

}  // namespace dreal
