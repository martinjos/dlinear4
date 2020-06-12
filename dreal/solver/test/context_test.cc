#include "dreal/solver/context.h"

#include <memory>
#include <gtest/gtest.h>

#include "dreal/symbolic/symbolic.h"
#include "dreal/symbolic/symbolic_test_util.h"
#include "dreal/api/api_test_util.h"
#include "dreal/util/logging.h"

using std::unique_ptr;
using std::make_unique;

namespace dreal {
namespace {

class ContextTest : public ::testing::Test {
  DrakeSymbolicGuard guard_;
 protected:
  ContextTest() = delete;
  explicit ContextTest(Config::LPSolver lp_solver) : guard_{lp_solver} {
    config_.mutable_lp_solver() = lp_solver;
    context_.reset(new Context(config_));
  }

  void SetUp() override {
    ::testing::Test::SetUp();
    context_->DeclareVariable(x_);
  }

  const Variable x_{"x"};
  Config config_;
  unique_ptr<Context> context_;
};

DREAL_TEST_F_PHASES(ContextTest, MultipleCheckSat) {
  context_->Assert(x_ >= 0);
  const auto result1 = context_->CheckSat();
  EXPECT_TRUE(result1);
  context_->Assert(x_ <= 5);
  const auto result2 = context_->CheckSat();
  EXPECT_TRUE(result2);
}

// QSopt_ex changes: assertions don't modify the Box any more
#if 0
TEST_F(ContextTest, AssertionsAndBox) {
  const Formula f1{x_ >= 0};
  const Formula f2{x_ <= 5};
  const Formula f3{sin(x_) == 1.0};
  const Formula f4{cos(x_) == 0.0};
  context_->Assert(f1);
  context_->Assert(f2);
  context_->Assert(f3);
  context_->Assert(f4);

  const auto& assertions{context_->assertions()};
  EXPECT_EQ(assertions.size(), 2);
  EXPECT_TRUE(assertions[0].EqualTo(f3));
  EXPECT_TRUE(assertions[1].EqualTo(f4));

  const auto& box{context_->box()};
  EXPECT_EQ(box[x_].lb(), 0.0);
  EXPECT_EQ(box[x_].ub(), 5.0);
}
#endif

}  // namespace
}  // namespace dreal
