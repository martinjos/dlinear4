#include "dreal/solver/qsoptex_sat_solver.h"
#include "dreal/solver/soplex_sat_solver.h"

#include <gtest/gtest.h>

#include "dreal/solver/config.h"
#include "dreal/symbolic/symbolic.h"
#include "dreal/symbolic/symbolic_test_util.h"

#define DREAL_TEST_F_SATSOLVERS(Class, Method) \
    class DRealTestFSatSolvers_##Class##_##Method : public Class { \
     protected: \
      void DRealTestFSatSolversImpl(); \
    }; \
    TEST_F(DRealTestFSatSolvers_##Class##_##Method, Method##_Soplex) { \
      sat_.Init<SoplexSatSolver>(config_); \
      DRealTestFSatSolversImpl(); \
    } \
    TEST_F(DRealTestFSatSolvers_##Class##_##Method, Method##_Qsoptex) { \
      sat_.Init<QsoptexSatSolver>(config_); \
      DRealTestFSatSolversImpl(); \
    } \
    void DRealTestFSatSolvers_##Class##_##Method::DRealTestFSatSolversImpl()

namespace dreal {
namespace {

using Model = std::pair<std::vector<Literal>, std::vector<Literal>>;

class SatSolverChecker {
 private:

  class ContainerBase {
   public:
    virtual ~ContainerBase() {}
    virtual void AddFormula(const Formula& f) = 0;
    virtual optional<Model> CheckSat(const Box& box) = 0;
  };

  template <typename SatSolver>
  class Container : public ContainerBase {
   public:
    explicit Container(const Config& config) : sat_solver_{config} {}
    virtual ~Container() {}
    void AddFormula(const Formula& f) {
      sat_solver_.AddFormula(f);
    }
    optional<Model> CheckSat(const Box& box) {
      return sat_solver_.CheckSat(box);
    }
   private:
    SatSolver sat_solver_;
  };

 public:

  SatSolverChecker() : container_{nullptr} {}

  virtual ~SatSolverChecker() {
    if (container_) {
      delete container_;
    }
  }

  template <typename SatSolver>
  void Init(const Config& config) {
    if (container_) {
      delete container_;
    }
    container_ = new Container<SatSolver>(config);
  }

  void AddFormula(const Formula& f) {
    DREAL_ASSERT(container_ != nullptr);
    container_->AddFormula(f);
  }
  optional<Model> CheckSat(const Box& box) {
    DREAL_ASSERT(container_ != nullptr);
    return container_->CheckSat(box);
  }

 private:
  ContainerBase* container_;
};

class SatSolverTest : public ::testing::Test {
  DrakeSymbolicGuard guard_;
 protected:
  SatSolverTest() {
    //config_.mutable_verbose_simplex() = 5;
  }

  const Variable b1_{"b1", Variable::Type::BOOLEAN};
  const Variable b2_{"b2", Variable::Type::BOOLEAN};
  const Box box_{Box({b1_, b2_})};

  SatSolverChecker sat_;
  Config config_;
};

DREAL_TEST_F_SATSOLVERS(SatSolverTest, Sat1) {
  // b1
  sat_.AddFormula(Formula{b1_});
  EXPECT_TRUE(sat_.CheckSat(box_));
}

DREAL_TEST_F_SATSOLVERS(SatSolverTest, Sat2) {
  // b2
  sat_.AddFormula(!b1_);
  EXPECT_TRUE(sat_.CheckSat(box_));
}

DREAL_TEST_F_SATSOLVERS(SatSolverTest, Sat3) {
  // b1 ∧ ¬b2
  sat_.AddFormula(b1_ && !b2_);
  EXPECT_TRUE(sat_.CheckSat(box_));
}

DREAL_TEST_F_SATSOLVERS(SatSolverTest, Sat4) {
  // b1 ∧ ¬b2
  sat_.AddFormula(Formula{b1_});
  sat_.AddFormula(!b2_);
  EXPECT_TRUE(sat_.CheckSat(box_));
}

DREAL_TEST_F_SATSOLVERS(SatSolverTest, Sat5) {
  // (b1 ∧ b2) ∧ (b1 ∨ b2)
  sat_.AddFormula(b1_ && b2_);
  sat_.AddFormula(b1_ || b2_);
  EXPECT_TRUE(sat_.CheckSat(box_));
}

DREAL_TEST_F_SATSOLVERS(SatSolverTest, UNSAT1) {
  // b1 ∧ ¬b1
  sat_.AddFormula(Formula{b1_});
  sat_.AddFormula(!b1_);
  EXPECT_FALSE(sat_.CheckSat(box_));
}

DREAL_TEST_F_SATSOLVERS(SatSolverTest, UNSAT2) {
  // (b1 ∧ b2) ∧ (¬b1 ∨ ¬b2)
  sat_.AddFormula(b1_ && b2_);
  sat_.AddFormula(!b1_ || !b2_);
  EXPECT_FALSE(sat_.CheckSat(box_));
}

}  // namespace
}  // namespace dreal
