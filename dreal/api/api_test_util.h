#pragma once

#include <gtest/gtest.h>

#if HAVE_SOPLEX
# define DREAL_TEST_F_PHASES(Class, Method) \
    class DRealTestFPhases_##Class##_##Method : public Class { \
     protected: \
      DRealTestFPhases_##Class##_##Method() = delete; \
      explicit DRealTestFPhases_##Class##_##Method(Config::LPSolver lp_solver) : Class{lp_solver} {} \
      void DRealTestFPhasesImpl(); \
    }; \
    class DRealTestFPhases_##Class##_##Method##_Soplex : public DRealTestFPhases_##Class##_##Method { \
     protected: \
      DRealTestFPhases_##Class##_##Method##_Soplex() : DRealTestFPhases_##Class##_##Method{Config::SOPLEX} {} \
    }; \
    class DRealTestFPhases_##Class##_##Method##_Qsoptex : public DRealTestFPhases_##Class##_##Method { \
     protected: \
      DRealTestFPhases_##Class##_##Method##_Qsoptex() : DRealTestFPhases_##Class##_##Method{Config::QSOPTEX} {} \
    }; \
    TEST_F(DRealTestFPhases_##Class##_##Method##_Soplex, Method##_Soplex_Phase1) { \
      config_.mutable_lp_solver() = Config::SOPLEX; \
      config_.mutable_use_phase_one_simplex() = true; \
      DRealTestFPhasesImpl(); \
    } \
    TEST_F(DRealTestFPhases_##Class##_##Method##_Soplex, Method##_Soplex_Phase2) { \
      config_.mutable_lp_solver() = Config::SOPLEX; \
      config_.mutable_use_phase_one_simplex() = false; \
      DRealTestFPhasesImpl(); \
    } \
    TEST_F(DRealTestFPhases_##Class##_##Method##_Qsoptex, Method##_Qsoptex_Phase1) { \
      config_.mutable_lp_solver() = Config::QSOPTEX; \
      config_.mutable_use_phase_one_simplex() = true; \
      DRealTestFPhasesImpl(); \
    } \
    TEST_F(DRealTestFPhases_##Class##_##Method##_Qsoptex, Method##_Qsoptex_Phase2) { \
      config_.mutable_lp_solver() = Config::QSOPTEX; \
      config_.mutable_use_phase_one_simplex() = false; \
      DRealTestFPhasesImpl(); \
    } \
    void DRealTestFPhases_##Class##_##Method::DRealTestFPhasesImpl()
#else
# define DREAL_TEST_F_PHASES(Class, Method) \
    class DRealTestFPhases_##Class##_##Method : public Class { \
     protected: \
      DRealTestFPhases_##Class##_##Method() = delete; \
      explicit DRealTestFPhases_##Class##_##Method(Config::LPSolver lp_solver) : Class{lp_solver} {} \
      void DRealTestFPhasesImpl(); \
    }; \
    class DRealTestFPhases_##Class##_##Method##_Qsoptex : public DRealTestFPhases_##Class##_##Method { \
     protected: \
      DRealTestFPhases_##Class##_##Method##_Qsoptex() : DRealTestFPhases_##Class##_##Method{Config::QSOPTEX} {} \
    }; \
    TEST_F(DRealTestFPhases_##Class##_##Method##_Qsoptex, Method##_Qsoptex_Phase1) { \
      config_.mutable_lp_solver() = Config::QSOPTEX; \
      config_.mutable_use_phase_one_simplex() = true; \
      DRealTestFPhasesImpl(); \
    } \
    TEST_F(DRealTestFPhases_##Class##_##Method##_Qsoptex, Method##_Qsoptex_Phase2) { \
      config_.mutable_lp_solver() = Config::QSOPTEX; \
      config_.mutable_use_phase_one_simplex() = false; \
      DRealTestFPhasesImpl(); \
    } \
    void DRealTestFPhases_##Class##_##Method::DRealTestFPhasesImpl()
#endif
