#pragma once

#include <gtest/gtest.h>

#define DREAL_TEST_F_PHASES(Class, Method) \
    class DRealTestFPhases_##Class##_##Method : public Class { \
     protected: \
      void DRealTestFPhasesImpl(); \
    }; \
    TEST_F(DRealTestFPhases_##Class##_##Method, Method##_Phase1) { \
      config_.mutable_use_phase_one_simplex() = true; \
      DRealTestFPhasesImpl(); \
    } \
    TEST_F(DRealTestFPhases_##Class##_##Method, Method##_Phase2) { \
      config_.mutable_use_phase_one_simplex() = false; \
      DRealTestFPhasesImpl(); \
    } \
    void DRealTestFPhases_##Class##_##Method::DRealTestFPhasesImpl()
