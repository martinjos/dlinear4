/// @file infty.h
#pragma once

#include <gmpxx.h>

namespace dreal {
namespace util {

extern mpq_class* mpq_class_infinity;
extern mpq_class* mpq_class_ninfinity;

void InftyStart(const mpq_t infty, const mpq_t ninfty);
void InftyStart(const mpq_class& val);
void InftyStart(double val);
void InftyFinish();

// Important: must call InftyStart() first!
// Also, if using QSXStart(), must call it before InftyStart().

const mpq_class& mpq_infty();
const mpq_class& mpq_ninfty();

}  // namespace util
}  // namespace dreal
