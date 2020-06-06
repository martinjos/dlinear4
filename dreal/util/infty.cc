/// @file infty.cc
///

#include "dreal/util/infty.h"

namespace dreal {
namespace util {

mpq_class* mpq_class_infinity = nullptr;
mpq_class* mpq_class_ninfinity = nullptr;

void InftyStart(double val) {
  mpq_class_infinity = new mpq_class(val);
  mpq_class_ninfinity = new mpq_class(-val);
}

void InftyStart(const mpq_class& val) {
  mpq_class_infinity = new mpq_class(val);
  mpq_class_ninfinity = new mpq_class(-val);
}

void InftyStart(const mpq_t infty, const mpq_t ninfty) {
  mpq_class_infinity = new mpq_class(infty);
  mpq_class_ninfinity = new mpq_class(ninfty);
}

void InftyFinish() {
  delete mpq_class_infinity;
  delete mpq_class_ninfinity;
}

// Important: must call InftyStart() first!
// Also, if using QSXStart(), must call it before InftyStart().

const mpq_class& mpq_infty() {
  return *mpq_class_infinity;
}

const mpq_class& mpq_ninfty() {
  return *mpq_class_ninfinity;
}

}  // namespace util
}  // namespace dreal
