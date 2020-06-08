/// @file gmp.h
///
/// This is the header file where we include GMP, and declare various
/// helpers.
///
/// Other files in dreal should include this file and should NOT
/// include gmp.h or gmpxx.h. Similarly, BUILD files
/// should only have a dependency on "//dreal/:gmp", not
/// "@linux_libs//:gmpxx" or "@linux_libs//:gmp".
///
#pragma once

#include <functional>

#include <gmpxx.h>

namespace std {

template <>
class hash<mpq_class> {
 public:
    size_t operator()(const mpq_class& val) const;
};

}  // namespace std

namespace dreal {
namespace gmp {

mpz_class floor(const mpq_class& val);
mpz_class ceil(const mpq_class& val);

// Important definitions from <gmpxx.h> and <gmp.h> (fair use):
//
//   mpq_srcptr mpq_class::get_mpq_t() const { return mp; }
//   mpq_ptr mpq_class::get_mpq_t() { return mp; }
//
//   typedef const __mpq_struct *mpq_srcptr;
//   typedef __mpq_struct *mpq_ptr;
//   typedef __mpq_struct mpq_t[1];
//
// We can cast mpq_ptr to mpq_t * (or mpq_srcptr to const mpq_t *).
// This is the same as casting (__mpq_struct *) to (__mpq_struct (*)[1]).
// It's okay because it converts a pointer to a struct, to a pointer to an
// array of that struct (which is always okay).
//
// We can then dereference the (mpq_t *) to obtain an mpq_t.
// Because mpq_t is an array type, it is still effectively treated as a pointer
// in certain contexts (such as when returning it from / passing it into a
// function).
// This pointer has the same value as the (mpq_t *).
//
// We can then take a reference to the mpq_t.
// The address of this reference also has the same value as the (mpq_t *).
//
const inline mpq_t& to_mpq_t(const mpq_class& cla) {
  return *reinterpret_cast<const mpq_t*>(cla.get_mpq_t());
}
// NOLINTNEXTLINE(runtime/references)
inline mpq_t& to_mpq_t(mpq_class& cla) {
  return *reinterpret_cast<mpq_t*>(cla.get_mpq_t());
}

// This works because the internal representation of an mpq_class is exactly
// the same as that of an mpq_t (and, because we only take a reference, no
// constructor or destructor is ever called).
//
const inline mpq_class& to_mpq_class(const mpq_t& mpq) {
  return reinterpret_cast<const mpq_class&>(mpq);
}
// NOLINTNEXTLINE(runtime/references)
inline mpq_class& to_mpq_class(mpq_t& mpq) {
  return reinterpret_cast<mpq_class&>(mpq);
}

}  // namespace gmp
}  // namespace dreal
