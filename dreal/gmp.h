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

}  // namespace gmp
}  // namespace dreal
