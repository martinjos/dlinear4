#pragma once

#include <cstdint>

#include "dreal/gmp.h"

namespace dreal {
/// Returns true if @p v is represented by `int`.
bool is_integer(double v);
bool is_integer(const mpq_class& v);

/// Converts @p v of int64_t to int.
/// @throw std::runtime_error if this conversion result in a loss of precision.
int convert_int64_to_int(std::int64_t v);

/// Converts @p v of int64_t to double.
/// @throw std::runtime_error if this conversion result in a loss of precision.
double convert_int64_to_double(std::int64_t v);

mpq_class convert_int64_to_rational(const int64_t v);

}  // namespace dreal
