/// @file qsopt_ex.h
///
/// This is the header file that we expose QSopt_ex inside of dreal
/// namespace.
///
/// Other files in dreal should include this file and should NOT
/// include files in qsopt_ex directory. Similarly, BUILD files
/// should only have a dependency "//dreal/:qsopt-ex", not
/// "//:qsopt-ex".
///
#pragma once

#include <gmpxx.h>

namespace dreal {
namespace qsopt_ex {

extern "C" {
#include <qsopt_ex/QSopt_ex.h>
}

}  // namespace qsopt_ex
}  // namespace dreal
