/// @file qsopt_ex.h
///
/// This is the header file where we expose QSopt_ex inside of the
/// dreal namespace.
///
/// Other files in dreal should include this file and should NOT
/// include files in the qsopt_ex directory. Similarly, BUILD files
/// should only have a dependency on "//dreal/:qsopt-ex", not
/// "//:qsopt-ex".
///
#pragma once

// All of these are included by QSopt_ex.h, and we need to make sure that their
// symbols are correctly namespaced.
#include <cerrno>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/resource.h>
#include <sys/time.h>
#include <unistd.h>

#include <string>
#include <gmpxx.h>

namespace dreal {
namespace qsopt_ex {

extern "C" {
#include <qsopt_ex/QSopt_ex.h>
}

mpq_class* StringToMpqPtr(const std::string& str);
mpq_class StringToMpq(const std::string& str);

}  // namespace qsopt_ex
}  // namespace dreal
