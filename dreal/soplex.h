/// @file soplex.h
///
/// This is the header file where we include SoPlex. We let it use its own
/// namespace, since it has one. However, we must first #define
/// SOPLEX_WITH_GMP, otherwise SoPlex behaves as if it doesn't have support for
/// GMP (even if it does).
///
/// Other files in dreal should include this file and should NOT
/// include files in the soplex directory. Similarly, BUILD files
/// should only have a dependency on "//dreal/:soplex", not
/// "//:soplex".
///
#pragma once

#include <gmp.h>

#define SOPLEX_WITH_GMP
#include <soplex.h>
