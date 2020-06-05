#pragma once

#include <set>
#include <utility>

#include "dreal/symbolic/symbolic.h"

namespace dreal {

using Literal = std::pair<Variable, bool>;

struct LiteralComparator {
  bool operator()(const Literal& a, const Literal& b) const;
};

using LiteralSet = std::set<Literal, LiteralComparator>;

}  // namespace dreal
