#include "dreal/util/literal.h"

namespace dreal {

bool LiteralComparator::operator()(const Literal& a, const Literal& b) const {
  if (a.first.get_id() < b.first.get_id()) {
    return true;
  } else if (a.first.get_id() > b.first.get_id()) {
    return false;
  }
  return a.second < b.second;
}

}  // namespace dreal
