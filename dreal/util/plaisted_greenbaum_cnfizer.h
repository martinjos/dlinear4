#pragma once

#include <map>
#include <unordered_map>
#include <vector>

#include "dreal/symbolic/symbolic.h"
#include "dreal/util/naive_cnfizer.h"

namespace dreal {
/// Transforms a symbolic formula @p f into an equi-satisfiable CNF
/// formula by introducing extra Boolean variables (Plaisted-Greenbaum
/// transformation: https://doi.org/10.1016/S0747-7171(86)80028-1).
class PlaistedGreenbaumCnfizer {
 public:
  /// Convert @p f into an equi-satisfiable formula @c f' in CNF.
  std::vector<Formula> Convert(const Formula& f);

  /// Returns a const reference of `map_` member.
  ///
  /// @note that this member `map_` is cleared at the beginning of `Convert`
  /// method.
  const std::vector<Variable>& vars() const { return vars_; }

 private:
  Formula Visit(const Formula& f);
  Formula VisitFalse(const Formula& f);
  Formula VisitTrue(const Formula& f);
  Formula VisitVariable(const Formula& f);
  Formula VisitEqualTo(const Formula& f);
  Formula VisitNotEqualTo(const Formula& f);
  Formula VisitGreaterThan(const Formula& f);
  Formula VisitGreaterThanOrEqualTo(const Formula& f);
  Formula VisitLessThan(const Formula& f);
  Formula VisitLessThanOrEqualTo(const Formula& f);
  Formula VisitConjunction(const Formula& f);
  Formula VisitDisjunction(const Formula& f);
  Formula VisitNegation(const Formula& f);
  Formula VisitForall(const Formula& f);

  const Nnfizer nnfizer_{};

  // To transform nested formulas inside of universal quantifications.
  const NaiveCnfizer naive_cnfizer_{};

  // Set of auxilliary clauses collected during conversion.
  //
  // @note that this aux_ is cleared at the beginning of `Convert`
  // call.
  std::vector<Formula> aux_;

  // Set of variables generated during conversion.
  //
  // @note that this vars_ is cleared at the beginning of `Convert`
  // call.
  std::vector<Variable> vars_;

  // Makes VisitFormula a friend of this class so that it can use private
  // operator()s.
  friend Formula drake::symbolic::VisitFormula<Formula, PlaistedGreenbaumCnfizer>(
      PlaistedGreenbaumCnfizer*, const Formula&);
};
}  // namespace dreal
