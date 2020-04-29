#pragma once

#include <iostream>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dreal/util/assert.h"
#include "dreal/symbolic/symbolic.h"
#include "dreal/gmp.h"

namespace dreal {

/// Represents a n-dimensional interval vector.
class Box {
 public:
  class Interval {
   public:
    Interval() : lb_(1), ub_(0) {}
    Interval(Interval&& other) noexcept;
    Interval(const Interval& other) : lb_(other.lb_), ub_(other.ub_) {}
    explicit Interval(const mpq_class& val) : lb_(val), ub_(val) {}
    Interval(const mpq_class& lb, const mpq_class& ub) : lb_(lb), ub_(ub) {
      DREAL_ASSERT(lb <= ub);
    }
    bool is_empty() const { return lb_ == 1 && ub_ == 0; }
    bool is_degenerated() const { return lb_ == ub_; }
    bool is_bisectable() const { return lb_ < ub_; }
    mpq_class lb() const { return lb_; }
    mpq_class ub() const { return ub_; }
    mpq_class mid() const { return (lb_ + ub_) / 2; }
    mpq_class diam() const { return is_empty() ? mpq_class(0) : mpq_class(ub_ - lb_); }
    std::pair<Interval, Interval> bisect(const mpq_class& p) const;
    bool operator==(const Interval& other) const { return lb_ == other.lb_ && ub_ == other.ub_; }
    bool operator!=(const Interval& other) const { return lb_ != other.lb_ || ub_ != other.ub_; }
    Interval& operator=(const mpq_class& val) { lb_ = ub_ = val; return *this; }
    Interval& operator=(const Interval& other) { lb_ = other.lb_; ub_ = other.ub_; return *this; }
    // Mutators
    void set_empty() { lb_ = 1; ub_ = 0; }
    friend std::ostream& operator<<(std::ostream& os, const Interval& iv);

   private:
    mpq_class lb_, ub_;
  };

  class IntervalVector : public std::vector<Interval> {
    using vector::vector;
  };

  /// Constructs a zero-dimensional box.
  Box();

  /// Constructs a box from @p variables.
  explicit Box(const std::vector<Variable>& variables);

  /// Default copy constructor.
  Box(const Box&) = default;

  /// Default move constructor.
  Box(Box&&) = default;

  /// Default copy assign operator.
  Box& operator=(const Box&) = default;

  /// Default move assign operator.
  Box& operator=(Box&&) = default;

  /// Default destructor.
  ~Box() = default;

  /// Adds @p v to the box.
  void Add(const Variable& v);

  /// Adds @p v to the box and sets its domain using @p lb and @p ub.
  void Add(const Variable& v, const mpq_class& lb, const mpq_class& ub);

  /// Checks if this box is empty.
  bool empty() const;

  /// Make this box empty.
  void set_empty();

  /// Returns the size of the box.
  int size() const;

  /// Returns @p i -th interval in the box.
  Interval& operator[](int i);

  /// Returns an interval associated with @p var.
  Interval& operator[](const Variable& var);

  /// Returns @p i -th interval in the box.
  const Interval& operator[](int i) const;

  /// Returns an interval associated with @p var.
  const Interval& operator[](const Variable& var) const;

  /// Returns the variables in the box.
  const std::vector<Variable>& variables() const;

  /// Returns i-th variable in the box.
  const Variable& variable(int i) const;

  /// Checks if this box has @p var.
  bool has_variable(const Variable& var) const;

  /// Returns the interval vector of the box.
  const IntervalVector& interval_vector() const;

  /// Returns the interval vector of the box.
  IntervalVector& mutable_interval_vector();

  /// Returns the index associated with @p var.
  int index(const Variable& var) const;

  /// Returns the max diameter of the box and the associated index .
  std::pair<mpq_class, int> MaxDiam() const;

  /// Bisects the box at @p i -th dimension.
  /// @throws std::runtime if @p i -th dimension is not bisectable.
  std::pair<Box, Box> bisect(int i) const;

  /// Bisects the box at @p the dimension represented by @p var.
  /// @throws std::runtime if @p i -th dimension is not bisectable.
  std::pair<Box, Box> bisect(const Variable& var) const;

  /// Updates the current box by taking union with @p b.
  ///
  /// @pre variables() == b.variables().
  //Box& InplaceUnion(const Box& b);

 private:
  /// Bisects the box at @p i -th dimension.
  /// @pre i-th variable is bisectable.
  /// @pre i-th variable is of integer type.
  std::pair<Box, Box> bisect_int(int i) const;

  /// Bisects the box at @p i -th dimension.
  /// @pre i-th variable is bisectable.
  /// @pre i-th variable is of continuous type.
  std::pair<Box, Box> bisect_continuous(int i) const;

  std::shared_ptr<std::vector<Variable>> variables_;

  IntervalVector values_;

  std::shared_ptr<std::unordered_map<Variable, int, hash_value<Variable>>>
      var_to_idx_;

  std::shared_ptr<std::unordered_map<int, Variable>> idx_to_var_;

  friend std::ostream& operator<<(std::ostream& os, const Box& box);
};

std::ostream& operator<<(std::ostream& os, const Box::Interval& iv);

std::ostream& operator<<(std::ostream& os, const Box& box);

bool operator==(const Box& b1, const Box& b2);

bool operator!=(const Box& b1, const Box& b2);

std::ostream& DisplayDiff(std::ostream& os,
                          const std::vector<Variable>& variables,
                          const Box::IntervalVector& old_iv,
                          const Box::IntervalVector& new_iv);

}  // namespace dreal
