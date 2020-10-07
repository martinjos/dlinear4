#pragma once

#include <chrono>
#include <iostream>
#include <type_traits>
#include <cstdint>

namespace dreal {

/// Simple timer class to profile performance.
template <class T>
class TimerBase {
 public:
  using clock = T;
  typedef typename clock::duration duration;
  typedef typename clock::time_point time_point;

  TimerBase();

  /// Starts the timer.
  void start();

  /// Pauses the timer.
  void pause();

  /// Resumes the timer.
  void resume();

  /// Checks if the timer is running.
  bool is_running() const;

  /// Returns the elapsed time as duration.
  duration elapsed() const;

  /// Returns the elapsed time in secionds.
  std::chrono::duration<double>::rep seconds() const;

 protected:
  time_point now() const { return clock::now(); }

 private:
  // Whether the timer is running or not.
  bool running_{false};

  // Last time_point when the timer is started or resumed.
  time_point last_start_{};

  // Elapsed time so far. This doesn't include the current fragment if
  // it is running.
  duration elapsed_{};
};

template<class T>
std::ostream& operator<<(std::ostream& os, const TimerBase<T>& timer);

// Use high_resolution clock if it's steady. Otherwise use steady_clock.
using chosen_steady_clock = std::conditional<std::chrono::high_resolution_clock::is_steady,
                                             std::chrono::high_resolution_clock,
                                             std::chrono::steady_clock>::type;

extern template class TimerBase<chosen_steady_clock>;
class Timer : public TimerBase<chosen_steady_clock> {};

struct user_clock {  // Implements the Clock interface of std::chrono
  typedef uint64_t rep;
  typedef std::micro period;
  typedef std::chrono::duration<rep, period> duration;
  typedef std::chrono::time_point<user_clock> time_point;
  const bool is_steady = false;  // Not sure how this should be interpreted here
  static time_point now();
};

extern template class TimerBase<user_clock>;
class UserTimer : public TimerBase<user_clock> {};

/// Pauses the passed timer object when the guard object is destructed
/// (e.g. going out of scope).
class TimerGuard {
 public:
  /// Constructs the timer guard object with @p timer.
  ///
  /// If @p enabled is false, this class does not do anything.
  /// If @p start_timer is true, starts the @p timer in the
  /// constructor. Otherwise, it does not start it and a user has to
  /// call `resume()` to start it.
  TimerGuard(Timer* timer, bool enabled, bool start_timer = true);

  TimerGuard(const TimerGuard&) = delete;
  TimerGuard(TimerGuard&&) = delete;
  TimerGuard& operator=(const TimerGuard&) = delete;
  TimerGuard& operator=(TimerGuard&&) = delete;


  /// Destructs the timer guard object. It pauses the embedded timer object.
  ~TimerGuard();

  /// Pauses the embedded timer object.
  void pause();

  /// Resumes the embedded timer object.
  void resume();

 private:
  Timer* const timer_;
  const bool enabled_{false};
};

extern UserTimer main_timer;

}  // namespace dreal
