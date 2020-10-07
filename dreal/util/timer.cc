#include "dreal/util/timer.h"

#include <stdexcept>
#include <sys/time.h>
#include <sys/resource.h>

namespace dreal {

using std::ostream;
using std::runtime_error;

template <class T>
TimerBase<T>::TimerBase() : last_start_{now()} {}

template <class T>
void TimerBase<T>::start() {
  last_start_ = now();
  elapsed_ = duration{0};
  running_ = true;
}

template <class T>
void TimerBase<T>::pause() {
  if (running_) {
    running_ = false;
    elapsed_ += (now() - last_start_);
  }
}

template <class T>
void TimerBase<T>::resume() {
  if (!running_) {
    last_start_ = now();
    running_ = true;
  }
}

template <class T>
bool TimerBase<T>::is_running() const { return running_; }

template <class T>
typename TimerBase<T>::duration TimerBase<T>::elapsed() const {
  if (running_) {
    return elapsed_ + (now() - last_start_);
  } else {
    return elapsed_;
  }
}

template <class T>
std::chrono::duration<double>::rep TimerBase<T>::seconds() const {
  // double representation of seconds.
  using seconds_in_double = std::chrono::duration<double>;
  return std::chrono::duration_cast<seconds_in_double>(elapsed()).count();
}

template <class T>
ostream& operator<<(ostream& os, const TimerBase<T>& timer) {
  return os << timer.seconds() << "s";
}

user_clock::time_point user_clock::now() {
  struct rusage usage;
  if (0 != getrusage(RUSAGE_SELF, &usage)) {
    throw runtime_error("Failed to get current resource usage (getrusage)");
  }
  return time_point(duration(uint64_t(usage.ru_utime.tv_sec) * std::micro::den
                           + uint64_t(usage.ru_utime.tv_usec)));
}

// Explicit instantiations
template class TimerBase<chosen_steady_clock>;
template class TimerBase<user_clock>;

TimerGuard::TimerGuard(Timer* const timer, const bool enabled,
                       const bool start_timer)
    : timer_{timer}, enabled_{enabled} {
  if (enabled_) {
    if (start_timer) {
      timer_->resume();
    }
  }
}

TimerGuard::~TimerGuard() {
  if (enabled_) {
    timer_->pause();
  }
}

void TimerGuard::pause() {
  if (enabled_) {
    timer_->pause();
  }
}

void TimerGuard::resume() {
  if (enabled_) {
    timer_->resume();
  }
}

UserTimer main_timer;

}  // namespace dreal
