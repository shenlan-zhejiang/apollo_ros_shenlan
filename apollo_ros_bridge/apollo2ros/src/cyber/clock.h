#ifndef CYBER_TIME_CLOCK_H_
#define CYBER_TIME_CLOCK_H_

#include <mutex>

#include "run_mode_conf.pb.h"

#include "atomic_rw_lock.h"
#include "macros.h"
#include "time.h"

namespace apollo {
namespace cyber {

using ::apollo::cyber::proto::ClockMode;

/**
 * @class Clock
 * @brief a singleton clock that can be used to get the current
 * timestamp. The source can be either system(cyber) clock or a mock clock.
 * Mock clock is for testing purpose mainly.
 */
class Clock {
 public:
  static constexpr int64_t PRECISION =
      std::chrono::system_clock::duration::period::den /
      std::chrono::system_clock::duration::period::num;

  /// PRECISION >= 1000000 means the precision is at least 1us.
  static_assert(PRECISION >= 1000000,
                "The precision of the system clock should be at least 1 "
                "microsecond.");

  /**
   * @brief get current time.
   * @return a Time object representing the current time.
   */
  static Time Now();

  /**
   * @brief gets the current time in second.
   * @return the current time in second.
   */
  static double NowInSeconds();

  /**
   * @brief This is for mock clock mode only. It will set the timestamp
   * for the mock clock.
   */
  static void SetNow(const Time& now);

  /**
   * @brief Set the behavior of the \class Clock.
   * @param The new clock mode to be set.
   */
  static void SetMode(ClockMode mode);

  /**
   * @brief Gets the current clock mode.
   * @return The current clock mode.
   */
  static ClockMode mode();

  /**
   * @brief This is for mock clock mode only. It will set the timestamp
   * for the mock clock with UNIX timestamp in seconds.
   */
  static void SetNowInSeconds(const double seconds) {
    Clock::SetNow(Time(seconds));
  }

 private:
  ClockMode mode_;
  Time mock_now_;
  ::apollo::cyber::base::AtomicRWLock rwlock_;

  DECLARE_SINGLETON(Clock)
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIME_CLOCK_H_
