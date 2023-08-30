#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <assert.h>
#include <stdint.h>
#include <time.h>

/**
 * @brief Timer for measuring elapsed time based on clock_monotonic
 *
 */
class Timer {
public:
    /**
     * @brief Construct and StartTimer timer
     *
     */
    explicit Timer() {
        StartTimer();
    }

    /**
     * @brief Start the timer
     *
     */
    void StartTimer() {
        clock_gettime( CLOCK_MONOTONIC, &start_time_ );
    }

    /**
     * @brief Get elapsed time, unit is milliseconds
     *
     * @return double
     */
    double GetElapsedMilliseconds() {
        return ( double )GetElapsedNanoseconds() / 1.e6;
    }

    /**
     * @brief Get elapsed time, unit is nanoseconds
     *
     * @return int64_t
     */
    int64_t GetElapsedNanoseconds() {
        struct timespec now;
        clock_gettime( CLOCK_MONOTONIC, &now );
        return ( int64_t )( now.tv_nsec - start_time_.tv_nsec ) + 1000000000 * ( now.tv_sec - start_time_.tv_sec );
    }

    /**
     * @brief Get elapsed time, unit is seconds
     *
     * @return double
     */
    double GetElapsedSeconds() {
        return ( double )GetElapsedNanoseconds() / 1.e9;
    }

private:
    struct timespec start_time_;
};

#endif  // TIMER_HPP_
