#include <cmath>
#include <string>
#include <unistd.h>
#ifdef linux
#include <sys/stat.h>
#include <sys/timerfd.h>
#endif

#include "utilities/periodic_task.hpp"
#include "utilities/timer.hpp"
#include "utilities/utilities_print.hpp"

/**
 * @brief Construct a new Periodic Task:: Periodic Task object
 *
 * @param task_manager Parent task manager
 * @param period duration of one loop
 * @param name name of task
 */
PeriodicTask::PeriodicTask( PeriodicTaskManager* task_manager, float period, std::string name ) : period_( period ), task_name_( name ) {
    task_manager->AddTask( this );
    // if ( task_name_ == "robot-control" || task_name_ == "spi" ) {
    //     struct stat st;
    //     if ( stat( "/tmp/log", &st ) == -1 ) {
    //         mkdir( "/tmp/log", 0700 );
    //     }
    //     task_log_file_.open( "/tmp/log/time_log_" + name + ".txt" );
    //     if ( !task_log_file_.is_open() ) {
    //         std::cout << "Failed to open period log file time_log.txt, exit" << std::endl;
    //         exit( -1 );
    //     }
    // }
}

/**
 * @brief Begin running task
 *
 */
void PeriodicTask::StartTask() {
    if ( running_ ) {
        printf( "[PeriodicTask] Tried to start %s but it was already running!\n", task_name_.c_str() );
        return;
    }
    InitTask();
    running_ = true;
    // Create independent thread
    thread_ = std::thread( &PeriodicTask::LoopFunction, this );
    // Set thread priority
    static int         policy;
    static sched_param sch_params;
    // static pid_t       cur_pid = getpid();
    // set priority for process
    // sch_params.sched_priority = sched_get_priority_max( SCHED_RR );
    // std::cout << "[PeriodicTask] Check max prority  " << sch_params.sched_priority << std::endl;
    // sched_setscheduler( cur_pid, SCHED_RR, &sch_params );
    if ( task_name_ == "spi" || task_name_ == "robot-control" ) {
        policy                    = SCHED_RR;
        sch_params.sched_priority = sched_get_priority_max( policy );
        pthread_setschedparam( thread_.native_handle(), policy, &sch_params );
        int rc = pthread_getschedparam( thread_.native_handle(), &policy, &sch_params );
        std::cout << "[PeriodicTask] Check pthread  " << task_name_ << "   " << rc << "   " << policy << "   " << sch_params.sched_priority << std::endl;
    }
}

/**
 * @brief Stop running task
 *
 */
void PeriodicTask::StopTask() {
    if ( !running_ ) {
        printf( "[PeriodicTask] Tried to stop %s but it wasn't running!\n", task_name_.c_str() );
        return;
    }
    running_ = false;
    printf( "[PeriodicTask] Waiting for %s to stop...\n", task_name_.c_str() );
    thread_.join();
    printf( "[PeriodicTask] Done!\n" );
    CleanUp();
}

/**
 * @brief If max period is more than 30% over desired period, it is slow
 *
 * @return true
 * @return false
 */
bool PeriodicTask::IsSlow() {
    return maximum_period_ > period_ * 1.3f || maximum_running_time_ > period_;
}

/**
 * @brief Reset max statistics
 *
 */
void PeriodicTask::ClearMaximumValue() {
    maximum_period_       = 0;
    maximum_running_time_ = 0;
}

/**
 * @brief Print the status of this task in the table format
 *
 */
void PeriodicTask::PrintfStatus() {
    if ( !running_ )
        return;
    if ( IsSlow() ) {
        PrintfColor( PrintColor::kRed, "|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f\n", task_name_.c_str(), last_running_time_, maximum_running_time_, period_, last_period_, maximum_period_ );
    }
    else {
        printf( "|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f\n", task_name_.c_str(), last_running_time_, maximum_running_time_, period_, last_period_, maximum_period_ );
    }
}

/**
 * @brief Call the task in a timed loop using a timerfd
 *
 */
void PeriodicTask::LoopFunction() {
#ifdef linux
    auto timerFd = timerfd_create( CLOCK_MONOTONIC, 0 );
#endif
    int seconds     = ( int )period_;
    int nanoseconds = ( int )( 1e9 * std::fmod( period_, 1.f ) );

    Timer t;

#ifdef linux
    itimerspec timerSpec;
    timerSpec.it_interval.tv_sec  = seconds;
    timerSpec.it_value.tv_sec     = seconds;
    timerSpec.it_value.tv_nsec    = nanoseconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;

    timerfd_settime( timerFd, 0, &timerSpec, nullptr );
#endif

    printf( "[PeriodicTask] Start %s (%d s, %d ns)\n", task_name_.c_str(), seconds, nanoseconds );
    while ( running_ ) {
        last_period_ = ( float )t.GetElapsedSeconds();
        t.StartTimer();
        RunTask();
        last_running_time_ = ( float )t.GetElapsedSeconds();

#ifdef linux
        unsigned long long missed = 0;
        int                m      = read( timerFd, &missed, sizeof( missed ) );
        ( void )m;
        if ( task_log_file_ ) {
            task_log_file_ << last_period_ << " " << last_running_time_ << std::endl;
        }
#endif
        maximum_period_       = std::max( maximum_period_, last_period_ );
        maximum_running_time_ = std::max( maximum_running_time_, last_running_time_ );
    }
    printf( "[PeriodicTask] %s has stopped!\n", task_name_.c_str() );
}

/**
 * @brief Destroy the Periodic Task Manager:: Periodic Task Manager object
 *
 */
PeriodicTaskManager::~PeriodicTaskManager() {}

/**
 * @brief Add a new task to a task manager
 *
 * @param task defined PeriodicTask
 */
void PeriodicTaskManager::AddTask( PeriodicTask* task ) {
    tasks_.push_back( task );
}

/**
 * @brief Print the status of all tasks and rest max statistics
 *
 */
void PeriodicTaskManager::PrintfStatus() {
    printf( "\n----------------------------TASKS----------------------------\n" );
    printf( "|%-20s|%-6s|%-6s|%-6s|%-6s|%-6s\n", "name", "rt", "rt-max", "T-des", "T-act", "T-max" );
    printf( "-----------------------------------------------------------\n" );
    for ( auto& task : tasks_ ) {
        task->PrintfStatus();
        task->ClearMaximumValue();
    }
    printf( "-------------------------------------------------------------\n\n" );
}

/**
 * @brief Print the status of only the slow tasks
 *
 */
void PeriodicTaskManager::PrintStatusOfSlowTasks() {
    for ( auto& task : tasks_ ) {
        if ( task->IsSlow() ) {
            task->PrintfStatus();
            task->ClearMaximumValue();
        }
    }
}

/**
 * @brief Stop all tasks
 *
 */
void PeriodicTaskManager::StopAllTasks() {
    for ( auto& task : tasks_ ) {
        task->StopTask();
    }
}
