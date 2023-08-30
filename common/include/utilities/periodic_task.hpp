#ifndef PERIODIC_TASK_HPP_
#define PERIODIC_TASK_HPP_

#include <fstream>
#include <pthread.h>
#include <string>
#include <thread>
#include <vector>

class PeriodicTaskManager;

/**
 * @brief Implementation of a periodic function running in a separate thread.
 * Periodic tasks have a task manager, which measure how long they take to run.
 *
 */
class PeriodicTask {
public:
    PeriodicTask( PeriodicTaskManager* task_manager, float period, std::string name );
    void         StartTask();
    void         StopTask();
    void         PrintfStatus();
    void         ClearMaximumValue();
    bool         IsSlow();
    virtual void InitTask() = 0;
    virtual void RunTask()  = 0;
    virtual void CleanUp()  = 0;
    virtual ~PeriodicTask() {
        StopTask();
    }

    /**
     * @brief Return the desired period of the task
     *
     * @return float
     */
    float GetPeriod() {
        return period_;
    }

    /**
     * @brief Return running time that the RunTask() of last loop took
     *
     * @return float
     */
    float GetRuntime() {
        return last_running_time_;
    }

    /**
     * @brief Return the maximum period that recent loops took
     *
     * @return float
     */
    float GetMaxPeriod() {
        return maximum_period_;
    }

    /**
     * @brief Return the maximum running time that the RunTask() of recent loops took
     *
     * @return float
     */
    float GetMaxRuntime() {
        return maximum_running_time_;
    }

private:
    void LoopFunction();

    float         period_;
    volatile bool running_              = false;
    float         last_running_time_    = 0;
    float         last_period_          = 0;
    float         maximum_period_       = 0;
    float         maximum_running_time_ = 0;
    std::string   task_name_;
    std::thread   thread_;

    std::ofstream task_log_file_;
};

/**
 * @brief A collection of periodic tasks which can be monitored together
 *
 */
class PeriodicTaskManager {
public:
    PeriodicTaskManager() : tasks_(){};
    ~PeriodicTaskManager();
    void AddTask( PeriodicTask* task );
    void PrintfStatus();
    void PrintStatusOfSlowTasks();
    void StopAllTasks();

private:
    std::vector< PeriodicTask* > tasks_;
};

/**
 * @brief A periodic task for calling a function
 *
 */
class PeriodicFunction : public PeriodicTask {
public:
    PeriodicFunction( PeriodicTaskManager* task_manager, float period, std::string name, void ( *function )() ) : PeriodicTask( task_manager, period, name ), function_( function ) {}
    void CleanUp() {}
    void InitTask() {}
    void RunTask() {
        function_();
    }

    ~PeriodicFunction() {
        StopTask();
    }

private:
    void ( *function_ )() = nullptr;
};

/**
 * @brief A periodic task for printing the status of all tasks in the task manager
 *
 */
class PrintTaskStatus : public PeriodicTask {
public:
    PrintTaskStatus( PeriodicTaskManager* task_manager, float period ) : PeriodicTask( task_manager, period, "print-tasks" ), task_manager_( task_manager ) {}
    void RunTask() override {
        // DH: Disable printing
        // task_manager_->PrintfStatus();
    }

    void InitTask() override {}

    void CleanUp() override {}

private:
    PeriodicTaskManager* task_manager_;
};

/**
 * @brief A periodic task for calling a member function
 *
 */
template < typename T > class PeriodicMemberFunction : public PeriodicTask {
public:
    PeriodicMemberFunction( PeriodicTaskManager* task_manager, float period, std::string name, void ( T::*function )(), T* object )
        : PeriodicTask( task_manager, period, name ), function_( function ), object_( object ) {}

    void CleanUp() {}
    void InitTask() {}
    void RunTask() {
        ( object_->*function_ )();
    }

private:
    void ( T::*function_ )();
    T* object_;
};

#endif  // PERIODIC_TASK_HPP_
