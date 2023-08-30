#ifndef BODY_ORIENTATION_LOCAL_ROLL_TASK_HPP_
#define BODY_ORIENTATION_LOCAL_ROLL_TASK_HPP_

#include "wbc/task.hpp"

template < typename T > class FloatingBaseModel;

/**
 * @brief Construct subtask of body orientation, contains one dimensions thai is roll.
 *
 */
template < typename T > class LocalRollTask : public Task< T > {
public:
    LocalRollTask( const FloatingBaseModel< T >* );
    virtual ~LocalRollTask();

    DVec< T > kp, kd;

protected:
    // Update op_cmd_
    virtual bool UpdateCommand( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des );
    // Update Jt_
    virtual bool UpdateTaskJacobian();
    // Update JtDotQdot_
    virtual bool UpdateTaskJDotQdot();
    virtual bool AdditionalUpdate() {
        return true;
    }

    int                           link_index_;
    bool                          virtual_depend_;
    const FloatingBaseModel< T >* robot_dynamics_;
};

#endif  // BODY_ORIENTATION_LOCAL_ROLL_TASK_HPP_
