#ifndef BODY_ORIENTATION_TASK_HPP_
#define BODY_ORIENTATION_TASK_HPP_

#include "wbc/task.hpp"

template < typename T > class FloatingBaseModel;

/**
 * @brief Construct subtask of body orientation, contains three dimension that presented by quaternions.
 *
 */
template < typename T > class BodyOriTask : public Task< T > {
public:
    BodyOriTask( const FloatingBaseModel< T >* );
    virtual ~BodyOriTask();

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

#endif  // BODY_ORIENTATION_TASK_HPP_
