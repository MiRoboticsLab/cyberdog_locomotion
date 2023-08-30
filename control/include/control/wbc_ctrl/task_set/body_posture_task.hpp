#ifndef BODY_POSTURE_TASK_HPP_
#define BODY_POSTURE_TASK_HPP_

#include "wbc/task.hpp"

template < typename T > class FloatingBaseModel;

/**
 * @brief Construct subtask of body orientation, contains six dimensions that is quaternion and position.
 *
 */
template < typename T > class BodyPostureTask : public Task< T > {
public:
    BodyPostureTask( const FloatingBaseModel< T >* );
    virtual ~BodyPostureTask();

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

    const FloatingBaseModel< T >* robot_dynamics_;
};

#endif  // BODY_POSTURE_TASK_HPP_
