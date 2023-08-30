#ifndef BODY_ORIENTATION_PITCH_YAW_TASK_HPP_
#define BODY_ORIENTATION_PITCH_YAW_TASK_HPP_

#include "wbc/task.hpp"

template < typename T > class FloatingBaseModel;

/**
 * @brief Construct subtask of body orientation, contains two dimensions that is pitch and yaw.
 *
 */
template < typename T > class BodyRyRzTask : public Task< T > {
public:
    BodyRyRzTask( const FloatingBaseModel< T >* );
    virtual ~BodyRyRzTask();

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

#endif  // BODY_ORIENTATION_PITCH_YAW_TASK_HPP_
