#ifndef JOINT_POSITION_TASK_HPP_
#define JOINT_POSITION_TASK_HPP_

#include "wbc/task.hpp"

template < typename T > class FloatingBaseModel;

/**
 * @brief Construct subtask of joints, contains twelve dimensions.
 *
 */
template < typename T > class JPosTask : public Task< T > {
public:
    JPosTask( const FloatingBaseModel< T >* );
    virtual ~JPosTask();

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
    const FloatingBaseModel< T >* robot_sys_;
};

#endif  // JOINT_POSITION_TASK_HPP_
