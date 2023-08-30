#ifndef BODY_POSITION_TASK_HPP_
#define BODY_POSITION_TASK_HPP_

#include "wbc/task.hpp"

template < typename T > class FloatingBaseModel;

/**
 * @brief Construct subtask of body position, contains three dimensions.
 *
 */
template < typename T > class BodyPosTask : public Task< T > {
public:
    BodyPosTask( const FloatingBaseModel< T >* );
    virtual ~BodyPosTask();

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

#endif  // BODY_POSITION_TASK_HPP_
