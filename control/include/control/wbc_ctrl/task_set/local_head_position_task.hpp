#ifndef LOCAL_HEAD_POSITION_TASK_HPP_
#define LOCAL_HEAD_POSITION_TASK_HPP_

#include "wbc/task.hpp"

template < typename T > class FloatingBaseModel;

/**
 * @brief Construct subtask of the head position, contains three dimensions.
 *
 */
template < typename T > class LocalHeadPosTask : public Task< T > {
public:
    LocalHeadPosTask( const FloatingBaseModel< T >* );
    virtual ~LocalHeadPosTask();

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

#endif  // LOCAL_HEAD_POSITION_TASK_HPP_
