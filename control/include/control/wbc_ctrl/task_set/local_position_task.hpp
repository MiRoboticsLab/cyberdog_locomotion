#ifndef LOCAL_POSITION_TASK_HPP_
#define LOCAL_POSITION_TASK_HPP_

#include "wbc/task.hpp"

template < typename T > class FloatingBaseModel;

/**
 * @brief Construct subtask of the link position, contains three dimensions.
 *
 */
template < typename T > class LocalPosTask : public Task< T > {
public:
    LocalPosTask( const FloatingBaseModel< T >*, int link_idx, int local_frame_idx );
    virtual ~LocalPosTask();

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
    int                           _link_idx;
    int                           _local_frame_idx;
};

#endif  // LOCAL_POSITION_TASK_HPP_
