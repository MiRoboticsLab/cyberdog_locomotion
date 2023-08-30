#ifndef LINK_POSITION_TASK_HPP_
#define LINK_POSITION_TASK_HPP_

#include "wbc/task.hpp"

template < typename T > class FloatingBaseModel;

/**
 * @brief Construct subtask of foot position, contains three dimensions.
 *
 */
template < typename T > class LinkPosTask : public Task< T > {
public:
    LinkPosTask( const FloatingBaseModel< T >*, int link_idx, bool virtual_depend = true );
    virtual ~LinkPosTask();

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
    int                           link_index_;
    bool                          virtual_depend_;
};

#endif  // LINK_POSITION_TASK_HPP_
