#ifndef TASK_HPP_
#define TASK_HPP_

#include "cpp_types.hpp"

#define TK Task< T >

/**
 * @brief task parent class of whole body controller, other specific tasks should inherit from this   
 * 
 */
template < typename T > class Task {
public:
    Task( size_t dim ) : set_task_flag_( false ), dim_task_( dim ), op_cmd_( dim ), pos_err_( dim ), vel_des_( dim ), acc_des_( dim ) {
        op_cmd_.setZero();
        pos_err_.setZero();
        vel_des_.setZero();
        acc_des_.setZero();
    }

    virtual ~Task() {}

    void GetCommand( DVec< T >& op_cmd ) {
        op_cmd = op_cmd_;
    }

    void GetTaskJacobian( DMat< T >& Jt ) {
        Jt = Jt_;
    }

    void GetTaskJacobianDotQdot( DVec< T >& JtDotQdot ) {
        JtDotQdot = JtDotQdot_;
    }

    bool UpdateTask( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des ) {
        UpdateTaskJacobian();
        UpdateTaskJDotQdot();
        UpdateCommand( pos_des, vel_des, acc_des );
        AdditionalUpdate();
        set_task_flag_ = true;
        return true;
    }

    bool IsTaskSet() {
        return set_task_flag_;
    }

    size_t GetDimension() {
        return dim_task_;
    }

    void UnsetTask() {
        set_task_flag_ = false;
    }

    const DVec< T >& GetPosError() {
        return pos_err_;
    }

    const DVec< T >& GetDesVel() {
        return vel_des_;
    }

    const DVec< T >& GetDesAcc() {
        return acc_des_;
    }

protected:
    // Update op_cmd_
    virtual bool UpdateCommand( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des ) = 0;

    // Update Jt_
    virtual bool UpdateTaskJacobian() = 0;

    // Update JtDotQdot_
    virtual bool UpdateTaskJDotQdot() = 0;
    
    // Additional Update (defined in child classes)
    virtual bool AdditionalUpdate() = 0;

    bool   set_task_flag_;
    size_t dim_task_;

    DVec< T > op_cmd_;
    DVec< T > JtDotQdot_;
    DMat< T > Jt_;

    DVec< T > pos_err_;
    DVec< T > vel_des_;
    DVec< T > acc_des_;
};

#endif  // TASK_HPP_