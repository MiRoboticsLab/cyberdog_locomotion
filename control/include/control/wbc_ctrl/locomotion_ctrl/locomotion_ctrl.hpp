#ifndef LOCOMOTION_CTRL_HPP_
#define LOCOMOTION_CTRL_HPP_

#include "wbc_ctrl/wbc_ctrl.hpp"

template < typename T > struct LocomotionCtrlData {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit LocomotionCtrlData() {
        body_rpy_des.setZero();
    }

    Vec3< T > body_pos_des;
    Vec3< T > body_vel_des;
    Vec3< T > body_acc_des;
    Vec3< T > body_rpy_des;
    Vec3< T > body_omg_des;

    Vec3< T > foot_pos_des[ 4 ];
    Vec3< T > foot_vel_des[ 4 ];
    Vec3< T > foot_acc_des[ 4 ];
    Vec3< T > reaction_force_des[ 4 ];

    Vec4< T > contact_state;
};

template < typename T > class LocomotionCtrl : public WbcCtrl< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LocomotionCtrl( FloatingBaseModel< T > model );
    virtual ~LocomotionCtrl();
    void SetFriction( const T& mu );
    void SetTaskPD( const Vec3< T >& kp_body, const Vec3< T >& kd_body, const Vec3< T >& kp_ori, const Vec3< T >& kd_ori, const Vec3< T >& kp_foot, const Vec3< T >& kd_foot, const Vec3< T >& kp_joint,
                    const Vec3< T >& kd_joint );

protected:
    virtual void ContactTaskUpdate( void* input, ControlFsmData< T >& data );
    virtual void ContactTaskUpdateTEST( void* input, ControlFsmData< T >& data );
    void         ParameterSetup( const UserParameters* param );
    void         ParameterSetup();
    void         CleanUp();
    virtual void LcmPublishData( ControlFsmData< T >& data );

    bool task_pd_set_outside_ = false;

    LocomotionCtrlData< T >* input_data_;

    Task< T >* body_pos_task_;
    Task< T >* body_ori_task_;

    Task< T >*        foot_task_[ 4 ];
    ContactSpec< T >* foot_contact_[ 4 ];

    Vec3< T > kp_body_, kd_body_, kp_ori_, kd_ori_, kp_foot_, kd_foot_, kp_joint_, kd_joint_;

    Vec3< T > reaction_force_result_[ 4 ];
    Quat< T > quat_des_;
};

#endif  // LOCOMOTION_CTRL_HPP_
