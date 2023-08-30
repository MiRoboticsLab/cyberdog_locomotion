#ifndef OFFLINE_DATA_CTRL_HPP_
#define OFFLINE_DATA_CTRL_HPP_

#include "Configuration.h"
#include "controllers/leg_controller.hpp"
#include "data_reader.hpp"
#include "data_reader_ctrl.hpp"
#include "dynamics/floating_base_model.hpp"
#include "wbc_ctrl/locomotion_ctrl/locomotion_ctrl.hpp"

template < typename T > struct OfflineOptCtrlData {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit OfflineOptCtrlData() {
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

/**
 * @brief Controller for junmp3d
 * Data contains wbc and joint contoller information
 * Five-link, SRBM, SRBM_Kine can be used
 */
template < typename T > class OfflineDataCtrl : public DataReaderCtrl< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OfflineDataCtrl( DataReader*, float dt );
    virtual ~OfflineDataCtrl();
    virtual void OneStep( float current_time, bool b_preparation, LegControllerCommand< T >* command );
    virtual void OfflineDataCtrlRun( float current_time, bool b_preparation, OfflineOptCtrlData< T >* offline_opt_data, int pre_mode_duration );
    int          GetCurrentIteration();
    int          GetDataEndFlag();
    Vec3< T >    GetJointPosDesire( int leg_num );
    Vec3< T >    GetJointVelDesire( int leg_num );
    Vec3< T >    GetJointTorqueFeedforward( int leg_num );
    Vec3< T >    GetReactionForceDesire( int leg_num );
    void         SetBodyInit( Vec3< T > body_pos_init, Vec3< T > body_rpy_init );

protected:
    Vec3< T >   body_pos_init_;
    Vec3< T >   body_rpy_init_;
    RotMat< T > body_rot_matrix_init_;
    Vec3< T >   joint_pos_des_[ 4 ];
    Vec3< T >   joint_vel_des_[ 4 ];
    Vec3< T >   joint_torque_feedforward_[ 4 ];
    Vec3< T >   body_pos_des_;
    Vec3< T >   body_vel_des_;
    Vec3< T >   body_acc_des_;
    Vec3< T >   body_rpy_des_;
    Vec3< T >   body_omg_des_;
    Vec3< T >   foot_pos_des_[ 4 ];
    Vec3< T >   foot_vel_des_[ 4 ];
    Vec3< T >   foot_acc_des_[ 4 ];
    Vec3< T >   reaction_force_des_[ 4 ];
    Vec4< T >   contact_state_;
    int         pre_mode_duration_ = 100;
    bool        data_end_;  // whether it comes to end of data

    void UpdateRobotRef();
};

#endif  // OFFLINE_DATA_CTRL_HPP_
