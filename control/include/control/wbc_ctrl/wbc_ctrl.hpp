#ifndef WBC_CTRL_HPP_
#define WBC_CTRL_HPP_

#include <lcm/lcm-cpp.hpp>

#include "cpp_types.hpp"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "fsm_states/control_fsm_data.hpp"
#include "header/lcm_type/wbc_test_data_t.hpp"
#include "wbc/wbic/kin_wbc.hpp"
#include "wbc/wbic/wbic.hpp"

#define WBCtrl WbcCtrl< T >

class UserParameters;

/**
 * @brief Whole Body Control
 *
 */
template < typename T > class WbcCtrl {
public:
    WbcCtrl( FloatingBaseModel< T > model );
    virtual ~WbcCtrl();

    void Run( void* input, ControlFsmData< T >& data, const bool& use_absolute_dom = false );
    void RunVirtual( void* input, ControlFsmData< T >& data );

    virtual void SetFriction( const T& mu ) {
        ( void )( mu );
    }

    virtual void SetTaskPD( const Vec3< T >& kp_body, const Vec3< T >& kd_body, const Vec3< T >& kp_ori, const Vec3< T >& kd_ori, const Vec3< T >& kp_foot, const Vec3< T >& kd_foot,
                            const Vec3< T >& kp_joint, const Vec3< T >& kd_joint ) = 0;

    void SetFloatingBaseWeight( const T& weight ) {
        wbic_data_->body_pose_weight_ = DVec< T >::Constant( 6, weight );
    }

    void SetFloatingBaseWeight( const DVec< T >& weight_vec ) {
        for ( int i = 0; i < 6; i++ )
            wbic_data_->body_pose_weight_[ i ] = weight_vec[ i ];
    }

protected:
    virtual void ContactTaskUpdate( void* input, ControlFsmData< T >& data ) = 0;
    virtual void ContactTaskUpdateTEST( void* input, ControlFsmData< T >& data ) {
        ( void )input;
        ( void )data;
    }
    virtual void LcmPublishData( ControlFsmData< T >& data ) {
        ( void )data;
    }
    void UpdateModel( const StateEstimatorResult< T >& state_est, const LegControllerData< T >* leg_data, const bool& use_absolute_dom = false );
    void UpdateLegCmd( ControlFsmData< T >& data );
    void ComputeWbc();

    KinWbc< T >*        kin_wbc_;
    WBIC< T >*          wbic_;
    WbicExtraData< T >* wbic_data_;

    FloatingBaseModel< T >           model_;
    std::vector< ContactSpec< T >* > contact_list_;
    std::vector< Task< T >* >        task_list_;

    DMat< T > A_;
    DMat< T > Ainv_;
    DVec< T > grav_;
    DVec< T > coriolis_;

    FBModelState< T > state_;

    DVec< T > full_config_;
    DVec< T > tau_ff_;
    DVec< T > des_jpos_;
    DVec< T > des_jvel_;

    std::vector< T > kp_joint_, kd_joint_;

    unsigned long long iter_;

    lcm::LCM        wbc_lcm_;
    wbc_test_data_t wbc_data_lcm_;
};
#endif  // WBC_CTRL_HPP_