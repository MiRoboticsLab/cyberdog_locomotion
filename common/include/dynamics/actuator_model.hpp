#ifndef ACTUATOR_MODEL_HPP_
#define ACTUATOR_MODEL_HPP_

#include "utilities/utilities.hpp"

/**
 * @brief A model of an actuator containing friction and electrical effects.
 *
 */
template < typename T > class ActuatorModel {
public:
    /**
     * @brief Construct a new actuator model with the given parameters.
     *
     * @param gear_ratio Gear reduction
     * @param motor_KT Value of KT (torque constant) for the motor
     * @param motor_R Motor resistance
     * @param battery_V Battery voltage
     * @param damping Actuator damping (at the joint, Nm/(rad/sec))
     * @param dry_friction Actuator dry friction (at the joint, Nm)
     * @param tau_max Maximum torque output of the actuator
     * @param n1 Maximun speed (rad/s) at Maximun torque
     * @param n2 Maximun speed (rad/s) at accelerate
     * @param n3 Maximun speed (rad/s) at deceleration
     */
    ActuatorModel( T gear_ratio, T motor_KT, T motor_R, T battery_V, T damping, T dry_friction, T tau_max, T n1, T n2, T n3 )
        : gr_( gear_ratio ), kt_( motor_KT ), R_( motor_R ), V_( battery_V ), damping_( damping ), dry_friction_( dry_friction ), tau_max_( tau_max ), n_1_( n1 ), n_2_( n2 ), n_3_( n3 ) {}

    /**
     * @brief Construct a new actuator model.
     *
     */
    ActuatorModel() {}

    /**
     * @brief Compute actual actuator torque, given desired torque and speed.
     *        takes into account friction (dry and damping), voltage limits, and torque
     *        limits
     *
     * @param tau_des Desired torque
     * @param qd Current actuator velocity (at the joint)
     * @return Actual produced torque
     */
    T GetAcuatorTorque( T tau_des, T qd ) {
        // compute motor torque
        T tau_des_motor = tau_des / gr_;                  // motor torque
        T i_des         = tau_des_motor / ( kt_ * 1.5 );  // i = tau / KT
        // T bemf =  qd * gr_ * kt_ * 1.732;     // back emf
        T bemf          = qd * gr_ * kt_ * 2.;                   // back emf
        T v_des         = i_des * R_ + bemf;                     // v = I*R + emf
        T v_actual      = WrapRange( v_des, -V_, V_ );           // limit to battery voltage
        T tau_act_motor = 1.5 * kt_ * ( v_actual - bemf ) / R_;  // tau = Kt * I = Kt * V / R
        T tau_act       = gr_ * WrapRange( tau_act_motor, -tau_max_, tau_max_ );
        // TN curve
        if ( qd < -n_3_ ) {
            tau_act = 0;
            printf( "actuator speed less than min\n" );
        }

        if ( qd >= -n_3_ && qd < -n_2_ ) {
            if ( tau_act > gr_ * tau_max_ ) {
                tau_act = gr_ * tau_max_;
            }
            if ( tau_act < 0 ) {
                tau_act = 0;
            }
        }

        if ( qd >= -n_2_ && qd < -n_1_ ) {
            T tau_act_min = ( gr_ * tau_max_ ) / ( n_1_ - n_2_ ) * qd - ( -( ( gr_ * tau_max_ ) * n_2_ ) / ( n_1_ - n_2_ ) );
            if ( tau_act > gr_ * tau_max_ ) {
                tau_act = gr_ * tau_max_;
            }
            if ( tau_act < tau_act_min ) {
                tau_act = tau_act_min;
            }
        }

        if ( qd >= -n_1_ && qd < n_1_ ) {
            if ( tau_act > gr_ * tau_max_ ) {
                tau_act = gr_ * tau_max_;
            }
            if ( tau_act < -gr_ * tau_max_ ) {
                tau_act = -gr_ * tau_max_;
            }
        }

        if ( qd >= n_1_ && qd < n_2_ ) {
            T tau_act_max = ( gr_ * tau_max_ ) / ( n_1_ - n_2_ ) * qd + ( -( ( gr_ * tau_max_ ) * n_2_ ) / ( n_1_ - n_2_ ) );
            if ( tau_act > tau_act_max ) {
                tau_act = tau_act_max;
            }
            if ( tau_act < -gr_ * tau_max_ ) {
                tau_act = -gr_ * tau_max_;
            }
        }

        if ( qd >= n_2_ && qd < n_3_ ) {
            if ( tau_act < -gr_ * tau_max_ ) {
                tau_act = -gr_ * tau_max_;
            }
            if ( tau_act > 0 ) {
                tau_act = 0;
            }
        }

        if ( qd >= n_3_ ) {
            tau_act = 0;
            printf( "actuator speed over max\n" );
        }
        return tau_act;
    }

    /**
     * @brief Calculates the joint torque based on the desired torque and joint velocity.
     *
     * @param tau_des Desired torque
     * @param qd Current actuator velocity (at the joint)
     * @return The actual joint torque
     */
    T GetJointTorque( T tau_des, T qd ) {
        T tau_act = GetAcuatorTorque( tau_des, qd );
        if ( friction_enabled_ )
            tau_act = tau_act - damping_ * qd - dry_friction_ * MathSign( qd );
        return tau_act;
    }

    /**
     * @brief Control friction effects.
     *
     * @param enabled Enable/disable both dry and damping friction terms
     */
    void SetFriction( bool enabled ) {
        friction_enabled_ = enabled;
    }

private:
    T    gr_, kt_, R_, V_, damping_, dry_friction_, tau_max_, n_1_, n_2_, n_3_;
    bool friction_enabled_ = true;
};

#endif  // ACTUATOR_MODEL_HPP_