#ifndef FRONTJUMP_CTRL_HPP_
#define FRONTJUMP_CTRL_HPP_

#include "data_reader_ctrl.hpp"
#include "data_reader.hpp"
#include "controllers/leg_controller.hpp"
#include "dynamics/floating_base_model.hpp"

/**
 * @brief Controller of frontjump
 * Frontjump could be  divided into 5 stages:
 * Stage1: Four-leg on stance
 * Stage2: Rear-leg on stance, Front-leg  on flight
 * Stage3: Four-leg on flight
 * Stage4: Rear-leg on flight, Front-leg supported by obstacle
 * Stage5: Four-leg on flight, overcoming the obstacle
 */
template < typename T > class FrontJumpCtrl : public DataReaderCtrl< T > {
public:
    FrontJumpCtrl( DataReader*, float dt );
    virtual ~FrontJumpCtrl();
    // decide the command based on current_time 
    virtual void OneStep( float current_time, bool b_preparation, LegControllerCommand< T >* command );
    Vec12< T >   q_init_, q_cur_;

protected:
    // called in OneStep function
    void UpdateJointCommand();
};

#endif // FRONTJUMP_CTRL_HPP_
