#ifndef PREJUMP_CTRL_HPP_
#define PREJUMP_CTRL_HPP_

#include "data_reader_ctrl.hpp"
#include "data_reader.hpp"
#include "controllers/leg_controller.hpp"

/**
 * @brief Controller of prejump executed before WBC jump in crawljump
 *
 */
template < typename T > class PreJumpCtrl : public DataReaderCtrl< T > {
public:
    PreJumpCtrl( DataReader*, float dt );
    virtual ~PreJumpCtrl();
    // Decide the command based on current_time
    virtual void OneStep( float current_time, bool b_preparation, LegControllerCommand< T >* command );
    Vec12< T >   q_init_, q_cur_;

protected:
    // Be called in OneStep()
    void UpdateJointCommand();
};

#endif  // PREJUMP_CTRL_HPP_