#ifndef GAIT_HPP_
#define GAIT_HPP_

#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>

#include "cpp_types.hpp"
#include "offline_optimization_controller/data_reader.hpp"

class Gait {
public:
    virtual ~Gait() = default;

    virtual Vec4< float > GetContactState()                                            = 0;
    virtual Vec4< float > GetSwingState()                                              = 0;
    virtual int*          GetMpcTable()                                                = 0;
    virtual void          SetIterations( int iter_between_mpc, int current_iteration ) = 0;
    virtual float         GetCurrentStanceTime( float dt_mpc, int leg )                = 0;
    virtual float         GetCurrentSwingTime( float dt_mpc, int leg )                 = 0;
    virtual int           GetCurrentGaitPhase()                                        = 0;
    virtual int           GetSegmentNumber()                                           = 0;
    virtual void          DebugPrint() {}

protected:
    std::string gait_name_;
};

using Eigen::Array4f;
using Eigen::Array4i;

class OffsetDurationGait : public Gait {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OffsetDurationGait( int num_segment, Vec4< int > offset, Vec4< int > durations, const std::string& name );
    ~OffsetDurationGait();
    Vec4< float > GetContactState();
    Vec4< float > GetSwingState();
    int*          GetMpcTable();
    void          SetIterations( int iter_between_mpc, int current_iteration );
    float         GetCurrentStanceTime( float dt_mpc, int leg );
    float         GetCurrentSwingTime( float dt_mpc, int leg );
    int           GetCurrentGaitPhase();
    int           GetSegmentNumber();
    void          DebugPrint();

private:
    int*    mpc_table_;
    Array4i offsets_;          // offset in mpc segments
    Array4i durations_;        // duration of step in mpc segments
    Array4f offsets_float_;    // offsets in phase (0 to 1)
    Array4f durations_float_;  // durations in phase (0 to 1)
    Array4f stance_;
    Array4f swing_;
    int     iteration_;
    int     num_iterations_;  // same as num_segment
    float   phase_;           // increase during moc ( 0 to 1 )
    int     horizon_length_;  // mpc horizon length
};

class MixedFrequncyGait : public Gait {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MixedFrequncyGait( int num_segment, Vec4< int > periods, Vec4< float > duty_cycle, const std::string& name );
    ~MixedFrequncyGait();
    Vec4< float > GetContactState();
    Vec4< float > GetSwingState();
    int*          GetMpcTable();
    void          SetIterations( int iter_between_mpc, int current_iteration );
    float         GetCurrentStanceTime( float dt_mpc, int leg );
    float         GetCurrentSwingTime( float dt_mpc, int leg );
    int           GetCurrentGaitPhase();
    int           GetSegmentNumber();
    void          DebugPrint();

private:
    Array4f duty_cycle_;
    int*    mpc_table_;
    Array4i periods_;
    Array4f phase_;
    int     iteration_;
    int     num_iterations_;
};

class OffsetDurationContactGait : public Gait {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OffsetDurationContactGait( int num_segment, Vec4< int > offset, Vec4< int > durations, const std::string& name );
    ~OffsetDurationContactGait();
    Vec4< float > GetContactState();
    Vec4< float > GetSwingState();
    int*          GetMpcTable();
    void          SetIterations( int iter_between_mpc, int current_iteration );
    float         GetCurrentStanceTime( float dt_mpc, int leg );
    float         GetCurrentSwingTime( float dt_mpc, int leg );
    int           GetCurrentGaitPhase();
    int           GetSegmentNumber();
    void          SetContact( Vec4< bool > contact );
    void          DebugPrint();

private:
    int*         mpc_table_;
    Array4i      offsets_;          // offset in mpc segments
    Array4i      durations_;        // duration of step in mpc segments
    Array4f      offsets_float_;    // offsets in phase (0 to 1)
    Array4f      durations_float_;  // durations in phase (0 to 1)
    int          stance_;
    int          swing_;
    int          iteration_;
    int          num_iterations_;
    float        phase_;
    Vec4< bool > contact_;
};

class NonPeriodicGait : public Gait {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    NonPeriodicGait( std::string file_name, int horizon, int iter_between_mpc, const std::string& name );
    ~NonPeriodicGait();
    void          InitGait( std::string file_name, int horizon, int iter_between_mpc, const std::string& name );
    Vec4< float > GetContactState();
    Vec4< float > GetSwingState();
    int*          GetMpcTable();
    void          SetIterations( int iter_between_mpc, int current_iteration );
    float         GetCurrentStanceTime( float dt_mpc, int leg );
    float         GetCurrentSwingTime( float dt_mpc, int leg );
    int           GetCurrentGaitPhase();
    int           GetSegmentNumber();
    int           GetTotalDuration();

private:
    DVec< int > user_define_duration_;      // store user define duration
    DMat< int > user_define_state_;         // store user define contact state
    int         user_define_section_num_;   // compute total section number of user define file/string
    DMat< int > gait_scheduler_duration_;   // determine successive state number for each leg
    Vec4< int > gait_scheduler_section_;    // used act as a counter in scanning gait_scheduler_duration_
    int         gait_scheduler_iteration_;  // convert WBC iteration to MPC iteration
    int         total_mpc_steps_;           // sum of user_define_duration_
    Vec4< int > current_state_;             // current contact state
    Vec4< int > init_state_;                // the first contact state
    float       current_iter_;
    int         iter_between_mpc_;
    int         horizon_length_;

    int* mpc_table_;
    int* total_mpc_table_;
    int  mpc_table_section_;
    int  num_iterations_;

    std::vector< std::string > SplitString( const std::string& str, const std::string& pattern );
    std::vector< std::string > GetModeString( const std::string& str, const std::string& target );
};


#endif  // GAIT_HPP_