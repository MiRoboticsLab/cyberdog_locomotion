#include "convex_mpc/gait.hpp"

// Offset - Duration Gait
OffsetDurationGait::OffsetDurationGait( int num_segment, Vec4< int > offsets, Vec4< int > durations, const std::string& name )
    : offsets_( offsets.array() ), durations_( durations.array() ), num_iterations_( num_segment ) {

    gait_name_      = name;
    horizon_length_ = 10;
    // allocate memory for MPC gait table
    mpc_table_ = new int[ horizon_length_ * 4 ];

    offsets_float_   = offsets.cast< float >() / ( float )num_segment;
    durations_float_ = durations.cast< float >() / ( float )num_segment;

    for ( int i = 0; i < 4; i++ ) {
        stance_[ i ] = durations[ i ];
        swing_[ i ]  = num_segment - durations[ i ];
    }
}

MixedFrequncyGait::MixedFrequncyGait( int num_segment, Vec4< int > periods, Vec4< float > duty_cycle, const std::string& name ) {
    gait_name_      = name;
    duty_cycle_     = duty_cycle;
    mpc_table_      = new int[ num_segment * 4 ];
    periods_        = periods;
    num_iterations_ = num_segment;
    iteration_      = 0;
    phase_.setZero();
}

OffsetDurationGait::~OffsetDurationGait() {
    delete[] mpc_table_;
}

MixedFrequncyGait::~MixedFrequncyGait() {
    delete[] mpc_table_;
}

Vec4< float > OffsetDurationGait::GetContactState() {
    static Array4f progress;
    progress = phase_ - offsets_float_;

    for ( int i = 0; i < 4; i++ ) {
        if ( progress[ i ] < 0 )
            progress[ i ] += 1.;
        if ( progress[ i ] > durations_float_[ i ] ) {
            progress[ i ] = 0.;
        }
        else {
            progress[ i ] = progress[ i ] / durations_float_[ i ];
        }
    }

    // printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
    return progress.matrix();
}

Vec4< float > MixedFrequncyGait::GetContactState() {
    static Array4f progress;
    progress = phase_;

    for ( int i = 0; i < 4; i++ ) {
        if ( progress[ i ] < 0 )
            progress[ i ] += 1.;
        if ( progress[ i ] > duty_cycle_[ i ] ) {
            progress[ i ] = 0.;
        }
        else {
            progress[ i ] = progress[ i ] / duty_cycle_[ i ];
        }
    }

    // printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
    return progress.matrix();
}

Vec4< float > OffsetDurationGait::GetSwingState() {
    static Array4f swing_offset;
    static Array4f swing_duration;
    static Array4f progress;

    swing_offset = offsets_float_ + durations_float_;
    for ( int i = 0; i < 4; i++ )
        if ( swing_offset[ i ] > 1 )
            swing_offset[ i ] -= 1.;
    swing_duration = 1. - durations_float_;

    progress = phase_ - swing_offset;

    for ( int i = 0; i < 4; i++ ) {
        if ( progress[ i ] < 0 )
            progress[ i ] += 1.f;
        if ( progress[ i ] > swing_duration[ i ] ) {
            progress[ i ] = 0.;
        }
        else {
            if ( fabs( swing_duration[ i ] ) < 1e-5 ) {
                progress[ i ] = 0.0;
            }
            else {
                progress[ i ] = progress[ i ] / swing_duration[ i ];
            }
        }
    }

    // printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
    return progress.matrix();
}

Vec4< float > MixedFrequncyGait::GetSwingState() {
    static Array4f swing_duration;
    static Array4f progress;

    swing_duration = 1.f - duty_cycle_;
    progress       = phase_ - duty_cycle_;
    for ( int i = 0; i < 4; i++ ) {
        if ( progress[ i ] < 0 ) {
            progress[ i ] = 0;
        }
        else {
            if ( fabs( swing_duration[ i ] ) < 1e-5 ) {
                progress[ i ] = 0.0;
            }
            else {
                progress[ i ] = progress[ i ] / swing_duration[ i ];
            }
        }
    }

    // printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
    return progress.matrix();
}

int* OffsetDurationGait::GetMpcTable() {
    static Array4i progress;
    // printf("MPC table:\n");
    for ( int i = 0; i < horizon_length_; i++ ) {
        int iter = ( i + iteration_ + 1 ) % num_iterations_;
        progress = iter - offsets_;
        for ( int j = 0; j < 4; j++ ) {
            if ( progress[ j ] < 0 )
                progress[ j ] += num_iterations_;
            if ( progress[ j ] < durations_[ j ] )
                mpc_table_[ i * 4 + j ] = 1;
            else
                mpc_table_[ i * 4 + j ] = 0;

            // printf("%d ", mpc_table_[i*4 + j]);
        }
        // printf("\n");
    }

    return mpc_table_;
}

int* MixedFrequncyGait::GetMpcTable() {
    // printf("MPC table (%d):\n", iteration_);
    for ( int i = 0; i < num_iterations_; i++ ) {
        for ( int j = 0; j < 4; j++ ) {
            int progress = ( i + iteration_ + 1 ) % periods_[ j ];  // progress
            if ( progress < ( periods_[ j ] * duty_cycle_[ j ] ) ) {
                mpc_table_[ i * 4 + j ] = 1;
            }
            else {
                mpc_table_[ i * 4 + j ] = 0;
            }
            // printf("%d %d (%d %d) | ", mpc_table_[i*4 + j], progress, periods_[j], (int)(periods_[j] * duty_cycle_));
        }

        // printf("%d %d %d %d (%.3f %.3f %.3f %.3f)\n", mpc_table_[i*4], mpc_table_[i*4 + 1], mpc_table_[i*4 + ])
        // printf("\n");
    }
    return mpc_table_;
}

void OffsetDurationGait::SetIterations( int iterationsPerMPC, int current_iteration ) {
    iteration_ = ( current_iteration / iterationsPerMPC ) % num_iterations_;
    phase_     = ( float )( current_iteration % ( iterationsPerMPC * num_iterations_ ) ) / ( float )( iterationsPerMPC * num_iterations_ );
}

void MixedFrequncyGait::SetIterations( int iter_between_mpc, int current_iteration ) {
    iteration_ = ( current_iteration / iter_between_mpc );  // % num_iterations_;
    for ( int i = 0; i < 4; i++ ) {
        int progress_mult = current_iteration % ( iter_between_mpc * periods_[ i ] );
        phase_[ i ]       = ( ( float )progress_mult ) / ( ( float )iter_between_mpc * periods_[ i ] );
        // phase_[i] = (float)(current_iteration % (iter_between_mpc * periods_[i])) / (float) (iter_between_mpc * periods_[i]);
    }

    // printf("phase: %.3f %.3f %.3f %.3f\n", phase_[0], phase_[1], phase_[2], phase_[3]);
}

int OffsetDurationGait::GetCurrentGaitPhase() {
    return iteration_;
}

int MixedFrequncyGait::GetCurrentGaitPhase() {
    return 0;
}

int OffsetDurationGait::GetSegmentNumber() {
    return num_iterations_;
}

int MixedFrequncyGait::GetSegmentNumber() {
    return num_iterations_;
}

float OffsetDurationGait::GetCurrentSwingTime( float dt_mpc, int leg ) {
    ( void )leg;
    return dt_mpc * swing_[ leg ];
}

float MixedFrequncyGait::GetCurrentSwingTime( float dt_mpc, int leg ) {
    return dt_mpc * ( 1. - duty_cycle_[ leg ] ) * periods_[ leg ];
}

float OffsetDurationGait::GetCurrentStanceTime( float dt_mpc, int leg ) {
    ( void )leg;
    return dt_mpc * stance_[ leg ];
}

float MixedFrequncyGait::GetCurrentStanceTime( float dt_mpc, int leg ) {
    return dt_mpc * duty_cycle_[ leg ] * periods_[ leg ];
}

void OffsetDurationGait::DebugPrint() {}

void MixedFrequncyGait::DebugPrint() {}

OffsetDurationContactGait::OffsetDurationContactGait( int num_segment, Vec4< int > offsets, Vec4< int > durations, const std::string& name )
    : offsets_( offsets.array() ), durations_( durations.array() ), num_iterations_( num_segment ) {

    gait_name_ = name;
    // allocate memory for MPC gait table
    mpc_table_ = new int[ num_segment * 4 ];

    offsets_float_   = offsets.cast< float >() / ( float )num_segment;
    durations_float_ = durations.cast< float >() / ( float )num_segment;

    stance_ = durations[ 0 ];
    swing_  = num_segment - durations[ 0 ];
}

OffsetDurationContactGait::~OffsetDurationContactGait() {
    delete[] mpc_table_;
}

Vec4< float > OffsetDurationContactGait::GetContactState() {
    static Array4f progress;

    progress = phase_ - offsets_float_;

    for ( int i = 0; i < 4; i++ ) {
        if ( progress[ i ] < 0 )
            progress[ i ] += 1.;
        if ( progress[ i ] > durations_float_[ i ] ) {
            progress[ i ] = 0.;
        }
        else {
            progress[ i ] = progress[ i ] / durations_float_[ i ];
        }
    }

    // printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
    return progress.matrix();
}

Vec4< float > OffsetDurationContactGait::GetSwingState() {
    static Array4f swing_offset;
    static Array4f swing_duration;
    static Array4f progress;

    swing_offset = offsets_float_ + durations_float_;
    for ( int i = 0; i < 4; i++ )
        if ( swing_offset[ i ] > 1 )
            swing_offset[ i ] -= 1.;
    swing_duration = 1. - durations_float_;

    progress = phase_ - swing_offset;

    for ( int i = 0; i < 4; i++ ) {
        if ( progress[ i ] < 0 )
            progress[ i ] += 1.f;
        if ( progress[ i ] > swing_duration[ i ] ) {
            progress[ i ] = 0.;
        }
        else {
            if ( fabs( swing_duration[ i ] ) < 1e-5 ) {
                progress[ i ] = 0.0;
            }
            else {
                progress[ i ] = progress[ i ] / swing_duration[ i ];
            }
        }
    }

    // printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
    return progress.matrix();
}

void OffsetDurationContactGait::SetIterations( int iterationsPerMPC, int current_iteration ) {
    iteration_ = ( current_iteration / iterationsPerMPC ) % num_iterations_;
    phase_     = ( float )( current_iteration % ( iterationsPerMPC * num_iterations_ ) ) / ( float )( iterationsPerMPC * num_iterations_ );
}

int OffsetDurationContactGait::GetCurrentGaitPhase() {
    return iteration_;
}

int OffsetDurationContactGait::GetSegmentNumber() {
    return num_iterations_;
}

float OffsetDurationContactGait::GetCurrentSwingTime( float dt_mpc, int leg ) {
    ( void )leg;
    return dt_mpc * swing_;
}

float OffsetDurationContactGait::GetCurrentStanceTime( float dt_mpc, int leg ) {
    ( void )leg;
    return dt_mpc * stance_;
}

int* OffsetDurationContactGait::GetMpcTable() {
    static Array4i progress;
    // printf("MPC table:\n");
    for ( int i = 0; i < num_iterations_; i++ ) {
        int iter = ( i + iteration_ + 1 ) % num_iterations_;
        progress = iter - offsets_;
        for ( int j = 0; j < 4; j++ ) {
            if ( progress[ j ] < 0 )
                progress[ j ] += num_iterations_;
            if ( progress[ j ] < durations_[ j ] )
                mpc_table_[ i * 4 + j ] = 1;
            else
                mpc_table_[ i * 4 + j ] = ( contact_[ j ] && i < durations_[ j ] ) ? 1 : 0;

            // printf("%d ", mpc_table_[i*4 + j]);
        }
        // printf("\n");
    }
    return mpc_table_;
}

void OffsetDurationContactGait::SetContact( Vec4< bool > contact ) {
    contact_ = contact;
}

void OffsetDurationContactGait::DebugPrint() {}

NonPeriodicGait::NonPeriodicGait( std::string file_name, int horizon, int iter_between_mpc, const std::string& name ) {

    InitGait( file_name, horizon, iter_between_mpc, name );
}

NonPeriodicGait::~NonPeriodicGait() {
    delete[] mpc_table_;
    delete[] total_mpc_table_;
}

void NonPeriodicGait::InitGait( std::string file_name, int horizon, int iter_between_mpc, const std::string& name ) {
    gait_name_               = name;
    user_define_section_num_ = 0;
    gait_scheduler_section_.setZero();
    gait_scheduler_iteration_ = 0;
    total_mpc_steps_          = 0;
    current_state_.setConstant( 1 );
    init_state_.setConstant( 1 );
    current_iter_     = 0;
    iter_between_mpc_ = iter_between_mpc;
    horizon_length_   = horizon;

    std::string                buffer;
    std::stringstream          file_buffer;
    std::vector< std::string > elems;
    std::vector< int >         data;

    // read file or string
    if ( file_name.substr( file_name.length() - 4, file_name.length() ) == "toml" ) {  // Read file
        std::ifstream file( file_name );
        if ( !file.is_open() ) {
            std::cout << "[Gait] Error opening " << file_name << std::endl;
        }
        file_buffer << file.rdbuf();
        file.clear();
        file.close();
    }
    else {  // read string
        file_buffer << file_name;
        // std::cout << "[Gait] Get file string:" << file_buffer.str() <<std::endl;
    }
    // push all data into a string vector elems
    while ( std::getline( file_buffer, buffer, '\n' ) ) {
        if ( !buffer.empty() ) {
            elems.push_back( buffer );
        }
    }

    // scan the elems to determine total section number and push useful data into a int vector
    user_define_section_num_ = 0;
    for ( int i = 0; i < int( elems.size() ) - 2; i++ ) {
        if ( elems[ i ].substr( 0, 11 ) == "[[section]]" ) {
            for ( int j = 0; j < 4; j++ ) {
                data.push_back( stoi( GetModeString( elems[ i + 1 ], "contact" )[ j ] ) );
            }
            data.push_back( stoi( GetModeString( elems[ i + 2 ], "duration" )[ 0 ] ) );
            user_define_section_num_++;
        }
    }
    if ( data.size() < 5 ) {
        std::cout << "Wrong input file:" << file_name << std::endl;
        for ( int j = 0; j < 5; j++ ) {
            data.push_back( 1 );
        }
        user_define_section_num_ = 1;
    }
    // initialize dynamic matrix and vector based on user_define_section_num_
    user_define_duration_.setZero( user_define_section_num_ );
    user_define_state_.setZero( 4, user_define_section_num_ + 1 );  // It compares i and i + 1 to determine gait_scheduler_section_
    gait_scheduler_duration_.setZero( 4, user_define_section_num_ );
    user_define_state_.block( 0, user_define_section_num_, 4, 1 ) << 0, 0, 0, 0;  // The last zero vector is a sign of end
    // divide data into state and duration
    for ( int i = 0; i < user_define_section_num_; i++ ) {
        for ( int j = 0; j < 4; j++ ) {
            user_define_state_( j, i ) = data[ i * 5 + j ];
            // guarantee fully contact at user_define_section_num_ - 1
            if ( i == user_define_section_num_ - 1 )
                user_define_state_( j, i ) = 1;
        }
        user_define_duration_( i ) = data[ i * 5 + 4 ];
    }
    total_mpc_steps_ = user_define_duration_.sum();
    std::cout << "[Gait] user gait total mpc steps is " << total_mpc_steps_ << std::endl;
    // Compute gait_scheduler_duration_
    for ( int i = 0; i < 4; i++ ) {
        for ( int j = 0; j < user_define_section_num_; j++ ) {
            if ( user_define_state_( i, j ) != user_define_state_( i, j + 1 ) ) {
                if ( gait_scheduler_section_( i ) == 0 ) {
                    gait_scheduler_duration_( i, gait_scheduler_section_( i ) ) = user_define_duration_.segment( 0, j + 1 ).sum();
                    gait_scheduler_section_( i )++;
                }
                else {
                    gait_scheduler_duration_( i, gait_scheduler_section_( i ) ) =
                        user_define_duration_.segment( 0, j + 1 ).sum() - gait_scheduler_duration_.block( i, 0, 1, gait_scheduler_section_( i ) ).sum();
                    gait_scheduler_section_( i )++;
                }
            }
        }
    }
    gait_scheduler_section_.setZero();

    // allocate memory for MPC gait table
    mpc_table_       = new int[ horizon_length_ * 4 ];
    total_mpc_table_ = new int[ total_mpc_steps_ * 4 ];
    // assemble MPC table
    for ( int i = 0; i < 4; i++ ) {
        mpc_table_section_ = 0;
        for ( int j = 0; j < total_mpc_steps_; j++ ) {
            total_mpc_table_[ j * 4 + i ] = user_define_state_( i, mpc_table_section_ );
            if ( j >= user_define_duration_.segment( 0, mpc_table_section_ + 1 ).sum() - 1 ) {
                mpc_table_section_++;
            }
        }
    }

    // // DEBUG
    // std::cout << "user_define_section_num_: " << user_define_section_num_ << std::endl;
    // std::cout << "user_define_duration_: " << user_define_duration_.transpose() << std::endl;
    // std::cout << "user_define_state_: " << std::endl;   std::cout  << user_define_state_ << std::endl;
    // std::cout << "gait_scheduler_duration_: " << std::endl;  std::cout << gait_scheduler_duration_ << std::endl;
    // std::cout << "total_mpc_table_: " << std::endl;
    // for ( int i = 0; i < 4; i++ ) {
    //     for ( int j = 0; j < total_mpc_steps_; j++ ) {
    //         std::cout << total_mpc_table_[ j * 4 + i ] << "  ";
    //     }
    //     std::cout << std::endl;
    // }
}

Vec4< float > NonPeriodicGait::GetContactState() {
    static Vec4< float > progress;
    static Vec4< int >   local_iter;

    progress.setZero();
    local_iter.setZero();

    for ( int i = 0; i < 4; i++ ) {
        if ( gait_scheduler_section_( i ) < 1 ) {
            local_iter( i ) = current_iter_ + 1;
        }
        else {
            local_iter( i ) = current_iter_ + 1 - iter_between_mpc_ * gait_scheduler_duration_.block( i, 0, 1, gait_scheduler_section_( i ) ).sum();
        }

        if ( current_state_( i ) == 0 ) {
            progress( i ) = 0.0;
        }
        else {
            progress( i ) = ( float )local_iter( i ) / ( iter_between_mpc_ * gait_scheduler_duration_( i, gait_scheduler_section_( i ) ) );
        }
    }

    // DEBUG
    // std::cout << "contact_state: " << progress.transpose() << std::endl;

    return progress;
}

Vec4< float > NonPeriodicGait::GetSwingState() {

    static Vec4< float > progress;
    static Vec4< int >   local_iter;

    progress.setZero();
    local_iter.setZero();

    for ( int i = 0; i < 4; i++ ) {
        if ( gait_scheduler_section_( i ) < 1 ) {
            local_iter( i ) = current_iter_ + 1;
        }
        else {
            local_iter( i ) = current_iter_ + 1 - iter_between_mpc_ * gait_scheduler_duration_.block( i, 0, 1, gait_scheduler_section_( i ) ).sum();
        }

        if ( current_state_( i ) == 1 ) {
            progress( i ) = 0.0;
        }
        else {
            progress( i ) = ( float )local_iter( i ) / ( iter_between_mpc_ * gait_scheduler_duration_( i, gait_scheduler_section_( i ) ) );
        }
    }

    // DEBUG
    // std::cout << "swing_state: " << progress.transpose() << std::endl;

    return progress;
}

void NonPeriodicGait::SetIterations( int iter_between_mpc, int current_iteration ) {
    current_iter_     = current_iteration;
    iter_between_mpc_ = iter_between_mpc;

    // Reset gait_scheduler_section_
    if ( current_iter_ == 0 ) {
        gait_scheduler_section_.setZero();
        current_state_ = user_define_state_.block( 0, 0, 4, 1 );
        init_state_    = user_define_state_.block( 0, 0, 4, 1 );
    }
    // Hold current_iter_
    if ( current_iter_ >= total_mpc_steps_ * iter_between_mpc_ - 1 ) {
        current_iter_ = total_mpc_steps_ * iter_between_mpc_ - 1;
    }

    gait_scheduler_iteration_ = floor( current_iter_ / iter_between_mpc_ );
    for ( int i = 0; i < 4; i++ ) {
        if ( gait_scheduler_iteration_ >= gait_scheduler_duration_.block( i, 0, 1, gait_scheduler_section_( i ) + 1 ).sum() ) {
            gait_scheduler_section_( i )++;
            current_state_( i ) = fmod( gait_scheduler_section_( i ) + init_state_( i ), 2 );  // init_state_ determines the first current_state_
            if ( gait_scheduler_section_( i ) >= user_define_section_num_ - 1 ) {
                current_state_( i ) = 1;
            }
        }
    }

    // DEBUG
    // std::cout << "gait_scheduler_iteration_: " << gait_scheduler_iteration_ << std::endl;
    // std::cout << "current_iter_: " << current_iter_ << std::endl;
    // std::cout << "gait_scheduler_section_: " << gait_scheduler_section_.transpose() << std::endl;
    // std::cout << "current_state_: " << current_state_.transpose() << std::endl;
}

int NonPeriodicGait::GetCurrentGaitPhase() {
    return gait_scheduler_iteration_;
}

int NonPeriodicGait::GetSegmentNumber() {
    return num_iterations_;
}

float NonPeriodicGait::GetCurrentSwingTime( float dt_mpc, int leg ) {
    ( void )leg;

    if ( current_state_( leg ) == 1 ) {
        return 0.0;
    }
    else {
        return dt_mpc * gait_scheduler_duration_( leg, gait_scheduler_section_( leg ) );
    }
}

float NonPeriodicGait::GetCurrentStanceTime( float dt_mpc, int leg ) {
    ( void )leg;

    if ( current_state_( leg ) == 0 ) {
        return 0.0;
    }
    else {
        return dt_mpc * gait_scheduler_duration_( leg, gait_scheduler_section_( leg ) );
    }
}

int* NonPeriodicGait::GetMpcTable() {

    for ( int i = 0; i < 4; i++ ) {
        for ( int j = 0; j < horizon_length_; j++ ) {
            if ( gait_scheduler_iteration_ + horizon_length_ <= total_mpc_steps_ ) {
                mpc_table_[ j * 4 + i ] = total_mpc_table_[ ( gait_scheduler_iteration_ + j + 1 ) * 4 + i ];
            }
            else {
                mpc_table_[ j * 4 + i ] = 1;
            }
        }
    }

    // DEBUG
    // std::cout << "mpc_table_: " << std::endl;
    // for ( int i = 0; i < 4; i++ ) {
    //     for ( int j = 0; j < horizon_length_; j++ ) {
    //         std::cout <<  mpc_table_[ j * 4 + i ] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    return mpc_table_;
}

int NonPeriodicGait::GetTotalDuration() {
    return total_mpc_steps_ * iter_between_mpc_;
}

std::vector< std::string > NonPeriodicGait::SplitString( const std::string& str, const std::string& pattern ) {
    char* strc = new char[ strlen( str.c_str() ) + 1 ];
    strcpy( strc, str.c_str() );
    std::vector< std::string > resultVec;
    char*                      tmpStr = strtok( strc, pattern.c_str() );
    while ( tmpStr != NULL ) {
        resultVec.push_back( std::string( tmpStr ) );
        tmpStr = strtok( NULL, pattern.c_str() );
    }
    delete[] strc;
    return resultVec;
}

std::vector< std::string > NonPeriodicGait::GetModeString( const std::string& str, const std::string& target ) {
    // std::cout <<"\n get0  input: "<<str <<" target:" << target << std::endl;
    std::string value = "";
    if ( str.find( target ) != std::string::npos ) {
        if ( str.find( "[" ) == std::string::npos ) {
            std::vector< std::string > resultVec = SplitString( str, "=" );
            resultVec.erase( resultVec.begin() );
            // std::cout << "return1:" << resultVec[0] << std::endl;
            return resultVec;
        }
        value = SplitString( str, "[" )[ 1 ];
        value = SplitString( value, "]" )[ 0 ];
    }
    // std::cout << "return2:  " << SplitString(value,",")[0] << std::endl;
    return SplitString( value, "," );
}

