#ifndef CONTROL_PARAMETERS_HPP_
#define CONTROL_PARAMETERS_HPP_

#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include "c_types.h"
#include "param_helper.hpp"
#include "parameters.hpp"
#include "utilities/utilities.hpp"

#define kCONTROL_PARAMETER_MAXIMUM_NAME_LENGTH 64

#define INIT_PARAMETER( name ) param_##name( #name, name, collection_ )
#define DECLARE_PARAMETER( type, name ) \
    type             name;              \
    ControlParameter param_##name;

#define kMAX_SIZE_IN_DOUBLE 12

/**
 * @brief Data types supported for control parameters
 *
 */
enum class ControlParameterValueKind : u64 {
    kDOUBLE       = 1,
    kS64          = 2,
    kVEC_X_DOUBLE = 3,  // for template type derivation
    kMAT_X_DOUBLE = 4   // for template type derivation
};

std::string ControlParameterValueKindToString( ControlParameterValueKind valueKind );

/**
 * @brief Pointer to control parameter data
 *
 */
union ControlParameterValuePtr {
    double* d;
    s64*    i;
    double* vecXd;
    double* matXd;
};

/**
 * @brief Value of a control parameter, maximum length is 12 double
 *
 */
union ControlParameterValue {

    ControlParameterValue() {
        memset( this, 0, sizeof( ControlParameterValue ) );
    }

    ControlParameterValue( const ControlParameterValue& value ) {
        memcpy( this, &value, sizeof( ControlParameterValue ) );
    }

    ControlParameterValue& operator=( const ControlParameterValue& others ) {
        memcpy( this, &others, sizeof( ControlParameterValue ) );
        return *this;
    }

    double d;
    s64    i;
    double vecXd[ kMAX_SIZE_IN_DOUBLE ];
    double matXd[ kMAX_SIZE_IN_DOUBLE ];
};

std::string ControlParameterValueToString( ControlParameterValue v, ControlParameterValueKind kind );

class ControlParameter;

/**
 * @brief ControlParameterCollections contains a map of all the control parameters.
 *
 */
class ControlParameterCollection {
public:
    explicit ControlParameterCollection( const std::string& name ) : name_( name ) {}

    /**
     * @brief Use this to add a parameter for the first time in the
     * RobotControlParameters or SimulatorControlParameters. This should only be
     * used during initialization of a ControlParameter.
     *
     * Throws exception if you try to add a parameter twice.
     *
     * @param param
     * @param name
     */
    void AddParameter( ControlParameter* param, const std::string& name ) {
        if ( DoesMapContains( map_, name ) ) {
            printf( "[ERROR] ControlParameterCollection %s: tried to add parameter %s "
                    "twice!\n",
                    name_.c_str(), name.c_str() );
            throw std::runtime_error( "Control parameter error [" + name_ + "]: parameter " + name + " appears twice!" );
        }
        map_[ name ] = param;
    }

    /**
     * @brief Lookup a control parameter by its name.
     * This does not modify the set field of the control parameter!
     *
     * Throws exception if parameter isn't found
     *
     * @param name
     * @return ControlParameter&
     */
    ControlParameter& LookUp( const std::string& name ) {
        if ( DoesMapContains( map_, name ) ) {
            return *map_[ name ];
        }
        else {
            throw std::runtime_error( "Control parameter " + name + " wasn't found in parameter collection " + name_ );
        }
    }

    std::string PrintToYamlString();  //!< print all control parameters in the
                                      //!< YAML file format
    bool CheckIfAllSet();             //!< are all the control parameters initialized?
    void ClearAllSet();
    void DeleteAll();

    std::map< std::string, ControlParameter* > map_;

private:
    std::string name_;
};

/**
 * @brief A single control parameter. Note that this representation doesn't store the value, just a pointer
 * to an existing variable.
 *
 */
class ControlParameter {
public:
    /**
     * @brief Construct control parameter for a double
     *
     * @param name : name of parameter
     * @param value : reference to value
     * @param collection : collection to add to
     * @param units : name of units
     */
    ControlParameter( const std::string& name, double& value, ControlParameterCollection& collection, const std::string& units = "" ) {
        name_ = name;
        TruncateName();
        units_   = units;
        value_.d = &value;
        kind_    = ControlParameterValueKind::kDOUBLE;
        collection.AddParameter( this, name );
    }

    /**
     * @brief Construct control parameter for an s64
     *
     * @param name : name of parameter
     * @param value : reference to value
     * @param collection : collection to add to
     * @param units : name of units
     */
    ControlParameter( const std::string& name, s64& value, ControlParameterCollection& collection, const std::string& units = "" ) {
        name_ = name;
        TruncateName();
        units_   = units;
        value_.i = &value;
        kind_    = ControlParameterValueKind::kS64;
        collection.AddParameter( this, name );
    }

    /**
     * @brief Construct control parameter for a list of Rows doubles
     *
     * @param name : name of parameter
     * @param value : reference to value
     * @param collection : collection to add to
     * @param units : name of units
     */
    template < int Rows > ControlParameter( const std::string& name, VecX< double, Rows >& value, ControlParameterCollection& collection, const std::string& units = "" ) {
        name_ = name;
        TruncateName();
        units_       = units;
        value_.vecXd = value.data();  // TODO
        kind_        = ControlParameterValueKind::kVEC_X_DOUBLE;
        rows_        = Rows;
        cols_        = 1;
        static_assert( Rows <= kMAX_SIZE_IN_DOUBLE, "Parameter size must smaller than MAX_SIZE_IN_DOUBLE" );
        collection.AddParameter( this, name );
    }

    /**
     * @brief  Construct control parameter for a fixed size matrix
     *
     * @param name : name of parameter
     * @param value : reference to value
     * @param collection : collection to add to
     * @param units : name of units
     */
    template < int Rows, int Cols > ControlParameter( const std::string& name, Eigen::Matrix< double, Rows, Cols >& value, ControlParameterCollection& collection, const std::string& units = "" ) {
        name_ = name;
        TruncateName();
        units_       = units;
        value_.matXd = value.data();
        kind_        = ControlParameterValueKind::kMAT_X_DOUBLE;
        rows_        = Rows;
        cols_        = Cols;
        static_assert( Rows <= kMAX_SIZE_IN_DOUBLE, "Parameter size must smaller than MAX_SIZE_IN_DOUBLE" );
        collection.AddParameter( this, name );
    }

    /**
     * @brief Construct control parameter for a given type without
     * adding it to a collection
     *
     * @param name : name of parameter
     * @param kind : type of data to be stored
     */
    ControlParameter( const std::string& name, ControlParameterValueKind kind, int rows = 1, int cols = 1 ) {
        name_ = name;
        TruncateName();
        kind_ = kind;
        if ( kind_ == ControlParameterValueKind::kMAT_X_DOUBLE || kind_ == ControlParameterValueKind::kVEC_X_DOUBLE ) {
            rows_ = rows;
            cols_ = cols;
            assert( rows * cols <= kMAX_SIZE_IN_DOUBLE );
        }
        value_.vecXd = ( double* )&static_value_;  // assign a value buff
    }

    /**
     * @brief Make sure that the control parameter name is short enough to fit
     * in the control parameter request message.
     */
    void TruncateName() {
        if ( name_.length() > kCONTROL_PARAMETER_MAXIMUM_NAME_LENGTH ) {
            printf( "[Error] control parameter name %s is too long, shortening to ", name_.c_str() );
            name_.resize( kCONTROL_PARAMETER_MAXIMUM_NAME_LENGTH - 1 );
            printf( "%s\n", name_.c_str() );
        }
    }

    /**
     * @brief Set initial value of the control parameter.
     * Checks to see that the types are correct
     */
    void InitializeDouble( double d ) {
        if ( kind_ != ControlParameterValueKind::kDOUBLE ) {
            throw std::runtime_error( "Tried to initialize control parameter " + name_ + " as a double!" );
        }
        set_      = true;
        *value_.d = d;
    }

    /**
     * @brief  Set initial value of the control parameter.
     * Checks to see that the types are correct
     */
    void InitializeInteger( s64 i ) {
        if ( kind_ != ControlParameterValueKind::kS64 ) {
            throw std::runtime_error( "Tried to initialize control parameter " + name_ + " as an integer!" );
        }
        set_      = true;
        *value_.i = i;
    }

    void InitializeMatXd( const std::vector< std::vector< double > >& mat ) {
        if ( kind_ != ControlParameterValueKind::kMAT_X_DOUBLE ) {
            throw std::runtime_error( "Tried to initialize control parameter " + name_ + " as a vectorNd" );
        }
        if ( !Check2DArraySize( mat, rows_, cols_ ) ) {
            throw std::runtime_error( StringFormat( "MatXd initialize row&col mismatch, expect: (%d, %d), ", rows_, cols_ ) );
        }
        set_ = true;
        for ( int j = 0; j < cols_; j++ ) {
            for ( int i = 0; i < rows_; i++ ) {
                value_.matXd[ j * rows_ + i ] = mat[ i ][ j ];
            }
        }
    }

    /**
     * @brief  Set initial value of the control parameter.
     * Checks to see that the types are correct
     */
    void InitializeVecXd( const std::vector< double >& v ) {
        if ( kind_ != ControlParameterValueKind::kVEC_X_DOUBLE ) {
            throw std::runtime_error( "Tried to initialize control parameter " + name_ + " as a vector3d" );
        }
        if ( ( int )v.size() != rows_ ) {
            throw std::runtime_error( StringFormat( "VecXd initialize row mismatch, expect %d, "
                                                    "got %d",
                                                    rows_, ( int )v.size() ) );
        }
        set_ = true;
        for ( int i = 0; i < rows_; i++ ) {
            value_.vecXd[ i ] = v[ i ];
        }
    }

    /**
     * @brief Set a control parameter by value.
     * Performs type checking
     *
     * @param value : value to set
     * @param kind : kind of the value
     */
    void Set( ControlParameterValue value, ControlParameterValueKind kind ) {
        if ( kind != kind_ ) {
            throw std::runtime_error( "Control parameter type mismatch in set" );
        }
        switch ( kind ) {
        case ControlParameterValueKind::kDOUBLE:
            *value_.d = value.d;
            break;
        case ControlParameterValueKind::kS64:
            *value_.i = value.i;
            break;
        case ControlParameterValueKind::kVEC_X_DOUBLE:
            for ( int i = 0; i < rows_; i++ ) {
                value_.vecXd[ i ] = value.vecXd[ i ];
            }
            break;
        case ControlParameterValueKind::kMAT_X_DOUBLE:
            for ( int t = 0; t < rows_ * cols_; t++ ) {
                value_.matXd[ t ] = value.matXd[ t ];
            }
            break;
        default:
            throw std::runtime_error( "Control parameter invalid kind in set" );
        }
        set_ = true;
    }

    /**
     * @brief Get the value of a control parameter.  Does type checking - you must provide
     * the correct type.
     *
     * @param kind : the kind of the control parameter
     * @return the value of the control parameter
     */
    ControlParameterValue Get( ControlParameterValueKind kind ) {
        ControlParameterValue value;
        if ( kind != kind_ ) {
            throw std::runtime_error( "Control parameter type mismatch in get" );
        }
        switch ( kind_ ) {
        case ControlParameterValueKind::kDOUBLE:
            value.d = *value_.d;
            break;
        case ControlParameterValueKind::kS64:
            value.i = *value_.i;
            break;
        case ControlParameterValueKind::kVEC_X_DOUBLE:
            for ( int t = 0; t < rows_; t++ ) {
                value.vecXd[ t ] = value_.vecXd[ t ];
            }
            break;
        case ControlParameterValueKind::kMAT_X_DOUBLE:
            for ( int t = 0; t < rows_ * cols_; t++ ) {
                value.matXd[ t ] = value_.matXd[ t ];
            }
            break;
        default:
            throw std::runtime_error( "Control parameter invalid kind in get" );
        }
        return value;
    }

    /**
     * @brief Convert the value to a string that works in a YAML file
     */
    std::string ToString() {
        std::string                  result;
        Eigen::IOFormat              CommaVecFmt( Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]" );
        Eigen::IOFormat              CommaMatFmt( Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "[", "]", "[", "]" );
        Eigen::Map< DMat< double > > mat( value_.matXd, rows_, cols_ );
        Eigen::Map< DVec< double > > vec( value_.vecXd, rows_ );
        std::stringstream            ss;
        switch ( kind_ ) {
        case ControlParameterValueKind::kDOUBLE:
            result += NumberToString( *value_.d );
            break;
        case ControlParameterValueKind::kS64:
            result += std::to_string( *value_.i );
            break;
        case ControlParameterValueKind::kVEC_X_DOUBLE:
            ss << vec.format( CommaVecFmt );
            result += ss.str();
            break;
        case ControlParameterValueKind::kMAT_X_DOUBLE:
            ss << mat.format( CommaMatFmt );
            result += ss.str();
            break;
        default:
            result += "<unknown type " + std::to_string( ( u32 )( kind_ ) ) + "> (add it yourself in control_parameters.hpp!)";
            break;
        }
        return result;
    }

    /**
     * @brief Set the value of a control parameter from a string.
     * Numbers are just numbers.
     * Vec3's are like [1,2,3]
     *
     * @param value : the string representing the value
     * @return if the set was successful
     */
    bool SetFromString( const std::string& value ) {
        switch ( kind_ ) {
        case ControlParameterValueKind::kDOUBLE:
            *value_.d = std::stod( value );
            break;
        case ControlParameterValueKind::kS64:
            *value_.i = std::stoll( value );
            break;
        case ControlParameterValueKind::kVEC_X_DOUBLE: {
            auto vv = loadVectorFromString< double >( value );
            assert( ( int )vv.size() == rows_ );
            // Eigen store matrix by col-major
            for ( int i = 0; i < rows_; i++ ) {
                value_.vecXd[ i ] = vv[ i ];
            }
        } break;
        case ControlParameterValueKind::kMAT_X_DOUBLE: {
            auto mm = loadMatFromString< double >( value );
            assert( Check2DArraySize( mm, rows_, cols_ ) );
            // Eigen store matrix by col-major
            for ( int j = 0; j < cols_; j++ ) {
                for ( int i = 0; i < rows_; i++ ) {
                    value_.matXd[ j * rows_ + i ] = mm[ i ][ j ];
                }
            }
        } break;
        default:
            return false;
        }
        return true;
    }

    void GetSize( int& rows, int& cols ) {
        rows = rows_;
        cols = cols_;
    }

    bool                      set_ = false;
    ControlParameterValuePtr  value_;
    std::string               name_;
    std::string               units_;
    ControlParameterValueKind kind_;
    ControlParameterValue     static_value_;

    // for Eigen Mat parameters
    int rows_ = 1;
    int cols_ = 1;

private:
};

/**
 * @brief Parent class for groups of parameters
 * RobotParameters and SimulatorParameters inherit from this class.
 * This will track if all parameters are initialized so you are sure
 * that the robot has received all parameters before starting
 */
class ControlParameters {
public:
    /**
     * @brief Construct a control parameter group
     *
     * @param name : Each control parameter group must have a unique name so the ini files don't
     * mixed up
     */
    ControlParameters( const std::string& name ) : collection_( name ), name_( name ) {}

    virtual ~ControlParameters() {}

    /**
     * @brief If true, all parameters have been initialized in one way or another
     */
    bool IsFullyInitialized() {
        return collection_.CheckIfAllSet();
    }

    /**
     * @brief Lock the access mutex. Control parameters are shared between UI, LCM, simulator, and robot threads
     */
    void LockMutex() {
        mutex_.lock();
    }

    /**
     * @brief Unlock the access mutex. Control parameters are shared between UI, LCM, simulator, and robot threads
     */
    void UnlockMutex() {
        mutex_.unlock();
    }

    void InitializeFromYamlFile( const std::string& path );
    void DefineAndInitializeFromYamlFile( const std::string& path );

    void WriteToYamlFile( const std::string& path );

    std::string GenerateUnitializedList();

    ControlParameterCollection collection_;

protected:
    std::string name_;
    std::mutex  mutex_;
};

#endif  // CONTROL_PARAMETERS_HPP_
