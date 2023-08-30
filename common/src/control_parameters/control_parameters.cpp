#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "ParamHandler.hpp"
#include "control_parameters/control_parameters.hpp"
#include "utilities/utilities.hpp"

using Type = YAML::NodeType;

/**
 * @brief Convert control parameter data types into strings
 *
 */
std::string ControlParameterValueKindToString( ControlParameterValueKind valueKind ) {
    switch ( valueKind ) {
    case ControlParameterValueKind::kS64:
        return "s64";
    case ControlParameterValueKind::kDOUBLE:
        return "double";
    case ControlParameterValueKind::kVEC_X_DOUBLE:
        return "vecXd";
    case ControlParameterValueKind::kMAT_X_DOUBLE:
        return "matXd";
    default:
        return "unknown-ControlParameterValueKind";
    }
}

/**
 * @brief Convert a control parameter value into a human readable string.
 *
 */
std::string ControlParameterValueToString( ControlParameterValue value, ControlParameterValueKind kind ) {
    std::string                                       result;
    Eigen::IOFormat                                   CommaVecFmt( Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]" );
    Eigen::Map< VecX< double, kMAX_SIZE_IN_DOUBLE > > vec( value.vecXd );
    std::stringstream                                 ss;
    switch ( kind ) {
    case ControlParameterValueKind::kDOUBLE:
        result += NumberToString( value.d );
        break;
    case ControlParameterValueKind::kS64:
        result += std::to_string( value.i );
        break;
    case ControlParameterValueKind::kVEC_X_DOUBLE:  // Vec&Mat share same to string method
    case ControlParameterValueKind::kMAT_X_DOUBLE:  // We may change the way this function used
        ss << vec.format( CommaVecFmt );
        result += ss.str();
        break;
    default:
        result += "<unknown type " + std::to_string( ( u32 )( kind ) ) + "> (add it yourself in control_parameter_interface.hpp!)";
        break;
    }
    return result;
}

/**
 * @brief Check if all parameters have had their value set
 *
 * @return if all set
 */
bool ControlParameterCollection::CheckIfAllSet() {
    for ( auto& kv : map_ ) {
        if ( !kv.second->set_ ) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Mark all parameters as not set
 *
 */
void ControlParameterCollection::ClearAllSet() {
    for ( auto& kv : map_ ) {
        kv.second->set_ = false;
    }
}

/**
 * @brief Remove all parameters
 *
 */
void ControlParameterCollection::DeleteAll() {
    for ( auto& kv : map_ ) {
        delete kv.second;
    }
    map_.clear();
}

/**
 * @brief Write a list of uninitialized parameters to a string
 *
 */
std::string ControlParameters::GenerateUnitializedList() {
    std::string result;
    for ( auto& kv : collection_.map_ ) {
        if ( !kv.second->set_ ) {
            result += kv.second->name_ + "    :\n";
        }
    }

    return result;
}

/**
 * @brief Print all parameters to a string which can be used as a YAML file
 *
 */
std::string ControlParameterCollection::PrintToYamlString() {
    std::string result = "# Generated on " + GetCurrentTimeAndDate() + "\n";

    result += YAML_COLLECTION_NAME_KEY;
    result += ": ";
    result += name_ + "\n\n";

    std::vector< std::string > lines;

    // names
    int maxLength_name = 0;
    for ( auto& kv : map_ ) {
        maxLength_name = std::max( maxLength_name, ( int )kv.first.length() );
        lines.push_back( kv.first );
    }

    // name pad, :, and number
    size_t i                = 0;
    int    maxLength_number = 0;
    for ( auto& kv : map_ ) {
        int charsToAdd = maxLength_name - ( int )lines[ i ].length();
        assert( charsToAdd >= 0 );
        for ( int j = 0; j < charsToAdd; j++ ) {
            lines[ i ].push_back( ' ' );
        }
        lines[ i ] += ": ";
        lines[ i ] += kv.second->ToString();
        maxLength_number = std::max( maxLength_number, ( int )lines[ i ].length() );
        i++;
    }

    // combine lines
    for ( auto& line : lines ) {
        result += line + "\n";
    }

    return result;
}

/**
 * @brief Write all parameters and values to a YAML file
 *
 * @param path : the file name
 */
void ControlParameters::WriteToYamlFile( const std::string& path ) {
    WriteStringToFile( path, collection_.PrintToYamlString() );
}

// early declaration
ControlParameterValueKind FindParamKind( const std::string& key, ParamHandler& param_handler, int& rows, int& cols );

/**
 * @brief Add and initialize parameters from a YAML file
 *
 */
void ControlParameters::DefineAndInitializeFromYamlFile( const std::string& path ) {
    ParamHandler paramHandler( path );

    if ( !paramHandler.fileOpenedSuccessfully() ) {
        printf( "[ERROR] Could not open yaml file %s : not initializing control "
                "parameters!\n",
                path.c_str() );
        throw std::runtime_error( "yaml file bad" );
    }

    std::string name;
    if ( !paramHandler.getString( YAML_COLLECTION_NAME_KEY, name ) ) {
        printf( "[ERROR] YAML doesn't have a a collection name field named %s\n", YAML_COLLECTION_NAME_KEY );
        throw std::runtime_error( "yaml file bad" );
    }

    if ( name != name_ ) {
        printf( "[ERROR] YAML file %s has collection name %s which cannot be used to "
                "initialize %s\n",
                path.c_str(), name.c_str(), name_.c_str() );
        throw std::runtime_error( "yaml file bad" );
    }

    std::vector< std::string > keys = paramHandler.getKeys();

    for ( auto& key : keys ) {
        if ( key == YAML_COLLECTION_NAME_KEY )
            continue;
        std::string valueString;
        paramHandler.getString( key, valueString );
        int                       rows = 1, cols = 1;
        ControlParameterValueKind kind = FindParamKind( key, paramHandler, rows, cols );

        ControlParameter* cp = new ControlParameter( key, kind, rows, cols );
        collection_.AddParameter( cp, key );
        switch ( cp->kind_ ) {
        case ControlParameterValueKind::kDOUBLE: {
            double d;
            assert( paramHandler.getValue( key, d ) );
            cp->InitializeDouble( d );
        } break;

        case ControlParameterValueKind::kS64: {
            s64 f;
            assert( paramHandler.getValue( key, f ) );
            cp->InitializeInteger( f );
        } break;

        case ControlParameterValueKind::kVEC_X_DOUBLE: {
            std::vector< double > vv;
            paramHandler.getVector( key, vv );
            cp->InitializeVecXd( vv );
        } break;

        case ControlParameterValueKind::kMAT_X_DOUBLE: {
            std::vector< std::vector< double > > mm;
            paramHandler.get2DArray( key, mm );
            cp->InitializeMatXd( mm );
        } break;

        default:
            throw std::runtime_error( StringFormat( "can't read type %d from yaml file", cp->kind_ ) );
            break;
        }
    }
}

/**
 * @brief Set parameters from a YAML file
 *
 * @param path : the file name
 */
void ControlParameters::InitializeFromYamlFile( const std::string& path ) {
    ParamHandler paramHandler( path );

    if ( !paramHandler.fileOpenedSuccessfully() ) {
        printf( "[ERROR] Could not open yaml file %s : not initializing control "
                "parameters!\n",
                path.c_str() );
        return;
        throw std::runtime_error( "yaml file bad" );
    }

    std::string name;
    // if (!paramHandler.getString(YAML_COLLECTION_NAME_KEY, name)) {
    if ( !paramHandler.getString( YAML_COLLECTION_NAME_KEY, name ) ) {
        printf( "[ERROR] YAML doesn't have a a collection name field named %s\n", YAML_COLLECTION_NAME_KEY );
        throw std::runtime_error( "yaml file bad" );
    }

    if ( name != name_ ) {
        printf( "[ERROR] YAML file %s has collection name %s which cannot be used to "
                "initialize %s\n",
                path.c_str(), name.c_str(), name_.c_str() );
        throw std::runtime_error( "yaml file bad" );
    }

    std::vector< std::string > keys = paramHandler.getKeys();

    for ( auto& key : keys ) {
        if ( key == YAML_COLLECTION_NAME_KEY )
            continue;
        ControlParameter& cp = collection_.LookUp( key );
        switch ( cp.kind_ ) {
        case ControlParameterValueKind::kDOUBLE: {
            double d;
            assert( paramHandler.getValue( key, d ) );
            cp.InitializeDouble( d );
        } break;

        case ControlParameterValueKind::kS64: {
            s64 f;
            assert( paramHandler.getValue( key, f ) );
            cp.InitializeInteger( f );
        } break;

        case ControlParameterValueKind::kVEC_X_DOUBLE: {
            std::vector< double > vv;
            assert( paramHandler.getVector( key, vv ) );
            cp.InitializeVecXd( vv );
        } break;

        case ControlParameterValueKind::kMAT_X_DOUBLE: {
            std::vector< std::vector< double > > mm;
            paramHandler.get2DArray( key, mm );
            cp.InitializeMatXd( mm );
        } break;

        default:
            throw std::runtime_error( StringFormat( "can't read type %d from yaml file", cp.kind_ ) );
            break;
        }
    }
}

/**
 * @brief Find parameter type directly by yaml
 *
 * @param key : the key in yaml
 */
ControlParameterValueKind FindParamKind( const std::string& key, ParamHandler& param_handler, int& rows, int& cols ) {
    auto type = param_handler.getType( key );
    switch ( type ) {
    case Type::Scalar:  // TODO: it will treat many other type as scalar e.g. String
        return ControlParameterValueKind::kDOUBLE;
        break;
    case Type::Sequence: {
        // Sequence including 2 kine: vector and matrix
        std::vector< double >                vv;
        std::vector< std::vector< double > > mm;
        if ( param_handler.getVector( key, vv ) ) {
            // the parameter is a vector
            rows = vv.size();
            cols = 1;
            return ControlParameterValueKind::kVEC_X_DOUBLE;
        }
        else if ( param_handler.get2DArray( key, mm ) ) {
            // the parameter is a matrix
            size_t size = mm[ 0 ].size();
            for ( auto& line : mm ) {
                if ( size != line.size() ) {
                    throw std::runtime_error( StringFormat( "Invalid Matrix, current line size %d differ from "
                                                            "expect %d",
                                                            line.size(), size ) );
                }
            }
            rows = mm.size();
            cols = mm[ 0 ].size();
            return ControlParameterValueKind::kMAT_X_DOUBLE;
        }
        else {
            throw std::runtime_error( "unknown parameter [" + key + "], expect a vector or matrix, got: " );
        }
    } break;
    default:
        throw std::runtime_error( "unknown parameter type: " + std::to_string( ( int )type ) );
        break;
    }
    return ControlParameterValueKind::kDOUBLE;
}
