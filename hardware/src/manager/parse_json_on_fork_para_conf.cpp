#include "parse_json_on_rork_para_conf.hpp"

// debug control surname for printf info
//#define DEBUG_PRINTF

ParseJsonOnForkParaConf::ParseJsonOnForkParaConf() {
    std::cout << "ParseJsonOnForkParaConf() is invoked!" << std::endl;
}

ParseJsonOnForkParaConf::~ParseJsonOnForkParaConf() {
    std::cout << "~ParseJsonOnForkParaConf() is invoked!" << std::endl;
}

std::vector< ForkParaConfJson > ParseJsonOnForkParaConf::ParseJsonData( const char* fileName ) {
    std::string                     strJson = ReadFile( fileName );
    rapidjson::Document             readDoc;
    std::vector< ForkParaConfJson > jsonDatas;
    ForkParaConfJson                jsonObject;

    if ( readDoc.Parse( strJson.c_str() ).HasParseError() ) {
        std::cout << "parse error,the json format maybe is wrong,please check the json file!" << std::endl;
        return std::vector< ForkParaConfJson >();
    }

    if ( !readDoc.IsObject() ) {
        std::cout << "json is not an object,please check the json file!" << std::endl;
        return std::vector< ForkParaConfJson >();
    }

    auto object = readDoc.GetObject();

    for ( auto iter = object.MemberBegin(); iter != object.MemberEnd(); ++iter ) {
        if ( iter->value.IsObject() && iter->value.HasMember( "fork_config" ) ) {
            if ( iter->value[ "fork_config" ].IsObject() ) {
                if ( iter->value[ "fork_config" ].HasMember( "name" ) ) {
                    auto name_object = iter->value[ "fork_config" ][ "name" ].GetString();

                    if ( iter->value[ "fork_config" ].HasMember( "object_path" ) ) {
                        auto object_path = iter->value[ "fork_config" ][ "object_path" ].GetString();

                        if ( iter->value[ "fork_config" ].HasMember( "log_path" ) ) {
                            auto log_path = iter->value[ "fork_config" ][ "log_path" ].GetString();

                            if ( iter->value[ "fork_config" ].HasMember( "para_values" ) ) {
                                const rapidjson::Value& info_object = iter->value[ "fork_config" ][ "para_values" ].GetArray();

                                jsonObject.para_values.clear();

                                if ( info_object.IsArray() ) {
                                    for ( std::size_t j = 0; j < info_object.Size(); j++ ) {
                                        jsonObject.para_values.push_back( info_object[ j ].GetString() );
                                    }
                                }

                                jsonObject.name        = name_object;
                                jsonObject.object_path = object_path;
                                jsonObject.log_path    = log_path;
                            }

#ifdef DEBUG_PRINTF
                            std::cout << "name: " << name_object << ", "
                                      << "object_path: " << object_path << ", "
                                      << "log_path: " << log_path << ", "
                                      << "para1: " << jsonObject.para_values.at( 0 ) << ", "
                                      << "para2: " << jsonObject.para_values.at( 1 ) << ", "
                                      << "para3: " << jsonObject.para_values.at( 2 ) << std::endl;
#endif

                            jsonDatas.push_back( jsonObject );
                        }
                    }
                }
            }
        }
    }

    return jsonDatas;
}

std::string ParseJsonOnForkParaConf::ReadFile( const char* fileName ) {
    FILE* fp = fopen( fileName, "rb" );
    if ( !fp ) {
        printf( "open failed! file: %s", fileName );
        return "";
    }

    char* buf = new char[ 1024 * 16 ];
    int   n   = fread( buf, 1, 1024 * 16, fp );
    fclose( fp );

    std::string result;
    if ( n >= 0 ) {
        result.append( buf, 0, n );
    }
    delete[] buf;

    return result;
}

bool ParseJsonOnForkParaConf::GetParaValueByKey( ForkParaConfJson& jsonCong, int key, std::string jsonValue ) {
    ( void )jsonCong;
    ( void )key;
    ( void )jsonValue;
    return true;
}
