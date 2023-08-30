#include <unistd.h>

#include "parse_json_on_execute_command.hpp"

// debug control surname for printf info
#define DEBUG_PRINTF

ParseJsonOnExecuteCommand::ParseJsonOnExecuteCommand() {
    std::cout << "ParseJsonOnExecuteCommand() is invoked!" << std::endl;
}

ParseJsonOnExecuteCommand::~ParseJsonOnExecuteCommand() {
    std::cout << "~ParseJsonOnExecuteCommand() is invoked!" << std::endl;
}

std::vector< ExecuteCommandJson > ParseJsonOnExecuteCommand::ParseJsonData( const char* fileName ) {
    std::string                       strJson = ReadFile( fileName );
    rapidjson::Document               readDoc;
    std::vector< ExecuteCommandJson > jsonDatas;

    if ( readDoc.Parse( strJson.c_str() ).HasParseError() ) {
        std::cout << "parse error,the json format maybe is wrong,please check the json file!" << std::endl;
        return std::vector< ExecuteCommandJson >();
    }

    if ( !readDoc.IsObject() ) {
        std::cout << "json is not an object,please check the json file!" << std::endl;
        return std::vector< ExecuteCommandJson >();
    }

    auto object = readDoc.GetObject();

    for ( auto iter = object.MemberBegin(); iter != object.MemberEnd(); ++iter ) {
        if ( iter->value.IsObject() && iter->value.MemberCount() == 4 ) {

            if ( iter->value.HasMember( "cmd" ) ) {
                auto cmd_object = iter->value[ "cmd" ].GetString();

                if ( iter->value.HasMember( "path" ) ) {
                    auto path_object = iter->value[ "path" ].GetString();

                    if ( iter->value.HasMember( "name" ) ) {
                        auto name_object = iter->value[ "name" ].GetString();

                        if ( iter->value.HasMember( "pc_cmd" ) ) {
                            auto pc_cmd_object = iter->value[ "pc_cmd" ].GetString();
                            jsonDatas.push_back( { cmd_object, path_object, name_object, pc_cmd_object } );

#ifdef DEBUG_PRINTF
                            std::cout << "cmd: " << cmd_object << ", "
                                      << "path: " << path_object << ", "
                                      << "name: " << name_object << ", "
                                      << "pc_cmd: " << pc_cmd_object << std::endl;
#endif
                        }
                    }
                }
            }
        }
    }

    return jsonDatas;
}

std::string ParseJsonOnExecuteCommand::ReadFile( const char* fileName ) {
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