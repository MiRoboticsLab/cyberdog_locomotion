#ifndef MANAGER_HANDLE_OTA_CLIENT_HPP_
#define MANAGER_HANDLE_OTA_CLIENT_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "libraries.h"

void HandleOtaClient( const int connfd, const pid_t extprog_pid ); /* main ota handle function */

void Push( const int connfd, const pid_t extprog_pid ); /* client upload new firmware */
void Flash( int connfd, const char* buf );              /* start flashing firmware */
void Query( const int connfd );                         /* client query firmware version */
void RebootOperateSystem( const int connfd );           /* reboot controller board */
void TokenWrite( int connfd, const char* buf );         /* write token from nx to mr813*/
void TokenRead( int connfd );                           /* read token from mr813 to token */
void LogUpload( int connfd );
void TokenWriteExpand( int connfd, const char* buf );
void TokenReadExpand( int connfd, const char* buf );
void SplitString( const std::string& src, const std::string& separator, std::vector< std::string >& dest );
/**
 * @brief fetch all mcu&os version info after ota finished, if any error occurred, the
 *        function will catch it, give a default message and put the crash reason into
 *        "error" tag
 *
 * @return std::string  json string
 */
std::string FetchOtaReport();

#endif  // MANAGER_HANDLE_OTA_CLIENT_HPP_
