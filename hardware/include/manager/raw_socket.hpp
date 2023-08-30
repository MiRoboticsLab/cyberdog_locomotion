
#ifndef MANAGER_RAW_SOCKET_HPP_
#define MANAGER_RAW_SOCKET_HPP_

void String2Upper( char* temp ); /* convert string to uppercase */

void ReadCommandByNewLine( const int connfd, char* buf, const int maxsize ); /* read command from client - by new line ending */
void WriteCommandByNewLine( const int connfd, const char* buf );             /* write responce to client - by new line ending */

void ReadCommandByLength( const int connfd, char* buf, const int maxsize ); /* read command from client - by length specified */
void WriteCommandByLength( const int connfd, const char* buf );             /* write responce to client - by length specified */

int HandShake( const int connfd, const int maxsize ); /* HandShake with client */

#endif  // MANAGER_RAW_SOCKET_HPP_
