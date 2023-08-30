#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "libraries.h"
#include "raw_socket.hpp"

/**
 * @brief Convert string to uppercase.
 *
 */
void String2Upper( char* temp ) {
    char* s = temp;
    while ( *s ) {
        *s = toupper( ( unsigned char )*s );
        s++;
    }
}

/**
 * @brief Read command from client, by new line ending.
 *
 */
void ReadCommandByNewLine( const int connfd, char* buf, const int maxsize ) {
    buf[ 0 ]       = '\0';
    char char_buf  = 0;
    int  err_count = 0;
    while ( 1 ) {
        int read_len = 0;

        if ( ( read_len = read( connfd, &char_buf, 1 ) ) == 0 ) {
            fprintf( stdout, "Client on PID %d closed the connection unexpectedly.\n", getpid() );
            exit( 0 );
        }
        else if ( read_len == -1 ) {
            err_count++;
            if ( err_count >= 3 ) {
                fprintf( stderr, "Client on PID %d read timeout or error. Closing Connection.\n", getpid() );
                exit( EXIT_FAILURE );
            }
        }
        else {
            if ( char_buf == '\r' )
                ;
            else if ( char_buf == '\n' ) {
                fprintf( stdout, "Received from client on PID %d: %s\n", getpid(), buf );
                return;
            }
            else
                strncat( buf, &char_buf, 1 );

            /* Buffer overflowed */
            if ( static_cast< int >( strlen( buf ) ) >= maxsize ) {
                fprintf( stdout, "(Buffer overflowed) Received from client on PID %d: %s\n", getpid(), buf );
                return;
            }
        }
    }
}

/**
 * @brief Read command from client, by length specified.
 *
 */
void ReadCommandByLength( const int connfd, char* buf, const int maxsize ) {
    char     char_buf    = 0;
    uint32_t data_length = 0;
    int      counter     = 0;
    int      state       = 0;

    while ( 1 ) {
        int read_len = 0;
        if ( ( read_len = read( connfd, &char_buf, 1 ) ) == 0 ) {
            fprintf( stdout, "Client on PID %d closed the connection unexpectedly.\n", getpid() );
            exit( 0 );
        }
        else if ( read_len == -1 ) {
            fprintf( stderr, "Client on PID %d read timeout or error. Closing Connection.\n", getpid() );
            exit( EXIT_FAILURE );
        }

        /* state: 0 = waiting for 0x00, 1 = waiting for 0x01, 2 = reading length int, 3 = reading data */
        switch ( state ) {
        case 0:
            if ( char_buf == 0x00 )
                state = 1;
            break;

        case 1:
            if ( char_buf == 0x01 )
                state = 2;
            else
                state = 0;
            break;

        case 2:
            *( ( char* )&data_length + counter ) = char_buf;
            counter += read_len;
            if ( counter >= 4 ) {
                data_length = ntohl( data_length );
                counter     = 0;
                state       = 3;
            }
            break;

        case 3:
            *( buf + counter ) = char_buf;
            counter += read_len;
            if ( counter >= static_cast< int >( data_length ) || counter >= maxsize - 1 ) {
                *( buf + counter ) = '\0';
                fprintf( stdout, "Received from client on PID %d: %s\n", getpid(), buf );
                return;
            }
            break;
        }
    }
}

/**
 * @brief write responce to client, by new line ending.
 *
 */
void WriteCommandByNewLine( const int connfd, const char* buf ) {
    fprintf( stdout, "Sending responce to client on PID %d (by new line ending): %s\n", getpid(), buf );
    int err_count = 0;
    while ( write( connfd, buf, strlen( buf ) ) == -1 ) {
        err_count++;
        if ( err_count >= 3 ) {
            fprintf( stderr, "Client on PID %d write timeout or error. Closing Connection.\n", getpid() );
            exit( EXIT_FAILURE );
        }
    }
    write( connfd, "\n", 1 );
}

/**
 * @brief write responce to client, by length specified.
 *
 */
void WriteCommandByLength( const int connfd, const char* buf ) {
    fprintf( stdout, "Sending responce to client on PID %d (by length specified): %s\n", getpid(), buf );
    write( connfd, "\x00\x01", 2 );
    uint32_t data_length = strlen( buf );
    data_length          = htonl( data_length );
    write( connfd, &data_length, sizeof( data_length ) );
    if ( write( connfd, buf, strlen( buf ) ) == -1 ) {
        fprintf( stderr, "Client on PID %d write timeout or error. Closing Connection.\n", getpid() );
        exit( EXIT_FAILURE );
    }
}

/**
 * @brief Handshake with client.
 *
 */
int HandShake( const int connfd, const int maxsize ) {
    char buf[ maxsize + 1 ];
    char buf_uppercase[ maxsize + 1 ];
    int  client_type = 0;

    /* set a shorter read timout before HandShake */
    struct timeval timeout;
    timeout.tv_sec  = 10;
    timeout.tv_usec = 0;
    setsockopt( connfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof( timeout ) );

    WriteCommandByNewLine( connfd, "100 SERVER HELLO" );

    ReadCommandByNewLine( connfd, buf, maxsize );

    strncpy( buf_uppercase, buf, maxsize );
    String2Upper( buf_uppercase );

    /* ota client responce */
    if ( strncmp( buf_uppercase, "CLIENT HELLO", 12 ) == 0 )
        client_type = 1;

    /* leg test client responce */
    else if ( strncmp( buf_uppercase, "PRODUCTION LEG TEST CLIENT HELLO", 32 ) == 0 )
        client_type = 2;

    /* not valid client */
    else
        return -1;

    WriteCommandByNewLine( connfd, "101 READY" );

    /* set a longer read timout after HandShake */
    struct timeval timeout2;
    timeout2.tv_sec  = 300;
    timeout2.tv_usec = 0;
    setsockopt( connfd, SOL_SOCKET, SO_RCVTIMEO, &timeout2, sizeof( timeout2 ) );
    return client_type;
}
