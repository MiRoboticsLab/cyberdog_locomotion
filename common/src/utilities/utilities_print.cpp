#include "utilities/utilities_print.hpp"

/**
 * @brief Printf with color.
 *
 * @param color
 * @param format
 * @param ...
 */
void PrintfColor( PrintColor color, const char* format, ... ) {
    auto color_id = ( uint32_t )color;
    if ( color_id )
        printf( "\033[1;%dm", ( uint32_t )color + 30 );
    va_list args;
    va_start( args, format );
    vprintf( format, args );
    va_end( args );
    printf( "\033[0m" );
}

/**
 * @brief Print  with color (used to print color to STDERR)
 *
 * @param color
 * @param stream
 * @param format
 * @param ...
 */
void FprintfColor( PrintColor color, FILE* stream, const char* format, ... ) {
    auto color_id = ( uint32_t )color;
    if ( color_id )
        fprintf( stream, "\033[1;%dm", ( uint32_t )color + 30 );
    va_list args;
    va_start( args, format );
    vfprintf( stream, format, args );
    va_end( args );
    fprintf( stream, "\033[0m" );
}