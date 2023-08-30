#include "calc_md5.hpp"

typedef struct Table {
    unsigned int  sin; /* floor (2^32 × abs(sin(i+1))) */
    unsigned char x;   /* index into data block */
    unsigned char rot; /* per-round shift amounts */
} Table;

Table tab[] = {

    /* round 1 */
    { 0xd76aa478, 0, 7 },
    { 0xe8c7b756, 1, 12 },
    { 0x242070db, 2, 17 },
    { 0xc1bdceee, 3, 22 },
    { 0xf57c0faf, 4, 7 },
    { 0x4787c62a, 5, 12 },
    { 0xa8304613, 6, 17 },
    { 0xfd469501, 7, 22 },
    { 0x698098d8, 8, 7 },
    { 0x8b44f7af, 9, 12 },
    { 0xffff5bb1, 10, 17 },
    { 0x895cd7be, 11, 22 },
    { 0x6b901122, 12, 7 },
    { 0xfd987193, 13, 12 },
    { 0xa679438e, 14, 17 },
    { 0x49b40821, 15, 22 },

    /* round 2 */
    { 0xf61e2562, 1, 5 },
    { 0xc040b340, 6, 9 },
    { 0x265e5a51, 11, 14 },
    { 0xe9b6c7aa, 0, 20 },
    { 0xd62f105d, 5, 5 },
    { 0x02441453, 10, 9 },
    { 0xd8a1e681, 15, 14 },
    { 0xe7d3fbc8, 4, 20 },
    { 0x21e1cde6, 9, 5 },
    { 0xc33707d6, 14, 9 },
    { 0xf4d50d87, 3, 14 },
    { 0x455a14ed, 8, 20 },
    { 0xa9e3e905, 13, 5 },
    { 0xfcefa3f8, 2, 9 },
    { 0x676f02d9, 7, 14 },
    { 0x8d2a4c8a, 12, 20 },

    /* round 3 */
    { 0xfffa3942, 5, 4 },
    { 0x8771f681, 8, 11 },
    { 0x6d9d6122, 11, 16 },
    { 0xfde5380c, 14, 23 },
    { 0xa4beea44, 1, 4 },
    { 0x4bdecfa9, 4, 11 },
    { 0xf6bb4b60, 7, 16 },
    { 0xbebfbc70, 10, 23 },
    { 0x289b7ec6, 13, 4 },
    { 0xeaa127fa, 0, 11 },
    { 0xd4ef3085, 3, 16 },
    { 0x04881d05, 6, 23 },
    { 0xd9d4d039, 9, 4 },
    { 0xe6db99e5, 12, 11 },
    { 0x1fa27cf8, 15, 16 },
    { 0xc4ac5665, 2, 23 },

    /* round 4 */
    { 0xf4292244, 0, 6 },
    { 0x432aff97, 7, 10 },
    { 0xab9423a7, 14, 15 },
    { 0xfc93a039, 5, 21 },
    { 0x655b59c3, 12, 6 },
    { 0x8f0ccc92, 3, 10 },
    { 0xffeff47d, 10, 15 },
    { 0x85845dd1, 1, 21 },
    { 0x6fa87e4f, 8, 6 },
    { 0xfe2ce6e0, 15, 10 },
    { 0xa3014314, 6, 15 },
    { 0x4e0811a1, 13, 21 },
    { 0xf7537e82, 4, 6 },
    { 0xbd3af235, 11, 10 },
    { 0x2ad7d2bb, 2, 15 },
    { 0xeb86d391, 9, 21 },
};

typedef struct Md5State {
    unsigned int len;
    unsigned int state[ 4 ];
} Md5State;

/**
 * @brief Encodes input (unsigned int) into output (unsigned char). Assumes len is a multiple of 4.
 *
 */
void Encode( unsigned char* output, unsigned int* input, unsigned int len ) {
    for ( unsigned char* e = output + len; output < e; ) {
        unsigned int x = *input++;
        *output++      = x;
        *output++      = x >> 8;
        *output++      = x >> 16;
        *output++      = x >> 24;
    }
}

/**
 * @brief Decodes input (unsigned char) into output (unsigned int). Assumes len is a multiple of 4.
 *
 */
void Decode( unsigned int* output, unsigned char* input, unsigned int len ) {
    for ( unsigned char* e = input + len; input < e; input += 4 )
        *output++ = input[ 0 ] | ( input[ 1 ] << 8 ) | ( input[ 2 ] << 16 ) | ( input[ 3 ] << 24 );
}

/* calculate md5 from file, put result hex in digest */

/**
 * @brief Calculate md5 from file, put result hex in digest.
 *
 */
void CalcMd5( unsigned char* digest, FILE* fd ) {

    /* initialize md5 starting variables */
    Md5State s   = {};
    s.state[ 0 ] = 0x67452301;
    s.state[ 1 ] = 0xefcdab89;
    s.state[ 2 ] = 0x98badcfe;
    s.state[ 3 ] = 0x10325476;

    /* read file and calculate in blocks of 128*64 (8KB) */
    while ( 1 ) {
        unsigned char file_buf[ 128 * 64 + 1 ];
        int           bytes_read = fread( file_buf, 1, 128 * 64, fd );
        unsigned int  a, b, c, d, tmp, x[ 16 ] = {}, quit = 0;
        Table*        t;

        s.len += bytes_read;

        /* if not in multiple of 64 */
        if ( bytes_read % 64 || bytes_read == 0 ) {
            quit = 1; /* reach end of file */

            /* pad the input with zero */
            int remainder = bytes_read % 64;
            if ( remainder < 56 )
                remainder = 56 - remainder;
            else
                remainder = 120 - remainder;
            memset( file_buf + bytes_read, 0, remainder );
            file_buf[ bytes_read ] = 0x80;
            bytes_read += remainder;

            /* append the count */
            x[ 0 ] = s.len << 3;
            x[ 1 ] = s.len >> 29;
            Encode( file_buf + bytes_read, x, 8 );
        }

        /* calculate */
        for ( unsigned char* loop = file_buf; loop < file_buf + bytes_read; loop += 64 ) {
            a = s.state[ 0 ];
            b = s.state[ 1 ];
            c = s.state[ 2 ];
            d = s.state[ 3 ];
            Decode( x, loop, 64 );

            for ( int i = 0; i < 64; i++ ) {
                t = tab + i;
                switch ( i >> 4 ) {
                case 0:
                    a += ( b & c ) | ( ~b & d );
                    break;
                case 1:
                    a += ( b & d ) | ( c & ~d );
                    break;
                case 2:
                    a += b ^ c ^ d;
                    break;
                case 3:
                    a += c ^ ( b | ~d );
                    break;
                }
                a += x[ t->x ] + t->sin;
                a = ( a << t->rot ) | ( a >> ( 32 - t->rot ) );
                a += b;

                /* rotate variables */
                tmp = d;
                d   = c;
                c   = b;
                b   = a;
                a   = tmp;
            }

            s.state[ 0 ] += a;
            s.state[ 1 ] += b;
            s.state[ 2 ] += c;
            s.state[ 3 ] += d;
        }

        if ( quit )
            break; /* end calculation */
    }

    Encode( digest, s.state, 16 );
}
