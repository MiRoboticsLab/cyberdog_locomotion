#ifndef SHM_HELP_HPP_
#define SHM_HELP_HPP_

#include <fcntl.h>
#include <string>
#include <sys/errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

/**
 * @brief Create a POSIX shared memory, if there is one with 
 * same name, then unlink it and create one.
 *
 * @param name name of shared memry
 * @param size size of the new shm, in byte
 * @return void* if succeed, return the pointer of shared memory, else return NULL
 */
inline void* CreateNewShm( const std::string& name, size_t size ) {
    if ( shm_unlink( name.c_str() ) ) {
        printf( "[Shared Memory] Error in shm_unlink: %s\n", strerror( errno ) );
    }

    int fd = shm_open( name.c_str(), O_RDWR | O_CREAT, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IROTH );
    if ( fd == -1 ) {
        printf( "[Shared Memory] shm_open %s failed: %s\n", name.c_str(), strerror( errno ) );
        return NULL;
    }

    struct stat s;
    if ( fstat( fd, &s ) ) {
        printf( "[ERROR] CreateNewShm(%s) stat: %s\n", name.c_str(), strerror( errno ) );
        return NULL;
    }

    if ( ftruncate( fd, size ) ) {
        printf( "[ERROR] SharedMemoryObject::createNew(%s) ftruncate(%ld): %s\n", name.c_str(), size, strerror( errno ) );
        return NULL;
    }

    void* mem = mmap( nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0 );
    if ( mem == MAP_FAILED ) {
        printf( "[ERROR] CreateNewShm(%s) mmap fail: %s\n", name.c_str(), strerror( errno ) );
        return NULL;
    }

    // there is a chance that the shared memory is not zeroed if we are reusing
    // old memory. this causes all sorts of weird issues, especially if the
    // layout of the object in memory has changed.
    memset( mem, 0, size );

    return mem;
}

/**
 * @brief Attach to a exist shared memory.
 *
 * @param name name of the shared memory
 * @param size size of the shared memory
 * @return void* pointer of the shared memory, NULL if open failed
 */
inline void* AttachShm( const std::string& name, size_t size ) {
    printf( "[Shared Memory] open existing %s size %ld bytes\n", name.c_str(), size );
    int fd = shm_open( name.c_str(), O_RDWR, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IROTH );

    if ( fd == -1 ) {
        printf( "[ERROR] SharedMemoryObject::Attach shm_open(%s) failed: %s\n", name.c_str(), strerror( errno ) );
        throw std::runtime_error( "Failed to create shared memory!" );
        return NULL;
    }

    struct stat s;
    if ( fstat( fd, &s ) ) {
        printf( "[ERROR] SharedMemoryObject::Attach(%s) stat: %s\n", name.c_str(), strerror( errno ) );
        throw std::runtime_error( "Failed to create shared memory!" );
        return NULL;
    }

    if ( ( size_t )s.st_size < size ) {
        printf( "[ERROR] SharedMemoryObject::Attach(%s) on something that was "
                "incorrectly "
                "sized (size is %d bytes, should be %ld)\n",
                name.c_str(), ( int )s.st_size, size );
        return NULL;
    }

    void* mem = mmap( nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0 );
    if ( mem == MAP_FAILED ) {
        printf( "[ERROR] SharedMemory::Attach(%s) mmap fail: %s\n", name.c_str(), strerror( errno ) );
        throw std::runtime_error( "Failed to create shared memory!" );
        return NULL;
    }

    return mem;
}

#endif  // SHM_HELP_HPP_
