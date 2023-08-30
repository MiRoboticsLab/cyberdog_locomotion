#ifndef SHARED_MEMORY_HPP_
#define SHARED_MEMORY_HPP_

#include <cassert>
#include <cstring>
#include <fcntl.h>
#include <semaphore.h>
#include <stdexcept>
#include <string>
#include <sys/errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "c_types.h"

#define DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME "development-simulator"

/**
 * @brief Shared memory utilities for connecting the simulator program to the
 * robot program.
 *
 * A POSIX semaphore for shared memory.
 *
 */
class SharedMemorySemaphore {
public:
    /**
     * @brief If semaphore is unitialized, initialize it and set its value.  This can be
     * called as many times as you want safely. It must be called at least once.
     * Only one process needs to call this, even if it is used in multiple
     * processes.
     *
     * @param name the name of the semaphore
     * @param value the initial value of the semaphore
     * @param is_host default is false
     */
    void Init( const char* name, unsigned int value, bool is_host = false ) {
        if ( !initialized_ ) {
            if ( is_host ) {
                if ( sem_unlink( name ) == -1 ) {
                    printf( "[ERROR] Failed to unlink named semaphore: %s\n", strerror( errno ) );
                }
                sem_ = sem_open( name, O_CREAT | O_EXCL, 0644, value );
            }
            else {
                sem_ = sem_open( name, O_RDWR );
            }
            if ( sem_ == SEM_FAILED ) {
                printf( "[ERROR] Failed to initialize shared memory semaphore: %s\n", strerror( errno ) );
            }
            else {
                initialized_ = true;
            }
        }
    }

    /**
     * @brief Increment the value of the semaphore.
     *
     */
    void Increment() {
        sem_post( sem_ );
    }

    /**
     * @brief If the semaphore's value is > 0, decrement the value.
     * Otherwise, wait until its value is > 0, then decrement.
     *
     */
    void Decrement() {
        sem_wait( sem_ );
    }

    /**
     * @brief If the semaphore's value is > 0, decrement the value and return true
     * Otherwise, return false (doesn't decrement or wait).
     *
     * @return true
     * @return false
     */
    bool TryDecrement() {
        return ( sem_trywait( sem_ ) ) == 0;
    }

    /**
     * @brief Like decrement, but after waiting milliseconds, will give up
     * Returns true if the semaphore is successfully decremented.
     *
     * @param seconds
     * @param nanoseconds
     * @return true
     * @return false
     */
    bool DecrementTimeout( u64 seconds, u64 nanoseconds ) {
        struct timespec ts;
        clock_gettime( CLOCK_REALTIME, &ts );
        ts.tv_nsec += nanoseconds;
        ts.tv_sec += seconds;
        ts.tv_sec += ts.tv_nsec / 1000000000;
        ts.tv_nsec %= 1000000000;
#ifdef linux
        return ( sem_timedwait( sem_, &ts ) == 0 );
#else
        // TODO: we need to implement a timeout mechanism for MacOS like system
        int res = sem_wait( sem_ );
        if ( res != 0 ) {
            printf( "[ERROR] sem_wait failed: %s\n", strerror( errno ) );
        }
        return res == 0;
#endif
    }

private:
    sem_t* sem_;
    bool   initialized_ = false;
};

/**
 * @brief A container class for an object which is stored in shared memory. This
 * object can then be viewed in multiple processes or programs. Note that there
 * is significant overhead when creating a shared memory object, so it is
 * recommended that two programs that communicate should have one single large
 * SharedMemoryObject instead of many small ones.
 *
 * A name string is used to identify shared objects across different programs.
 *
 * Before a shared memory object can be used, you must either allocate new
 * memory, or connect it to an existing shared memory object.
 *
 * Creating/deleting the memory can be done with CreateNew/CloseNew.
 * Viewing an existing object allocated with CreateNew can be done with
 * Attach/Detach.
 *
 */
template < typename T > class SharedMemoryObject {
public:
    SharedMemoryObject() = default;

    /**
     * @brief Allocate memory for the shared memory object and Attach to it.
     *
     * @param name the name of the shared memory object
     * @param allow_over_write If allow_over_write is true, and there's already an object with this name,
     * the old object is overwritten.
     * Note that if this happens, the object may be initialized in a very weird state.
     * Otherwise, if an object with the name already exists, throws a std::runtime_error.
     * @return true
     * @return false
     */
    bool CreateNew( const std::string& name, bool allow_over_write = false ) {
        bool had_to_delete = false;
        assert( !data_ );
        name_ = name;
        size_ = sizeof( T );
        printf( "[Shared Memory] open new %s, size %ld bytes\n", name.c_str(), size_ );

        if ( shm_unlink( name.c_str() ) ) {
            printf( "[Shared Memory] Error in shm_unlink: %s\n", strerror( errno ) );
        }

        fd_ = shm_open( name.c_str(), O_RDWR | O_CREAT, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IROTH );
        if ( fd_ == -1 ) {
            printf( "[ERROR] SharedMemoryObject shm_open failed: %s\n", strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return false;
        }

        struct stat s;
        if ( fstat( fd_, &s ) ) {
            printf( "[ERROR] SharedMemoryObject::CreateNew(%s) stat: %s\n", name.c_str(), strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return false;
        }

        if ( s.st_size ) {
            printf( "[Shared Memory] SharedMemoryObject::CreateNew(%s) on something that "
                    "wasn't new (size is %ld bytes)\n",
                    name_.c_str(), s.st_size );
            had_to_delete = true;
            if ( !allow_over_write )
                throw std::runtime_error( "Failed to create shared memory - it already exists." );

            printf( "\tusing existing shared memory!\n" );
            // return false;
        }

        if ( ftruncate( fd_, size_ ) ) {
            printf( "[ERROR] SharedMemoryObject::CreateNew(%s) ftruncate(%ld): %s\n", name.c_str(), size_, strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return false;
        }

        void* mem = mmap( nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0 );
        if ( mem == MAP_FAILED ) {
            printf( "[ERROR] SharedMemory::CreateNew(%s) mmap fail: %s\n", name_.c_str(), strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return false;
        }

        // there is a chance that the shared memory is not zeroed if we are reusing
        // old memory. this causes all sorts of weird issues, especially if the
        // layout of the object in memory has changed.
        memset( mem, 0, size_ );

        data_ = ( T* )mem;
        return had_to_delete;
    }

    /**
     * @brief Attach to an existing shared memory object.
     *
     * @param name the name of the shared memory object
     */
    void Attach( const std::string& name ) {
        assert( !data_ );
        name_ = name;
        size_ = sizeof( T );
        printf( "[Shared Memory] open existing %s size %ld bytes\n", name.c_str(), size_ );
        fd_ = shm_open( name.c_str(), O_RDWR, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IROTH );
        if ( fd_ == -1 ) {
            printf( "[ERROR] SharedMemoryObject::Attach shm_open(%s) failed: %s\n", name_.c_str(), strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return;
        }

        struct stat s;
        if ( fstat( fd_, &s ) ) {
            printf( "[ERROR] SharedMemoryObject::Attach(%s) stat: %s\n", name.c_str(), strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return;
        }

        if ( ( size_t )s.st_size < size_ ) {
            printf( "[ERROR] SharedMemoryObject::Attach(%s) on something that was "
                    "incorrectly "
                    "sized (size is %ld bytes, should be %ld)\n",
                    name_.c_str(), s.st_size, size_ );
            throw std::runtime_error( "Failed to create shared memory!" );
            return;
        }

        void* mem = mmap( nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0 );
        if ( mem == MAP_FAILED ) {
            printf( "[ERROR] SharedMemory::Attach(%s) mmap fail: %s\n", name_.c_str(), strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return;
        }

        data_ = ( T* )mem;
    }

    /**
     * @brief Free memory associated with the current open shared memory object.
     * The object could have been opened with either Attach or CreateNew.
     * After calling this, no process can use this shared object
     *
     */
    void CloseNew() {
        assert( data_ );
        // first, unmap
        if ( munmap( ( void* )data_, size_ ) ) {
            printf( "[ERROR] SharedMemoryObject::CloseNew (%s) munmap %s\n", name_.c_str(), strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return;
        }

        data_ = nullptr;

        if ( shm_unlink( name_.c_str() ) ) {
            printf( "[ERROR] SharedMemoryObject::CloseNew (%s) shm_unlink %s\n", name_.c_str(), strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return;
        }

        // close fd
        if ( close( fd_ ) ) {
            printf( "[ERROR] SharedMemoryObject::CloseNew (%s) close %s\n", name_.c_str(), strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return;
        }

        fd_ = 0;
    }

    /**
     * @brief Close this view of the currently opened shared memory object. The object
     * can be opened with either Attach or CreateNew. After calling this, this
     * process can no longer use this shared object, but other processes still can.
     *
     */
    void Detach() {
        assert( data_ );
        // first, unmap
        if ( munmap( ( void* )data_, size_ ) ) {
            printf( "[ERROR] SharedMemoryObject::Detach (%s) munmap %s\n", name_.c_str(), strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return;
        }

        data_ = nullptr;

        // close fd
        if ( close( fd_ ) ) {
            printf( "[ERROR] SharedMemoryObject::Detach (%s) close %s\n", name_.c_str(), strerror( errno ) );
            throw std::runtime_error( "Failed to create shared memory!" );
            return;
        }

        fd_ = 0;
    }

    /*********************       For Syncronize           ************************/

    /**
     * @brief The init() method should only be called *after* shared memory is connected!
     * This initializes the shared memory semaphores used to keep things in sync.
     *
     * @param is_host
     */
    void Init( bool is_host = false ) {
        robot_to_sim_semaphore_.Init( "/robot2sim", 0, is_host );
        sim_to_robot_semaphore_.Init( "/sim2robot", 0, is_host );
    }

    /**
     * @brief Wait for the simulator to respond.
     *
     */
    void WaitForSimulator() {
        sim_to_robot_semaphore_.Decrement();
    }

    /**
     * @brief Simulator signals that it is done.
     *
     */
    void SimulatorIsDone() {
        sim_to_robot_semaphore_.Increment();
    }

    /**
     * @brief Wait for the robot to finish.
     *
     */
    void WaitForRobot() {
        robot_to_sim_semaphore_.Decrement();
    }

    /**
     * @brief Check if the robot is done.
     *
     * @return true if true, the robot is done
     * @return false if true, the robot is not done
     */
    bool TryWaitForRobot() {
        return robot_to_sim_semaphore_.TryDecrement();
    }

    /**
     * @brief Wait for the robot to finish with a timeout.
     *
     * @return true if true, we finished before timing out
     * @return false if true, we don's finish before timing out
     */
    bool WaitForRobotWithTimeout() {
        // TODO: for DEBUG ONLY
        return robot_to_sim_semaphore_.DecrementTimeout( 10000000, 0 );
    }

    /**
     * @brief Signal that the robot is done.
     *
     */
    void RobotIsDone() {
        robot_to_sim_semaphore_.Increment();
    }

    /**
     * @brief Get the shared memory object.
     *
     */
    T* Get() {
        assert( data_ );
        return data_;
    }

    /**
     * @brief Get the shared memory object.
     *
     */
    T& operator()() {
        assert( data_ );
        return *data_;
    }

private:
    SharedMemorySemaphore robot_to_sim_semaphore_, sim_to_robot_semaphore_;
    T*                    data_ = nullptr;
    std::string           name_;
    size_t                size_;
    int                   fd_;
};

#endif  // SHARED_MEMORY_HPP_
