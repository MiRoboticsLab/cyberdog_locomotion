#ifndef RING_QUEUE_HPP_
#define RING_QUEUE_HPP_

#include <vector>

template < typename T, typename C > struct RingQueueIteratorType {
    RingQueueIteratorType( C& collection, int index ) : collection_( collection ) {
        idx_ = index;
    }
    RingQueueIteratorType( RingQueueIteratorType const& a ) : collection_( a.collection_ ), idx_( a.idx_ ) {}

    RingQueueIteratorType const& operator++() {
        idx_++;
        return *this;
    }
    T const& operator*() {
        return collection_.at( idx_ );
    }
    bool operator!=( RingQueueIteratorType const& other ) const {
        return idx_ != other.idx_;
    }

private:
    C&  collection_;
    int idx_;
};

template < typename T, typename T_alloc > class RingQueue {
public:
    explicit RingQueue( int size ) : que_( size ), all_size_( size ) {
        head_     = 0;
        tail_     = 0;
        cur_size_ = 0;
    }
    ~RingQueue() {}

    bool PushBack( T& value ) {
        if ( all_size_ <= 0 ) {
            return false;
        }

        if ( cur_size_ < all_size_ ) {
            que_[ tail_ ] = value;
            SizePlus1( tail_ );
            cur_size_++;
        }
        else {
            que_[ head_ ] = value;
            SizePlus1( head_ );
            SizePlus1( tail_ );
        }
        return true;
    }

    void Clear() {
        tail_     = head_;
        cur_size_ = 0;
    }

    int GetCurrentSize() {
        return cur_size_;
    }
    bool IsFull() {
        return ( cur_size_ == all_size_ );
    }

    T& at( int idx ) {
        return que_[ ( head_ + idx ) % all_size_ ];
    }

    RingQueueIteratorType< T, RingQueue > Begin() {
        return RingQueueIteratorType< T, RingQueue >( *this, 0 );
    }
    RingQueueIteratorType< T, RingQueue > End() {
        return RingQueueIteratorType< T, RingQueue >( *this, cur_size_ );
    }

private:
    inline void SizePlus1( int& size_value ) {
        size_value = ( size_value + 1 ) % all_size_;
    }

    // TODO: what if T is a eigen vector?
    std::vector< T, T_alloc > que_;
    int                       all_size_;
    int                       head_;
    int                       tail_;
    int                       cur_size_;
};

#endif  // RING_QUEUE_HPP_
