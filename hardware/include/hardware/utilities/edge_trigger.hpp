#ifndef EDGE_TRIGGER_HPP_
#define EDGE_TRIGGER_HPP_

template < typename T > class EdgeTrigger {
public:
    EdgeTrigger( T initial_state ) : state_( initial_state ) {}

    bool Trigger( T& x ) {
        if ( state_ == x ) {
            return false;
        }
        else {
            state_ = x;
            return true;
        }
    }

private:
    T state_;
};

#endif  // EDGE_TRIGGER_HPP_
