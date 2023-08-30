#ifndef FILTERS_HPP_
#define FILTERS_HPP_

template < typename T > class Filter {
public:
    Filter( void ) {}
    virtual ~Filter( void ) {}
    virtual void Input( T input_value ) = 0;
    virtual T    Output( void )         = 0;
    virtual void Clear( void )          = 0;
};

template < typename T > class ButterWorthFilter : public Filter< T > {
public:
    ButterWorthFilter( int num_sample, T dt, T cutoff_frequency );
    virtual ~ButterWorthFilter( void );
    virtual void Input( T input_value );
    virtual T    Output( void );
    virtual void SetTimeStep( T dt );
    virtual void SetSampleNum( int num_sample );
    virtual void SetCutoffFrequency( T cutoff_frequency );
    virtual void Clear( void );

private:
    T*  buffer_;
    int cur_idx_;
    int sample_num_;
    T   dt_;
    T   cutoff_freq_;
    T   value_;
};

template < typename T > class DigitalLpFilter : public Filter< T > {
public:
    DigitalLpFilter( T w_c, T t_s );
    virtual ~DigitalLpFilter( void );
    virtual void Input( T input_value );
    virtual T    Output( void );
    virtual void Update( void );
    virtual void SetWc( T w_c );
    virtual void SetTs( T t_s );
    virtual void Clear( void );

private:
    T lpf_in_prev_[ 2 ];
    T lpf_out_prev_[ 2 ];
    T lpf_in1_, lpf_in2_, lpf_in3_, lpf_out1_, lpf_out2_;
    T lpf_out_;
    T wc_;
    T ts_;
};

template < typename T > class MovingAverageFilter : public Filter< T > {
public:
    MovingAverageFilter( int num_data );
    virtual ~MovingAverageFilter();
    virtual void Input( T input_value );
    virtual T    Output( void );
    virtual void SetDataNum( int num_data );
    virtual int  GetDataNum( void );
    virtual void Clear( void );

private:
    T*  buffer_;
    int num_data_;
    int idx_;
    T   sum_;
};

template < typename T > class DerivLpFilter : public Filter< T > {
public:
    DerivLpFilter( T w_c, T t_s );
    virtual ~DerivLpFilter( void );
    virtual void Input( T input_value );
    virtual T    Output( void );
    virtual void Clear( void );

private:
    T lpf_in_prev_[ 2 ];
    T lpf_out_prev_[ 2 ];
    T lpf_in1_, lpf_in2_, lpf_in3_, lpf_out1_, lpf_out2_;
    T lpf_out_;
};

template < typename T > class Ff01Filter : public Filter< T > {
public:
    Ff01Filter( float t_s, float w_c );
    virtual ~Ff01Filter( void );
    virtual void Input( T input_value );
    virtual T    Output( void );
    virtual void Clear( void );

private:
    T lpf_in_prev_[ 2 ];
    T lpf_out_prev_[ 2 ];
    T lpf_in1_, lpf_in2_, lpf_in3_, lpf_out1_, lpf_out2_;
    T lpf_out_;
};

template < typename T > class Ff02Filter : public Filter< T > {
public:
    Ff02Filter( float t_s, float w_c );
    virtual ~Ff02Filter( void );
    virtual void Input( T input_value );
    virtual T    Output( void );
    virtual void Clear( void );

private:
    T lpf_in_prev_[ 2 ];
    T lpf_out_prev_[ 2 ];
    T lpf_in1_, lpf_in2_, lpf_in3_, lpf_out1_, lpf_out2_;
    T lpf_out_;
};

template < typename T > class AverageFilter : public Filter< T > {
public:
    AverageFilter( T dt, T t_const, T limit );
    virtual ~AverageFilter();
    virtual void Input( T input_value );
    virtual T    Output( void );
    virtual void Clear( void );

private:
    T est_value_;
    T dt_;
    T t_const_;
    T limit_;
};

#endif  // FILTERS_HPP_
