#ifndef CPU_MLP_HPP_
#define CPU_MLP_HPP_

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace cyberdog {
namespace cpu_mlp {
    enum class ActivationType { elu, leakyrelu, relu, tanh, softsign };

    template < typename Dtype, ActivationType activation_type > struct Activation {
        inline void NonLinearity( Eigen::Matrix< Dtype, -1, -1 >& output ) {}
        inline void NonLinearity( Eigen::Matrix< Dtype, -1, 1 >& output ) {}
    };

    template < typename Dtype > struct Activation< Dtype, ActivationType::elu > {
        inline void NonLinearity( Eigen::Matrix< Dtype, -1, -1 >& output ) {
            for ( int i = 0; i < output.size(); i++ ) {
                if ( output[ i ] < 0.0 )
                    output[ i ] = std::exp( output[ i ] ) - 1.0;
            }
        }

        inline void NonLinearity( Eigen::Matrix< Dtype, -1, 1 >& output ) {
            for ( int i = 0; i < output.size(); i++ ) {
                if ( output[ i ] < 0.0 )
                    output[ i ] = std::exp( output[ i ] ) - 1.0;
            }
        }
    };

    template < typename Dtype > struct Activation< Dtype, ActivationType::leakyrelu > {
        inline void NonLinearity( Eigen::Matrix< Dtype, -1, -1 >& output ) {
            output = output.cwiseMax( 1e-2 * output );
        }

        inline void NonLinearity( Eigen::Matrix< Dtype, -1, 1 >& output ) {
            output = output.cwiseMax( 1e-2 * output );
        }
    };

    template < typename Dtype > struct Activation< Dtype, ActivationType::relu > {
        inline void NonLinearity( Eigen::Matrix< Dtype, -1, -1 >& output ) {
            output = output.cwiseMax( 0.0 );
        }

        inline void NonLinearity( Eigen::Matrix< Dtype, -1, 1 >& output ) {
            output = output.cwiseMax( 0.0 );
        }
    };

    template < typename Dtype > struct Activation< Dtype, ActivationType::tanh > {
        inline void NonLinearity( Eigen::Matrix< Dtype, -1, -1 >& output ) {
            output = output.array().tanh();
        }
        inline void NonLinearity( Eigen::Matrix< Dtype, -1, 1 >& output ) {
            output = output.array().tanh();
        }
    };

    template < typename Dtype > struct Activation< Dtype, ActivationType::softsign > {
        inline void NonLinearity( Eigen::Matrix< Dtype, -1, -1 >& output ) {
            for ( int i = 0; i < output.size(); i++ ) {
                output[ i ] = output[ i ] / ( std::abs( output[ i ] ) + 1.0 );
            }
        }

        inline void NonLinearity( Eigen::Matrix< Dtype, -1, 1 >& output ) {
            for ( int i = 0; i < output.size(); i++ ) {
                output[ i ] = output[ i ] / ( std::abs( output[ i ] ) + 1.0 );
            }
        }
    };

    /**
     * @brief Implement a fully connected neural network.
     *
     */
    template < typename Dtype, int state_dim, int action_dim, ActivationType activation_type > class MlpFullyConnected {
    public:
        typedef Eigen::Matrix< Dtype, action_dim, 1 > Action;
        typedef Eigen::Matrix< Dtype, state_dim, 1 >  State;

        /**
         * @brief Construct a new MLP Fully Connected neural network object.
         *
         * @param hidden_layer_sizes example: {512, 256, 128}
         */
        MlpFullyConnected( std::vector< int > hidden_layer_sizes ) {
            // layer sizes: [input hidden output]
            layer_sizes_.push_back( state_dim );
            layer_sizes_.reserve( layer_sizes_.size() + hidden_layer_sizes.size() );
            layer_sizes_.insert( layer_sizes_.end(), hidden_layer_sizes.begin(), hidden_layer_sizes.end() );
            layer_sizes_.push_back( action_dim );

            // model parameters
            params_.resize( 2 * ( layer_sizes_.size() - 1 ) );
            ws_.resize( layer_sizes_.size() - 1 );
            bs_.resize( layer_sizes_.size() - 1 );
            lo_.resize( layer_sizes_.size() );

            for ( int i = 0; i < ( int )( params_.size() ); i++ ) {
                if ( i % 2 == 0 ) {  // w resize
                    ws_[ i / 2 ].resize( layer_sizes_[ i / 2 + 1 ], layer_sizes_[ i / 2 ] );
                    params_[ i ].resize( layer_sizes_[ i / 2 ] * layer_sizes_[ i / 2 + 1 ] );
                }
                else if ( i % 2 == 1 ) {  // b resize
                    bs_[ ( i - 1 ) / 2 ].resize( layer_sizes_[ ( i + 1 ) / 2 ] );
                    params_[ i ].resize( layer_sizes_[ ( i + 1 ) / 2 ] );
                }
            }
        }

        /**
         * @brief Load model parameters.
         *
         * @param file_name model path
         */
        void UpdateParamsFromTxt( std::string file_name ) {
            std::ifstream indata;
            indata.open( file_name );
            std::string line;
            getline( indata, line );
            std::stringstream line_stream( line );
            std::string       cell;

            // assign parameters
            for ( int i = 0; i < ( int )( params_.size() ); i++ ) {
                int param_size = 0;

                while ( std::getline( line_stream, cell, ',' ) ) {  // read params
                    params_[ i ]( param_size++ ) = std::stod( cell );
                    if ( param_size == params_[ i ].size() )
                        break;
                }

                if ( i % 2 == 0 ) {  // w copy
                    memcpy( ws_[ i / 2 ].data(), params_[ i ].data(), sizeof( Dtype ) * ws_[ i / 2 ].size() );
                }
                else if ( i % 2 == 1 ) {  // b copy
                    memcpy( bs_[ ( i - 1 ) / 2 ].data(), params_[ i ].data(), sizeof( Dtype ) * bs_[ ( i - 1 ) / 2 ].size() );
                }
            }
        }

        /**
         * @brief Perform neural network model inference.
         *
         * @param state model input
         * @return action to perform
         */
        inline Action Forward( State& state ) {
            lo_[ 0 ] = state;

            for ( int cnt = 0; cnt < ( int )( ws_.size() ) - 1; cnt++ ) {
                lo_[ cnt + 1 ] = ws_[ cnt ] * lo_[ cnt ] + bs_[ cnt ];
                activation_.NonLinearity( lo_[ cnt + 1 ] );
            }

            lo_[ lo_.size() - 1 ] = ws_[ ws_.size() - 1 ] * lo_[ lo_.size() - 2 ] + bs_[ bs_.size() - 1 ];  // output layer
            return lo_.back();
        }

    private:
        std::vector< Eigen::Matrix< Dtype, -1, 1 > >  params_;
        std::vector< Eigen::Matrix< Dtype, -1, -1 > > ws_;
        std::vector< Eigen::Matrix< Dtype, -1, 1 > >  bs_;
        std::vector< Eigen::Matrix< Dtype, -1, 1 > >  lo_;

        Activation< Dtype, activation_type > activation_;

        std::vector< int > layer_sizes_;
    };
}  // namespace cpu_mlp
}  // namespace cyberdog

#endif  // CPU_MLP_HPP_
