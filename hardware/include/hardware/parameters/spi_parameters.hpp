#ifndef SPI_PARAMETERS_HPP_
#define SPI_PARAMETERS_HPP_

#include "control_parameters/control_parameters.hpp"

/**
 * @brief This class contains all the spi parameters represent definition difference
 * between motion-controller and motor-controller.
 *
 */
class SpiParameters : public ControlParameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct a new Spi Parameters object
     *
     */
    SpiParameters()
        : ControlParameters( "spi-parameters" ), INIT_PARAMETER( abad_side_sign ), INIT_PARAMETER( hip_side_sign ), INIT_PARAMETER( knee_side_sign ), INIT_PARAMETER( abad_offset ),
          INIT_PARAMETER( hip_offset ), INIT_PARAMETER( knee_offset ), INIT_PARAMETER( hip_range ), INIT_PARAMETER( hip_range_offset )

    {}

    DECLARE_PARAMETER( Vec4< double >, abad_side_sign )
    DECLARE_PARAMETER( Vec4< double >, hip_side_sign )
    DECLARE_PARAMETER( Vec4< double >, knee_side_sign )

    DECLARE_PARAMETER( Vec4< double >, abad_offset )
    DECLARE_PARAMETER( Vec4< double >, hip_offset )
    DECLARE_PARAMETER( Vec4< double >, knee_offset )

    DECLARE_PARAMETER( Vec8< double >, hip_range )
    DECLARE_PARAMETER( Vec4< double >, hip_range_offset )
};

#endif  // SPI_PARAMETERS_HPP_
