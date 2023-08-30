#ifndef KIN_WBC_HPP_
#define KIN_WBC_HPP_

#include <vector>

#include "wbc/contact_spec.hpp"
#include "wbc/task.hpp"

/**
 * @brief Kinematic whole body controller
 *
 */
template < typename T > class KinWbc {
public:
    KinWbc( size_t num_qdot );
    ~KinWbc() {}

    bool FindConfiguration( const DVec< T >& curr_config, const std::vector< Task< T >* >& task_list, const std::vector< ContactSpec< T >* >& contact_list, DVec< T >& jpos_cmd, DVec< T >& jvel_cmd );

private:
    void PseudoInverseWrapper( const DMat< T > jacobian, DMat< T >& inverse_jacobian );
    void BuildProjectionMatrix( const DMat< T >& jacobian, DMat< T >& project_matrix );
    void BuildProjectionMatrix( const DMat< T >& jacobian, const DMat< T >& pseudo_inverse_jacobian, DMat< T >& project_matrix );

    double    threshold_;
    size_t    num_qdot_;
    size_t    num_act_joint_;
    DMat< T > inertia_matrix_;
};
#endif  // KIN_WBC_HPP_