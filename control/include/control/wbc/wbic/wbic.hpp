#ifndef WBIC_HPP_
#define WBIC_HPP_

#include <Goldfarb_Optimizer/QuadProg++.hh>

#include "utilities/utilities_print.hpp"
#include "wbc/contact_spec.hpp"
#include "wbc/task.hpp"
#include "wbc/wbc.hpp"

template < typename T > class WbicExtraData {
public:
    // Output
    DVec< T > opt_result_;
    DVec< T > qddot_;
    DVec< T > reaction_force_;

    // Input
    DVec< T > body_pose_weight_;
    DVec< T > reaction_force_weight_;
    T         abad_joint_limit_;
    T         hip_joint_limit_;
    T         knee_joint_limit_;

    WbicExtraData() {}
    ~WbicExtraData() {}
};

template < typename T > class WBIC : public WBC< T > {
public:
    WBIC( size_t num_qdot, const std::vector< ContactSpec< T >* >* contact_list, const std::vector< Task< T >* >* task_list );
    virtual ~WBIC() {}

    virtual void UpdateSetting( const DMat< T >& A, const DMat< T >& Ainv, const DVec< T >& cori, const DVec< T >& grav, void* extra_setting = NULL );

    virtual void MakeTorque( DVec< T >& cmd, void* extra_input = NULL );

private:
    void SetEqualityConstraint( const DVec< T >& qddot );
    void SetInEqualityConstraint();
    void ContactBuilding();

    void GetSolution( const DVec< T >& qddot, DVec< T >& cmd );
    void SetCost();
    void SetOptimizationSize();

    const std::vector< ContactSpec< T >* >* contact_list_;
    const std::vector< Task< T >* >*        task_list_;

    size_t dim_opt_;      // Contact pt delta, First task delta, reaction force
    size_t dim_eq_cstr_;  // equality constraints

    size_t dim_rf_;  // inequality constraints, reaction force
    size_t dim_uf_;

    size_t dim_floating_;  // floating base dimension, body pose

    WbicExtraData< T >* data_;

    GolDIdnani::GVect< double > z_;
    // Cost
    GolDIdnani::GMatr< double > G_;
    GolDIdnani::GVect< double > g0_;

    // Equality
    GolDIdnani::GMatr< double > CE_;
    GolDIdnani::GVect< double > ce0_;

    // Inequality
    GolDIdnani::GMatr< double > CI_;
    GolDIdnani::GVect< double > ci0_;

    DMat< T > dyn_CE_;
    DVec< T > dyn_ce0_;
    DMat< T > dyn_CI_;
    DVec< T > dyn_ci0_;

    DMat< T > eye_;

    DMat< T > Uf_;
    DVec< T > Uf_ieq_vec_;

    DMat< T > Jc_;
    DVec< T > JcDotQdot_;
    DVec< T > force_des_;
};

#endif  // WBIC_HPP_