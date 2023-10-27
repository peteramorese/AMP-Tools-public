#pragma once
#include "hw/HW7.h"


namespace ampprivate_hw7 {

struct _HW7ReconstructorBase {};

template <class ALG_T, class BASE_ALG_T, class _CTOR_ARGS_TUP>
struct _Reconstructor : public BASE_ALG_T, public _HW7ReconstructorBase {
    _Reconstructor(const _CTOR_ARGS_TUP& constructor_arguments) 
        : args_tuple(constructor_arguments) {}

    virtual amp::Path2D plan(const amp::Problem2D& problem) override {
        ALG_T algo = std::make_from_tuple<ALG_T>(args_tuple);
        return algo.plan(problem);
    }

    _CTOR_ARGS_TUP args_tuple;

    virtual ~_Reconstructor() {}
};

}

/// @brief No ctor params
template <class PRM_ALG_T, class RRT_ALG_T>
int amp::HW7::grade(const std::string& email, int argc, char** argv) {
    ampprivate_hw7::_Reconstructor<PRM_ALG_T, amp::PRM2D, std::tuple<>> reconstructor_prm_algo(std::tuple<>{});
    ampprivate_hw7::_Reconstructor<RRT_ALG_T, amp::GoalBiasRRT2D, std::tuple<>> reconstructor_rrt_algo(std::tuple<>{});
    return grade(reconstructor_prm_algo, reconstructor_rrt_algo, email, argc, argv);
}

/// @brief With ctor params
template <class PRM_ALG_T, class RRT_ALG_T, class _PRM_CTOR_ARGS_TUP, class _RRT_CTOR_ARGS_TUP>
int amp::HW7::grade(const std::string& email, int argc, char** argv, const _PRM_CTOR_ARGS_TUP& prm_ctor_args_tuple, const _RRT_CTOR_ARGS_TUP& rrt_ctor_args_tuple) {
    ampprivate_hw7::_Reconstructor<PRM_ALG_T, amp::PRM2D, _PRM_CTOR_ARGS_TUP> reconstructor_prm_algo(prm_ctor_args_tuple);
    ampprivate_hw7::_Reconstructor<RRT_ALG_T, amp::GoalBiasRRT2D, _RRT_CTOR_ARGS_TUP> reconstructor_rrt_algo(rrt_ctor_args_tuple);
    return grade(reconstructor_prm_algo, reconstructor_rrt_algo, email, argc, argv);
}
