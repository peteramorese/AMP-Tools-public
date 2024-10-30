#pragma once
#include "hw/HW9.h"


namespace ampprivate_hw9 {

struct _HW9ReconstructorBase {};

template <class ALG_T, class BASE_ALG_T, class _CTOR_ARGS_TUP>
struct _Reconstructor : public BASE_ALG_T, public _HW9ReconstructorBase {
    _Reconstructor(const _CTOR_ARGS_TUP& constructor_arguments) 
        : args_tuple(constructor_arguments) {}

    virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem) override {
        ALG_T algo = std::make_from_tuple<ALG_T>(args_tuple);
        return algo.plan(problem);
    }

    _CTOR_ARGS_TUP args_tuple;

    virtual ~_Reconstructor() {}
};

}

/// @brief No ctor params
template <class RRT_ALG_T>
int amp::HW9::grade(const std::string& email, int argc, char** argv) {
    ampprivate_hw9::_Reconstructor<RRT_ALG_T, amp::KinodynamicRRT, std::tuple<>> reconstructor_rrt_algo(std::tuple<>{});
    return grade(reconstructor_rrt_algo, email, argc, argv);
}

/// @brief With ctor params
template <class RRT_ALG_T, class _RRT_CTOR_ARGS_TUP>
int amp::HW9::grade(const std::string& email, int argc, char** argv, const _RRT_CTOR_ARGS_TUP& rrt_ctor_args_tuple) {
    ampprivate_hw9::_Reconstructor<RRT_ALG_T, amp::KinodynamicRRT, _RRT_CTOR_ARGS_TUP> reconstructor_rrt_algo(rrt_ctor_args_tuple);
    return grade(reconstructor_rrt_algo, email, argc, argv);
}
