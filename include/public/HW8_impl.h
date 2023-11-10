#pragma once
#include "hw/HW8.h"


namespace ampprivate_hw8 {

struct _HW8ReconstructorBase {};

template <class ALG_T, class BASE_ALG_T, class _CTOR_ARGS_TUP>
struct _Reconstructor : public BASE_ALG_T, public _HW8ReconstructorBase {
    _Reconstructor(const _CTOR_ARGS_TUP& constructor_arguments) 
        : args_tuple(constructor_arguments) {}

    virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override {
        ALG_T algo = std::make_from_tuple<ALG_T>(args_tuple);
        return algo.plan(problem);
    }

    _CTOR_ARGS_TUP args_tuple;

    virtual ~_Reconstructor() {}
};

}

/// @brief No ctor params
template <class C_MA_ALGO, class DC_MA_ALGO>
int amp::HW8::grade(const std::string& email, int argc, char** argv) {
    ampprivate_hw8::_Reconstructor<C_MA_ALGO, amp::CentralizedMultiAgentRRT, std::tuple<>> reconstructor_c_algo(std::tuple<>{});
    ampprivate_hw8::_Reconstructor<DC_MA_ALGO, amp::DecentralizedMultiAgentRRT, std::tuple<>> reconstructor_dc_algo(std::tuple<>{});
    return grade(reconstructor_c_algo, reconstructor_dc_algo, email, argc, argv);
}

/// @brief With ctor params
template <class C_MA_ALGO, class DC_MA_ALGO, class _C_MA_CTOR_ARGS_TUP, class _DC_MA_CTOR_ARGS_TUP>
int amp::HW8::grade(const std::string& email, int argc, char** argv, const _C_MA_CTOR_ARGS_TUP& c_ma_ctor_args_tuple, const _DC_MA_CTOR_ARGS_TUP& dc_ma_ctor_args_tuple) {
    ampprivate_hw8::_Reconstructor<C_MA_ALGO, amp::CentralizedMultiAgentRRT, _C_MA_CTOR_ARGS_TUP> reconstructor_c_algo(c_ma_ctor_args_tuple);
    ampprivate_hw8::_Reconstructor<DC_MA_ALGO, amp::DecentralizedMultiAgentRRT, _C_MA_CTOR_ARGS_TUP> reconstructor_dc_algo(dc_ma_ctor_args_tuple);
    return grade(reconstructor_c_algo, reconstructor_dc_algo, email, argc, argv);
}
