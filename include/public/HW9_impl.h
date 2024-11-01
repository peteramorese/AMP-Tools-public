#pragma once
#include "hw/HW9.h"


namespace ampprivate_hw9 {

struct _HW9ReconstructorBase {};

template <class ALG_T, class BASE_ALG_T, class _CTOR_ARGS_TUP>
struct _AlgoReconstructor : public BASE_ALG_T, public _HW9ReconstructorBase {
    _AlgoReconstructor(const _CTOR_ARGS_TUP& constructor_arguments) 
        : args_tuple(constructor_arguments) {}

    virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override {
        ALG_T algo = std::make_from_tuple<ALG_T>(args_tuple);
        return algo.plan(problem, agent);
    }

    _CTOR_ARGS_TUP args_tuple;

    virtual ~_AlgoReconstructor() {}
};

template <class AGENT_T, class BASE_AGENT_T, class _CTOR_ARGS_TUP>
struct _AgentReconstructor : public BASE_AGENT_T, public _HW9ReconstructorBase {
    _AgentReconstructor(const _CTOR_ARGS_TUP& constructor_arguments) 
        : args_tuple(constructor_arguments) {}

    virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {
        AGENT_T agent = std::make_from_tuple<AGENT_T>(args_tuple);
        agent.propagate(state, control, dt);
    }

    _CTOR_ARGS_TUP args_tuple;

    virtual ~_AgentReconstructor() {}
};

}

/// @brief No ctor params
template <class KINO_ALG_T, class SI_AGENT_T, class FO_AGENT_T, class SO_AGENT_T, class CAR_AGENT_T>
int amp::HW9::grade(const std::string& email, int argc, char** argv) {
    ampprivate_hw9::_AlgoReconstructor<KINO_ALG_T, amp::KinodynamicRRT, std::tuple<>> reconstructor_kino_algo(std::tuple<>{});
    ampprivate_hw9::_AgentReconstructor<SI_AGENT_T, amp::DynamicAgent, std::tuple<>> reconstructor_si_agent(std::tuple<>{});
    ampprivate_hw9::_AgentReconstructor<FO_AGENT_T, amp::DynamicAgent, std::tuple<>> reconstructor_fo_agent(std::tuple<>{});
    ampprivate_hw9::_AgentReconstructor<SO_AGENT_T, amp::DynamicAgent, std::tuple<>> reconstructor_so_agent(std::tuple<>{});
    ampprivate_hw9::_AgentReconstructor<CAR_AGENT_T, amp::DynamicAgent, std::tuple<>> reconstructor_car_agent(std::tuple<>{});
    return grade(reconstructor_kino_algo, reconstructor_si_agent, reconstructor_fo_agent, reconstructor_so_agent, reconstructor_car_agent, email, argc, argv);
}

/// @brief With ctor params
template <class KINO_ALG_T, class SI_AGENT_T, class FO_AGENT_T, class SO_AGENT_T, class CAR_AGENT_T, 
    class _KINO_CTOR_ARGS_TUP, class _SI_CTOR_ARGS_TUP, class _FO_CTOR_ARGS_TUP, class _SO_CTOR_ARGS_TUP, class _CAR_CTOR_ARGS_TUP>
int amp::HW9::grade(const std::string& email, int argc, char** argv, const _KINO_CTOR_ARGS_TUP& kino_ctor_args_tuple, const _SI_CTOR_ARGS_TUP& si_ctor_args_tuple, const _FO_CTOR_ARGS_TUP& fo_ctor_args_tuple, const _SO_CTOR_ARGS_TUP& so_ctor_args_tuple, const _CAR_CTOR_ARGS_TUP& car_ctor_args_tuple) {
    ampprivate_hw9::_AlgoReconstructor<KINO_ALG_T, amp::KinodynamicRRT, _KINO_CTOR_ARGS_TUP> reconstructor_kino_algo(kino_ctor_args_tuple);
    ampprivate_hw9::_AgentReconstructor<SI_AGENT_T, amp::DynamicAgent, _SI_CTOR_ARGS_TUP> reconstructor_si_agent(si_ctor_args_tuple);
    ampprivate_hw9::_AgentReconstructor<FO_AGENT_T, amp::DynamicAgent, _FO_CTOR_ARGS_TUP> reconstructor_fo_agent(fo_ctor_args_tuple);
    ampprivate_hw9::_AgentReconstructor<SO_AGENT_T, amp::DynamicAgent, _SO_CTOR_ARGS_TUP> reconstructor_so_agent(so_ctor_args_tuple);
    ampprivate_hw9::_AgentReconstructor<CAR_AGENT_T, amp::DynamicAgent, _CAR_CTOR_ARGS_TUP> reconstructor_car_agent(car_ctor_args_tuple);
    return grade(reconstructor_kino_algo, reconstructor_si_agent, reconstructor_fo_agent, reconstructor_so_agent, reconstructor_car_agent, email, argc, argv);
}
