#pragma once
#include "hw/HW6.h"


namespace ampprivate_hw6 {

struct _HW6ReconstructorBase {};

template <class POINT_WF_ALG_T, class _POINT_CTOR_ARGS_TUP>
struct _PointWFReconstructor : public amp::PointMotionPlanner2D, public _HW6ReconstructorBase {
    static_assert(std::is_base_of<amp::PointWaveFrontAlgorithm, POINT_WF_ALG_T>::value, "POINT_WF_ALG_T does not derive amp::PointWaveFrontAlgorithm");
    _PointWFReconstructor(const _POINT_CTOR_ARGS_TUP& constructor_arguments) 
        : args_tuple(constructor_arguments) {}

    virtual amp::Path2D plan(const amp::Problem2D& problem) override {
        POINT_WF_ALG_T algo = std::make_from_tuple<POINT_WF_ALG_T>(args_tuple);
        return algo.plan(problem);
    }

    _POINT_CTOR_ARGS_TUP args_tuple;

    virtual ~_PointWFReconstructor() {}
};

template <class MANIP_WF_ALG_T, class _MANIP_CTOR_ARGS_TUP>
struct _ManipWFReconstructor : public amp::LinkManipulatorMotionPlanner2D, public _HW6ReconstructorBase {
    static_assert(std::is_base_of<amp::ManipulatorWaveFrontAlgorithm, MANIP_WF_ALG_T>::value, "MANIP_WF_ALG_T does not derive amp::ManipulatorWaveFrontAlgorithm");
    _ManipWFReconstructor(const _MANIP_CTOR_ARGS_TUP& constructor_arguments) 
        : args_tuple(constructor_arguments) {}

    virtual amp::ManipulatorTrajectory2Link plan(const amp::LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
        MANIP_WF_ALG_T algo = std::make_from_tuple<MANIP_WF_ALG_T>(args_tuple);
        return algo.plan(link_manipulator_agent, problem);
    }

    _MANIP_CTOR_ARGS_TUP args_tuple;

    virtual ~_ManipWFReconstructor() {}
};

template <class ASTAR_ALG_T, class _ASTAR_CTOR_ARGS_TUP>
struct _AStarReconstructor : public amp::AStar {
    _AStarReconstructor(const _ASTAR_CTOR_ARGS_TUP& constructor_arguments) 
        : args_tuple(constructor_arguments) {}

    virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override {
        ASTAR_ALG_T algo = std::make_from_tuple<ASTAR_ALG_T>(args_tuple);
        return algo.search(problem, heuristic);
    }

    _ASTAR_CTOR_ARGS_TUP args_tuple;

    virtual ~_AStarReconstructor() {}
};

}

/// @brief No ctor params
template <class POINT_WF_ALG_T, class MANIP_WF_ALG_T, class ASTAR_ALG_T>
int amp::HW6::grade(const std::string& email, int argc, char** argv) {
    ampprivate_hw6::_PointWFReconstructor<POINT_WF_ALG_T, std::tuple<>> reconstructor_point_wf_algo(std::tuple<>{});
    ampprivate_hw6::_ManipWFReconstructor<MANIP_WF_ALG_T, std::tuple<>> reconstructor_manip_wf_algo(std::tuple<>{});
    ampprivate_hw6::_AStarReconstructor<ASTAR_ALG_T, std::tuple<>> reconstructor_astar_algo(std::tuple<>{});
    return grade(reconstructor_point_wf_algo, reconstructor_manip_wf_algo, reconstructor_astar_algo, email, argc, argv);
}

/// @brief With ctor params
template <class POINT_WF_ALG_T, class MANIP_WF_ALG_T, class ASTAR_ALG_T, class _POINT_CTOR_ARGS_TUP, class _MANIP_CTOR_ARGS_TUP, class _ASTAR_CTOR_ARGS_TUP>
int amp::HW6::grade(const std::string& email, int argc, char** argv, const _POINT_CTOR_ARGS_TUP& point_wf_ctor_args_tuple, const _MANIP_CTOR_ARGS_TUP& manip_wf_ctor_args_tuple, const _ASTAR_CTOR_ARGS_TUP& astar_ctor_args_tuple) {
    ampprivate_hw6::_PointWFReconstructor<POINT_WF_ALG_T, _POINT_CTOR_ARGS_TUP> reconstructor_point_wf_algo(point_wf_ctor_args_tuple);
    ampprivate_hw6::_ManipWFReconstructor<MANIP_WF_ALG_T, _MANIP_CTOR_ARGS_TUP> reconstructor_manip_wf_algo(manip_wf_ctor_args_tuple);
    ampprivate_hw6::_AStarReconstructor<ASTAR_ALG_T, _ASTAR_CTOR_ARGS_TUP> reconstructor_astar_algo(astar_ctor_args_tuple);
    return grade(reconstructor_point_wf_algo, reconstructor_manip_wf_algo, reconstructor_astar_algo, email, argc, argv);
}
