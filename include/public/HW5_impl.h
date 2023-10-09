#pragma once
#include "hw/HW5.h"

template <class GD_ALGORITHM_T, class... CTOR_ARGS_T>
int amp::HW5::grade(const std::string& email, int argc, char** argv, CTOR_ARGS_T&&... constructor_arguments) {
    struct _Reconstructor : public GDAlgorithm {
        _Reconstructor(CTOR_ARGS_T&&... constructor_arguments_) 
            : args_tuple(std::forward<CTOR_ARGS_T>(constructor_arguments_)...) {}
        virtual amp::Path2D plan(const amp::Problem2D& problem) override {
            GD_ALGORITHM_T algo = std::make_from_tuple<GD_ALGORITHM_T>(args_tuple);
            return algo.plan(problem);
        }
        std::tuple<CTOR_ARGS_T&&...> args_tuple;
    };
    _Reconstructor reconstructor_algo(std::forward<CTOR_ARGS_T>(constructor_arguments)...);
    return grade(reconstructor_algo, email, argc, argv);
}