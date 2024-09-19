#pragma once
#include "hw/HW4.h"

#include <type_traits>

template <class MANIPULATOR_T>
int amp::HW4::grade(ManipulatorCSConstructor& cspace_constructor, const std::string& email, int argc, char** argv) {
    static_assert(std::is_default_constructible<MANIPULATOR_T>::value, "Manipulator type must have a default consructor to use with grade()");
    MANIPULATOR_T manipulator;
    return grade(cspace_constructor, manipulator, email, argc, argv);
}