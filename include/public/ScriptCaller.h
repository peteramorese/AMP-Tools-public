#pragma once

#ifndef AMP_EXCLUDE_VIS

#include <Python.h>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <tuple>

#include "PythonObjects.h"
#include "tools/Logging.h"

namespace ampprivate {
namespace pybridge {

class ScriptCaller {
    public:
        template <typename... ARGS_T>
        static void call(const std::string& module, const std::string& function, const std::tuple<ARGS_T...>& args_tuple);
    private:
        // Initialize the python interpreter when the program starts, destroy it when the program terminates
        struct _Lifeline {
            _Lifeline();
            ~_Lifeline();
            PyGILState_STATE gil_state;
            std::unique_ptr<PythonObject> sys_path;
        };
        static inline _Lifeline s_lifeline;
    private:
};

}
}

#include "ScriptCaller_impl.h"

#endif