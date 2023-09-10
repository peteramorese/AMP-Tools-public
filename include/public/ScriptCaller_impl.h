#pragma once
#include "ScriptCaller.h"

template <typename... ARGS_T>
void ampprivate::pybridge::ScriptCaller::call(const std::string& module, const std::string& function, const std::tuple<ARGS_T...>& args_tuple) {
//, const PythonObject& python_object
    // Import the Python script
    PythonObject object(PyImport_ImportModule(module.c_str()));
    if (object) {
        // Get a reference to the function in your script

        PythonObject func(PyObject_GetAttrString(object.get(), function.c_str()));

        if (func && PyCallable_Check(func.get())) {
            // Call the Python function
            PythonObject args(std::apply(PyTuple_Pack, std::tuple_cat(std::make_tuple(sizeof...(ARGS_T)), args_tuple)));
            PythonObject result(PyObject_CallObject(func.get(), args.get()));
        } else {
            if (PyErr_Occurred()) {
                PyErr_Print();
            } else {
                ERROR("Cannot find function '" << function << "'\n");
            }
        }
    } else {
        PyErr_Print();
    }

}
