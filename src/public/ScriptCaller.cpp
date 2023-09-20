#include "public/ScriptCaller.h"
#include "public/VarsPublic.h"
#include "tools/Logging.h"

ampprivate::pybridge::PythonObject::PythonObject(PyObject* py_object) : m_py_object{py_object} {
    if (!m_py_object)
        PyErr_Print();
}

void ampprivate::pybridge::PythonObject::addDependentObject(PyObject* py_object) {
    m_dependent_objects.push_back(py_object);
}

void ampprivate::pybridge::PythonObject::print() {
    if (m_py_object != nullptr) {
        if (PyUnicode_Check(m_py_object)) { // Check if valid Unicode object
            // Print the object
            const char* result_str = PyUnicode_AsUTF8(m_py_object);
            if (result_str != nullptr) {
                LOG(result_str);
                //printf("%s\n", result_str);
            } else {
                PyErr_Print();
            }

        } else {
            PyErr_Print();
            // Handle the case where it is not a valid Unicode object
        }
    } else {
        PyErr_Print();
        // Handle other errors
    }
}

ampprivate::pybridge::PythonObject::~PythonObject() {
    for (PyObject* obj : m_dependent_objects) {
        if (obj)
            Py_DECREF(obj);
    }
    if (m_py_object)
        Py_DECREF(m_py_object);
}


ampprivate::pybridge::ScriptCaller::_Lifeline::_Lifeline() {
    Py_Initialize();

    gil_state = PyGILState_Ensure();

    // Append to python search path based off _Vars.h.in
    sys_path.reset(new PythonObject(PySys_GetObject((char*)"path")));
    if (*sys_path) {
        // Add the build directory to the path
        PyList_Append(sys_path->get(), PyUnicode_DecodeFSDefault(AMP_BINARY_DIR));
        //PyList_Append(sys_path->get(), PyUnicode_DecodeFSDefault(AMP_PYTHONPATH));
    } else {
        PyErr_Print();
    }
}

ampprivate::pybridge::ScriptCaller::_Lifeline::~_Lifeline() {
    PyGILState_Release(gil_state);
    Py_Finalize();
}