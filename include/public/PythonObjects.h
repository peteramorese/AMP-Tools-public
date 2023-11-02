#pragma once

#ifndef AMP_EXCLUDE_VIS

#include <Python.h>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <tuple>

namespace ampprivate {
namespace pybridge {

// Native type with auto single-reference decrementing
class PythonObject {
    public:
        PythonObject(PyObject* py_object);
        void print();

        // Add ownership of another object
        void addDependentObject(PyObject* py_object);

        PyObject* const get() {return m_py_object;}
        operator bool() const {return m_py_object;}
        ~PythonObject();

    private:
        PyObject* m_py_object;
        std::vector<PyObject*> m_dependent_objects;
};

template <typename T>
struct ListOfPairs {
    std::vector<std::array<T, 2>> list_of_tuples;
    std::unique_ptr<PythonObject> toPyList() const;
};

template <typename T>
struct ListOfQuadrouples {
    std::vector<std::array<T, 4>> list_of_tuples;
    std::unique_ptr<PythonObject> toPyList() const;
};

std::unique_ptr<PythonObject> makeBool(bool boolean);
std::unique_ptr<PythonObject> makeLong(uint64_t scalar);
std::unique_ptr<PythonObject> makeScalar(double scalar);
std::unique_ptr<PythonObject> makeString(const std::string& string);
std::unique_ptr<PythonObject> makeList(std::vector<std::unique_ptr<PythonObject>>&& list_elements);
std::unique_ptr<PythonObject> makePair(std::pair<std::unique_ptr<PythonObject>, std::unique_ptr<PythonObject>>&& tuple_elements); // Tuple of size 2

}
}

#endif