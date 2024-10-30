#ifndef AMP_EXCLUDE_VIS

#include "public/PythonObjects.h"
#include "tools/Logging.h"

template <>
std::unique_ptr<ampprivate::pybridge::PythonObject> ampprivate::pybridge::ListOfPairs<double>::toPyList() const {
    std::unique_ptr<PythonObject> list = std::make_unique<PythonObject>(PyList_New(0));
    for (const auto& tuple : list_of_tuples) {
        PyObject* py_tuple = PyTuple_Pack(2, 
            PyFloat_FromDouble(tuple[0]), 
            PyFloat_FromDouble(tuple[1]));
        PyList_Append(list->get(), py_tuple);
        list->addDependentObject(py_tuple);
    }
    return list;
}

template <>
std::unique_ptr<ampprivate::pybridge::PythonObject> ampprivate::pybridge::ListOfTriples<double>::toPyList() const {
    std::unique_ptr<PythonObject> list = std::make_unique<PythonObject>(PyList_New(0));
    for (const auto& tuple : list_of_tuples) {
        PyObject* py_tuple = PyTuple_Pack(3, 
            PyFloat_FromDouble(tuple[0]), 
            PyFloat_FromDouble(tuple[1]),
            PyFloat_FromDouble(tuple[2]));
        PyList_Append(list->get(), py_tuple);
        list->addDependentObject(py_tuple);
    }
    return list;
}

template <>
std::unique_ptr<ampprivate::pybridge::PythonObject> ampprivate::pybridge::ListOfQuadrouples<double>::toPyList() const {
    std::unique_ptr<PythonObject> list = std::make_unique<PythonObject>(PyList_New(0));
    for (const auto& tuple : list_of_tuples) {
        PyObject* py_tuple = PyTuple_Pack(4, 
            PyFloat_FromDouble(tuple[0]), 
            PyFloat_FromDouble(tuple[1]), 
            PyFloat_FromDouble(tuple[2]), 
            PyFloat_FromDouble(tuple[3]));
        PyList_Append(list->get(), py_tuple);
        list->addDependentObject(py_tuple);
    }
    return list;
}

std::unique_ptr<ampprivate::pybridge::PythonObject> ampprivate::pybridge::makeBool(bool boolean) {
    return std::make_unique<ampprivate::pybridge::PythonObject>(PyBool_FromLong(static_cast<long>(boolean)));
}

std::unique_ptr<ampprivate::pybridge::PythonObject> ampprivate::pybridge::makeLong(uint64_t integer) {
    return std::make_unique<ampprivate::pybridge::PythonObject>(PyLong_FromLong(integer));
}

std::unique_ptr<ampprivate::pybridge::PythonObject> ampprivate::pybridge::makeScalar(double scalar) {
    return std::make_unique<ampprivate::pybridge::PythonObject>(PyFloat_FromDouble(scalar));
}

std::unique_ptr<ampprivate::pybridge::PythonObject> ampprivate::pybridge::makeString(const std::string& string) {
    return std::make_unique<ampprivate::pybridge::PythonObject>(PyUnicode_FromString(string.c_str()));
}

std::unique_ptr<ampprivate::pybridge::PythonObject> ampprivate::pybridge::makeList(std::vector<std::unique_ptr<PythonObject>>&& list_elements) {
    std::unique_ptr<PythonObject> list = std::make_unique<PythonObject>(PyList_New(0));
    for (auto& element : list_elements) {
        // Append the element
        PyList_Append(list->get(), element->get());
        // Add the element as a dependent object
        list->addDependentObject(element->get());
        // Release ownership of the object because it will be destroyed in the list
        element.release();
    }
    return list;
}

std::unique_ptr<ampprivate::pybridge::PythonObject> ampprivate::pybridge::makePair(std::pair<std::unique_ptr<PythonObject>, std::unique_ptr<PythonObject>>&& tuple_elements) {
    std::unique_ptr<PythonObject> tuple = std::make_unique<PythonObject>(PyTuple_Pack(2, tuple_elements.first->get(), tuple_elements.second->get()));

    // Add the elements as a dependent object
    tuple->addDependentObject(tuple_elements.first->get());
    tuple->addDependentObject(tuple_elements.second->get());

    // Release ownership of the object because it will be destroyed in the tuple
    tuple_elements.first.release();
    tuple_elements.second.release();
    return tuple;
}

#endif