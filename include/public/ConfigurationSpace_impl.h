#pragma once
#include "tools/ConfigurationSpace.h"

template <typename T>
amp::DenseArray2D<T>::DenseArray2D(std::size_t x0_cells, std::size_t x1_cells) : DenseArray2D(x0_cells, x1_cells, T{}) { }

template <typename T>
amp::DenseArray2D<T>::DenseArray2D(std::size_t x0_cells, std::size_t x1_cells, T default_element) 
    : m_x0_cells(x0_cells), m_x1_cells(x1_cells)
{
    m_data.resize(x0_cells * x1_cells, default_element);
}

template <typename T>
std::size_t amp::DenseArray2D<T>::getWrappedIndex(std::size_t i, std::size_t j) const {
    return j * m_x0_cells + i;
}

template <typename T>
std::pair<std::size_t, std::size_t> amp::DenseArray2D<T>::size() const {
    return std::make_pair(m_x0_cells, m_x1_cells);
}

template <typename T>
typename std::vector<T>::reference amp::DenseArray2D<T>::operator()(std::size_t i, std::size_t j) {
    return m_data.at(getWrappedIndex(i, j));
}

template <typename T>
typename std::vector<T>::const_reference amp::DenseArray2D<T>::operator()(std::size_t i, std::size_t j) const {
    return m_data.at(getWrappedIndex(i, j));
}

template <typename T>
const std::vector<bool>& amp::DenseArray2D<T>::data() const {
    return m_data;
}
