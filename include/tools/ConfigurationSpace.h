#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Serializer.h"

namespace amp {

/// @brief User implemented abstract class that accesses the continuous C-Space (bounded)
class ConfigurationSpace2D {
    public:
        /// @brief Constructor
        /// @param x0_min Lower bound on first configuration dimension
        /// @param x0_max Upper bound on first configuration dimension
        /// @param x1_min Lower bound on second configuration dimension
        /// @param x1_max Upper bound on second configuration dimension
        ConfigurationSpace2D(double x0_min, double x0_max, double x1_min, double x1_max)
            : m_x0_bounds(x0_min, x0_max)
            , m_x1_bounds(x1_min, x1_max)
            {}

        /******* User Implemented Methods ********/

        /// @brief Access the C-space with continuous variables (interpolation between cells)
        /// @param x0 Value of the first configuration variable
        /// @param x1 Value of the second configuration variable
        /// @return `true` if the the point is in collision, `false` if it is not
        virtual bool inCollision(double x0, double x1) const = 0;

        /*****************************************/

        /// @brief Get bounds for the first config dimension
        /// @return Bounds
        inline const std::pair<double, double>& x0Bounds() const {
            return m_x0_bounds;
        }

        /// @brief Get bounds for the second config dimension
        /// @return Bounds
        inline const std::pair<double, double>& x1Bounds() const {
            return m_x1_bounds;
        }

        /// @brief Virtual dtor
        virtual ~ConfigurationSpace2D() {}
    protected:
        std::pair<double, double> m_x0_bounds;
        std::pair<double, double> m_x1_bounds;
};

/* Some of the tools below might help you define your C-Space */

template <typename T = bool>
class DenseArray2D {
    public:
        /// @brief Constructor that initializes data to T{}
        /// @param x0_cells Number of cells along the first configuration dimension
        /// @param x1_cells Number of cells along the second configuration dimension
        DenseArray2D(std::size_t x0_cells, std::size_t x1_cells);

        /// @brief Constructor that initializes data to T{}
        /// @param x0_cells Number of cells along the first configuration dimension
        /// @param x1_cells Number of cells along the second configuration dimension
        /// @param default_element Use custom default element to fill array with
        DenseArray2D(std::size_t x0_cells, std::size_t x1_cells, T default_element);

        /// @brief Get the # of cells along x0 and number of cells along x1
        /// @return 
        inline std::pair<std::size_t, std::size_t> size() const;

        /// @brief Edit/Access an element with bounds checking
        /// @param i x0 index
        /// @param j x1 index
        /// @return Reference to the element (for use with vector<bool>)
        inline typename std::vector<T>::reference operator()(std::size_t i, std::size_t j);

        /// @brief Read an element with bounds checking
        /// @param i x0 index
        /// @param j x1 index
        /// @return Const reference to the element
        inline typename std::vector<T>::const_reference operator()(std::size_t i, std::size_t j) const;

        /// @brief Read the wrapped data
        /// @return Wrapped data
        inline const std::vector<bool>& data() const;

        /// @brief Virtual dtor
        virtual ~DenseArray2D() {}

    private:
        inline std::size_t getWrappedIndex(std::size_t i, std::size_t j) const;

    private:
        std::vector<T> m_data;
        const std::size_t m_x0_cells, m_x1_cells;
};

/// @brief Abstract type that can be used with the Visualizer
class GridCSpace2D : public ConfigurationSpace2D, public DenseArray2D<bool> {
    public:
        GridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : ConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max)
            , DenseArray2D<bool>(x0_cells, x1_cells)
            {}

        /// @brief Virtual dtor
        virtual ~GridCSpace2D() {}
};

}

#include "public/ConfigurationSpace_impl.h"
