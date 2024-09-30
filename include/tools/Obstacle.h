#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Serializer.h"

namespace amp {

class Polygon {
    public:
        /// @brief Used for deserialization
        Polygon() = default;
        
        /// @brief Construct from a set of vertices ASSUMED to be in counter clock wise order
        /// @param vertices_ccw Ordered counter clock wise order
        Polygon(std::vector<Eigen::Vector2d>& vertices_ccw);
        
        /// @brief Access the vertices in memory of the obstacle (counter clock wise coordinates)
        /// @return Reference to stored CCW vertices //ZACK does this mean a pointer? what is "reference to"
        std::vector<Eigen::Vector2d>& verticesCCW(); //ZACK explain this? difference between this and the one below?

        /// @brief Access the vertices in memory of the obstacle (counter clock wise coordinates)
        /// @return Reference to stored CCW vertices
        const std::vector<Eigen::Vector2d>& verticesCCW() const; 

        /// @brief Create a new array of vertices that are re-ordered clock-wise 
        /// @return Clockwise vertices
        std::vector<Eigen::Vector2d> verticesCW() const; 

        void serialize(Serializer& szr) const;
        void deserialize(const Deserializer& dszr);

        /// @brief Print the object
        /// @param heading Log what type of object is being printed
        void print(const std::string& heading = "Polygon") const;
    private:
        std::vector<Eigen::Vector2d> m_vertices_ccw;
};

using Obstacle2D = Polygon; //ZACK what is this

}
