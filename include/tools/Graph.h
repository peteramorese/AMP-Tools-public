#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <type_traits>
#include <memory>

#include "tools/Logging.h"
#include "public/GraphContainers.h"

namespace amp {

/// @brief Alias for the default node type for readability
using Node = uint32_t;

/// @brief Directed graph class that stores edge connections between nodes (integer type)
/// @tparam EDGE_T Data type of the edge. NOTE: Must have std::cout stream overload to use print().
/// @tparam NATIVE_NODE_T Data type of the node 'label'. NOTE: Must be an indexable integer type.
/// @tparam REVERSIBLE Allow efficient traversal backwards, while using occupying more memory. 
/// NOTE: Must be `true` to enable `incomingEdges` and `parents` methods.
template<class EDGE_T, typename NATIVE_NODE_T = Node, bool REVERSIBLE = true>
class Graph {
	public:
	 	typedef EDGE_T EdgeType; // used for dependent types
	 	typedef NATIVE_NODE_T NodeType; // used for dependent types

	public:
		Graph() = default;

		/// @brief Determine if the graph is reversible
		/// @return `true` if the graph is reversible
		constexpr bool isReversible();

		/// @brief Get the set of all nodes
		/// @return Constructed array of node 'labels'
		std::vector<NATIVE_NODE_T> nodes() const;

		/// @brief Connect two nodes with an edge, going from the source node to the destination node
		/// @param src Source node (start)
		/// @param dst Destination node (end)
		/// @param edge Edge going from `src` to `dst`
		void connect(NATIVE_NODE_T src, NATIVE_NODE_T dst, const EDGE_T& edge);

		/// @brief Read the set of all child nodes of a given node. NOTE: Returned in the same respective order as edges in `outgoingEdges()`
		/// @param node Node 
		/// @return Set of all child nodes
		inline const std::vector<NATIVE_NODE_T>& children(NATIVE_NODE_T node) const;
		
		/// @brief Read the set of all outgoing edges from a given node. NOTE: Returned in the same respective order as edges in `children()`
		/// @param node Node 
		/// @return Set of all outgoing edges
		inline const std::vector<EDGE_T>& outgoingEdges(NATIVE_NODE_T node) const;

		/// @brief Read the set of all parent nodes of a given node. Graph must be REVERSIBLE to enable this method.
		/// NOTE: Returned in the same respective order as edges in `incomingEdges()`.
		/// @param node Node 
		/// @return Set of all parent nodes
		inline const std::vector<NATIVE_NODE_T>& parents(NATIVE_NODE_T node) const;

		/// @brief Read the set of all incoming edges from a given node. Graph must be REVERSIBLE to enable this method.
		/// NOTE: Returned in the same respective order as edges in `parents()`.
		/// @param node Node
		/// @return Set of all incoming edges
		inline const std::vector<EDGE_T>& incomingEdges(NATIVE_NODE_T node) const;
		
		/// @brief Disconnect all edges between two nodes
		/// @param src Source node
		/// @param dst Destination node
		/// @return `true` if at least one edge was disconnected, `false` if none were found
		bool disconnect(NATIVE_NODE_T src, NATIVE_NODE_T dst);

		/// @brief Disconnect all edges that match a given edge (operator==) between two nodes
		/// @param src Source node
		/// @param dst Destination node
		/// @param edge Matching edge
		/// @return `true` if at least one edge was disconnected, `false` if none were found
		bool disconnect(NATIVE_NODE_T src, NATIVE_NODE_T dst, const EDGE_T& edge);

		/// @brief Reverse the order graph (parents become children and visa versa)
		void reverse();

		/// @brief Print the graph
		void print(const std::string& heading = "Graph") const;

		/// @brief Remove all contents of the graph
		void clear();

	private: 
		inline std::size_t size() const;
	
	private:
		ampprivate::RandomAccessList<ampprivate::BidirectionalConnectionList<EDGE_T, NATIVE_NODE_T>> m_graph;
};

/// @brief Search heuristic for A* graph search problems
struct SearchHeuristic {
	/// @brief Default heuristic that just returns 0. Override this to use a more informative heuristic. Make sure that the 
	/// heuristic is admissible.
	/// @param node Node to get the heuristic value h(node) for. 
	/// @return Heuristic value
	virtual double operator()(amp::Node node) const {return 0.0;}
};

/// @brief Object containing a graph and two nodes to find the shortest path between
struct ShortestPathProblem {
	/// @brief Graph object
	std::shared_ptr<Graph<double>> graph;

	/// @brief Initial node to search from
	amp::Node init_node;

	/// @brief Goal node to find the shortest path to
	amp::Node goal_node;
};

class GraphTools {
	public:
		/// @brief Generate a random graph with `double` type edge weights
		/// @param n_nodes Number of nodes in the graph
		/// @param min_edge_weight Minimum edge weight
		/// @param max_edge_weight Maximum edge weight
		/// @param max_outgoing_edges_per_node Maximum spanning factor for outgoing edges. Increase for more edges in graph
		/// @param connectedness Number in [0, 1] that dictates how connected the graph is (0 produces a DAG, 1 has high connectivity)
		/// @return Shared point to a graph object
		static std::shared_ptr<Graph<double>> generateRandomGraphDouble(uint32_t n_nodes, double min_edge_weight = 0.0, double max_edge_weight = 10.0, uint32_t max_outgoing_edges_per_node = 5, double connectedness = 0.5, uint32_t seed = 0u);

		/// @brief Generate a random Shortest Path Problem
		/// @param n_nodes Number of nodes in the graph
		/// @param min_edge_weight Minimum edge weight
		/// @param max_edge_weight Maximum edge weight
		/// @param max_outgoing_edges_per_node Maximum spanning factor for outgoing edges. Increase for more edges in graph
		/// @param connectedness Number in [0, 1] that dictates how connected the graph is (0 produces a DAG, 1 has high connectivity)
		/// @return Shortest Path Problem
		static ShortestPathProblem generateRandomSPP(uint32_t n_nodes, double min_edge_weight = 0.0, double max_edge_weight = 10.0, uint32_t max_outgoing_edges_per_node = 5, double connectedness = 0.5, uint32_t seed = 0u);
};
}


#include "public/Graph_impl.h"