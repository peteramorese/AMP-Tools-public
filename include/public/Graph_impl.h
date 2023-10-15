#pragma once

#include "tools/Graph.h"

namespace amp {

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
inline std::size_t amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::size() const {return m_graph.size();}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
constexpr bool amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::isReversible() {return REVERSIBLE;}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
std::vector<NATIVE_NODE_T> amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::nodes() const {
	std::vector<NATIVE_NODE_T> nodes;
	nodes.reserve(m_graph.size());
	for (Node n=0; n<m_graph.size(); ++n) {
		if (!m_graph[n].forward.empty() || !m_graph[n].backward.empty()) 
			nodes.push_back(n);
	}
	return nodes;
}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
void amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::connect(NATIVE_NODE_T src, NATIVE_NODE_T dst, const EDGE_T& edge) {
	if (src >= size() || dst >= size()) {
		NATIVE_NODE_T max_ind = (src > dst) ? src : dst;
		m_graph.resize(max_ind + 1);
	}
	m_graph[src];
	m_graph[src].forward.pushConnect(dst, edge);

	if constexpr (REVERSIBLE) m_graph[dst].backward.pushConnect(src, edge);
}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
inline const std::vector<EDGE_T>& amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::outgoingEdges(NATIVE_NODE_T node) const {
	return m_graph[node].forward.edges;
}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
inline const std::vector<NATIVE_NODE_T>& amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::children(NATIVE_NODE_T node) const {
	return m_graph[node].forward.nodes;
}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
inline const std::vector<EDGE_T>& amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::incomingEdges(NATIVE_NODE_T node) const {
	static_assert(REVERSIBLE, "Graph must be reversible");
	return m_graph[node].backward.edges;
}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
inline const std::vector<NATIVE_NODE_T>& amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::parents(NATIVE_NODE_T node) const {
	static_assert(REVERSIBLE, "Graph must be reversible");
	return m_graph[node].backward.nodes;
}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
bool amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::disconnect(NATIVE_NODE_T src, NATIVE_NODE_T dst) {
	ASSERT(src < size() && dst < size(), "Disconnection nodes not found");
	bool found = m_graph[src].forward.disconnect(dst);
	if (!found) return false;

	if constexpr (REVERSIBLE) {
		found = m_graph[dst].backward.disconnect(src);
		ASSERT(found, "Disconnection found in forward, but not backward");
	}
	return true;
}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
bool amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::disconnect(NATIVE_NODE_T src, NATIVE_NODE_T dst, const EDGE_T& edge) {
	ASSERT(src < size() && dst < size(), "Disconnection nodes not found");
	bool found = m_graph[src].forward.disconnect(dst, edge);
	if (!found) return false;

	if constexpr (REVERSIBLE) {
		found = m_graph[dst].backward.disconnect(src);
		ASSERT(found, "Disconnection found in forward, but not backward");
	}
	return true;
}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
void amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::reverse() {
	static_assert(REVERSIBLE, "Cannot reverse a graph that is not reversible");
	for (auto& adj_list : m_graph) {
		swap(adj_list.forward, adj_list.backward);
	}
}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
void amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::print(const std::string& heading) const {
    if (!heading.empty())
        LOG(heading + ":");
	NATIVE_NODE_T node = 0;
	for (const auto& list : m_graph) {
		for (uint32_t i=0; i < list.forward.size(); ++i) {
			if (i == 0) PRINT_NAMED("Node " << node, "is connected to:");
			PRINT_NAMED("    - child node " << list.forward.nodes[i], "with edge: " << list.forward.edges[i]);
		}
		++node;
	}
}

template<class EDGE_T, typename NATIVE_NODE_T, bool REVERSIBLE>
void amp::Graph<EDGE_T, NATIVE_NODE_T, REVERSIBLE>::clear() {m_graph.clear();}

}