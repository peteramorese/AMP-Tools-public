#pragma once

#include <vector>
#include <list>
#include <tuple>

#include "tools/Logging.h"

namespace ampprivate {

template <class T>
class RandomAccessList {
    public:
        RandomAccessList() = default;
        RandomAccessList(RandomAccessList&&) = default;
        RandomAccessList(const RandomAccessList& other) {
            m_list = other.m_list;
            m_access_ptrs.reserve(m_list.size());
            for (auto it = m_list.begin(); it != m_list.end(); ++it) {
                m_access_ptrs.push_back(&(*it));
            }
        }

        void operator=(const RandomAccessList& other) {
            m_list = other.m_list;
            m_access_ptrs.reserve(m_list.size());
            for (auto it = m_list.begin(); it != m_list.end(); ++it) {
                m_access_ptrs.push_back(&(*it));
            }
        }

        void push_back(const T& value) {
            m_list.push_back(value);
            m_access_ptrs.push_back(&m_list.back());
        }

        T& operator[](uint32_t i) {return *m_access_ptrs[i];}
        const T& operator[](uint32_t i) const {return *m_access_ptrs[i];}

        void clear() {
            m_list.clear();
            m_access_ptrs.clear(); 
            m_access_ptrs.shrink_to_fit();
        }
        void resize(std::size_t count) {
            auto list_it = std::prev(m_list.end());
            m_list.resize(count);
            m_access_ptrs.reserve(count);
            while (m_access_ptrs.size() < m_access_ptrs.capacity()) {
                m_access_ptrs.insert(m_access_ptrs.end(), &(*(++list_it)));
            }
        }
        std::size_t size() const {return m_list.size();}

        typename std::list<T>::iterator begin() {return m_list.begin();}
        typename std::list<T>::iterator end() {return m_list.end();}
        typename std::list<T>::const_iterator begin() const {return m_list.cbegin();}
        typename std::list<T>::const_iterator end() const {return m_list.cend();}

        friend void swap(RandomAccessList& lhs, RandomAccessList& rhs) {
            std::swap(lhs.m_list, rhs.m_list);
            std::swap(lhs.m_access_ptrs, rhs.m_access_ptrs);
        }
    private:
        std::list<T> m_list;
        std::vector<T*> m_access_ptrs;
};

template<class EDGE_T, typename NATIVE_NODE_T>
struct AdjacencyList {
    std::size_t pushConnect(NATIVE_NODE_T dst_node, const EDGE_T& edge) {
        edges.push_back(edge);
        nodes.push_back(dst_node);
        return nodes.size();
    }
    template <typename ... Args>
    constexpr std::size_t emplaceConnect(NATIVE_NODE_T dst_node, Args&& ... args) {
        edges.emplace_back(std::forward<Args>(args)...);
        nodes.push_back(dst_node);
        return nodes.size();
    }
    std::size_t disconnect(NATIVE_NODE_T dst_node) {
        auto removeConnection = [dst_node] (NATIVE_NODE_T n, const EDGE_T& e) -> bool {
            return n == dst_node;
        };
        return disconnectIf(removeConnection);
    }
    std::size_t disconnect(NATIVE_NODE_T dst_node, const EDGE_T& edge) {
        auto removeConnection = [dst_node, &edge] (NATIVE_NODE_T n, const EDGE_T& e) -> bool {
            return n == dst_node && e == edge;
        };
        return disconnectIf(removeConnection);
    }
    template <typename LAM>
    std::size_t disconnectIf(LAM removeConnection) {
        std::size_t n_removed;
        auto e_it = edges.begin();
        for (auto n_it = nodes.begin(); n_it != nodes.end();) {
            if (removeConnection(*n_it, *e_it)) {
                nodes.erase(n_it);
                edges.erase(e_it);
                ++n_removed;
            } else {
                ++n_it;
                ++e_it;
            }
        }
        return n_removed;
    }

    inline std::size_t size() const {return edges.size();}

    inline bool empty() const {return edges.empty();}

    friend void swap(AdjacencyList& lhs, AdjacencyList& rhs) {
        std::swap(lhs.nodes, rhs.nodes);
        std::swap(lhs.edges, rhs.edges);
    }

    std::vector<NATIVE_NODE_T> nodes;
    std::vector<EDGE_T> edges;
};

template<class EDGE_T, typename NATIVE_NODE_T>
struct BidirectionalConnectionList {
    AdjacencyList<EDGE_T, NATIVE_NODE_T> forward;
    AdjacencyList<EDGE_T, NATIVE_NODE_T> backward;
};

}