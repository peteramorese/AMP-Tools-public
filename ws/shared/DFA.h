#pragma once
#include "Triangulate.h"
#include "MyAStar.h"
#include <iostream>
#include <utility>
#include <unordered_map>
#include <vector>
#include <string>
#include <set>


// Struct to represent a transition between states
struct Transition {
    uint32_t sourceState;
    uint32_t targetState;
    char inputSymbol;
};

// struct HybridState {
//     int q;
//     xState x;
//     std::vector<int> transitions;
//     std::vector<double> weights;
// };

// Class to represent the DFA
class DFA {
public:

    // Add a state to the DFA
    void addState(uint32_t stateId) {
        states.insert(stateId);
    }

    void addStates(vector<int> stateIds) {
        for (uint32_t stateId : stateIds)
            states.insert(stateId);
    }

    // Add an input symbol to the DFA
    void addSymbol(char symbol) {
        inputSymbols.insert(symbol);
    }

    void addSymbols(vector<char> symbols) {
        for (char symbol : symbols)
            inputSymbols.insert(symbol);
    }

    // Add a transition to the DFA
    void addTransition(uint32_t source, char symbol, uint32_t target) {
        transitions[{source, symbol}] = target;
    }

    // Set the initial state of the DFA
    void setInitialState(uint32_t stateId) {
        initialState = stateId;
    }

    // Add an accepting state to the DFA
    void addAcceptingState(uint32_t stateId) {
        accStates.insert(stateId);
    }

    // Check if a string is accepted by the DFA
    bool acceptString(const std::string& str) {
        uint32_t currentState = initialState;
        for (char symbol : str) {
            if (inputSymbols.find(symbol) == inputSymbols.end()) {
                std::cerr << "Invalid input symbol: " << symbol << std::endl;
                return false;
            }
            currentState = transitions[{currentState, symbol}];
        }
        return accStates.find(currentState) != accStates.end();
    }

    uint32_t transition(uint32_t state, char symbol) const {
        auto it = transitions.find({state, symbol});
        if (it != transitions.end()) return it->second;
        else return state;
    }

    std::set<uint32_t> getStates() const {
        return states;
    }

    bool isAccepting(uint32_t q) const {
        return accStates.find(q) != accStates.end();
    }

    std::set<uint32_t> getAcceptingStates() const {
        return accStates;
    }

private:
    std::set<uint32_t> states;
    std::set<char> inputSymbols;
    uint32_t initialState;
    std::set<uint32_t> accStates;
    std::map<std::pair<uint32_t, char>, uint32_t> transitions; 
};

// Product automaton class definition
class ProductAutomaton {
public:
    // Constructor to build the product automaton
    ProductAutomaton(const DFA& dfa, const std::unordered_map<uint32_t, abstractionNode>& abstraction) : dfa(dfa), abstraction(abstraction) {
        combineStateSpaces();
        combineTransitionFunctions();
    }

    MyAStarAlgo::GraphSearchResult searchPath(u_int32_t init, std::map<uint32_t, std::vector<double>> dynamicWeights) {
        std::cout << "\nSearching Product Graph\n";
        std::cout << "Size of neighborMap "<< neighborMap.size() << std::endl;
        std::cout << "Size of weightMap " << weightMap.size() << std::endl;
        MyAStarAlgo algo(true);
        return algo.search2(init, acceptingStates, neighborMap, dynamicWeights);
    }

    void updateWeights() {

    }

    const std::unordered_map<uint32_t, abstractionNode>& getAbstraction() const {
        return abstraction;
    }

    const std::map<uint32_t, std::pair<uint32_t, uint32_t>>& getHybridMap() const {
        return hybridStates;
    }

    const std::map<std::pair<uint32_t, uint32_t>, uint32_t>& getHybridMapBack() const {
        return hybridMapBack;
    }

    const std::map<uint32_t, std::vector<uint32_t>>& getNeighborMap() const {
        return neighborMap;
    }
    
    const std::map<uint32_t, std::vector<double>>& getWeightMap() const {
        return weightMap;
    }

    const std::set<uint32_t>& getAcceptingStates() const {
        return acceptingStates;
    }

private:
    const DFA& dfa;
    const std::unordered_map<uint32_t, abstractionNode>& abstraction;
    std::map<uint32_t, std::pair<uint32_t, uint32_t>> hybridStates;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> hybridMapBack;
    std::map<uint32_t, std::vector<uint32_t>> neighborMap;
    std::map<uint32_t, std::vector<double>> weightMap;
    std::set<uint32_t> acceptingStates;
    int initialState;
    // std::map<int, std::unordered_set<char>> graphLabels;
    // std::map<std::pair<int, char>, int> combinedTransitions;
    // std::map<int, std::pair<int, int>> hybridMap;
    // std::map<int, HybridState> hybridGraph;


    // Method to combine state spaces
    void combineStateSpaces() {
        uint32_t i = 0;
        auto Q = dfa.getStates();
        for (const auto& pair : abstraction) {
            for (uint32_t q : Q) {
                // HybridState hybridState = {q, pair.second};
                hybridStates[i] = {q, pair.first};
                hybridMapBack[{q, pair.first}] = i;
                // std::cout << "Hybrid state states: (" << q << ", " << pair.first << ")" << std::endl;
                if (dfa.isAccepting(q)) acceptingStates.insert(i);
                i++;
            }
        }
        std::cout << "Accepting states: ";
        for (auto it = acceptingStates.begin(); it != acceptingStates.end(); ++it) std::cout << *it << " ";
        std::cout << std::endl;
    }

    // Method to combine transition functions
    void combineTransitionFunctions() {
        for (const auto state : hybridStates) {
            uint32_t q = state.second.first;
            uint32_t r = state.second.second;
            const abstractionNode region = abstraction.at(r);
            char o = region.observation;
            // std::cout << "Mode State: " << q << std::endl;
            // std::cout << "Observation: " << o << std::endl;
            uint32_t qp = dfa.transition(q, o);
            neighborMap[state.first] = {};
            for (uint32_t i = 0; i < region.neighbors.size(); ++i) {
                uint32_t r_n = region.neighbors[i];
                uint32_t pp = hybridMapBack[{qp, r_n}];
                neighborMap[state.first].push_back(pp); // Key: hybrid state index. Value: vector of hybrid state indices
                weightMap[state.first].push_back(1.0);
            }
            // std::cout << "Neighbors for " << state.first << ": " << neighborMap[state.first].size() << std::endl;
        }
        // for (const auto& entry : neighborMap) {
        //     uint32_t key = entry.first;
        //     const std::vector<uint32_t>& values = entry.second;
        //     std::cout << "Key: " << key << ", Values: ";
        //     for (const double value : values) std::cout << value << " ";
        //     std::cout << std::endl;
        // }
        // for (const auto& entry : weightMap) {
        //     uint32_t key = entry.first;
        //     const std::vector<double>& values = entry.second;
        //     std::cout << "Key: " << key << ", Values: ";
        //     for (const double value : values) std::cout << value << " ";
        //     std::cout << std::endl;
        // }
    }
};

DFA createDFA() {
    // Create a DFA representing the language of strings over {0,1} that end with '0'
    DFA dfa;
    dfa.addStates({0, 1, 2, 3});
    dfa.addSymbols({'e', 'a', 'g', 'u', 'o'});
    dfa.addTransition(0, 'e', 0);
    dfa.addTransition(1, 'e', 1);
    dfa.addTransition(2, 'e', 2);
    dfa.addTransition(0, 'a', 1);
    dfa.addTransition(1, 'g', 2);
    dfa.addTransition(0, 'u', 2);
    dfa.addTransition(0, 'o', 3);
    dfa.addTransition(1, 'o', 3);
    dfa.addTransition(2, 'o', 3);
    dfa.setInitialState(0);
    dfa.addAcceptingState(2);

    return dfa;
}