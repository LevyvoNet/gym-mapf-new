//
// Created by levyvonet on 31/10/2021.
//

/* Here there are going to be generic dictionary-like structures for storing multi agent states (MultiAgentState as key) */

#ifndef GYM_MAPF_STATE_STORAGE_H
#define GYM_MAPF_STATE_STORAGE_H

#include <gym_mapf/multiagent_action/multiagent_action.h>
#include <gym_mapf/multiagent_state/multiagent_state.h>

using namespace std;


template<>
class std::hash<Location> {
public:
    size_t operator()(const Location &l) const;
};


template<>
class std::hash<MultiAgentState> {
public:
    size_t operator()(const MultiAgentState &s) const;
};


template<>
class std::hash<MultiAgentAction> {
public:
    size_t operator()(const MultiAgentAction &a) const;
};

class ValueTable {
private:
    double default_value;
    std::unordered_map<MultiAgentState, double> *v;

public:
    ValueTable(double default_value);

    ValueTable(const ValueTable &other);

    double operator[](const MultiAgentState &s);

    void set(const MultiAgentState &s, double value);
};


#endif //GYM_MAPF_STATE_STORAGE_H
