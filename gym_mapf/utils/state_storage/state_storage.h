//
// Created by levyvonet on 31/10/2021.
//

/* Here there are going to be generic dictionary-like structures for storing multi agent states (MultiAgentState as key) */

#ifndef GYM_MAPF_STATE_STORAGE_H
#define GYM_MAPF_STATE_STORAGE_H

//#include <unordered_map>

#include <tsl/hopscotch_map.h>

#include <multiagent_action/multiagent_action.h>
#include <multiagent_state/multiagent_state.h>

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
    tsl::hopscotch_map<MultiAgentState, double> *v;

public:
    ValueTable(double default_value);

    ValueTable(const ValueTable &other);

    double operator[](const MultiAgentState &s);

    void set(const MultiAgentState &s, double value);
};

/**
 * Each agent here has its own hash table which its key is the agents local state (Location)
 * @tparam T
 */
template<typename T>
class MultiAgentStateStorage {
private:
    size_t n_agents;
    T default_value;
    tsl::hopscotch_map<Location, void *> *nested_hashmap;

public:
    MultiAgentStateStorage(size_t n_agents, T default_value);

    void set(const MultiAgentState &s, T value);

    T get(const MultiAgentState &s);
};


template<typename T>
void MultiAgentStateStorage<T>::set(const MultiAgentState &s, T value) {
    tsl::hopscotch_map<Location, void *> *d = NULL;
    tsl::hopscotch_map<Location, T> *d_last = NULL;
    d = this->nested_hashmap;
    size_t i = 0;

    if (this->n_agents > 2) {
        for (i = 0; i < this->n_agents - 2; ++i) {
            if (d->find(s.locations[i]) == d->end()) {
                (*d)[s.locations[i]] = new tsl::hopscotch_map<Location, void *>();
            }
            d = (tsl::hopscotch_map<Location, void *> *) ((*d)[s.locations[i]]);
        }
    }

    /* The last one is different */
    if (d->find(s.locations[i]) == d->end()) {
        (*d)[s.locations[i]] = new tsl::hopscotch_map<Location, T>();
    }
    d_last = (tsl::hopscotch_map<Location, T> *) ((*d)[s.locations[i]]);
    ++i;

    (*d_last)[s.locations[i]] = value;
}

template<typename T>
T MultiAgentStateStorage<T>::get(const MultiAgentState &s) {
    tsl::hopscotch_map<Location, void *> *d = NULL;
    tsl::hopscotch_map<Location, T> *d_last = NULL;
    d = this->nested_hashmap;
    size_t i = 0;

    if (this->n_agents > 2) {
        for (i = 0; i < this->n_agents - 2; ++i) {
            if (d->find(s.locations[i]) == d->end()) {
                return this->default_value;
            }
            d = (tsl::hopscotch_map<Location, void *> *) ((*d)[s.locations[i]]);
        }
    }

    /* The last one is different */
    if (d->find(s.locations[i]) == d->end()) {
        return this->default_value;
    }
    d_last = (tsl::hopscotch_map<Location, T> *) ((*d)[s.locations[i]]);
    ++i;


    if (d_last->find(s.locations[i]) == d_last->end()) {
        return this->default_value;
    }

    return (*d_last)[s.locations[i]];
}

template<typename T>
MultiAgentStateStorage<T>::MultiAgentStateStorage(size_t n_agents, T default_value) {
    this->n_agents = n_agents;
    this->default_value = default_value;
    this->nested_hashmap = new tsl::hopscotch_map<Location, void *>();
}


#endif //GYM_MAPF_STATE_STORAGE_H
