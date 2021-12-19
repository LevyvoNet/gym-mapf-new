//
// Created by levyvonet on 31/10/2021.
//

#ifndef GYM_MAPF_MULTIAGENT_STATE_H
#define GYM_MAPF_MULTIAGENT_STATE_H

#include "grid/grid.h"

using namespace std;


class MultiAgentState {
public:
    /* TODO: make this a pointer */
    vector<Location> locations;
    uint64_t id;

    MultiAgentState(const vector<Location> &locations, int64_t id);

    bool operator==(const MultiAgentState &other) const;

    bool operator!=(const MultiAgentState &other) const;

    friend std::ostream& operator<<(std::ostream& os, const MultiAgentState& s);
};

class MultiAgentStateIterator {
protected:
    size_t n_agents;
    vector<GridIterator> iters;
    const Grid *grid;

public:

    MultiAgentStateIterator(const Grid *grid, size_t n_agents);

    ~MultiAgentStateIterator();

    virtual void reach_end();

    virtual void reach_begin();

    MultiAgentState *operator->() const;

    MultiAgentState operator*() const;

    virtual MultiAgentStateIterator &operator++();

    bool operator==(const MultiAgentStateIterator &other) const;

    bool operator!=(const MultiAgentStateIterator &other) const;

    void set_locations(vector<Location> locations);

    MultiAgentState *ptr;
};

class MultiAgentStateSpace {
protected:
    size_t n_agents;
    const Grid *grid;

public:
    MultiAgentStateSpace(const Grid *grid, size_t n_agents);

    virtual MultiAgentStateIterator* begin();

    virtual MultiAgentStateIterator* end();
};

#endif //GYM_MAPF_MULTIAGENT_STATE_H
