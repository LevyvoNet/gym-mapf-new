//
// Created by levyvonet on 31/10/2021.
//

#ifndef GYM_MAPF_MULTIAGENT_STATE_H
#define GYM_MAPF_MULTIAGENT_STATE_H

#include "grid/grid.h"

using namespace std;


class MultiAgentState {
public:
    vector<Location> locations;

    MultiAgentState(const vector<Location> &locations);

    bool operator==(const MultiAgentState &other) const;
};
class MultiAgentStateIterator {
private:
    MultiAgentState *ptr;
    size_t n_agents;
    vector<GridIterator> iters;
    const Grid *grid;

public:

    MultiAgentStateIterator(const Grid *grid, size_t n_agents);

    void reach_end();

    MultiAgentState *operator->() const;

    MultiAgentState operator*() const;

    MultiAgentStateIterator operator++();

    bool operator==(const MultiAgentStateIterator &other) const;

    bool operator!=(const MultiAgentStateIterator &other) const;
};

class MultiAgentStateSpace {
private:
    size_t n_agents;
    const Grid *grid;

public:
    MultiAgentStateSpace(const Grid *grid, size_t n_agents);

    MultiAgentStateIterator begin();

    MultiAgentStateIterator end();
};

#endif //GYM_MAPF_MULTIAGENT_STATE_H
