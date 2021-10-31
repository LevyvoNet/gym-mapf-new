//
// Created by levyvonet on 31/10/2021.
//

#ifndef GYM_MAPF_MULTIAGENT_STATE_H
#define GYM_MAPF_MULTIAGENT_ACTION_H

#include <gym_mapf/grid/grid.h>

using namespace std;

class MultiAgentAction {
public:
    vector<Action> actions;

    MultiAgentAction(const vector<Action> &actions);

    bool operator==(const MultiAgentAction &other) const;

};


class MultiAgentActionIterator {
private:
    MultiAgentAction *ptr;
    size_t n_agents;

public:

    MultiAgentActionIterator(size_t n_agents);

    void reach_end();

    MultiAgentAction *operator->() const;

    MultiAgentAction operator*() const;

    MultiAgentActionIterator operator++();

    bool operator==(const MultiAgentActionIterator &other) const;

    bool operator!=(const MultiAgentActionIterator &other) const;
};

class MultiAgentActionSpace {
private:
    size_t n_agents;

public:
    MultiAgentActionSpace(size_t n_agents);

    MultiAgentActionIterator begin();

    MultiAgentActionIterator end();
};

#endif //GYM_MAPF_MULTIAGENT_STATE_H