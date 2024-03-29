//
// Created by levyvonet on 31/10/2021.
//

#ifndef GYM_MAPF_MULTIAGENT_STATE_H
#define GYM_MAPF_MULTIAGENT_ACTION_H

#include <grid/grid.h>

using namespace std;

class MultiAgentAction {
public:
    /* TODO: make this a pointer */
    vector<Action> actions;
    int64_t id;

    MultiAgentAction(const vector<Action> &actions, int64_t id);

    MultiAgentAction(size_t n_agents);

    bool operator==(const MultiAgentAction &other) const;

};


class MultiAgentActionIterator {
public:
    MultiAgentAction *ptr;
    size_t n_agents;

    MultiAgentActionIterator(size_t n_agents, vector<bool> is_effective_agent);

    ~MultiAgentActionIterator();

    void reach_end();

    void reach_begin();

    MultiAgentAction *operator->() const;

    MultiAgentAction operator*() const;

    virtual MultiAgentActionIterator& operator++();

    bool operator==(const MultiAgentActionIterator &other) const;

    bool operator!=(const MultiAgentActionIterator &other) const;

private:
    vector<bool> is_effective_agent;

};

class MultiAgentActionSpace {
public:
    size_t n_agents;

    MultiAgentActionSpace(size_t n_agents);
    MultiAgentActionSpace(size_t n_agents, vector<bool> is_effective_agent);

    virtual MultiAgentActionIterator* begin();

    virtual MultiAgentActionIterator* end();

private:
    vector<bool> is_effective_agent;
};

#endif //GYM_MAPF_MULTIAGENT_STATE_H
