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

    friend std::ostream &operator<<(std::ostream &os, const MultiAgentState &s);
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

    virtual MultiAgentStateIterator *begin();

    virtual MultiAgentStateIterator *end();
};


class AreaMultiAgentStateIterator : public MultiAgentStateIterator {
public:
    GridArea area;

    void _reach_begin();

    AreaMultiAgentStateIterator(const Grid *grid, GridArea area, size_t n_agents, vector<bool> is_effective_agent,
                                const MultiAgentState &current_state);

    virtual void reach_begin() override;

    virtual MultiAgentStateIterator &operator++() override;

private:
    size_t next_effective_agent(size_t curr_agent);
    vector<bool> is_effective_agent;
    const MultiAgentState& current_state;
    MultiAgentState dummy_state = MultiAgentState({}, -1);
};

class AreaMultiAgentStateSpace : public MultiAgentStateSpace {
public:
    AreaMultiAgentStateSpace(const Grid *grid, GridArea area, size_t n_agents);

    AreaMultiAgentStateSpace(const Grid *grid, GridArea area, size_t n_agents, vector<bool> is_effective_agent,
                             const MultiAgentState &current_state);

    virtual AreaMultiAgentStateIterator *begin() override;

    virtual AreaMultiAgentStateIterator *end() override;

protected:
    GridArea area;
    vector<bool> is_effective_agent;
    const MultiAgentState &current_state;
    MultiAgentState dummy_state = MultiAgentState({}, -1);


};


class GirthMultiAgentStateIterator : public MultiAgentStateIterator {

public:
    GridArea area;

    GirthMultiAgentStateIterator(const Grid *grid, GridArea area, size_t n_agents);

    virtual MultiAgentStateIterator &operator++() override;

    void _reach_begin();

    virtual void reach_begin() override;

};

class GirthMultiAgentStateSpace : public MultiAgentStateSpace {
protected:
    GridArea area;

public:
    GirthMultiAgentStateSpace(const Grid *grid, GridArea area, size_t n_agents);

    virtual GirthMultiAgentStateIterator *begin() override;

    virtual GirthMultiAgentStateIterator *end() override;
};

#endif //GYM_MAPF_MULTIAGENT_STATE_H
