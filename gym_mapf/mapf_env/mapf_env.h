//
// Created by levyvonet on 21/10/2021.
//

#ifndef GYM_MAPF_MAPF_ENV_H
#define GYM_MAPF_MAPF_ENV_H

#include <list>
#include <iterator>
#include <unordered_map>

#include <gym_mapf/grid/grid.h>

using namespace std;

class MultiAgentState {
public:
    vector<Location> locations;

    MultiAgentState(const vector<Location> &locations);

    bool operator==(const MultiAgentState &other) const;
};

template<>
class std::hash<MultiAgentState> {
public:
    inline size_t operator()(const MultiAgentState &s) const;
};

inline size_t hash<MultiAgentState>::operator()(const MultiAgentState &s) const {
    size_t h = 0;
    int i = 0;

    for (i = 0; i < s.locations.size(); i++) {
        h += i ^ hash<Location>()(s.locations[i]);
    }

    return h;
}

/* Forward declaration */
class MapfEnv;

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

class MultiAgentAction {
public:
    vector<Action> actions;

    MultiAgentAction(const vector<Action> &actions);

    bool operator==(const MultiAgentAction &other) const;

};

template<>
class std::hash<MultiAgentAction> {
public:
    inline size_t operator()(const MultiAgentAction &a) const;
};

size_t hash<MultiAgentAction>::operator()(const MultiAgentAction &a) const {
    size_t h = 0;
    for (size_t i = 0; i < a.actions.size(); ++i) {
        h += i ^ a.actions[i];
    }

    return h;
}


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


class Transition {
public:
    double p;
    MultiAgentState *next_state;
    int reward;
    bool done;
    bool is_collision;

    Transition(double p,
               MultiAgentState *next_state,
               int reward,
               bool done,
               bool is_collision);

    Transition(double p,
               const MultiAgentState &next_state,
               int reward,
               bool done,
               bool is_collision);

    bool operator==(const Transition &other) const;
};


class MapfEnv {
private:
    void calc_transition_reward(const MultiAgentState *prev_state, const MultiAgentAction *action,
                                const MultiAgentState *next_state, int *reward, bool *done, bool *is_collision);

    int calc_living_reward(const MultiAgentState *prev_state, const MultiAgentAction *action);

    /* Caches */
    std::unordered_map<MultiAgentState, std::unordered_map<MultiAgentAction, list < Transition * > *>>
    transition_cache;
    std::unordered_map<MultiAgentState, std::unordered_map<MultiAgentAction, int>> living_reward_cache;

public:
    const Grid *grid_ptr;
    size_t n_agents;
    const MultiAgentState *start_state;
    const MultiAgentState *goal_state;
    float fail_prob;
    int reward_of_collision;
    int reward_of_goal;
    int reward_of_living;
    MultiAgentState *s;
    MultiAgentStateSpace *observation_space;
    MultiAgentActionSpace *action_space;
    uint64_t hits;

    MapfEnv(const Grid *grid,
            size_t n_agents,
            const MultiAgentState *start_state,
            const MultiAgentState *goal_state,
            float fail_prob,
            int collision_reward,
            int goal_reward,
            int living_reward);

    list<Transition *> *get_transitions(const MultiAgentState &state, const MultiAgentAction &action);

    void step(const MultiAgentAction &action, MultiAgentState *next_state, int *reward, bool *done, bool *is_collision);

    bool is_terminal_state(const MultiAgentState &state);

    MultiAgentState *reset();
};

#endif //GYM_MAPF_MAPF_ENV_H
