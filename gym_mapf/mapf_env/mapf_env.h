//
// Created by levyvonet on 21/10/2021.
//

#ifndef GYM_MAPF_MAPF_ENV_H
#define GYM_MAPF_MAPF_ENV_H

#include <list>
#include <gym_mapf/grid/grid.h>

using namespace std;

class MultiAgentState {
public:
    vector<Location> locations;

    MultiAgentState(const vector<Location> &locations);

    bool operator==(const MultiAgentState &other) const;
};

class MultiAgentAction {
public:
    vector<Action> actions;

    MultiAgentAction(const vector<Action> &actions);

    bool operator==(const MultiAgentAction &other) const;

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
                                const MultiAgentState *next_state, int *reward, bool *done, bool *is_collision) const;

    int calc_living_reward(const MultiAgentState *prev_state, const MultiAgentAction *action) const;

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

    MapfEnv(const Grid *grid,
            size_t n_agents,
            const MultiAgentState *start_state,
            const MultiAgentState *goal_state,
            float fail_prob,
            int collision_reward,
            int goal_reward,
            int living_reward);

    list<Transition *> *get_transitions(const MultiAgentState &state, const MultiAgentAction &action);

    void step(const MultiAgentAction &action, MultiAgentState *next_state, int *reward, bool *done);

    bool is_terminal_state(const MultiAgentState &state) const;
};

#endif //GYM_MAPF_MAPF_ENV_H
