//
// Created by levyvonet on 21/10/2021.
//

#ifndef GYM_MAPF_MAPF_ENV_H
#define GYM_MAPF_MAPF_ENV_H

#include <list>
#include <iterator>
#include <unordered_map>
#include <cmath>

#include <gym_mapf/grid/grid.h>
#include <gym_mapf/multiagent_action/multiagent_action.h>
#include <gym_mapf/multiagent_state/multiagent_state.h>
#include <gym_mapf/utils/state_storage/state_storage.h>

using namespace std;

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
//    MultiAgentStateStorage<std::unordered_map<MultiAgentAction, list < Transition * > *>*> *transition_cache;
//    MultiAgentStateStorage<std::unordered_map<MultiAgentAction, int>*> *living_reward_cache;
//    MultiAgentStateStorage<bool*> *is_terminal_cache;

public:
    Grid *grid_ptr;
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

    MapfEnv(Grid *grid,
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
