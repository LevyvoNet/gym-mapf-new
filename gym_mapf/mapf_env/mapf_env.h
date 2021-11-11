//
// Created by levyvonet on 21/10/2021.
//

#ifndef GYM_MAPF_MAPF_ENV_H
#define GYM_MAPF_MAPF_ENV_H

#include <list>
#include <iterator>
//#include <unordered_map>
#include <cmath>

#include <tsl/hopscotch_map.h>

#include <grid/grid.h>
#include <multiagent_action/multiagent_action.h>
#include <multiagent_state/multiagent_state.h>
#include <utils/state_storage/state_storage.h>

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

    ~Transition();

    bool operator==(const Transition &other) const;
};

class TransitionsList {
public:
    list<Transition *> *transitions;

    TransitionsList();

    ~TransitionsList();
};

class ActionToTransitionStorage{
public:
    tsl::hopscotch_map<MultiAgentAction, TransitionsList *> * m;

    ActionToTransitionStorage();

    ~ActionToTransitionStorage();

};

class MapfEnv {
private:
    void calc_transition_reward(const MultiAgentState *prev_state, const MultiAgentAction *action,
                                const MultiAgentState *next_state, int *reward, bool *done, bool *is_collision);

    int calc_living_reward(const MultiAgentState *prev_state, const MultiAgentAction *action);

    /* Caches */
    MultiAgentStateStorage<ActionToTransitionStorage *> *transition_cache;
//    MultiAgentStateStorage<std::unordered_map<MultiAgentAction, int>*> *living_reward_cache;
//    MultiAgentStateStorage<bool*> *is_terminal_cache;

public:
    /* Parameters */
    Grid *grid;
    size_t n_agents;
    MultiAgentState *start_state;
    MultiAgentState *goal_state;
    float fail_prob;
    int reward_of_collision;
    int reward_of_goal;
    int reward_of_living;

    /* Constants */
    MultiAgentStateSpace *observation_space;
    MultiAgentActionSpace *action_space;
    uint64_t nS;
    uint64_t nA;

    /* State */
    MultiAgentState *s;

    MapfEnv(Grid *grid,
            size_t n_agents,
            const vector<Location> &start_locations,
            const vector<Location> &goal_locations,
            float fail_prob,
            int collision_reward,
            int goal_reward,
            int living_reward);

    ~MapfEnv();

    TransitionsList *get_transitions(const MultiAgentState &state, const MultiAgentAction &action);

    void step(const MultiAgentAction &action, MultiAgentState *next_state, int *reward, bool *done, bool *is_collision);

    bool is_terminal_state(const MultiAgentState &state);

    MultiAgentState *reset();

    /* Conversions between integers and vectors */

    MultiAgentState *locations_to_state(const vector<Location> &locations);

    MultiAgentAction *actions_to_action(const vector<Action> &actions);

    MultiAgentState *id_to_state(int64_t id);

    MultiAgentAction *id_to_action(int64_t id);
};

#endif //GYM_MAPF_MAPF_ENV_H
