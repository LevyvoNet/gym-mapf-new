//
// Created by levyvonet on 21/10/2021.
//

#ifndef GYM_MAPF_MAPF_ENV_H
#define GYM_MAPF_MAPF_ENV_H

#include <list>
#include <iterator>
//#include <unordered_map>
#include <unordered_set>
#include <cmath>

#include <tsl/hopscotch_map.h>

#include <grid/grid.h>
#include <multiagent_action/multiagent_action.h>
#include <multiagent_state/multiagent_state.h>
#include <utils/state_storage/state_storage.h>
//#include <mapf_env/goal_predicator.h>

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

class ActionToIntStorage {
public:
    tsl::hopscotch_map<MultiAgentAction, int *> *m;

    ActionToIntStorage();

    ~ActionToIntStorage();

};

class ActionToTransitionStorage {
public:
    tsl::hopscotch_map<MultiAgentAction, TransitionsList *> *m;

    ActionToTransitionStorage();

    ~ActionToTransitionStorage();

};


#define MOUNTAIN_NOISE_FACTOR (2)

/* Forward decleration of MapfEnv */
class MapfEnv;

class Constraint {
public:
    virtual bool is_violated(const MultiAgentState *prev_state, const MultiAgentState *next_state) = 0;
};

struct GoalDecision {
    bool is_goal;
    int reward;
};

class GoalDefinition {
public:
    virtual GoalDecision is_goal(const MapfEnv *env, const MultiAgentState &s) = 0;

    virtual ~GoalDefinition();
};

class SingleStateGoalDefinition : public GoalDefinition {
public:
    SingleStateGoalDefinition(const MultiAgentState &goal_state);

    virtual GoalDecision is_goal(const MapfEnv *env, const MultiAgentState &s) override;

private:
    const MultiAgentState goal_state_;
};

class MapfEnv {
public:
    /* Parameters */
    Grid *grid;
    size_t n_agents;
    MultiAgentState *start_state;
//    std::unique_ptr<GoalPredicator> goal_predicator;
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
    vector<GridArea> *mountains;
//    tsl::hopscotch_map<size_t, std::unordered_set<Location> *> *constraints;
    vector<Constraint *> constraints;
    unique_ptr<GoalDefinition> goal_definition;


    MapfEnv(Grid *grid,
            size_t n_agents,
            const vector<Location> &start_locations,
            const vector<Location> &goal_locations,
            float fail_prob,
            int collision_reward,
            int goal_reward,
            int living_reward);
//
//    MapfEnv(Grid *grid,
//            size_t n_agents,
//            const vector<Location> &start_locations,
//            std::unique_ptr<GoalPredicator> goal_predicator,
//            float fail_prob,
//            int collision_reward,
//            int goal_reward,
//            int living_reward);

    ~MapfEnv();

    void reset_cache();

    TransitionsList *get_transitions(const MultiAgentState &state, const MultiAgentAction &action, bool cache = true);

    void step(const MultiAgentAction &action,
              MultiAgentState *next_state, int *reward, bool *done, bool *is_collision,
              bool cache = true);

    bool is_terminal_state(const MultiAgentState &state, bool cache);

    MultiAgentState *reset();

    /* Conversions between integers and vectors */

    MultiAgentState *locations_to_state(const vector<Location> &locations);

    MultiAgentState *id_to_state(int64_t id);

    MultiAgentAction *id_to_action(int64_t id);

    void add_mountain(GridArea mountain_area);

    bool is_in_mountain(const Location &l);

    bool is_collision_transition(const MultiAgentState *prev_state, const MultiAgentState *next_state);

    void add_constraint(Constraint *constraint);

    void set_goal_definition(std::unique_ptr<GoalDefinition> goal);


private:


    void calc_transition_reward(const MultiAgentState *prev_state, const MultiAgentAction *action,
                                const MultiAgentState *next_state, int *reward, bool *done, bool *is_collision,
                                bool cache);

    int calc_living_reward(const MultiAgentState *prev_state, const MultiAgentAction *action, bool cache);

    /* Caches */
    /* TODO: add another hierarchical structure for multi agent actions as well */
    MultiAgentStateStorage<ActionToTransitionStorage *> *transition_cache;
    MultiAgentStateStorage<ActionToIntStorage *> *living_reward_cache;
    MultiAgentStateStorage<bool *> *is_terminal_cache;
};

MapfEnv *get_local_view(const MapfEnv *, vector<size_t> agents);


MultiAgentAction *actions_to_action(const vector<Action> &actions);

#endif //GYM_MAPF_MAPF_ENV_H
