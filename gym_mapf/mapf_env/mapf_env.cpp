//
// Created by levyvonet on 21/10/2021.
//

#include <cmath>
#include <iostream>
#include <string>
#include <map>
#include <random>

#include <mapf_env/mapf_env.h>
#include <set>


/** Constants **********************************************************************************************/
#define NO_DISRUPTION (0)
#define CLOCKWISE (1)
#define COUNTERCLOCKWISE (2)
#define INVALID_DISRUPTION (3)


/** Globals ***************************************************************************************************/
Action g_action_noise_to_action[5][3] = {
        {STAY,  STAY,  STAY},
        {UP,    RIGHT, LEFT},
        {RIGHT, DOWN,  UP},
        {DOWN,  LEFT,  RIGHT},
        {LEFT,  UP,    DOWN}
};

/** Transition ***********************************************************************************************/

Transition::Transition(double p, MultiAgentState *next_state, int reward, bool done, bool is_collision) {
    this->p = p;
    this->next_state = next_state;
    this->reward = reward;
    this->done = done;
    this->is_collision = is_collision;
}

Transition::Transition(double p, const MultiAgentState &next_state, int reward, bool done, bool is_collision) {
    this->p = p;
    this->next_state = new MultiAgentState(next_state.locations, next_state.id);
    this->reward = reward;
    this->done = done;
    this->is_collision = is_collision;
}

bool Transition::operator==(const Transition &other) const {
    return ((this->p == other.p) &&
            (*this->next_state == *other.next_state) &&
            (this->reward == other.reward) &&
            (this->done == other.done) &&
            (this->is_collision == other.is_collision));
}

Transition::~Transition() {
    delete this->next_state;
}


/** MapfEnv *****************************************************************************************************/

MapfEnv::MapfEnv(Grid *grid,
                 size_t n_agents,
                 const vector<Location> &start_locations,
                 const vector<Location> &goal_locations,
                 float fail_prob,
                 int collision_reward,
                 int goal_reward,
                 int living_reward) {
    size_t agent_idx = 0;

    /* Copy constant fields */
    this->grid = grid;
    this->n_agents = n_agents;
    this->start_state = this->locations_to_state(start_locations);
    this->goal_state = this->locations_to_state(goal_locations);
    this->fail_prob = fail_prob;
    this->reward_of_collision = collision_reward;
    this->reward_of_goal = goal_reward;
    this->reward_of_living = living_reward;

    /* Set the state and action spaces for the env */
    this->nS = pow(this->grid->id_to_loc.size(), this->n_agents);
    this->nA = pow(ACTIONS_COUNT, this->n_agents);
    this->observation_space = new MultiAgentStateSpace(this->grid, this->n_agents);
    this->action_space = new MultiAgentActionSpace(this->n_agents);

    /* State */
    this->s = new MultiAgentState(start_state->locations, start_state->id);

    /* Caches */
    this->transition_cache = new MultiAgentStateStorage<ActionToTransitionStorage *>(this->n_agents, NULL);
//    this->living_reward_cache = new MultiAgentStateStorage<tsl::hopscotch_map<MultiAgentAction, int> *>(this->n_agents,
//                                                                                                        NULL);
//    this->is_terminal_cache = new MultiAgentStateStorage<bool *>(this->n_agents, NULL);

    /* Reset the env to its starting state */
    this->reset();

}

bool is_collision_transition(const MultiAgentState *prev_state, const MultiAgentState *next_state) {
    size_t i = 0;
    size_t j = 0;
    size_t n_agents = prev_state->locations.size();


    for (i = 0; i < n_agents; ++i) {
        for (j = 0; j < n_agents; j++) {
            if (i == j) {
                continue;
            }

            /* Vertex collision */
            if (next_state->locations[i] == next_state->locations[j]) {
                return true;
            }

            /* Edge collision */
            if ((prev_state->locations[j] == next_state->locations[i])
                && (prev_state->locations[i] == next_state->locations[j])) {
                return true;
            }
        }
    }

    return false;
}

int MapfEnv::calc_living_reward(const MultiAgentState *prev_state, const MultiAgentAction *action) {
    size_t agent_idx = 0;
    int living_reward = 0;
//    tsl::hopscotch_map<MultiAgentAction, int> *cached_state = NULL;

//    cached_state = this->living_reward_cache->get(*prev_state);
//    if (NULL != cached_state) {
//        if (cached_state->find(*action) != cached_state->end()) {
//            return (*cached_state)[*action];
//        }
//    } else {
//        cached_state = new tsl::hopscotch_map<MultiAgentAction, int>();
//        this->living_reward_cache->set(*prev_state, cached_state);
//    }


    for (agent_idx = 0; agent_idx < this->n_agents; agent_idx++) {
        if ((prev_state->locations[agent_idx] == this->goal_state->locations[agent_idx]) &&
            (action->actions[agent_idx] == STAY)) {
            continue;
        }

        living_reward += this->reward_of_living;
    }

//    (*cached_state)[*action] = living_reward;
    return living_reward;

}

void MapfEnv::calc_transition_reward(const MultiAgentState *prev_state, const MultiAgentAction *action,
                                     const MultiAgentState *next_state, int *reward, bool *done,
                                     bool *is_collision) {
    *reward = 0;
    *done = false;
    *is_collision = false;
    int living_reward = 0;

    living_reward = this->calc_living_reward(prev_state, action);

    if (is_collision_transition(prev_state, next_state)) {
        *reward = living_reward + this->reward_of_collision;
        *done = true;
        *is_collision = true;
        return;
    }

    if (*this->goal_state == *next_state) {
        *reward = living_reward + this->reward_of_goal;
        *done = true;
        *is_collision = false;
        return;
    }

    *reward = living_reward;
    *done = false;
    *is_collision = false;
}

bool MapfEnv::is_terminal_state(const MultiAgentState &state) {
    size_t i = 0;
    size_t j = 0;
    bool *cached = NULL;

//    cached = this->is_terminal_cache->get(state);
//    if (NULL != cached) {
//        return *cached;
//    }

    cached = new bool[1];

    /* Collision between two agents */
    for (i = 0; i < this->n_agents; i++) {
        for (j = 0; j < this->n_agents; j++) {
            if ((i != j) && (state.locations[i] == state.locations[j])) {
                *cached = true;
//                this->is_terminal_cache->set(state, cached);
                return true;
            }
        }

    }

    /* Goal state */
    if (state == *this->goal_state) {
        *cached = true;
//        this->is_terminal_cache->set(state, cached);
        return true;
    }

    /* None of the conditions satisfied, this state is not terminal */
    *cached = false;
//    this->is_terminal_cache->set(state, cached);
    return false;
}

/* TODO: calculate next state and living reward as part of the main loop instead of helper functions (inline it) */
/* TODO: sum all of the transitions to the same next_state to a single one with the sum of probabilities */
TransitionsList *MapfEnv::get_transitions(const MultiAgentState &state, const MultiAgentAction &action) {
    TransitionsList *transitions = NULL;
    vector<int> disruptions(this->n_agents);
    unsigned int i = 0;
    size_t curr_agent_idx = 0;
    double curr_prob = 0;
    unsigned long n_disruptions = (unsigned long) pow((double) INVALID_DISRUPTION, this->n_agents);
    double disrupt_ratio = 0.5 * this->fail_prob / (1 - this->fail_prob);
    double normal_ratio = 2 * (1 - this->fail_prob) / this->fail_prob;
    /* TODO: make sure that this the default copy c'tor copies the underlying vector */
    MultiAgentAction t_action = MultiAgentAction(action);
    MultiAgentState *t_state = NULL;
    int t_reward = 0;
    bool t_done = false;
    bool t_collision = false;
    size_t j = 0;
    vector<Location> *t_state_locations = NULL;

    /* Try to fetch from cache */
    ActionToTransitionStorage *state_cache = this->transition_cache->get(state);
    if (NULL != state_cache) {
        if (state_cache->m->find(action) != state_cache->m->end()) {
            return (*state_cache->m)[action];
        }
    } else {
        state_cache = new ActionToTransitionStorage();
        this->transition_cache->set(state, state_cache);
    }

    transitions = new TransitionsList();
    if (this->is_terminal_state(state)) {
        transitions->transitions->push_back(
                new Transition(1.0, new MultiAgentState(state.locations, state.id), 0, true, false));
        (*state_cache->m)[action] = transitions;
        return transitions;
    }

    /* Initialize disruptions */
    for (i = 0; i < n_agents; ++i) {
        disruptions[i] = 0;
    }
    disruptions[0] = -1;
    curr_prob = pow((1 - this->fail_prob), this->n_agents);

    /* Iterate over all possible disruptions and compute the matching transition */
    for (i = 1; i <= n_disruptions; ++i) {
        /* Increment and handle the "carry" */
        curr_agent_idx = 0;
        if (disruptions[curr_agent_idx] == NO_DISRUPTION) {
            curr_prob *= disrupt_ratio;
        }
        disruptions[curr_agent_idx]++;
        while (disruptions[curr_agent_idx] == INVALID_DISRUPTION) {
            disruptions[curr_agent_idx] = 0;
            t_action.actions[curr_agent_idx] = g_action_noise_to_action[action.actions[curr_agent_idx]][disruptions[curr_agent_idx]];
            curr_prob *= normal_ratio;
            curr_agent_idx++;
            if (disruptions[curr_agent_idx] == 0) {
                curr_prob *= disrupt_ratio;
            }
            disruptions[curr_agent_idx]++;
        }

        /* In case there was not "carry", set the proper action */
        t_action.actions[curr_agent_idx] = g_action_noise_to_action[action.actions[curr_agent_idx]][disruptions[curr_agent_idx]];

        t_state_locations = new vector<Location>();
//        t_state = new MultiAgentState(state);
        /* TODO: do it as part of the previous loops instead of nesting another loop */
        /* Execute the whole action */
        for (j = 0; j < this->n_agents; ++j) {
            t_state_locations->push_back(this->grid->execute(state.locations[j], t_action.actions[j]));
//            t_state->locations[j] = this->grid->execute(state.locations[j], t_action.actions[j]);
        }
        t_state = this->locations_to_state(*t_state_locations);
        delete t_state_locations;
        t_state_locations = nullptr;

        /* We have the probability and the disrupted action, calculate the next state and reward from it */
        this->calc_transition_reward(&state, &t_action, t_state, &t_reward, &t_done, &t_collision);

        /* Add the transition to the result */
        transitions->transitions->push_back(new Transition(curr_prob, t_state, t_reward, t_done, t_collision));
    }

    (*state_cache->m)[action] = transitions;
    return transitions;
}

void MapfEnv::step(const MultiAgentAction &action, MultiAgentState *next_state, int *reward, bool *done,
                   bool *is_collision) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d({1 - this->fail_prob, this->fail_prob / 2, this->fail_prob / 2});
    int noise_idx = 0;
    Action noised_action = STAY;
    size_t agent_idx = 0;
    MultiAgentState *next_state_local = nullptr;

    if (this->is_terminal_state(*this->s)) {
        *next_state = *this->s;
        *reward = 0;
        *done = true;
        return;
    }

    /* Set the next state locations - for each agent sample an action and set its next location properly */
    for (agent_idx = 0; agent_idx < this->n_agents; ++agent_idx) {
        noise_idx = d(gen);
        noised_action = g_action_noise_to_action[action.actions[agent_idx]][noise_idx];
        next_state->locations[agent_idx] = this->grid->execute(this->s->locations[agent_idx], noised_action);
    }
    next_state_local = this->locations_to_state(next_state->locations);
    next_state->id = next_state_local->id;

    /* Set the reward and done */
    this->calc_transition_reward(this->s, &action, next_state, reward, done, is_collision);

    /* Update the current state of the env */
    this->s->id = next_state_local->id;
    this->s->locations = next_state_local->locations;
    delete next_state_local;
}

MultiAgentState *MapfEnv::reset() {
    *(this->s) = MultiAgentState(this->start_state->locations, this->start_state->id);
    return this->s;
}

MultiAgentState *MapfEnv::locations_to_state(const vector<Location> &locations) {
    int mul = 1;
    int sum = 0;
    int n_options = this->grid->id_to_loc.size();

    sum += locations[0].id * mul;

    for (size_t i = 1; i < locations.size(); ++i) {
        mul *= n_options;
        sum += locations[i].id * mul;
    }

    return new MultiAgentState(locations, sum);
}

MultiAgentAction *MapfEnv::actions_to_action(const vector<Action> &actions) {
    int mul = 1;
    int sum = 0;
    int n_options = ACTIONS_COUNT;

    sum += actions[0] * mul;

    for (size_t i = 1; i < actions.size(); ++i) {
        mul *= n_options;
        sum += actions[i] * mul;
    }

    return new MultiAgentAction(actions, sum);
}

MultiAgentState *MapfEnv::id_to_state(int64_t id) {
    int orig_id = id;
    vector<Location> locations;
    int n_options = this->grid->id_to_loc.size();

    for (size_t i = 0; i < this->n_agents; ++i) {
        locations.push_back(*this->grid->id_to_loc[id % n_options]);
        id /= n_options;
    }

    return new MultiAgentState(locations, orig_id);
}

MultiAgentAction *MapfEnv::id_to_action(int64_t id) {
    int orig_id = id;
    vector<Action> actions;
    int n_options = ACTIONS_COUNT;

    for (size_t i = 0; i < this->n_agents; ++i) {
        actions.push_back((Action) (id % n_options));
        id /= n_options;
    }

    return new MultiAgentAction(actions, orig_id);
}

MapfEnv::~MapfEnv() {
    delete this->transition_cache;
    delete this->action_space;
    delete this->observation_space;
//    delete this->grid;
    delete this->s;
}


TransitionsList::TransitionsList() {
    this->transitions = new list<Transition *>();

}

TransitionsList::~TransitionsList() {
    for (Transition *t: *this->transitions) {
        delete t;
    }

    delete this->transitions;
}

ActionToTransitionStorage::ActionToTransitionStorage() {
    this->m = new tsl::hopscotch_map<MultiAgentAction, TransitionsList *>();
}

ActionToTransitionStorage::~ActionToTransitionStorage() {
    for (auto item: *this->m) {
        delete item.second;
    }
    delete this->m;
}
