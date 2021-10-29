//
// Created by levyvonet on 21/10/2021.
//

#include <cmath>
#include <iostream>
#include <string>
#include <map>
#include <random>

#include <gym_mapf/mapf_env/mapf_env.h>
#include <set>


/** Constants **********************************************************************************************/
#define NO_DISRUPTION (0)
#define CLOCKWISE (1)
#define COUNTERCLOCKWISE (2)
#define DISRUPTION_COUNT (3)


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
    this->next_state = new MultiAgentState(next_state.locations);
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

/** MultiAgentState *************************************************************************************************/

MultiAgentState::MultiAgentState(const vector<Location> &locations) {
    this->locations = locations;
}

bool MultiAgentState::operator==(const MultiAgentState &other) const {
    size_t agent_idx = 0;

    if (this->locations.size() != other.locations.size()) {
        return false;
    }

    for (agent_idx = 0; agent_idx < this->locations.size(); ++agent_idx) {

        if (this->locations[agent_idx] != other.locations[agent_idx]) {
            return false;
        }
    }

    return true;
}


/** MultiAgentStateIterator ****************************************************************************************/

MultiAgentStateIterator::MultiAgentStateIterator(const Grid *grid, size_t n_agents) {
    this->grid = grid;
    this->n_agents = n_agents;
    vector<Location> locs;
    for (size_t i = 0; i < this->n_agents; ++i) {
        this->iters.push_back(this->grid->begin());
        locs.push_back(*(this->iters[i]));
    }


    this->ptr = new MultiAgentState(locs);
}

MultiAgentState *MultiAgentStateIterator::operator->() const {
    return this->ptr;
}

MultiAgentState MultiAgentStateIterator::operator*() const {
    return *(this->ptr);
}

MultiAgentStateIterator MultiAgentStateIterator::operator++() {
    size_t agent_idx = 0;
    bool carry = false;

    /* Increment the first agent, then handle the "carry" */

    do {
        ++(this->iters[agent_idx]);
        if (this->iters[agent_idx] == this->grid->end()) {
            this->iters[agent_idx] = this->grid->begin();
            carry = true;
        } else {
            carry = false;
        }

        this->ptr->locations[agent_idx] = *(this->iters[agent_idx]);
        agent_idx++;

    } while ((agent_idx < this->n_agents) && carry);



    /* Check if we are out because of reaching the last state. If so, return the end */
    if (agent_idx == this->n_agents && carry) {
        this->reach_end();
    }

    return *this;

}

bool MultiAgentStateIterator::operator==(const MultiAgentStateIterator &other) const {
    size_t i = 0;

    if (this->iters.size() != other.iters.size()) {
        return false;
    }

    for (i = 0; i < this->iters.size(); ++i) {
        if (this->iters[i] != other.iters[i]) {
            return false;
        }
    }

    return true;
}

bool MultiAgentStateIterator::operator!=(const MultiAgentStateIterator &other) const {
    return !(*this == other);
}

void MultiAgentStateIterator::reach_end() {
    size_t agent_idx = 0;

    for (agent_idx = 0; agent_idx < this->n_agents; agent_idx++) {
        this->iters[agent_idx] = this->grid->end();
    }
}

/** MultiAgentStateSpace ***************************************************************************************/
MultiAgentStateIterator MultiAgentStateSpace::begin() {
    return MultiAgentStateIterator(this->grid, this->n_agents);
}

MultiAgentStateIterator MultiAgentStateSpace::end() {
    MultiAgentStateIterator iter = MultiAgentStateIterator(this->grid, this->n_agents);
    iter.reach_end();

    return iter;
}

MultiAgentStateSpace::MultiAgentStateSpace(const Grid *grid, size_t n_agents) {
    this->grid = grid;
    this->n_agents = n_agents;
}


/** MultiAgentAction *********************************************************************************************/
MultiAgentAction::MultiAgentAction(const vector<Action> &actions) {
    this->actions = actions;
}

bool MultiAgentAction::operator==(const MultiAgentAction &other) const {
    size_t agent_idx = 0;

    if (this->actions.size() != other.actions.size()) {
        return false;
    }

    for (agent_idx = 0; agent_idx < this->actions.size(); ++agent_idx) {

        if (this->actions[agent_idx] != other.actions[agent_idx]) {
            return false;
        }
    }

    return true;
}

/** MultiAgentActionIterator *************************************************************************************/
MultiAgentActionIterator::MultiAgentActionIterator(size_t n_agents) {
    this->n_agents = n_agents;
    vector<Action> all_stay(n_agents);

    for (size_t i = 0; i < n_agents; ++i) {
        all_stay[i] = STAY;
    }

    this->ptr = new MultiAgentAction(all_stay);
}

void MultiAgentActionIterator::reach_end() {
    for (size_t i = 0; i < this->n_agents; ++i) {
        this->ptr->actions[i] = LAST_INVALID_ACTION;
    }
}

MultiAgentAction *MultiAgentActionIterator::operator->() const {
    return this->ptr;
}

MultiAgentAction MultiAgentActionIterator::operator*() const {
    return *(this->ptr);
}

MultiAgentActionIterator MultiAgentActionIterator::operator++() {
    size_t agent_idx = 0;
    bool carry = false;

    /* Increment the first and handle the "carry" */
    do {
        carry = false;

        this->ptr->actions[agent_idx] = (Action) (int(this->ptr->actions[agent_idx]) + 1);
        if (this->ptr->actions[agent_idx] == LAST_INVALID_ACTION) {
            this->ptr->actions[agent_idx] = STAY;
            carry = true;
        }

        agent_idx++;


    } while (carry && (agent_idx < this->n_agents));

    if (agent_idx >= this->n_agents && carry) {
        this->reach_end();
    }

    return *this;
}

bool MultiAgentActionIterator::operator==(const MultiAgentActionIterator &other) const {
    for (size_t i = 0; i < this->n_agents; ++i) {
        if (this->ptr->actions[i] != other.ptr->actions[i]) {
            return false;
        }
    }

    return true;
}

bool MultiAgentActionIterator::operator!=(const MultiAgentActionIterator &other) const {
    for (size_t i = 0; i < this->n_agents; ++i) {
        if (this->ptr->actions[i] == other.ptr->actions[i]) {
            return false;
        }
    }

    return true;
}

/** MultiAgentActionSpace ****************************************************************************************/
MultiAgentActionSpace::MultiAgentActionSpace(size_t n_agents) {
    this->n_agents = n_agents;
}

MultiAgentActionIterator MultiAgentActionSpace::begin() {
    return MultiAgentActionIterator(this->n_agents);
}

MultiAgentActionIterator MultiAgentActionSpace::end() {
    MultiAgentActionIterator iter = MultiAgentActionIterator(this->n_agents);
    iter.reach_end();

    return iter;
}


/** MapfEnv *****************************************************************************************************/

MapfEnv::MapfEnv(const Grid *grid,
                 size_t n_agents,
                 const MultiAgentState *start_state,
                 const MultiAgentState *goal_state,
                 float fail_prob,
                 int collision_reward,
                 int goal_reward,
                 int living_reward) {
    size_t agent_idx = 0;

    /* Copy constant fields */
    this->grid_ptr = grid;
    this->n_agents = n_agents;
    this->start_state = new MultiAgentState(start_state->locations);
    this->goal_state = new MultiAgentState(goal_state->locations);
    this->fail_prob = fail_prob;
    this->reward_of_collision = collision_reward;
    this->reward_of_goal = goal_reward;
    this->reward_of_living = living_reward;

    /* Set the state and action spaces for the env */
    this->observation_space = new MultiAgentStateSpace(this->grid_ptr, this->n_agents);
    this->action_space = new MultiAgentActionSpace(this->n_agents);


    /* Reset the env to its starting state */
    this->reset();

    /* TODO: delete me */
    this->hits = 0;

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

    if (this->living_reward_cache[*prev_state].find(*action) != this->living_reward_cache[*prev_state].end()) {
        return this->living_reward_cache[*prev_state][*action];
    }

    for (agent_idx = 0; agent_idx < this->n_agents; agent_idx++) {
        if ((prev_state->locations[agent_idx] == this->goal_state->locations[agent_idx]) &&
            (action->actions[agent_idx] == STAY)) {
            continue;
        }

        living_reward += this->reward_of_living;
    }

    this->living_reward_cache[*prev_state][*action] = living_reward;
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

//    if (this->is_terminal_cache.find(state) != this->is_terminal_cache.end()){
//        return this->is_terminal_cache[state];
//    }

    /* Collision between two agents */
    for (i = 0; i < this->n_agents; i++) {
        for (j = 0; j < this->n_agents; j++) {
            if ((i != j) && (state.locations[i] == state.locations[j])) {
//                this->is_terminal_cache[state] = true;
                return true;
            }
        }

    }

    /* Goal state */
    if (state == *this->goal_state) {
//        this->is_terminal_cache[state] = true;
        return true;
    }

    /* None of the conditions satisfied, this state is not terminal */
//    this->is_terminal_cache[state] = false;
    return false;
}

/* TODO: calculate next state and living reward as part of the main loop instead of helper functions (inline it) */
/* TODO: sum all of the transitions to the same next_state to a single one with the sum of probabilities */
list<Transition *> *MapfEnv::get_transitions(const MultiAgentState &state, const MultiAgentAction &action) {
    list<Transition *> *transitions = new list<Transition *>();
    vector<int> disruptions(this->n_agents);
    unsigned int i = 0;
    size_t curr_agent_idx = 0;
    double curr_prob = 0;
    unsigned long n_disruptions = (unsigned long) pow((double) DISRUPTION_COUNT, this->n_agents);
    double disrupt_ratio = 0.5 * this->fail_prob / (1 - this->fail_prob);
    double normal_ratio = 2 * (1 - this->fail_prob) / this->fail_prob;
    /* TODO: make sure that this the default copy c'tor copies the underlying vector */
    MultiAgentAction t_action = MultiAgentAction(action);
    MultiAgentState *t_state = NULL;
    int t_reward = 0;
    bool t_done = false;
    bool t_collision = false;
    size_t j = 0;

    /* Try to fetch from cache */
    if (this->transition_cache.find(state) != this->transition_cache.end()) {
        if (this->transition_cache[state].find(action) != this->transition_cache[state].end()) {
            this->hits++;
            return this->transition_cache[state][action];
        }
    }

    if (this->is_terminal_state(state)) {
        transitions->push_back(new Transition(1.0, new MultiAgentState(state.locations), 0, true, false));
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
        while (disruptions[curr_agent_idx] == 3) {
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

        t_state = new MultiAgentState(state);
        /* TODO: do it as part of the previous loops instead of nesting another loop */
        /* Execute the whole action */
        for (j = 0; j < this->n_agents; ++j) {
            t_state->locations[j] = this->grid_ptr->execute(state.locations[j], t_action.actions[j]);
        }

        /* We have the probability and the disrupted action, calculate the next state and reward from it */
        this->calc_transition_reward(&state, &t_action, t_state, &t_reward, &t_done, &t_collision);

        /* Add the transition to the result */
        transitions->push_back(new Transition(curr_prob, t_state, t_reward, t_done, t_collision));


    }


    this->transition_cache[state][action] = transitions;
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

    if (this->is_terminal_state(*this->s)) {
        *next_state = *this->s;
        *reward = 0;
        *done = true;
        return;
    }

    /* Set the next state - for each agent sample an action and set its next location properly */
    for (agent_idx = 0; agent_idx < this->n_agents; ++agent_idx) {
        noise_idx = d(gen);
        noised_action = g_action_noise_to_action[action.actions[agent_idx]][noise_idx];
        next_state->locations[agent_idx] = this->grid_ptr->execute(this->s->locations[agent_idx], noised_action);
    }

    /* Set the reward and done */
    this->calc_transition_reward(this->s, &action, next_state, reward, done, is_collision);

    /* Update the current state of the env */
    for (agent_idx = 0; agent_idx < this->n_agents; ++agent_idx) {
        this->s->locations[agent_idx] = next_state->locations[agent_idx];
    }
}

MultiAgentState *MapfEnv::reset() {
    this->s = new MultiAgentState(start_state->locations);

    return this->s;
}
