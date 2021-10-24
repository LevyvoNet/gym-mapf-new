//
// Created by levyvonet on 21/10/2021.
//

#include <cmath>
#include <iostream>
#include <string>
#include <map>
#include <random>

#include <mapf_env.h>
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

bool Transition::operator==(const Transition &other) {
    return ((this->p == other.p) &&
            (*this->next_state == *other.next_state) &&
            (this->reward == other.reward) &&
            (this->done == other.done) &&
            (this->is_collision == other.is_collision));
}

MultiAgentState::MultiAgentState(const vector<Location> &locations) {
    this->locations = locations;
}

bool MultiAgentState::operator==(const MultiAgentState &other) const{
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



MapfEnv::MapfEnv(const Grid *grid,
                 size_t n_agents,
                 const MultiAgentState *start_state,
                 const MultiAgentState *goal_state,
                 float fail_prob,
                 int reward_of_collision,
                 int reward_of_goal,
                 int reward_of_living) {
    size_t agent_idx = 0;

    /* Copy constant fields */
    this->grid_ptr = grid;
    this->n_agents = n_agents;
    this->start_state = new MultiAgentState(start_state->locations);
    this->goal_state = new MultiAgentState(goal_state->locations);
    this->fail_prob = fail_prob;
    this->reward_of_collision = reward_of_collision;
    this->reward_of_goal = reward_of_goal;
    this->reward_of_living = reward_of_living;

    /* Initialize the env state to the starting one */
    this->s = new MultiAgentState(start_state->locations);

}

bool is_collision_transition(const MultiAgentState *prev_state, const MultiAgentState *next_state) {
    return false;
}

int MapfEnv::calc_living_reward(const MultiAgentState *prev_state, const MultiAgentAction *action) const {
    return this->reward_of_living;
}

void MapfEnv::calc_transition_reward(const MultiAgentState *prev_state, const MultiAgentAction *action,
                                     const MultiAgentState *next_state, int *reward, bool *done,
                                     bool *is_collision) const {
    *reward = this->reward_of_living;
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

    return transitions;
}

void MapfEnv::step(const MultiAgentAction &action, MultiAgentState *next_state, int *reward, bool *done) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d({1 - this->fail_prob, this->fail_prob / 2, this->fail_prob / 2});
    int noise_idx = 0;
    Action noised_action = STAY;
    size_t agent_idx = 0;
    bool collision = false;

    /* Set the next state - for each agent sample an action and set its next location properly */
    for (agent_idx = 0; agent_idx < this->n_agents; ++agent_idx) {
        noise_idx = d(gen);
        noised_action = g_action_noise_to_action[action.actions[agent_idx]][noise_idx];
        next_state->locations[agent_idx] = this->grid_ptr->execute(this->s->locations[agent_idx], noised_action);
    }

    /* Set the reward and done */
    this->calc_transition_reward(this->s, nullptr, next_state, reward, done, &collision);

    /* Update the current state of the env */
    for (agent_idx = 0; agent_idx < this->n_agents; ++agent_idx) {
        this->s->locations[agent_idx] = next_state->locations[agent_idx];
    }
}


