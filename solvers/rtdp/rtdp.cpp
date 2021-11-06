//
// Created by levyvonet on 03/11/2021.
//

#include "rtdp.h"

/** Constants ***************************************************************************************************/
#define MAX_ITERATIONS (10000)
#define BATCH_SIZE (100)
#define MAX_STEPS (1000)
#define EPSILON (0.01)

/** Private ****************************************************************************************************/


bool RtdpPolicy::single_iteration() {
    bool done = false;
    bool is_collision = false;
    int reward = 0;
    int total_reward = 0;
    vector<MultiAgentState> path;
    int steps = 0;
    std::chrono::steady_clock::time_point init_begin = std::chrono::steady_clock::now();
    MultiAgentAction *a = nullptr;
    int diff = 0;
    bool converged = true;

    MultiAgentState *s = this->env->reset();

    MultiAgentStateStorage<double *> *prev_v = this->v;
    this->v = new MultiAgentStateStorage<double *>(this->env->n_agents, NULL);

    while (!done && steps < MAX_STEPS) {
        ++steps;

        /* Select action */
        /* TODO: this is a problem because it is choosing the best action from v instead of prev_v */
        a = this->select_max_value_action(*s);

        /* Bellman update the current state */
        diff = this->update(s, prev_v);
        if (std::abs(diff) > EPSILON) {
            converged = false;
        }

        /* Sample the next state from the transition function */
        this->env->step(*a, s, &reward, &done, &is_collision);
        total_reward += reward;

        /* Add the next state to the path */
        path.push_back(*s);
    }

    for (size_t i = path.size() - 1; i >= 0; ++i) {
        update(&path[i], prev_v);
    }

    this->env->reset();

    return converged;
}


/** Public ****************************************************************************************************/
RtdpPolicy::RtdpPolicy(MapfEnv *env, float gamma,
                       const string &name, Heuristic *h) : ValueFunctionPolicy(env, gamma, name) {
    this->h = h;
    this->v = new MultiAgentStateStorage<double *>(this->env->n_agents, NULL);

}


void RtdpPolicy::train() {
    /* Initialize the heuristic and measure the time for it */
    std::chrono::steady_clock::time_point init_begin = std::chrono::steady_clock::now();
    this->h->init(this->env);
    std::chrono::steady_clock::time_point init_end = std::chrono::steady_clock::now();
    auto elapsed_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            init_end - init_begin).count();
    float elapsed_time_seconds = float(elapsed_time_milliseconds) / 1000;
    (*(this->train_info->additional_data))["init_time"] = round(elapsed_time_seconds * 100) / 100;
    bool converged = false;
    size_t i = 0;

    /* Run RTDP iterations until convergence */
    std::chrono::steady_clock::time_point train_begin = std::chrono::steady_clock::now();
    for (i = 0; i < MAX_ITERATIONS && !converged; ++i) {
        converged = this->single_iteration();
    }
    std::chrono::steady_clock::time_point train_end = std::chrono::steady_clock::now();

    /* Set the train info */
    elapsed_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(train_end - train_begin).count();
    elapsed_time_seconds = float(elapsed_time_milliseconds) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;
    (*(this->train_info->additional_data))["n_iterations"] = std::to_string(i + 1);

}

double RtdpPolicy::get_value(MultiAgentState *s) {
    double *value = this->v->get(*s);
    if (nullptr == value) {
        return (*(this->h))(s);
    }

    return *value;
}

int RtdpPolicy::update(MultiAgentState *s, MultiAgentStateStorage<double *> *prev_v) {
    MultiAgentActionIterator a = this->env->action_space->begin();
    double q_sa = 0;
    list<Transition *> *transitions = NULL;
    double new_value = -std::numeric_limits<double>::max();

    double prev_value = this->get_value(s);

    /* Calculate Q(s,a) and keep the maximum one */
    for (a = this->env->action_space->begin(); a != this->env->action_space->end(); ++a) {
        q_sa = 0;
        transitions = this->env->get_transitions(*s, *a);
        for (Transition *t: *transitions) {
            if (t->is_collision) {
                q_sa = -std::numeric_limits<double>::max();
                break;
            }

            q_sa += t->p * (t->reward + this->gamma * (*prev_v->get(*t->next_state)));
        }

        if (q_sa > new_value) {
            new_value = q_sa;
        }
    }

    /* Update the value table and the diff */
    double *new_value_ptr = new double;
    *new_value_ptr = new_value;
    this->v->set(*s, new_value_ptr);
    return std::abs(prev_value - new_value);
}


