//
// Created by levyvonet on 03/11/2021.
//

#include "rtdp.h"

#include <iostream>

/** Constants ***************************************************************************************************/
#define MAX_ITERATIONS (10000)
#define BATCH_SIZE (100)
#define MAX_STEPS (1000)
#define MDR_EPSILON (0.1)

/** Private ****************************************************************************************************/
void RtdpPolicy::single_iteration() {
    bool done = false;
    bool is_collision = false;
    int reward = 0;
    int total_reward = 0;
    vector<MultiAgentState> path;
    int steps = 0;
    std::chrono::steady_clock::time_point init_begin = std::chrono::steady_clock::now();
    MultiAgentAction *a = nullptr;
    int diff = 0;
    double new_value = 0;
    double *new_value_ptr = nullptr;

    MultiAgentState *s = this->env->reset();

    vector<Action> all_stay_vector(this->env->n_agents);
    for (size_t i = 0; i < this->env->n_agents; ++i) {
        all_stay_vector[i] = STAY;
    }
    MultiAgentAction all_stay(all_stay_vector, 0);

    while (!done && steps < MAX_STEPS) {
        ++steps;

        /* Select action */
        a = this->select_max_value_action(*s, &new_value);
        diff = std::abs(this->get_value(s) - new_value);

        /* Bellman update the current state */
        new_value_ptr = new double;
        *new_value_ptr = new_value;
        this->v->set(*s, new_value_ptr);

        /* Sample the next state from the transition function */
        this->env->step(*a, s, &reward, &done, &is_collision);
        total_reward += reward;

        /* Add the next state to the path */
        path.push_back(*s);
    }

    for (int i = path.size() - 1; i >= 0; --i) {
        s = &path[i];
        this->select_max_value_action(*s, &new_value);
        new_value_ptr = new double;
        *new_value_ptr = new_value;
        this->v->set(*s, new_value_ptr);
    }

    this->env->reset();

    this->train_rewards.push_back(total_reward);
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
    size_t iters_count = 0;
    EvaluationInfo *prev_eval_info = NULL;
    EvaluationInfo *eval_info = NULL;
    float total_eval_time = 0;

    /* Run RTDP iterations until convergence */
    std::chrono::steady_clock::time_point train_begin = std::chrono::steady_clock::now();
    while (iters_count < MAX_ITERATIONS) {
        for (size_t i = 0; i < BATCH_SIZE; ++i) {
            this->single_iteration();
            iters_count++;
        }
        prev_eval_info = eval_info;
        eval_info = this->evaluate(100, 1000, 0);

        std::chrono::steady_clock::time_point eval_begin = std::chrono::steady_clock::now();
        if (nullptr != prev_eval_info && nullptr != eval_info) {
            if (std::abs(eval_info->mdr - prev_eval_info->mdr) < MDR_EPSILON) {
                break;
            }
        }
        std::chrono::steady_clock::time_point eval_end = std::chrono::steady_clock::now();
        total_eval_time += std::chrono::duration_cast<std::chrono::milliseconds>(
                eval_end - eval_begin).count();
    }
    std::chrono::steady_clock::time_point train_end = std::chrono::steady_clock::now();

    /* Set the train info */
    elapsed_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(train_end - train_begin).count();
    elapsed_time_seconds = float(elapsed_time_milliseconds) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;
    (*(this->train_info->additional_data))["n_iterations"] = std::to_string(iters_count + 1);
    total_eval_time = float(total_eval_time) / 1000;
    (*(this->train_info->additional_data))["eval_time"] = round(total_eval_time * 100) / 100;


}

double RtdpPolicy::get_value(MultiAgentState *s) {
    double *value = this->v->get(*s);
    if (nullptr == value) {
        return (*(this->h))(s);
    }

    return *value;
}



