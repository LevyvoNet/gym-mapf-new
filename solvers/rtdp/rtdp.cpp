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
#define SUCCESS_RATE_EPSILON (0.05)
#define MIN_SUCCESS_RATE (50)

/** Private ****************************************************************************************************/
void RtdpPolicy::single_iteration() {
    bool done = false;
    bool is_collision = false;
    int reward = 0;
    int total_reward = 0;
    vector<MultiAgentState> path;
    int steps = 0;
    std::chrono::steady_clock::time_point init_begin = std::chrono::steady_clock::now();
    MultiAgentAction *a = new MultiAgentAction(this->env->n_agents);
    int diff = 0;
    double new_value = 0;
    double *new_value_ptr = nullptr;

    MultiAgentState *s = this->env->reset();


    while (!done && steps < MAX_STEPS) {
        ++steps;

        /* Select action */
        this->select_max_value_action(*s, &new_value, a);

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
        this->select_max_value_action(path[i], &new_value, nullptr);
        new_value_ptr = new double;
        *new_value_ptr = new_value;
        this->v->set(path[i], new_value_ptr);
    }

    this->env->reset();

    this->train_rewards.push_back(total_reward);

l_cleanup:
    delete a;
}

bool should_stop(EvaluationInfo *prev_eval_info, EvaluationInfo *curr_eval_info) {
    if (nullptr == prev_eval_info || nullptr == curr_eval_info) {
        return false;
    }

    if (curr_eval_info->success_rate < MIN_SUCCESS_RATE) {
        return false;
    }

    if ((std::abs(curr_eval_info->mdr - prev_eval_info->mdr) / std::abs(prev_eval_info->mdr)) >= MDR_EPSILON) {
        return false;
    }

    if ((std::abs(curr_eval_info->success_rate - prev_eval_info->success_rate) /
         std::abs(prev_eval_info->success_rate)) >= SUCCESS_RATE_EPSILON) {
        return false;
    }

    return true;
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
    (*(this->train_info->additional_data))["init_time"] = std::to_string(round(elapsed_time_seconds * 100) / 100);
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

        /* Evaluate */
        if (nullptr != prev_eval_info) {
            delete prev_eval_info;
        }
        prev_eval_info = eval_info;
        std::chrono::steady_clock::time_point eval_begin = std::chrono::steady_clock::now();
        eval_info = this->evaluate(100, 1000, 0);
        std::chrono::steady_clock::time_point eval_end = std::chrono::steady_clock::now();
        total_eval_time += std::chrono::duration_cast<std::chrono::milliseconds>(eval_end - eval_begin).count();

        /* Check if there is no improvement since the last batch */
        if (should_stop(prev_eval_info, eval_info)) {
            break;
        }
    }
    std::chrono::steady_clock::time_point train_end = std::chrono::steady_clock::now();

    /* Set the train info */
    elapsed_time_milliseconds += std::chrono::duration_cast<std::chrono::milliseconds>(train_end - train_begin).count();
    elapsed_time_seconds = float(elapsed_time_milliseconds) / 1000;
    total_eval_time = float(total_eval_time) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;
    (*(this->train_info->additional_data))["eval_time"] = std::to_string(round(total_eval_time * 100) / 100);
    (*(this->train_info->additional_data))["n_iterations"] = std::to_string(iters_count + 1);

l_cleanup:
    if (nullptr != eval_info) {
        delete eval_info;
    }
    if (nullptr != prev_eval_info) {
        delete prev_eval_info;
    }
}

double RtdpPolicy::get_value(MultiAgentState *s) {
    double *value = this->v->get(*s);
    if (nullptr == value) {
        return (*(this->h))(s);
    }

    return *value;
}

RtdpPolicy::~RtdpPolicy() {
    delete this->v;
    delete this->h;
}



