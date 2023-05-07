//
// Created by levyvonet on 03/11/2021.
//

#include "rtdp.h"

#include <iostream>

/** Constants ***************************************************************************************************/
#define MAX_ITERATIONS (30000)
#define BATCH_SIZE (50)
#define MAX_STEPS (1000)
#define MDR_EPSILON (1)
#define MIN_SUCCESS_RATE (100)
#define MIN_CONSECUTIVE_SUCCESS_COUNT (2)

#define NON_EXISTING_DEFAULT_VALUE (99999999)

/** Private ****************************************************************************************************/
void RtdpPolicy::single_iteration(double timeout_ms) {
    MEASURE_TIME;
    bool done = false;
    bool is_collision = false;
    int reward = 0;
    int total_reward = 0;
    vector<MultiAgentState> path;
    int steps = 0;
    MultiAgentAction *a = new MultiAgentAction(this->env->n_agents);
    int diff = 0;
    double new_value = 0;

    MultiAgentState *s = this->env->reset();

    while (!done && steps < MAX_STEPS) {
        ++steps;

        /* Add the current state to the path */
        path.push_back(*s);

        /* Select action */
        this->select_max_value_action(*s, &new_value, a, timeout_ms - ELAPSED_TIME_MS);
        if (ELAPSED_TIME_MS >= timeout_ms) {
            return;
        }

//        /* Bellman update the current state */
//        this->v->set(s->id, new_value);

        /* Sample the next state from the transition function */
        this->env->step(*a, s, &reward, &done, &is_collision);
        total_reward += reward;
    }

    /* Backward update */
    for (int i = path.size() - 1; i >= 0; --i) {
        this->select_max_value_action(path[i], &new_value, nullptr, timeout_ms - ELAPSED_TIME_MS);
        if (ELAPSED_TIME_MS >= timeout_ms) {
            return;
        }
        this->v->set(path[i].id, new_value);
    }

    this->env->reset();

    this->train_rewards.push_back(total_reward);

    l_cleanup:
    delete a;
}

bool RtdpPolicy::should_stop(EvaluationInfo *prev_eval_info, EvaluationInfo *curr_eval_info) {
    if (nullptr == prev_eval_info || nullptr == curr_eval_info) {
        return false;
    }

    if (curr_eval_info->success_rate >= MIN_SUCCESS_RATE) {
        this->consecutive_success++;
    } else {
        this->consecutive_success = 0;
    }

    if (this->consecutive_success < MIN_CONSECUTIVE_SUCCESS_COUNT) {
        return false;
    }

    if (prev_eval_info->success_rate < MIN_SUCCESS_RATE || curr_eval_info->success_rate < MIN_SUCCESS_RATE) {
        return false;
    }

    if ((std::abs(curr_eval_info->mdr - prev_eval_info->mdr) / std::abs(prev_eval_info->mdr)) >= MDR_EPSILON) {
        return false;
    }

    return true;
}

void RtdpPolicy::clear_cache() {
    delete this->cache;
    this->cache = new MultiAgentStateStorage<MultiAgentAction *>(this->env->n_agents, nullptr);
}


/** Public ****************************************************************************************************/
RtdpPolicy::RtdpPolicy(MapfEnv *env, float gamma,
                       const string &name, Heuristic *h) :
        ValueFunctionPolicy(env, gamma, name) {
    this->h = h;
    this->v = new Dictionary(NON_EXISTING_DEFAULT_VALUE);
    this->cache = new MultiAgentStateStorage<MultiAgentAction *>(this->env->n_agents, nullptr);
    this->in_train = true;
    this->consecutive_success = 0;
}


void RtdpPolicy::train(double timeout_ms) {
    /* Initialize the heuristic and measure the time for it */
    MEASURE_TIME;
    this->h->init(this->env, timeout_ms - ELAPSED_TIME_MS);
//    cout << "h init took " << ELAPSED_TIME_MS / 1000 << " SECONDS" << endl;
    double init_time_sec = ELAPSED_TIME_MS / 1000;
    (*(this->train_info->additional_data))["init_time"] = std::to_string((int) round(init_time_sec * 100) / 100);
    if (ELAPSED_TIME_MS >= timeout_ms) {
        return;
    }

    bool converged = false;
    size_t iters_count = 0;
    EvaluationInfo *prev_eval_info = NULL;
    EvaluationInfo *eval_info = NULL;
    float total_eval_time = 0;


    /* Run RTDP iterations until convergence */
    while (iters_count < MAX_ITERATIONS) {
        for (size_t i = 0; i < BATCH_SIZE; ++i) {
            this->single_iteration(timeout_ms - ELAPSED_TIME_MS);
            if (ELAPSED_TIME_MS >= timeout_ms) {
                return;
            }
            iters_count++;
        }

        /* Evaluate */
        if (nullptr != prev_eval_info) {
            delete prev_eval_info;
        }
        prev_eval_info = eval_info;
        const auto eval_begin = clk::now();
        this->clear_cache();
        this->in_train = false;
        eval_info = this->evaluate(10, 1000, (timeout_ms - ELAPSED_TIME_MS) / 30, false);
        this->in_train = true;
        const auto eval_end = clk::now();
        total_eval_time += ((ms) (eval_end - eval_begin)).count();

        /* Check if there is no improvement since the last batch */
        if (should_stop(prev_eval_info, eval_info)) {
            break;
        }
    }

    cout << "RTDP took " << iters_count << " to converge" << endl;

    /* Set the train info */
    float elapsed_time_seconds = float(ELAPSED_TIME_MS) / 1000;
    total_eval_time = float(total_eval_time) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;
    (*(this->train_info->additional_data))["eval_time"] = std::to_string((int) round(total_eval_time * 100) / 100);
    (*(this->train_info->additional_data))["n_iterations"] = std::to_string(iters_count + 1);

    /* Finish training */
    this->in_train = false;

    l_cleanup:
    if (nullptr != eval_info) {
        delete eval_info;
    }
    if (nullptr != prev_eval_info) {
        delete prev_eval_info;
    }
}

double RtdpPolicy::get_value(MultiAgentState *s) {
    double value = this->v->get(s->id);
    double h_value = 0;
    if (NON_EXISTING_DEFAULT_VALUE == value) {
        h_value = (*(this->h))(s);
//        if (this->in_train) {
//            value = new double;
//            *value = h_value;
//            this->v->set(*s, value);
//        }

        return h_value;
    }

    return value;
}

RtdpPolicy::~RtdpPolicy() {
    delete this->v;
    delete this->h;
    delete this->cache;
}

MultiAgentAction *RtdpPolicy::act(const MultiAgentState &state, double timeout_ms) {
    MEASURE_TIME;
    MultiAgentAction *a = nullptr;

    a = this->cache->get(state);
    if (nullptr != a) {
        return new MultiAgentAction(a->actions, a->id);
    }

    /* If this is an unfamiliar state, return all stay action */
//    if (nullptr == this->v->get(state)) {
//        MultiAgentActionIterator a_iter = this->env->action_space->begin();
//        return new MultiAgentAction(a_iter->actions, a_iter->id);
//    }

    a = ValueFunctionPolicy::act(state, timeout_ms);
    if (ELAPSED_TIME_MS >= timeout_ms) {
        return nullptr;
    }
    this->cache->set(state, new MultiAgentAction(a->actions, a->id));

    return a;
}

/** RTDP merger ************************************************************************************************/
Policy *RtdpMerger::operator()(MapfEnv *env,
                               float gamma,
                               double timeout_ms,
                               size_t group1,
                               size_t group2,
                               CrossedPolicy *joint_policy) {
    /* Create the merged env */
    MapfEnv *merged_env = merge_groups_envs(joint_policy, group1, group2);
    /* This is for the destructor to not destroy the received policies */

    Heuristic *solution_sum = new SolutionSumHeuristic(
            {
                    (ValueFunctionPolicy *) joint_policy->policies[group1],
                    (ValueFunctionPolicy *) joint_policy->policies[group2]},
            {
                    joint_policy->groups[group1],
                    joint_policy->groups[group2]});

    RtdpPolicy *policy = new RtdpPolicy(merged_env, gamma, "", solution_sum);
    policy->train(timeout_ms);

    return policy;
}
