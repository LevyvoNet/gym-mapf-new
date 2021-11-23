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
#define MIN_SUCCESS_RATE (90)

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

//    if ((std::abs(curr_eval_info->mdr - prev_eval_info->mdr) / std::abs(prev_eval_info->mdr)) >= MDR_EPSILON) {
//        return false;
//    }

    return true;
}

void RtdpPolicy::clear_cache() {
    delete this->cache;
    this->cache = new MultiAgentStateStorage<MultiAgentAction *>(this->env->n_agents, nullptr);
}


/** Public ****************************************************************************************************/
RtdpPolicy::RtdpPolicy(MapfEnv *env, float gamma,
                       const string &name, Heuristic *h) : ValueFunctionPolicy(env, gamma, name) {
    this->h = h;
    this->v = new MultiAgentStateStorage<double *>(this->env->n_agents, nullptr);
    this->cache = new MultiAgentStateStorage<MultiAgentAction *>(this->env->n_agents, nullptr);

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
        this->clear_cache();
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
    delete this->cache;
}

MultiAgentAction *RtdpPolicy::act(const MultiAgentState &state) {
    MultiAgentAction *a = nullptr;
    MultiAgentAction *all_stay = nullptr;

    a = this->cache->get(state);
    if (nullptr != a) {
        return new MultiAgentAction(a->actions, a->id);
    }

    /* If this is an unfamiliar state, return all stay action */
    if (nullptr == this->v->get(state)) {
        MultiAgentActionIterator a_iter = this->env->action_space->begin();
        return new MultiAgentAction(a_iter->actions, a_iter->id);
    }

    a = ValueFunctionPolicy::act(state);
    this->cache->set(state, new MultiAgentAction(a->actions, a->id));

    return a;
}

/** RTDP merger ************************************************************************************************/
class SolutionSumHeuristic : public Heuristic {
public:
    ValueFunctionPolicy *p1;
    ValueFunctionPolicy *p2;
    vector<size_t> g1;
    vector<size_t> g2;
    MapfEnv *env;

    SolutionSumHeuristic(ValueFunctionPolicy *p1,
                         ValueFunctionPolicy *p2,
                         vector<size_t> g1,
                         vector<size_t> g2) :
            p1(p1), p2(p2), g1(g1), g2(g2) {}

    ~SolutionSumHeuristic(){
//        delete this->p1->env;
//        delete this->p2->env;
//        delete this->p1;
//        delete this->p2;
    }

    virtual void init(MapfEnv *env_param) {
        this->env = env_param;
    }


    virtual double operator()(MultiAgentState *s) {
        MultiAgentState *s1 = nullptr;
        MultiAgentState *s2 = nullptr;
        vector<Location> l1;
        vector<Location> l2;
        double v1 = 0;
        double v2 = 0;
        double sum = 0;
        size_t relevant_values = 0;

        /* Extract the local states of the two groups from the merged state */
        for (size_t i = 0; i < s->locations.size(); ++i) {
            if (i < this->g1.size()) {
                l1.push_back(s->locations[i]);
            } else {
                l2.push_back(s->locations[i]);
            }
        }
        s1 = this->p1->env->locations_to_state(l1);
        s2 = this->p2->env->locations_to_state(l2);

        v1 = this->p1->get_value(s1);
        v2 = this->p2->get_value(s2);

        /* Sum only the states which are not goal states */
        if (s1 == this->p1->env->goal_state) {
            sum += v1;
            ++relevant_values;
        }
        if (s2 == this->p2->env->goal_state) {
            sum += v2;
            ++relevant_values;
        }

        delete s1;
        delete s2;


        if (0 == relevant_values) {
            return 0;
        }

        return sum - (relevant_values - 1) * env->reward_of_goal;
    }


};

Policy *RtdpMerger::operator()(MapfEnv *env, float gamma, vector<vector<size_t>> groups, size_t group1, size_t group2,
                               Policy *policy1, Policy *policy2) {
    /* Create the merged env */
    CrossedPolicy joint_policy(env, gamma, "", groups, {policy1, policy2});
    MapfEnv *merged_env = merge_groups_envs(&joint_policy, group1, group2);
    /* This is for the destructor to not destroy the received policies */
    joint_policy.policies.clear();

    Heuristic *solution_sum = new SolutionSumHeuristic((ValueFunctionPolicy *) policy1,
                                                       (ValueFunctionPolicy *) policy2,
                                                       groups[group1],
                                                       groups[group2]);
    RtdpPolicy *policy = new RtdpPolicy(merged_env, gamma, "", solution_sum);
    policy->train();


    return policy;
}
