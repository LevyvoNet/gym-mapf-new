//
// Created by levyvonet on 03/11/2021.
//

#ifndef GYM_MAPF_RTDP_H
#define GYM_MAPF_RTDP_H

#include <gym_mapf/gym_mapf.h>
#include "solvers/utils/policy/policy.h"
#include <solvers/utils/policy/value_function_policy.h>
#include "solvers/heuristics/heuristic.h"
#include "solvers/heuristics/solution_sum_heuristic.h"
#include "solvers/id/id.h"

class RtdpPolicy : public ValueFunctionPolicy {
private:
    MultiAgentStateStorage<double *> *v;
    Heuristic *h;
    vector<int> train_rewards;
    MultiAgentStateStorage<MultiAgentAction *> *cache;
    bool in_train;
    int consecutive_success;

    void single_iteration(double timeout_ms);

    void clear_cache();

public:

    RtdpPolicy(MapfEnv *env, float gamma, const string &name, Heuristic *h);

    bool should_stop(EvaluationInfo *prev_eval_info, EvaluationInfo *curr_eval_info)

    virtual ~RtdpPolicy();

    virtual void train(double timeout_ms) override;

    virtual MultiAgentAction *act(const MultiAgentState &state, double timeout_ms) override;

    virtual double get_value(MultiAgentState *s) override;

};


class RtdpMerger : public PolicyMerger {
public:
    virtual Policy *operator()(MapfEnv *env,
                               float gamma,
                               double timeout_ms,
                               size_t group1,
                               size_t group2,
                               CrossedPolicy *joint_policy);
};

#endif //GYM_MAPF_RTDP_H
