//
// Created by levyvonet on 03/11/2021.
//

#ifndef GYM_MAPF_RTDP_H
#define GYM_MAPF_RTDP_H

#include <gym_mapf/gym_mapf.h>
#include <solvers/utils/policy/value_function_policy.h>
#include "solvers/heuristics/heuristic.h"
#include "solvers/id/id.h"

class RtdpPolicy : public ValueFunctionPolicy {
private:
    MultiAgentStateStorage<double *> *v;
    Heuristic *h;
    vector<int> train_rewards;
    MultiAgentStateStorage<MultiAgentAction *> *cache;

    void single_iteration();
    void clear_cache();

public:

    RtdpPolicy(MapfEnv *env, float gamma, const string &name, Heuristic *h);

    virtual ~RtdpPolicy();

    virtual void train() override;

    virtual MultiAgentAction *act(const MultiAgentState &state) override;

    virtual double get_value(MultiAgentState *s) override;

};


class RtdpMerger: public PolicyMerger {
public:
    virtual Policy *operator()(MapfEnv *env,
                               float gamma,
                               vector<vector<size_t>> groups,
                               size_t group1,
                               size_t group2,
                               Policy* policy1,
                               Policy* policy2);
};

#endif //GYM_MAPF_RTDP_H
