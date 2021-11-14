//
// Created by levyvonet on 03/11/2021.
//

#ifndef GYM_MAPF_RTDP_H
#define GYM_MAPF_RTDP_H

#include <gym_mapf/gym_mapf.h>
#include <solvers/utils/policy/value_function_policy.h>
#include "solvers/heuristics/heuristic.h"

class RtdpPolicy : public ValueFunctionPolicy {
private:
    MultiAgentStateStorage<double *> *v;
    Heuristic *h;
    vector<int> train_rewards;
    MultiAgentStateStorage<MultiAgentAction *> *cache;

    virtual double get_value(MultiAgentState *s) override;

    void single_iteration();
    void clear_cache();

public:

    RtdpPolicy(MapfEnv *env, float gamma, const string &name, Heuristic *h);

    virtual ~RtdpPolicy();

    virtual void train() override;

    virtual MultiAgentAction *act(const MultiAgentState &state) override;

};


#endif //GYM_MAPF_RTDP_H
