//
// Created by levyvonet on 03/11/2021.
//

#ifndef GYM_MAPF_RTDP_H
#define GYM_MAPF_RTDP_H

#include <gym_mapf/gym_mapf.h>
#include <solvers/utils/policy/policy.h>

class RtdpPolicy : public Policy {
private:
    MultiAgentStateStorage<double*> *v;

public:

    RtdpPolicy(MapfEnv *env, float gamma, const string &name);

    virtual MultiAgentAction *act(const MultiAgentState &state) override;

    virtual void train() override;

    virtual TrainInfo *get_train_info() override;
};


#endif //GYM_MAPF_RTDP_H
