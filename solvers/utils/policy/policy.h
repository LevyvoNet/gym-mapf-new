//
// Created by levyvonet on 26/10/2021.
//

#ifndef GYM_MAPF_POLICY_H
#define GYM_MAPF_POLICY_H

#include <unordered_map>
#include <chrono>
#include <cmath>

#include <gym_mapf/gym_mapf.h>

class EvaluationInfo {
public:
    float mdr;
    float success_rate;
    float mean_episode_time;
    bool collision_happened;
    vector<float> episodes_rewards;
    vector<float> episodes_times;

    EvaluationInfo();
};

class TrainInfo {
public:
    float time;
    std::unordered_map<std::string, std::string> *additional_data;

    TrainInfo();
};

class Policy {

protected:

    float gamma;

    TrainInfo *train_info;

    void evaluate_single_episode(std::size_t max_steps, EvaluationInfo *eval_info);

    virtual void eval_episode_info_update();

    virtual void eval_episode_info_process();

public:
    std::string name;
    MapfEnv *env;

    Policy(MapfEnv *env, float gamma, const std::string &name = "");

    void reset();

    TrainInfo *get_train_info();

    EvaluationInfo *evaluate(std::size_t n_episodes, std::size_t max_steps, double min_success_rate = 0);

    virtual MultiAgentAction *act(const MultiAgentState &state) = 0;

    virtual void train() = 0;
};

#endif //GYM_MAPF_POLICY_H
