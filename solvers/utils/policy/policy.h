//
// Created by levyvonet on 26/10/2021.
//

#ifndef GYM_MAPF_POLICY_H
#define GYM_MAPF_POLICY_H

#include <unordered_map>
#include <chrono>
#include <cmath>

#include <gym_mapf/gym_mapf.h>

using ms = std::chrono::duration<double, std::milli>;
using clk = std::chrono::steady_clock;

#define ELAPSED_TIME_MS (((ms)(clk::now() - start_time)).count())
#define MEASURE_TIME const auto start_time = clk::now()
#define EPISODE_SUCCEED(episode_info) (!episode_info.collision && !episode_info.timeout)


class EpisodeInfo{
public:
    int reward;
    double time;
    bool collision;
    bool timeout;

    EpisodeInfo(int reward, double time, bool collision, bool timeout);
};

class EvaluationInfo {
public:
    float mdr;
    float success_rate;
    float mean_episode_time;
    float collision_rate;
    float timeout_rate;
    vector<EpisodeInfo> episodes_info;

    std::unordered_map<std::string, std::string> *additional_data;

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

    EpisodeInfo evaluate_single_episode(std::size_t max_steps, double timeout_ms);

    virtual void eval_episode_info_update(EpisodeInfo episode_info);

    virtual void eval_episodes_info_process(EvaluationInfo* eval_info);

public:
    std::string name;
    MapfEnv *env;

    Policy(MapfEnv *env, float gamma, const std::string &name = "");

    virtual ~Policy();

    virtual void reset();

    TrainInfo *get_train_info();

    EvaluationInfo *evaluate(std::size_t n_episodes,
                             std::size_t max_steps,
                             double episode_timeout_ms,
                             double min_success_rate = 0);

    virtual MultiAgentAction *act(const MultiAgentState &state, double timeout_ms) = 0;

    virtual void train(double timeout_milliseconds) = 0;
};

#endif //GYM_MAPF_POLICY_H
