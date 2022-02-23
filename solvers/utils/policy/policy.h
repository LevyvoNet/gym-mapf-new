//
// Created by levyvonet on 26/10/2021.
//

#ifndef GYM_MAPF_POLICY_H
#define GYM_MAPF_POLICY_H

#include <unordered_map>
#include <chrono>
#include <cmath>
#include <string.h>
//#include <time.h>

#include <gym_mapf/gym_mapf.h>

using ms = std::chrono::duration<double, std::milli>;
using clk = std::chrono::steady_clock;

#define ELAPSED_TIME_MS (((ms)(clk::now() - start_time)).count())
#define MEASURE_TIME const auto start_time = clk::now()
#define EPISODES_TIMEOUT_LIMIT (3)

//#define ELAPSED_TIME_SEC (((double) (clock() - start_time)) / CLOCKS_PER_SEC)
//#define ELAPSED_TIME_MS (ELAPSED_TIME_SEC * 1000)
//#define MEASURE_TIME const clock_t start_time = clock()
//#define EPISODES_TIMEOUT_LIMIT (3)

struct episode_info {
    /* General fields */
    int reward;
    double time;
    bool collision;
    bool timeout;
    bool stuck;

    /* Concrete policy proprietary fields */
    int replans_max_size;
    double replans_count;

};

class EvaluationInfo {
public:
    float mdr;
    float mdr_stderr;
    float success_rate;
    float mean_episode_time;
    float mean_episode_time_stderr;
    float collision_rate;
    float timeout_rate;
    float stuck_rate;

    vector<episode_info> episodes_info;

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

    episode_info evaluate_single_episode(std::size_t max_steps, double timeout_ms);

    virtual void eval_episode_info_update(episode_info *episode_info);

    virtual void eval_episodes_info_process(EvaluationInfo *eval_info);

public:
    std::string name;
    MapfEnv *env;

    Policy(MapfEnv *env, float gamma, const std::string &name = "");

    virtual ~Policy();

    virtual void reset();

    TrainInfo *get_train_info();

    EvaluationInfo *evaluate(size_t n_episodes, size_t max_steps, double episode_timeout_ms);

    virtual MultiAgentAction *act(const MultiAgentState &state, double timeout_ms) = 0;

    virtual void train(double timeout_milliseconds) = 0;
};

#endif //GYM_MAPF_POLICY_H
