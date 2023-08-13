//
// Created by levyvonet on 26/10/2021.
//

#ifndef GYM_MAPF_POLICY_H
#define GYM_MAPF_POLICY_H

#include <unordered_map>
#include <chrono>
#include <cmath>
#include <string.h>
#include <unistd.h>
//#include <time.h>

#include <gym_mapf/gym_mapf.h>

using ms = std::chrono::duration<double, std::milli>;
using clk = std::chrono::steady_clock;

#define ELAPSED_TIME_MS (((ms)(clk::now() - start_time)).count())
#define MEASURE_TIME const auto start_time = clk::now()
#define EPISODES_TIMEOUT_LIMIT (3)
#define MAX_RAM (16000000000)

//#define ELAPSED_TIME_SEC (((double) (clock() - start_time)) / CLOCKS_PER_SEC)
//#define ELAPSED_TIME_MS (ELAPSED_TIME_SEC * 1000)
//#define MEASURE_TIME const clock_t start_time = clock()
//#define EPISODES_TIMEOUT_LIMIT (3)

enum episode_status_code {
    EPISODE_INVALID = 0,
    EPISODE_SUCCESS = 1,
    EPISODE_TIMEOUT = 2,
    EPISODE_STUCK = 3,
    EPISODE_COLLISION = 4,
    EPISODE_OUT_OF_MEMORY = 5,
    EPISODE_UNKNOWN_FAILURE = 6,
};

struct episode_info {
    /* Metadata */
    bool child_exited_normally;
    bool child_exited_by_signal;
    size_t child_exit_status;
    ssize_t read_syscall_result;
    size_t signal;
    pid_t waitpid_result;

    /* General fields */
    episode_status_code end_reason;
    int reward;
    double time;
    uint64_t steps;
    double memory_used;

    /* Concrete policy proprietary fields */
    int replans_max_size;
    int max_agents_replan_area;
    double replans_count;
    int max_steps_window;
    int max_reached_window;
    int max_expanded_window;
    int livelock_count;

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
    float oom_rate;

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

    episode_info evaluate_single_episode(std::size_t max_steps, double timeout_ms, bool debug_print);

    virtual void eval_episode_info_update(episode_info *episode_info);

    virtual void eval_episodes_info_process(EvaluationInfo *eval_info);

public:
    std::string name;
    MapfEnv *env;

    Policy(MapfEnv *env, float gamma, const std::string &name = "");

    virtual ~Policy();

    virtual void reset();

    TrainInfo *get_train_info();

    EvaluationInfo *
    evaluate(size_t n_episodes, size_t max_steps, double episode_timeout_ms, bool forked, bool debug_print = false);

    virtual MultiAgentAction *act(const MultiAgentState &state, double timeout_ms) = 0;

    virtual void train(double timeout_milliseconds) = 0;
};

#endif //GYM_MAPF_POLICY_H
