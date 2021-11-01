//
// Created by levyvonet on 21/10/2021.
//
#include <iostream>
#include <string>

#include <gym_mapf/gym_mapf.h>
#include <solvers/value_iteartion/value_iteration.h>

int main(int argc, char **argv) {
    MapfEnv *env = create_mapf_env("empty-8-8", 1, 2, 0.2, -1000, 100, -1);

    ValueIterationPolicy policy = ValueIterationPolicy(env, 1.0, "vi");

    policy.train();
    TrainInfo *train_info = policy.get_train_info();
    EvaluationInfo *eval_info = policy.evaluate(100, 1000, 0);

    std::cout << "MDR:" << eval_info->mdr << " rate:" << eval_info->success_rate << " train_time:"
              << train_info->time;
    std::cout << " exec_time:" << eval_info->mean_episode_time << " solver:" << policy.name;

    for (auto item: *train_info->additional_data) {
        std::cout << " " << item.first << ":" << item.second;
    }

    std::cout << endl;

    /* Just check out how much time it takes to iterate over the states */
    MapfEnv *other_env = create_mapf_env("empty-8-8", 1, 2, 0.2, -1000, 0, -1);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    MultiAgentStateIterator s = other_env->observation_space->begin();
    MultiAgentActionIterator a = other_env->action_space->begin();
    std::stringstream sstream((*(train_info->additional_data))["n_iterations"]);
    size_t n_iters;
    sstream >> n_iters;
    for (size_t i = 0; i < n_iters; ++i) {
        for (s = other_env->observation_space->begin(); s != other_env->observation_space->end(); ++s) {
            for (a = other_env->action_space->begin(); a != other_env->action_space->end(); ++a) {
                for (Transition *t: *other_env->get_transitions(*s, *a)) {

                }
            }
        }
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto elapsed_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    float elapsed_time_seconds = float(elapsed_time_milliseconds) / 1000;
    elapsed_time_seconds = round(elapsed_time_seconds * 100) / 100;
    cout << n_iters << " iterations took " << elapsed_time_seconds << " seconds" << endl;
}