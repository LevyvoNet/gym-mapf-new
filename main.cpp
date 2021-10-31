//
// Created by levyvonet on 21/10/2021.
//
#include <iostream>
#include <string>

#include <gym_mapf/gym_mapf.h>
#include <solvers/value_iteartion/value_iteration.h>

int main(int argc, char **argv) {
    MapfEnv *env = create_mapf_env("empty-8-8", 1, 2, 0.2, -1000, 0, -1);

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


}