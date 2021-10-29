//
// Created by levyvonet on 21/10/2021.
//
#include <iostream>
#include <string>

#include <gym_mapf/gym_mapf.h>
#include <solvers/value_iteartion/value_iteration.h>

int main(int argc, char **argv) {
    std::vector<std::string> symmetrical_bottleneck{{'.', '.', '@', '.', '.', '.'},
                                                    {'.', '.', '@', '.', '.', '.'},
                                                    {'.', '.', '.', '.', '.', '.'},
                                                    {'.', '.', '@', '.', '.', '.'},
                                                    {'.', '.', '@', '.', '.', '.'}};
    Grid g(symmetrical_bottleneck);
    MultiAgentState start_state = MultiAgentState({Location(2, 0), Location(2, 5)});
    MultiAgentState goal_state = MultiAgentState({Location(2, 5), Location(2, 0)});

    MapfEnv env(&g, 2, &start_state, &goal_state, 0.2, -1000, 0, -1);

    ValueIterationPolicy policy = ValueIterationPolicy(&env, 1.0, "vi");

    policy.train();
    TrainInfo *train_info = policy.get_train_info();
//    EvaluationInfo* eval_info = policy.evaluate(100, 1000, 0);



    std::cout << "iterations: " << (*(train_info->additional_data))["n_iterations"] << " time: " << train_info->time
              << " hits: " << env.hits << std::endl;

}