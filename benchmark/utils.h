//
// Created by levyvonet on 16/01/2022.
//

#ifndef GYM_MAPF_BENCHMARK_UTILS_H
#define GYM_MAPF_BENCHMARK_UTILS_H

#include "gym_mapf/gym_mapf.h"
#include "solvers/solvers.h"
#include "available_solvers_envs.h"

#define BENCHMARK_LONG_TIME_SEC (60)
#define BENCHMARK_LONG_TIME_MS (BENCHMARK_LONG_TIME_SEC * 1000)

void add_mountains_to_env(MapfEnv *env);


#endif //GYM_MAPF_BENCHMARK_UTILS_H
