//
// Created by levyvonet on 19/11/2021.
//

#ifndef GYM_MAPF_SOLVERS_UTILS_H
#define GYM_MAPF_SOLVERS_UTILS_H

using namespace std;

class EnvCreator {
public:
    string map_name;
    int scen_id;
    int n_agents;
    string name;

    EnvCreator(string name) : name(name) {}

    virtual MapfEnv *operator()() = 0;
};

class SolverCreator {
public:
    string name;

    SolverCreator(string name):name(name){}

    virtual Policy *operator()(MapfEnv *env, float gamma) = 0;
};



#endif //GYM_MAPF_SOLVERS_UTILS_H
