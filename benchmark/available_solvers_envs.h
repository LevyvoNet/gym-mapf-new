//
// Created by levyvonet on 03/01/2022.
//

#ifndef GYM_MAPF_AVAILABLE_SOLVERS_ENVS_H
#define GYM_MAPF_AVAILABLE_SOLVERS_ENVS_H

#include "gym_mapf/gym_mapf.h"
#include "solvers/solvers.h"

#define FAIL_PROB (0.3)

/** Envs **********************************************************************************************************/
class EmptyGrid : public EnvCreator {
public:
    size_t n_agents;
    size_t grid_size;
    int goal_reward;

    EmptyGrid(string name, size_t grid_size, size_t n_agents, int goal_reward);

    virtual MapfEnv *operator()();
};


class SymmetricalBottleneck : public EnvCreator {
public:
    int goal_reward;

    SymmetricalBottleneck(string name, int goal_reward);

    virtual MapfEnv *operator()();


};

class ASymmetricalBottleneck : public EnvCreator {
public:
    int goal_reward;

    ASymmetricalBottleneck(string name, int goal_reward);

    virtual MapfEnv *operator()();


};

class RoomEnv : public EnvCreator {
public:
    size_t room_size;
    size_t scen_id;
    size_t n_agents;
    size_t n_rooms;

    RoomEnv(string name, size_t room_size, size_t n_rooms, size_t scen_id, size_t n_agents);

    virtual MapfEnv *operator()();
};

class SanityEnv : public EnvCreator {
public:
    size_t room_size;
    size_t n_agents;
    size_t n_rooms;


    SanityEnv(string name, size_t n_rooms, size_t room_size, size_t n_agents);

    virtual MapfEnv *operator()();
};

class MazeEnv : public EnvCreator {
public:
    size_t maze_size;
    size_t scen_id;
    size_t n_rooms;
    size_t n_agents;

    MazeEnv(string name, size_t maze_size, size_t n_rooms, size_t scen_id, size_t n_agents);

    virtual MapfEnv *operator()();

};

class BerlinEnv : public EnvCreator {
public:
    size_t n_agents;
    size_t scen_id;

    BerlinEnv(string name, size_t scen_id, size_t n_agents);

    virtual MapfEnv *operator()();
};


class GeneralEnv: public EnvCreator{
public:
    string map_name;
    size_t scen_id;
    size_t n_agents;

    GeneralEnv(string name, string map_name, size_t scen_id, size_t n_agents);

    virtual MapfEnv *operator()();


};
/** Policies *******************************************************************************************************/
class vi : public SolverCreator {
public:
    vi(string name);

    virtual Policy *operator()(MapfEnv *env, float gamma);
};

class rtdp_dijkstra : public SolverCreator {
public:
    rtdp_dijkstra(string name);

    virtual Policy *operator()(MapfEnv *env, float gamma);
};

class rtdp_dijkstra_rtdp : public SolverCreator {
public:
    rtdp_dijkstra_rtdp(string name);

    virtual Policy *operator()(MapfEnv *env, float gamma);
};

class id_rtdp : public SolverCreator {
public:
    id_rtdp(string name);

    virtual Policy *operator()(MapfEnv *env, float gamma);
};

class id_rtdp_default : public SolverCreator {
public:
    id_rtdp_default(string name);

    virtual Policy *operator()(MapfEnv *env, float gamma);
};

class online_replan : public SolverCreator {
    int k;
    SolverCreator *low_level_planner;
public:
    online_replan(string name, int k, SolverCreator *low_level_planner);

    virtual Policy *operator()(MapfEnv *env, float gamma);
};

class dijkstra_baseline : public SolverCreator {
public:
    dijkstra_baseline(string name);

    virtual Policy *operator()(MapfEnv *env, float gamma);
};






#endif //GYM_MAPF_AVAILABLE_SOLVERS_ENVS_H
