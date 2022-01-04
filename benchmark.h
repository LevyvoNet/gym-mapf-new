//
// Created by levyvonet on 03/01/2022.
//

#ifndef GYM_MAPF_BENCHMARK_H
#define GYM_MAPF_BENCHMARK_H

#include <gym_mapf/gym_mapf.h>
#include <solvers/solvers.h>

#define BENCHMARK_LONG_TIME_SEC (60)
#define BENCHMARK_LONG_TIME_MS (BENCHMARK_LONG_TIME_SEC * 1000)
#define FAIL_PROB (0.3)

/** Envs **********************************************************************************************************/
class EmptyGrid : public EnvCreator {
public:
    size_t n_agents;
    size_t grid_size;
    int goal_reward;

    EmptyGrid(string name, size_t grid_size, size_t n_agents, int goal_reward) : EnvCreator(name), grid_size(grid_size),
                                                                                 n_agents(n_agents),
                                                                                 goal_reward(goal_reward) {}

    virtual MapfEnv *operator()() {
        std::ostringstream map_name;
        map_name << "empty-" << this->grid_size << "-" << this->grid_size;

        return create_mapf_env(map_name.str(), 3, this->n_agents, FAIL_PROB, -1000, this->goal_reward, -1);
    }
};


class SymmetricalBottleneck : public EnvCreator {
public:
    int goal_reward;

    SymmetricalBottleneck(string name, int goal_reward) : EnvCreator(name), goal_reward(goal_reward) {}

    virtual MapfEnv *operator()() {
        vector<std::string> map_lines({
                                              "..@...",
                                              "..@...",
                                              "......",
                                              "..@...",
                                              "..@..."
                                      });

        Grid *g = new Grid(map_lines);

        return new MapfEnv(g,
                           2,
                           {g->get_location(2, 0), g->get_location(2, 5)},
                           {g->get_location(2, 5), g->get_location(2, 0)},
                           FAIL_PROB,
                           -1000,
                           goal_reward,
                           -1);

    }


};

class ASymmetricalBottleneck : public EnvCreator {
public:
    int goal_reward;

    ASymmetricalBottleneck(string name, int goal_reward) : EnvCreator(name), goal_reward(goal_reward) {}

    virtual MapfEnv *operator()() {
        vector<std::string> map_lines({
                                              "..@..",
                                              "..@..",
                                              ".....",
                                              "..@..",
                                              "..@.."
                                      });

        Grid *g = new Grid(map_lines);

        return new MapfEnv(g,
                           2,
                           {g->get_location(2, 0), g->get_location(2, 4)},
                           {g->get_location(2, 4), g->get_location(2, 0)},
                           FAIL_PROB,
                           -1000,
                           goal_reward,
                           -1);

    }


};

class RoomEnv : public EnvCreator {
public:
    size_t room_size;
    size_t scen_id;
    size_t n_agents;
    size_t n_rooms;

    RoomEnv(string name, size_t room_size, size_t n_rooms, size_t scen_id, size_t n_agents) : EnvCreator(name),
                                                                                              room_size(room_size),
                                                                                              scen_id(scen_id),
                                                                                              n_agents(n_agents),
                                                                                              n_rooms(n_rooms) {}

    virtual MapfEnv *operator()() {
        std::ostringstream map_name;
        map_name << "room-" << this->room_size << "-" << this->room_size << "-" << this->n_rooms;
        return create_mapf_env(map_name.str(), this->scen_id, this->n_agents, FAIL_PROB, -1000, 0, -1);
    }
};

class SanityEnv : public EnvCreator {
public:
    size_t room_size;
    size_t n_agents;
    size_t n_rooms;


    SanityEnv(string name, size_t n_rooms, size_t room_size, size_t n_agents) : EnvCreator(name),
                                                                                room_size(room_size),
                                                                                n_agents(n_agents),
                                                                                n_rooms(n_rooms) {}

    virtual MapfEnv *operator()() {
        return create_sanity_mapf_env(this->n_rooms, this->room_size, this->n_agents, FAIL_PROB, -1000, 0, -1);
    }
};

class MazeEnv : public EnvCreator {
public:
    size_t maze_size;
    size_t scen_id;
    size_t n_rooms;
    size_t n_agents;

    MazeEnv(string name, size_t maze_size, size_t n_rooms, size_t scen_id, size_t n_agents) : EnvCreator(name),
                                                                                              maze_size(maze_size),
                                                                                              scen_id(scen_id),
                                                                                              n_agents(n_agents),
                                                                                              n_rooms(n_rooms) {}

    virtual MapfEnv *operator()() {
        std::ostringstream map_name;
        map_name << "maze-" << this->maze_size << "-" << this->maze_size << "-" << this->n_rooms;
        return create_mapf_env(map_name.str(), this->scen_id, this->n_agents, FAIL_PROB, -1000, 0, -1);
    }

};

class BerlinEnv : public EnvCreator {
public:
    size_t n_agents;
    size_t scen_id;

    BerlinEnv(string name, size_t scen_id, size_t n_agents) : EnvCreator(name), n_agents(n_agents), scen_id(scen_id) {}

    virtual MapfEnv *operator()() {
        return create_mapf_env("Berlin_1_256", this->scen_id, this->n_agents, FAIL_PROB, -1000, 0, -1);
    }
};

/** Policies *******************************************************************************************************/
class vi : public SolverCreator {
public:
    vi(string name) : SolverCreator(name) {}

    virtual Policy *operator()(MapfEnv *env, float gamma) {
        return new ValueIterationPolicy(env, gamma, this->name);
    }
};

class rtdp_dijkstra : public SolverCreator {
public:
    rtdp_dijkstra(string name) : SolverCreator(name) {}

    virtual Policy *operator()(MapfEnv *env, float gamma) {
        return new RtdpPolicy(env, gamma, this->name, new DijkstraHeuristic());
    }
};

class rtdp_dijkstra_rtdp : public SolverCreator {
public:
    rtdp_dijkstra_rtdp(string name) : SolverCreator(name) {}

    virtual Policy *operator()(MapfEnv *env, float gamma) {
        return new RtdpPolicy(env, gamma, this->name, new RtdpDijkstraHeuristic(gamma));
    }
};

class id_rtdp : public SolverCreator {
public:
    id_rtdp(string name) : SolverCreator(name) {}

    virtual Policy *operator()(MapfEnv *env, float gamma) {
        return new IdPolicy(env, gamma, this->name, new rtdp_dijkstra_rtdp(""), new RtdpMerger());
    }
};

class id_rtdp_default : public SolverCreator {
public:
    id_rtdp_default(string name) : SolverCreator(name) {}

    virtual Policy *operator()(MapfEnv *env, float gamma) {
        return new IdPolicy(env, gamma, this->name, new rtdp_dijkstra_rtdp(""), nullptr);
    }
};

class online_replan : public SolverCreator {
    int k;
    SolverCreator *low_level_planner;
public:
    online_replan(string name, int k, SolverCreator *low_level_planner) : SolverCreator(name),
                                                                          k(k), low_level_planner(low_level_planner) {}

    virtual Policy *operator()(MapfEnv *env, float gamma) {
        return new OnlineReplanPolicy(env, gamma, this->name, this->low_level_planner, this->k);
    }
};

class dijkstra_baseline : public SolverCreator {
public:
    dijkstra_baseline(string name) : SolverCreator(name) {}

    virtual Policy *operator()(MapfEnv *env, float gamma) {
        return new DijkstraBaselinePolicy(env, gamma, this->name);
    }
};

/** Utilities ********************************************************************************************************/
void add_mountains_to_env(MapfEnv *env) {
    int grid_area = (env->grid->max_row + 1) * ((env->grid->max_col + 1));
    vector<Location> mountain_centers;


    vector<vector<size_t>> groups(env->n_agents);
    for (size_t i = 0; i < env->n_agents; ++i) {
        groups[i] = {i};
    }
    CrossedPolicy *p = solve_local_and_cross(env,
                                             1.0,
                                             BENCHMARK_LONG_TIME_MS,
                                             new dijkstra_baseline(""),
                                             &groups);

    for (size_t agent_idx = 0; agent_idx < env->n_agents; ++agent_idx) {
        MultiAgentState *s = p->env->locations_to_state({p->env->start_state->locations[agent_idx]});
        Location l = s->locations[0];
        int path_length = ((DijkstraBaselinePolicy *) (p->policies[agent_idx]))->h->distance[0][l.id];

        /* Add a mountain in middle of path */
        size_t j = 0;
        for (; j < path_length / 2; ++j) {
            MultiAgentAction *a = p->policies[agent_idx]->act(MultiAgentState({l}, l.id), BENCHMARK_LONG_TIME_MS);
            l = p->env->grid->execute(l, a->actions[0]);
            delete a;
        }
        int mountain_dim = min(3, path_length / 8);

        int mountain_top = max(0, l.row - mountain_dim);
        int mountain_bottom = min((int) env->grid->max_row, l.row +mountain_dim);
        int mountain_left = max(0, l.col - mountain_dim);
        int mountain_right = min((int) env->grid->max_col, l.col + mountain_dim);
        env->add_mountain(GridArea(mountain_top, mountain_bottom, mountain_left, mountain_right));
    }
}

#endif //GYM_MAPF_BENCHMARK_H
