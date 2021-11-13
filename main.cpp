//
// Created by levyvonet on 21/10/2021.
//
#include <iostream>
#include <string>

#include <gym_mapf/gym_mapf.h>
#include <solvers/solvers.h>


/** Abstracts ********************************************************************************************************/
class EnvCreator {
public:
    string name;

    EnvCreator(string name) : name(name) {}

    virtual MapfEnv *operator()() = 0;
};

class SolverCreator {
public:
    virtual Policy *operator()(MapfEnv *env, float gamma) = 0;
};


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

        return create_mapf_env(map_name.str(), 1, this->n_agents, 0.2, -1000, this->goal_reward, -1);
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
                           0.2,
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
                           0.2,
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
        return create_mapf_env(map_name.str(), this->scen_id, this->n_agents, 0.2, -1000, 0, -1);
    }
};

/** Policies *******************************************************************************************************/
class vi : public SolverCreator {
public:
    virtual Policy *operator()(MapfEnv *env, float gamma) {
        return new ValueIterationPolicy(env, gamma, "VI");
    }
};

class rtdp_dijkstra : public SolverCreator {
    virtual Policy *operator()(MapfEnv *env, float gamma) {
        return new RtdpPolicy(env, gamma, "RTDP", new DijkstraHeuristic());
    }
};

/** Constants *******************************************************************************************************/
vector<vector<EnvCreator *>> env_creators(
        {   /* lvl 0 */
                {
                        new EmptyGrid("empty_8X8_single_agent", 8, 1, 0),
                        new EmptyGrid("empty_8X8_2_agents_large_goal", 8, 2, 100),
                        new EmptyGrid("empty_8X8_2_agents", 8, 2, 0),
                        new SymmetricalBottleneck("symmetrical_bottleneck", 0),
                        new SymmetricalBottleneck("symmetrical_bottleneck_large_goal", 100),
                        new ASymmetricalBottleneck("asymmetrical_bottleneck", 0),
                        new ASymmetricalBottleneck("asymmetrical_bottleneck_large_goal", 100),
                        new EmptyGrid("empty_16X16_2-agents", 16, 2, 0),
                },
                /* lvl 1 */
                {
                        new RoomEnv("room-32-32-4_scen-12_2-agents", 32, 4, 12, 2),
                        new RoomEnv("room-32-32-4_scen-1_2-agents", 32, 4, 1, 2),
                }

        }
);

vector<vector<SolverCreator *>> solver_creators(
        {   /* lvl 0 */
                {
                        new vi(),

                },

                /* lvl 1 */
                {
                        new rtdp_dijkstra(),
                }
        }
);

void benchmark_solver_on_env(Policy *policy) {
    policy->train();
    TrainInfo *train_info = policy->get_train_info();
    EvaluationInfo *eval_info = policy->evaluate(100, 1000, 0);

    std::cout << "MDR:" << eval_info->mdr << " rate:" << eval_info->success_rate << " train_time:" << train_info->time;
    std::cout << " exec_time:" << eval_info->mean_episode_time << " solver:" << policy->name;

    for (auto item: *train_info->additional_data) {
        std::cout << " " << item.first << ":" << item.second;
    }

    std::cout << endl;
}

int main(int argc, char **argv) {
    Policy *policy = nullptr;
    MapfEnv *env = nullptr;

    for (size_t env_lvl = 0; env_lvl < env_creators.size(); ++env_lvl) {
        for (EnvCreator *env_creator: env_creators[env_lvl]) {
            cout << endl << env_creator->name << endl;
            for (size_t solver_lvl = env_lvl; solver_lvl < solver_creators.size(); ++solver_lvl) {
                for (SolverCreator *solver_creator: solver_creators[solver_lvl]) {
                    env = (*env_creator)();
                    policy = (*solver_creator)(env, 1.0);
                    benchmark_solver_on_env(policy);
                    delete env;
                    delete policy;
                }
            }
        }
    }

    /* Delete all env creators */

    /* Delete all solver creators */
}