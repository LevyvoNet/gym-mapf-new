//
// Created by levyvonet on 21/10/2021.
//
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <gym_mapf/gym_mapf.h>
#include <solvers/solvers.h>

/** Results ********************************************************************************************************/
std::string RESULT_OK = "OK";
std::string RESULT_COLLISION = "COLLISION";

class InstanceResult {
public:
    std::string env;
    std::string solver;
    std::string result;

    InstanceResult(std::string env, std::string solver, std::string result) :
            solver(solver), env(env), result(result) {}
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

        return create_mapf_env(map_name.str(), 1, this->n_agents, 0.21, -1000, this->goal_reward, -1);
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
                           0.21,
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
                           0.21,
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
        return create_mapf_env(map_name.str(), this->scen_id, this->n_agents, 0.21, -1000, 0, -1);
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
        return create_sanity_mapf_env(this->n_rooms, this->room_size, this->n_agents, 0.21, -1000, 0, -1);
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

/** Constants *******************************************************************************************************/
vector<vector<EnvCreator *>> env_creators(
        {   /* lvl 0 */
                {
//                        new EmptyGrid("empty_8X8_single_agent", 8, 1, 0),
//                        new EmptyGrid("empty_8X8_2_agents_large_goal", 8, 2, 100),
//                        new EmptyGrid("empty_8X8_2_agents", 8, 2, 0),
//                        new SymmetricalBottleneck("symmetrical_bottleneck", 0),
//                        new SymmetricalBottleneck("symmetrical_bottleneck_large_goal", 100),
//                        new ASymmetricalBottleneck("asymmetrical_bottleneck", 0),
//                        new ASymmetricalBottleneck("asymmetrical_bottleneck_large_goal", 100),
//                        new EmptyGrid("empty_16X16_2-agents", 16, 2, 0),
//                        new EmptyGrid("empty_16X16_2-agents_large_goal", 16, 2, 100)
                },
                /* lvl 1 */
                {
//                        new RoomEnv("room-32-32-4_scen-12_2-agents", 32, 4, 12, 2),
//                        new SanityEnv("independent_8X8_3-agents", 3, 8, 3),
                },
                /* lvl 2 */
                {
//                        new RoomEnv("room-32-32-4_scen_1_2-agents", 32, 4, 1, 2),
                        new SanityEnv("conflict_between_pair_and_single_large_map", 2, 32, 3),
                }

        }
);

vector<vector<SolverCreator *>> solver_creators(
        {   /* lvl 0 */
                {
                        new vi("vi"),

                },

                /* lvl 1 */
                {
                        new rtdp_dijkstra("rtdp_dijkstra"),

                },
                /* lvl 2 */
                {
//                        new rtdp_dijkstra_rtdp("rtdp_dijkstra_rtdp"),
//                        new id_rtdp_default("id_rtdp_default"),
                        new id_rtdp("id_rtdp"),
                }
        }
);

std::string benchmark_solver_on_env(EnvCreator *env_creator, SolverCreator *solver_creator) {
    /* Create the policy */
    Policy *policy = nullptr;
    MapfEnv *env = nullptr;
    env = (*env_creator)();
    policy = (*solver_creator)(env, 1.0);

    /* Train and evaluate */
    policy->train();
    TrainInfo *train_info = policy->get_train_info();
    EvaluationInfo *eval_info = policy->evaluate(100, 1000, 0);

    /* Print results */
    std::cout << "MDR:" << eval_info->mdr;
    std::cout << " rate:" << eval_info->success_rate;
    std::cout << " exec_time:" << eval_info->mean_episode_time;
    std::cout << " train_time:" << train_info->time;
    std::cout << " solver:" << policy->name;

    for (auto item: *train_info->additional_data) {
        std::cout << " " << item.first << ":" << item.second;
    }

    std::cout << endl;

//    /* NOTE: this is redundant when running in fork */
//    delete env->grid;
//    delete env;
//    delete policy;

    if (eval_info->collision_happened) {
        return RESULT_COLLISION;
    }

    return RESULT_OK;
}


int main(int argc, char **argv) {
    vector<InstanceResult> results;
    std::string result;
    InstanceResult instance_result("", "", "");
    int fds[2] = {0};
    pid_t pid = 0;
    ssize_t written_bytes = 0;
    char c_result[20];

    for (size_t env_lvl = 0; env_lvl < env_creators.size(); ++env_lvl) {
        for (EnvCreator *env_creator: env_creators[env_lvl]) {
            cout << endl << env_creator->name << endl;
            for (size_t solver_lvl = env_lvl; solver_lvl < solver_creators.size(); ++solver_lvl) {
                for (SolverCreator *solver_creator: solver_creators[solver_lvl]) {
                    /* Open a pipe for the new child and fork*/
                    pipe(fds);
                    std::cout.flush();
//                    pid = fork();

                    /* Child process, solve the instance and return the result */
                    if (0 == pid) {
                        close(fds[0]);
                        result = benchmark_solver_on_env(env_creator, solver_creator);
                        do {
                            written_bytes = write(fds[1], result.c_str(), result.size() + 1);
                        } while (written_bytes < result.size());
                        close(fds[1]);
                        exit(0);
                    }

                        /* Parent process, read the result after the child is finished */
                    else {
                        close(fds[1]);
                        waitpid(pid, nullptr, 0);
                        result = read(fds[0], c_result, 20);
                        instance_result = InstanceResult(env_creator->name, solver_creator->name,
                                                         std::string(c_result));
                        results.push_back(instance_result);
                    }

                }
            }
        }
    }

    cout << endl;

    bool error_occurred = false;
    for (InstanceResult r: results) {
        if (r.result != RESULT_OK) {
            cout << r.env << ", " << r.solver << ", " << ": " << r.result << endl;
            error_occurred = true;
        }
    }

    if (!error_occurred) {
        cout << "-----------------------------------------Success-----------------------------------------" << endl;
    }

    return 0;
}
