//
// Created by levyvonet on 17/01/2022.
//

#include <iostream>
#include <string>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cmath>

#include <gym_mapf/gym_mapf.h>
#include <solvers/solvers.h>
#include "benchmark/available_solvers_envs.h"
#include "benchmark/utils.h"
#include "benchmark/infra.h"

/** Experiment Settings ********************************************************************************************/
#define EPISODE_TRAIN_TIMEOUT_SEC (600)
#define EPISODE_EXEC_TIMEOUT_SEC (120)
#define EPISODE_TRAIN_TIMEOUT_MS (EPISODE_TRAIN_TIMEOUT_SEC * 1000)
#define EPISODE_EXEC_TIMEOUT_MS (EPISODE_EXEC_TIMEOUT_SEC * 1000)
#define EPISODE_TIMEOUT_MS (EPISODE_TIMEOUT_SEC * 1000)
#define MAX_STEPS (4000)
#define WORKERS_LIMIT (1)

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
                        new LongCorridorEnv("corridor", 0)

                },
                /* lvl 1 */
                {
                        new RoomEnv("room-32-32-4_scen-12_2-agents", 32, 4, 12, 2),
                        new SanityEnv("independent_8X8_3-agents", 3, 8, 3),
                        new EmptyGrid("empty_16X16_2-agents", 16, 2, 0),
                        new EmptyGrid("empty_16X16_2-agents_large_goal", 16, 2, 100)
                },
                /* lvl 2 */
                {
                        new RoomEnv("room-32-32-4_scen_1_2-agents", 32, 4, 1, 2),
                },
                /* lvl 3 */
                {
                        new RoomEnv("room-64-64-16_scen_1_2-agents", 64, 16, 1, 2),
                        new RoomEnv("room-64-64-8-scen_1_2-agents", 64, 8, 1, 2),
                },
                /* lvl 4 */
                {

                        /* City */
//                        new BerlinEnv("paris_1_256_scen_2_4-agents", 2, 4),

                        /* Dragon Age */
                        new GeneralEnv("", "ost003d", 4, 4),

                        /* Open */
//                        new GeneralEnv("empty-48-48_scen_1_4-agents", "empty-48-48", 1, 4),

                        /* Open + obstacles */
//                        new GeneralEnv("", "random-64-64-10", 1, 4),

                        /* Maze */
//                       new MazeEnv("", 128, 10, 1, 5),

                        /* Room */
//                        new GeneralEnv("", "room-64-64-16", 1, 6),
//                        new GeneralEnv("", "room-64-64-16", 1, 8),
                }
        }
);

vector<vector<SolverCreator *>> solver_creators(
        {   /* lvl 0 */
                {
//                        new vi("vi"),

                },

                /* lvl 1 */
                {
                        new rtdp_dijkstra("rtdp_dijkstra"),

                },
                /* lvl 2 */
                {
                        new rtdp_dijkstra_rtdp("rtdp_dijkstra_rtdp"),
                },
                /* lvl 3 */
                {
//                        new id_rtdp_default("id_rtdp_default"),
//                        new id_rtdp("id_rtdp"),
                },
                /* lvl 4 */
                {
                        new online_window("online_window_vi_2_vi", 2, new vi("vi"), window_planner_vi),
//                        new online_window("online_window_dijkstra_2_vi", 2, new dijkstra_baseline(""),
//                                          window_planner_vi),
                        new online_window("online_window_vi_2_rtdp", 2, new vi("vi"), window_planner_rtdp),
//                        new online_window("online_window_vi_2_rtdp_only_bonus", 2, new vi("vi"), window_planner_rtdp_only_bonus),
                }
        }
);


list<problem_instance> *generate_problems() {
    list<problem_instance> *res = new list<problem_instance>();
    int id_iter = 0;

    for (size_t env_lvl = 0; env_lvl < env_creators.size(); ++env_lvl) {
        for (EnvCreator *env_creator: env_creators[env_lvl]) {
            for (size_t solver_lvl = env_lvl; solver_lvl < solver_creators.size(); ++solver_lvl) {
                for (SolverCreator *solver_creator: solver_creators[solver_lvl]) {
                    struct problem_instance problem;
                    problem.solver_creator = solver_creator;
                    problem.env_creator = env_creator;
                    problem.id = id_iter;
                    ++id_iter;
                    res->push_back(problem);
                }
            }
        }
    }

    return res;
}


class SanityBenchmarksResultDatabase : public ResultDatabase {
public:
    vector<struct problem_instance_result> results;

    SanityBenchmarksResultDatabase(int problems_count) {
        this->results = vector<struct problem_instance_result>(problems_count);
    }

    virtual void insert(struct problem_instance_result result) {
        this->results[result.id] = result;
    }
};

int sanity_benchmark() {
    list<problem_instance> *problems = nullptr;
    bool sanity_test_failed = false;
    std::string last_name = "";
    int last_scen_id = -1;
    int last_n_agents = -1;

    /* Generate the problems to solve */
    problems = generate_problems();

    /* Create the sanity benchmark db */
    SanityBenchmarksResultDatabase db(problems->size());

    solve_problems(problems,
                   WORKERS_LIMIT,
                   &db,
                   EPISODE_TRAIN_TIMEOUT_MS,
                   EPISODE_EXEC_TIMEOUT_MS,
                   EPISODE_COUNT,
                   MAX_STEPS,
                   "sanity_temp",
                   false);

    /* Print every result */
    for (problem_instance_result result: db.results) {
        if (PROBLEM_STATUS_FAILED(result) || result.timeout_rate > 0) {
            sanity_test_failed = true;
        }
        if (std::string(result.map_name) != last_name || result.scen_id != last_scen_id ||
            result.n_agents != last_n_agents) {
            cout << endl << std::string(result.map_name) << " scen:" << result.scen_id << " n_agents:"
                 << result.n_agents << endl;
            last_name = std::string(result.map_name);
            last_scen_id = result.scen_id;
            last_n_agents = result.n_agents;
        }

        std::cout << "ADR:" << result.adr;
        std::cout << " rate:" << result.rate << "%";
        std::cout << " total_time:" << result.total_time;
        std::cout << " exec_time:" << result.exec_time;
        std::cout << " train_time:" << result.train_time;
        std::cout << " timeout_rate:" << result.timeout_rate << "%";
        std::cout << " stuck_rate:" << result.stuck_rate << "%";
        std::cout << " collision_rate:" << result.collision_rate << "%";
//        std::cout << " ADR_STDERR:" << result.adr_stderr;
//        std::cout << " exec_time_STDERR:" << result.exec_time_stderr;
        std::cout << " solver:" << std::string(result.solver_name);

        if (strcmp(result.init_time, "-")) {
            std::cout << " init_time: " << result.init_time;
        }
        if (strcmp(result.eval_time, "-")) {
            std::cout << " eval_time: " << result.eval_time;
        }
        if (strcmp(result.replans_max_size, "-")) {
            std::cout << " replans_max_size: " << result.replans_max_size;
        }
        if (strcmp(result.replans_mean, "-")) {
            std::cout << " replans_mean: " << result.replans_mean;
        }
        if (strcmp(result.n_conflicts, "-")) {
            std::cout << " n_conflicts: " << result.n_conflicts;
        }
        if (strcmp(result.conflicts_time, "-")) {
            std::cout << " conflicts_time: " << result.conflicts_time;
        }
        if (strcmp(result.n_iterations, "-")) {
            std::cout << " n_iterations: " << result.n_iterations;
        }
        cout << endl;
    }

    if (sanity_test_failed) {
        return 1;
    }
    return 0;
}

/** TOOD: delete all of this */

int restore_online_window_rtdp_crash() {
    SolverCreator *online_window_rtdp = new online_window("online_window_vi_2_rtdp", 2, new vi("vi"),
                                                          window_planner_rtdp);
    EnvCreator *maze_creator = new GeneralEnv("", "maze-32-32-4", 12, 4);

    MapfEnv *env = (*maze_creator)();

//    env->start_state = env->locations_to_state({
//                                                       env->grid->get_location(6, 5),
//                                                       env->grid->get_location(17, 18),
//                                                       env->grid->get_location(19, 16),
//                                                       env->grid->get_location(17, 19),
//                                               });

    Policy *policy = (*online_window_rtdp)(env, 1.0);

    policy->train(EPISODE_TRAIN_TIMEOUT_MS);

    policy->evaluate(1,
                     MAX_STEPS,
                     EPISODE_EXEC_TIMEOUT_MS,
                     false,
                     true);

    return 0;

}

int main(int argc, char **argv) {
    return sanity_benchmark();

//    return restore_online_window_rtdp_crash();
}
