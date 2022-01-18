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
#define EPISODE_TIMEOUT_SEC (180)
#define EPISODE_TIMEOUT_MS (EPISODE_TIMEOUT_SEC * 1000)
#define MAX_STEPS (2000)
#define EPISODE_COUNT (30)
#define WORKERS_LIMIT (2)

/** Constants *******************************************************************************************************/
vector<vector<EnvCreator *>> env_creators(
        {   /* lvl 0 */
                {
                        new EmptyGrid("empty_8X8_single_agent", 8, 1, 0),
//                        new EmptyGrid("empty_8X8_2_agents_large_goal", 8, 2, 100),
//                        new EmptyGrid("empty_8X8_2_agents", 8, 2, 0),
                        new SymmetricalBottleneck("symmetrical_bottleneck", 0),
                        new SymmetricalBottleneck("symmetrical_bottleneck_large_goal", 100),
                        new ASymmetricalBottleneck("asymmetrical_bottleneck", 0),
                        new ASymmetricalBottleneck("asymmetrical_bottleneck_large_goal", 100),
//                        new EmptyGrid("empty_16X16_2-agents", 16, 2, 0),
//                        new EmptyGrid("empty_16X16_2-agents_large_goal", 16, 2, 100)
                },
                /* lvl 1 */
                {
                        new RoomEnv("room-32-32-4_scen-12_2-agents", 32, 4, 12, 2),
                        new SanityEnv("independent_8X8_3-agents", 3, 8, 3),
                },
                /* lvl 2 */
                {
                        new RoomEnv("room-32-32-4_scen_1_2-agents", 32, 4, 1, 2),
                },
                /* lvl 3 */
                {
                        new RoomEnv("room-64-64-16_scen_1_2-agents", 64, 16, 1, 2),
                        new RoomEnv("room-64-64-16_scen_1_3-agents", 64, 16, 1, 3),
                        new RoomEnv("room-64-64-8-scen_1_2-agents", 64, 8, 1, 2),
                },
                /* lvl 4 */
                {
                        new RoomEnv("room-64-64-8-scen_1_5-agents", 64, 8, 1, 5),
                        new MazeEnv("maze-128-128-10_scen_2_5-agents", 128, 10, 2, 5),
                        new RoomEnv("room-64-64-16_scen_1_10-agents", 64, 16, 1, 10),
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
//                        new rtdp_dijkstra("rtdp_dijkstra"),

                },
                /* lvl 2 */
                {
//                        new rtdp_dijkstra_rtdp("rtdp_dijkstra_rtdp"),
                },
                /* lvl 3 */
                {
//                        new id_rtdp_default("id_rtdp_default"),
//                        new id_rtdp("id_rtdp"),
                },
                /* lvl 4 */
                {
//                        new online_replan("online_replan_rtdp_2", 2, new rtdp_dijkstra_rtdp("")),
//                        new online_replan("online_replan_rtdp_3", 3, new rtdp_dijkstra_rtdp("")),
//                        new online_replan("online_replan_dijkstra_2", 2, new dijkstra_baseline("")),
//                        new online_replan("online_replan_dijkstra_3", 3, new dijkstra_baseline("")),
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

int main(int argc, char **argv) {
    list<problem_instance> *problems = nullptr;
    bool sanity_test_failed = false;
    std::string last_name = "";

    /* Generate the problems to solve */
    problems = generate_problems();

    /* Create the sanity benchmark db */
    SanityBenchmarksResultDatabase db(problems->size());

    solve_problems(problems, WORKERS_LIMIT, &db, EPISODE_TIMEOUT_MS, EPISODE_COUNT, MAX_STEPS);

    /* Print every result */
    for (problem_instance_result result: db.results) {
        if (PROBLEM_STATUS_FAILED(result) || result.timeout_rate > 0) {
            sanity_test_failed = true;
        }
        if (std::string(result.env_name) != last_name) {
            cout << endl << std::string(result.env_name) << endl;
            last_name = std::string(result.env_name);
        }

        std::cout << "ADR:" << result.adr;
        std::cout << " rate:" << result.rate << "%";
        std::cout << " total_time:" << result.total_time;
        std::cout << " exec_time:" << result.exec_time;
        std::cout << " train_time:" << result.train_time;
        std::cout << " timeout_rate:" << result.timeout_rate << "%";
        std::cout << " stuck_rate:" << result.stuck_rate << "%";
        std::cout << " solver:" << std::string(result.solver_name);
        cout << endl;
    }

    if (sanity_test_failed) {
        return 1;
    }
    return 0;

}
