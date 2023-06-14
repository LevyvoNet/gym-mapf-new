//
// Created by levyvonet on 20/01/2022.
//

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
#define EPISODE_TRAIN_TIMEOUT_SEC (120)
#define EPISODE_EXEC_TIMEOUT_SEC (180)
#define EPISODE_TRAIN_TIMEOUT_MS (EPISODE_TRAIN_TIMEOUT_SEC * 1000)
#define EPISODE_EXEC_TIMEOUT_MS (EPISODE_EXEC_TIMEOUT_SEC * 1000)
#define MAX_STEPS (4000)
#define WORKERS_LIMIT (10)

/** Constants *******************************************************************************************************/
#define MIN_SCEN_ID (1)
#define MAX_SCEN_ID (25)
#define MIN_AGENTS (2)
#define MAX_AGENTS (6)
#define AGENTS_INCREASE (2)
vector<string> MAPS{
        /* City */
//        "berlin_1_256",

        /* Dragon Age */
//        "ost003d",

        /* Open */
//        "empty-8-8",
//                "empty-16-16",
        //        "empty-32-32",
//        "empty-48-48",

        /* Open + obstacles */
//        "random-64-64-10",

        /* Maze */
//        "maze-128-128-10",
        "maze-32-32-4",

        /* Room */
//        "room-32-32-4",
//        "room-64-64-8",
//        "room-64-64-16",
};

vector<SolverCreator *> SOLVERS{
//        new vi("vi"),
//        new rtdp_dijkstra_rtdp("rtdp_dijkstra_rtdp"),
//        new id_rtdp("id_rtdp"),
//        new online_window("online_window_rtdp_2", 2, new rtdp_dijkstra_rtdp(""), window_planner_vi),
//        new online_window("online_window_dijkstra_2", 2, new dijkstra_baseline(""), window_planner_vi),
        new online_window("online_window_vi_2", 2, new vi("vi"), window_planner_vi),
        new online_window("online_window_rtdp_2", 2, new vi("vi"), window_planner_rtdp),

};

vector<EnvCreator *> generate_env_creators() {
    vector<EnvCreator *> res;
    for (string map: MAPS) {
        for (size_t n_agents = MIN_AGENTS; n_agents <= MAX_AGENTS; n_agents += AGENTS_INCREASE) {
            for (size_t scen_id = MIN_SCEN_ID; scen_id <= MAX_SCEN_ID; ++scen_id) {
                std::ostringstream env_name;
                env_name << map << "_scen" << "-" << scen_id << "_agents=" << n_agents;
                res.push_back(new GeneralEnv(env_name.str(), map, scen_id, n_agents));
            }
        }

    }

    return res;
}


list<problem_instance> *generate_problems() {
    list<problem_instance> *res = new list<problem_instance>();
    int id_iter = 0;
    vector<EnvCreator *> env_creators = generate_env_creators();
    for (EnvCreator *env_creator: env_creators) {
        for (SolverCreator *solver_creator: SOLVERS) {
            struct problem_instance problem;
            problem.solver_creator = solver_creator;
            problem.env_creator = env_creator;
            problem.id = id_iter;
            ++id_iter;
            res->push_back(problem);
        }

    }

    return res;
}



int main(int argc, char **argv) {
    list<problem_instance> *problems = generate_problems();

    solve_problems(problems,
                   WORKERS_LIMIT,
                   EPISODE_TRAIN_TIMEOUT_MS,
                   EPISODE_EXEC_TIMEOUT_MS,
                   EPISODE_COUNT,
                   MAX_STEPS,
                   argv[1]);

    return 0;

}
