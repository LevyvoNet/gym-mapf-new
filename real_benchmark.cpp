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
#define EPISODE_TIMEOUT_SEC (180)
#define EPISODE_TIMEOUT_MS (EPISODE_TIMEOUT_SEC * 1000)
#define MAX_STEPS (2000)
#define EPISODE_COUNT (30)
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
                        new RoomEnv("room-64-64-16_scen_1_3-agents", 64, 16, 1, 3),
                        new RoomEnv("room-64-64-8-scen_1_2-agents", 64, 8, 1, 2),
                        new SanityEnv("conflict_between_pair_and_single_large_map", 2, 32, 3),
                },
                /* lvl 4 */
                {
                        new RoomEnv("room-64-64-8-scen_1_5-agents", 64, 8, 1, 5),
                        new MazeEnv("maze-128-128-10_scen_2_5-agents", 128, 10, 2, 5),
                        new RoomEnv("room-64-64-16_scen_1_10-agents", 64, 16, 1, 10),

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
                        new rtdp_dijkstra_rtdp("rtdp_dijkstra_rtdp"),
                },
                /* lvl 3 */
                {
                        new id_rtdp_default("id_rtdp_default"),
                        new id_rtdp("id_rtdp"),
                },
                /* lvl 4 */
                {
                        new online_replan("online_replan_rtdp_2", 2, new rtdp_dijkstra_rtdp("")),
                        new online_replan("online_replan_rtdp_3", 3, new rtdp_dijkstra_rtdp("")),
                        new online_replan("online_replan_dijkstra_2", 2, new dijkstra_baseline("")),
                        new online_replan("online_replan_dijkstra_3", 3, new dijkstra_baseline("")),
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


class CsvResultDataBase : public ResultDatabase {
public:
    string file_name;
    ofstream csv_file;

    CsvResultDataBase(string name) {
        std::ostringstream file_name;
        file_name << name << ".csv";
        this->file_name = file_name.str();

        /* Open the file */
        this->csv_file.open(this->file_name, ios::ate);

        /* Write the columns name row */
        this->csv_file << "col1" << "col2";

        this->csv_file.close();
    }

    virtual void insert(struct problem_instance_result result) {

        /* Open the file */
        this->csv_file.open(this->file_name, ios::ate);

        this->csv_file << result.env_name;
        this->csv_file << "," << result.solver_name;
        this->csv_file << "," << result.adr;
        this->csv_file << "," << result.rate;
        this->csv_file << "," << result.total_time;
        this->csv_file << "," << result.exec_time;
        this->csv_file << "," << result.train_time;
        this->csv_file << "," << result.timeout_rate;
        this->csv_file << "," << result.stuck_rate;
        this->csv_file << "," << result.collision_rate;

        this->csv_file << endl;

        this->csv_file.close();
    }
};

int main(int argc, char **argv) {
    list<problem_instance> *problems = nullptr;
    bool sanity_test_failed = false;
    std::string last_name = "";

    /* Generate the problems to solve */
    problems = generate_problems();

    /* Create the sanity benchmark db */
    CsvResultDataBase db("test");

    solve_problems(problems, WORKERS_LIMIT, &db, EPISODE_TIMEOUT_MS, EPISODE_COUNT, MAX_STEPS);

    return 0;

}
