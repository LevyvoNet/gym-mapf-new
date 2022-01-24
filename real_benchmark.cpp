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
#define WORKERS_LIMIT (2)

/** Constants *******************************************************************************************************/
#define MIN_SCEN_ID (1)
#define MAX_SCEN_ID (25)
#define MIN_AGENTS (3)
#define MAX_AGENTS (5)
vector<string> MAPS{
        "empty-8-8",
        "empty-16-16",
        "empty-32-32",
        "empty-48-48",

        "room-32-32-4",
        "room-64-64-8",
        "room-64-64-16",

        "maze-128-128-10",

};

vector<SolverCreator *> SOLVERS{
//        new vi("vi"),
//        new rtdp_dijkstra_rtdp("rtdp_dijkstra_rtdp"),
//        new id_rtdp("id_rtdp"),
        new online_replan("online_replan_rtdp_2", 2, new rtdp_dijkstra_rtdp("")),
        new online_replan("online_replan_rtdp_3", 3, new rtdp_dijkstra_rtdp("")),
        new online_replan("online_replan_dijkstra_2", 2, new dijkstra_baseline("")),
        new online_replan("online_replan_dijkstra_3", 3, new dijkstra_baseline("")),
};

vector<EnvCreator *> generate_env_creators() {
    vector<EnvCreator *> res;
    for (string map: MAPS) {
        for (size_t n_agents = MIN_AGENTS; n_agents <= MAX_AGENTS; ++n_agents) {
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


class CsvResultDataBase : public ResultDatabase {
public:
    string file_name;
    ofstream csv_file;

    CsvResultDataBase(string name) {
        std::ostringstream file_name;
        file_name << name << ".csv";
        this->file_name = file_name.str();

        /* Open the file */
        this->csv_file.open(this->file_name, ios::out);

        /* Write the column names row */
        this->csv_file << "env_name";
        this->csv_file << "," << "solver_name";
        this->csv_file << "," << "adr";
        this->csv_file << "," << "rate";
        this->csv_file << "," << "total_time";
        this->csv_file << "," << "exec_time";
        this->csv_file << "," << "train_time";
        this->csv_file << "," << "timeout_rate";
        this->csv_file << "," << "stuck_rate";
        this->csv_file << "," << "collision_rate";

        this->csv_file << endl;

        this->csv_file.close();
    }

    virtual void insert(struct problem_instance_result result) {

        /* Open the file */
        this->csv_file.open(this->file_name, ios::app);

        /* Write the result data */
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
    list<problem_instance> *problems = generate_problems();


    /* Create the sanity benchmark db */
    CsvResultDataBase db(argv[1]);

    solve_problems(problems,
                   WORKERS_LIMIT,
                   &db,
                   EPISODE_TIMEOUT_MS,
                   EPISODE_COUNT,
                   MAX_STEPS);

    return 0;

}
