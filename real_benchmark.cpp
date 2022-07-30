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
#define MAX_AGENTS (2)
#define AGENTS_INCREASE (2)
vector<string> MAPS{
        /* City */
//        "berlin_1_256",

        /* Dragon Age */
//        "ost003d"

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
        new online_window("online_window_dijkstra_2", 2, new dijkstra_baseline(""), window_planner_vi),
        new online_window("online_window_vi_2", 2, new vi("vi"), window_planner_vi),

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
        this->csv_file << "map_name";
        this->csv_file << "," << "scen_id";
        this->csv_file << "," << "n_agents";
        this->csv_file << "," << "solver_name";
        this->csv_file << "," << "adr";
        this->csv_file << "," << "adr_stderr";
        this->csv_file << "," << "rate";
        this->csv_file << "," << "total_time";
        this->csv_file << "," << "exec_time";
        this->csv_file << "," << "exec_time_stderr";
        this->csv_file << "," << "train_time";
        this->csv_file << "," << "timeout_rate";
        this->csv_file << "," << "stuck_rate";
        this->csv_file << "," << "collision_rate";
        this->csv_file << "," << "replans_max_size";
        this->csv_file << "," << "replans_mean";
        this->csv_file << "," << "n_conflicts";
        this->csv_file << "," << "eval_time";
        this->csv_file << "," << "init_time";
        this->csv_file << "," << "conflicts_time";
        this->csv_file << "," << "n_iterations";
        this->csv_file << endl;

        this->csv_file.close();
    }

    virtual void insert(struct problem_instance_result result) {

        /* Open the file */
        this->csv_file.open(this->file_name, ios::app);

        /* Write the result data */
        this->csv_file << result.map_name;
        this->csv_file << "," << result.scen_id;
        this->csv_file << "," << result.n_agents;
        this->csv_file << "," << result.solver_name;
        this->csv_file << "," << result.adr;
        this->csv_file << "," << result.adr_stderr;
        this->csv_file << "," << result.rate;
        this->csv_file << "," << result.total_time;
        this->csv_file << "," << result.exec_time;
        this->csv_file << "," << result.exec_time_stderr;
        this->csv_file << "," << result.train_time;
        this->csv_file << "," << result.timeout_rate;
        this->csv_file << "," << result.stuck_rate;
        this->csv_file << "," << result.collision_rate;
        this->csv_file << "," << result.replans_max_size;
        this->csv_file << "," << result.replans_mean;
        this->csv_file << "," << result.n_conflicts;
        this->csv_file << "," << result.eval_time;
        this->csv_file << "," << result.init_time;
        this->csv_file << "," << result.conflicts_time;
        this->csv_file << "," << result.n_iterations;
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
                   EPISODE_TRAIN_TIMEOUT_MS,
                   EPISODE_EXEC_TIMEOUT_MS,
                   EPISODE_COUNT,
                   MAX_STEPS,
                   argv[1]);

    return 0;

}
