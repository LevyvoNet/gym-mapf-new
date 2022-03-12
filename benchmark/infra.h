//
// Created by levyvonet on 16/01/2022.
//

#ifndef GYM_MAPF_INFRA_H
#define GYM_MAPF_INFRA_H

#include <sys/wait.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/resource.h>

#include "gym_mapf/gym_mapf.h"
#include "solvers/solvers.h"
#include "benchmark/utils.h"

/** Constants *******************************************************************************************************/
#define MAX_MAP_NAME (100)
#define MAX_SOLVER_NAME (50)
#define EPISODE_COUNT (30)

#define MAX_RAM (1000000000)

#define BGU_CLUSTER_WORKER_LIMIT (20)
#define POLL_SLEEP_TIME_nS (200000)

/** Structs *********************************************************************************************************/

struct worker_data {
    int problem_id;
    pid_t pid;
    int fd;

    char env_name[MAX_MAP_NAME];
    char solver_name[MAX_SOLVER_NAME];
    int scen_id;
    int n_agents;
};

struct problem_instance {
    int id;

    EnvCreator *env_creator;
    SolverCreator *solver_creator;

};

enum problem_status_code {
    PROBLEM_SUCCESS = 0,
    PROBLEM_FAIL = 1
};

struct problem_instance_result {
    int id;

    problem_status_code status;

    char map_name[MAX_MAP_NAME];
    int scen_id;
    int n_agents;

    char solver_name[MAX_SOLVER_NAME];

    /* Aggregated data */
    double adr;
    int rate;
    double total_time;
    double exec_time;
    double train_time;
    float timeout_rate;
    float stuck_rate;
    float collision_rate;
    float adr_stderr;
    float exec_time_stderr;


    /* Raw episode data */
    struct episode_info episodes_data[EPISODE_COUNT];

    /* Solver proprietary information */
    char replans_max_size[8];
    char replans_mean[8];
    char n_conflicts[8];
    char eval_time[8];
    char init_time[8];
    char conflicts_time[8];
    char n_iterations[8];


};

class ResultDatabase {
public:

    virtual void insert(struct problem_instance_result) = 0;
};


//class CsvResultDatabase: public ResultDatabase{
//
//    virtual void push(struct problem_instance_result);
//};

/** Macros **********************************************************************************************************/
#define PROBLEM_STATUS_FAILED(result) (result.status != PROBLEM_SUCCESS)

struct problem_instance_result
solve(struct problem_instance problem, double timeout_ms, int episode_count, int max_steps);

void solve_problems(list<struct problem_instance> *problems,
                    size_t workers_limit,
                    ResultDatabase *db,
                    double train_timeout_ms,
                    double episode_exec_timeout_ms,
                    int eval_episodes_count,
                    int max_steps,
                    string log_file);


#endif //GYM_MAPF_INFRA_H
