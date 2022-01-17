//
// Created by levyvonet on 16/01/2022.
//

#ifndef GYM_MAPF_INFRA_H
#define GYM_MAPF_INFRA_H

#include <sys/wait.h>
#include <sys/types.h>
#include <unistd.h>

#include "gym_mapf/gym_mapf.h"
#include "solvers/solvers.h"
#include "benchmark/utils.h"

/** Constants *******************************************************************************************************/
#define BGU_CLUSTER_WORKER_LIMIT (20)
#define POLL_SLEEP_TIME_nS (200000)

#define ANY_CHILD (-1)


/** Structs *********************************************************************************************************/

struct worker_data {
    int problem_id;
    pid_t pid;
    int fd;
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
    char env_name[30];
    char solver_name[30];
    double adr;
    int rate;
    double total_time;
    double exec_time;
    double train_time;
    float timeout_rate;
    float stuck_rate;
    float collision_rate;

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

struct problem_instance_result solve(struct problem_instance problem,
                                     double timeout_ms,
                                     int episode_count,
                                     int max_steps);

void solve_problems(list<struct problem_instance> *problems,
                    size_t workers_limit, ResultDatabase *db,
                    double episode_timeout_ms,
                    int eval_episodes_count,
                    int max_steps);


#endif //GYM_MAPF_INFRA_H