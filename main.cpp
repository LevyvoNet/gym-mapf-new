//
// Created by levyvonet on 21/10/2021.
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

/** Results ********************************************************************************************************/
std::string RESULT_OK = "OK";
std::string RESULT_COLLISION = "COLLISION";
std::string RESULT_ERROR = "ERROR";
std::string RESULT_CHILD_ERROR = "CHILD_ERROR";
std::string RESULT_NOT_SOLVED = "NOT_SOLVED";

/** Experiment Settings ********************************************************************************************/
#define EPISODE_TIMEOUT_SEC (180)
#define EPISODE_TIMEOUT_MS (EPISODE_TIMEOUT_SEC * 1000)
#define MAX_STEPS (2000)
#define EPISODE_COUNT (30)

class TestResult {
public:
    std::string env;
    std::string solver;
    std::string result;

    TestResult(std::string env, std::string solver, std::string result) :
            solver(solver), env(env), result(result) {}
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

std::string benchmark_solver_on_env(EnvCreator *env_creator, SolverCreator *solver_creator) {
    /* Create the policy */
    Policy *policy = nullptr;
    MapfEnv *env = nullptr;
    env = (*env_creator)();
    policy = (*solver_creator)(env, 1.0);
    double timeout = EPISODE_TIMEOUT_MS;

    /* Add the mountains to env */
    add_mountains_to_env(env);

    MEASURE_TIME;

    /* Train and evaluate */
    policy->train(timeout);
    TrainInfo *train_info = policy->get_train_info();
    EvaluationInfo *eval_info = policy->evaluate(EPISODE_COUNT,
                                                 MAX_STEPS,
                                                 EPISODE_TIMEOUT_MS - ELAPSED_TIME_MS);

    /* Print results */
    std::cout << "ADR:" << eval_info->mdr;
    std::cout << " rate:" << eval_info->success_rate << "%";
    std::cout << " total_time:" << eval_info->mean_episode_time + train_info->time;
    std::cout << " exec_time:" << eval_info->mean_episode_time;
    std::cout << " train_time:" << train_info->time;
    std::cout << " timeout_rate:" << eval_info->timeout_rate << "%";
    std::cout << " stuck_rate:" << eval_info->stuck_rate << "%";
    std::cout << " ADR_STDERR:" << eval_info->mdr_stderr;
    std::cout << " exec_time_STDERR:" << eval_info->mean_episode_time_stderr;
    std::cout << " solver:" << policy->name;

    /* Additional data */
    for (auto item: *train_info->additional_data) {
        std::cout << " " << item.first << ":" << item.second;
    }
    for (auto item: *eval_info->additional_data) {
        std::cout << " " << item.first << ":" << item.second;
    }

    std::cout << endl;

//    /* NOTE: this is redundant when running in fork */
//    delete env->grid;
//    delete env;
//    delete policy;

    if (eval_info->collision_rate > 0) {
        return RESULT_COLLISION;
    }

    if (eval_info->success_rate == 0) {
        return RESULT_NOT_SOLVED;
    }

    return RESULT_OK;
}


int run_benchmarks() {
    vector<TestResult> results;
    std::string result = RESULT_ERROR;
    TestResult instance_result("", "", "");
    int fds[2] = {0};
    pid_t pid = 0;
    ssize_t written_bytes = 0;
    char c_result[20];
    int waitpid_result = 0;
    int read_result = 0;
    int child_status = -1;

    for (size_t env_lvl = 0; env_lvl < env_creators.size(); ++env_lvl) {
        for (EnvCreator *env_creator: env_creators[env_lvl]) {
            cout << endl << env_creator->name << endl;
            for (size_t solver_lvl = env_lvl; solver_lvl < solver_creators.size(); ++solver_lvl) {
                for (SolverCreator *solver_creator: solver_creators[solver_lvl]) {
                    /* Open a pipe for the new child and fork*/
                    pipe(fds);
                    std::cout.flush();
                    pid = fork();

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
                        memset(c_result, 0, 20);
                        close(fds[1]);
                        waitpid_result = waitpid(pid, &child_status, 0);
                        if (child_status != 0) {
                            instance_result = TestResult(env_creator->name,
                                                         solver_creator->name,
                                                         RESULT_CHILD_ERROR);
                        } else {
                            read_result = read(fds[0], c_result, 20);
                            instance_result = TestResult(env_creator->name, solver_creator->name,
                                                         std::string(c_result));
                        }
                        results.push_back(instance_result);
                    }

                }
            }
        }
    }

    cout << endl;

    bool error_occurred = false;
    for (TestResult r: results) {
        if (r.result != RESULT_OK) {
            cout << r.env << ", " << r.solver << ": " << r.result << endl;
            error_occurred = true;
        }
    }

    if (error_occurred) {
        cout << "-----------------------------------------Failure-----------------------------------------" << endl;
        return 1;
    }

    cout << "-----------------------------------------Success-----------------------------------------" << endl;
    return 0;
}


int main(int argc, char **argv) {
    return run_benchmarks();
}
