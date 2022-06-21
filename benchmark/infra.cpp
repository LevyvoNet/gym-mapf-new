//
// Created by levyvonet on 16/01/2022.
//

#include "infra.h"


bool is_worker_active(struct worker_data worker) {
    int wstatus = 0;

    pid_t res = waitpid(worker.pid, &wstatus, WNOHANG);
    return 0 == res;
}

void close_all_fds(list<struct worker_data> other_workers) {
    for (struct worker_data other_worker: other_workers) {
        close(other_worker.fd);
    }
}

struct problem_instance_result solve(struct problem_instance problem,
                                     double train_timeout_ms,
                                     double exec_timeout_ms,
                                     int episode_count,
                                     int max_steps) {
    struct problem_instance_result res;
    Policy *policy = nullptr;
    MapfEnv *env = nullptr;
    bool should_log = false;
    EvaluationInfo *eval_info = nullptr;
    TrainInfo *train_info = nullptr;
    string field_value;

    /* Initialize structs */
    memset(&res, 0, sizeof(res));

    /* Create the policy */
    env = (*problem.env_creator)();
    policy = (*problem.solver_creator)(env, 1.0);

    /* Add the mountains to env */
    add_mountains_to_env(env);

    /* Limit memory usage */
    struct rlimit rl;

    rl.rlim_max = MAX_RAM;
    rl.rlim_cur = MAX_RAM;
    if (setrlimit(RLIMIT_AS, &rl) < 0) {
        std::cout << "failed to limit" << std::endl;
    }

    /* Train and evaluate */
    try {
        policy->train(train_timeout_ms);
        train_info = policy->get_train_info();

        MEASURE_TIME;
        eval_info = policy->evaluate(episode_count,
                                     max_steps,
                                     exec_timeout_ms - ELAPSED_TIME_MS);
    } catch (std::bad_alloc const &) {
        res.status = PROBLEM_FAIL;
        strncpy(res.map_name, problem.env_creator->map_name.c_str(), MAX_MAP_NAME);
        strncpy(res.solver_name, policy->name.c_str(), MAX_SOLVER_NAME);
        res.id = problem.id;
        res.scen_id = problem.env_creator->scen_id;
        res.n_agents = problem.env_creator->n_agents;
        field_value = "-";
        return res;
    }

    /* Set res fields */
    res.status = PROBLEM_SUCCESS;
    res.id = problem.id;
    strncpy(res.map_name, problem.env_creator->map_name.c_str(), MAX_MAP_NAME);
    res.scen_id = problem.env_creator->scen_id;
    res.n_agents = problem.env_creator->n_agents;
    strncpy(res.solver_name, policy->name.c_str(), MAX_SOLVER_NAME);
    res.adr = eval_info->mdr;
    res.rate = eval_info->success_rate;
    res.total_time = eval_info->mean_episode_time + train_info->time;
    res.exec_time = eval_info->mean_episode_time;
    res.train_time = train_info->time;
    res.timeout_rate = eval_info->timeout_rate;
    res.stuck_rate = eval_info->stuck_rate;
    res.collision_rate = eval_info->collision_rate;
    res.adr_stderr = eval_info->mdr_stderr;
    res.exec_time_stderr = eval_info->mean_episode_time_stderr;

    /* Set solvers train additional data */
    field_value = "-";
    if (train_info->additional_data->find("n_iterations") != train_info->additional_data->end()) {
        field_value = (*(train_info->additional_data))["n_iterations"];
    }
    strncpy(res.n_iterations, field_value.c_str(), 8);

    field_value = "-";
    if (train_info->additional_data->find("conflicts_time") != train_info->additional_data->end()) {
        field_value = (*(train_info->additional_data))["conflicts_time"];
    }
    strncpy(res.conflicts_time, field_value.c_str(), 8);

    field_value = "-";
    if (train_info->additional_data->find("n_conflicts") != train_info->additional_data->end()) {
        field_value = (*(train_info->additional_data))["n_conflicts"];
    }
    strncpy(res.n_conflicts, field_value.c_str(), 8);

    field_value = "-";
    if (train_info->additional_data->find("eval_time") != train_info->additional_data->end()) {
        field_value = (*(train_info->additional_data))["eval_time"];
    }
    strncpy(res.eval_time, field_value.c_str(), 8);

    field_value = "-";
    if (train_info->additional_data->find("init_time") != train_info->additional_data->end()) {
        field_value = (*(train_info->additional_data))["init_time"];
    }
    strncpy(res.init_time, field_value.c_str(), 8);


    /* Set solvers evaluation additional data */
    field_value = "-";
    if (eval_info->additional_data->find("replans_mean") != eval_info->additional_data->end()) {
        field_value = (*(eval_info->additional_data))["replans_mean"];
    }
    strncpy(res.replans_mean, field_value.c_str(), 8);

    field_value = "-";
    if (eval_info->additional_data->find("replans_max_size") != eval_info->additional_data->end()) {
        field_value = (*(eval_info->additional_data))["replans_max_size"];
    }
    strncpy(res.replans_max_size, field_value.c_str(), 8);


    /* Set the episodes raw data */
    for (size_t i = 0; i < EPISODE_COUNT; ++i) {
        res.episodes_data[i] = eval_info->episodes_info[i];
    }

    return res;

}

struct worker_data spawn_worker(list<struct worker_data> other_workers,
                                struct problem_instance problem,
                                double train_timeout_ms,
                                double exec_timeout_ms,
                                int episode_count,
                                int max_steps) {
    int fds[2] = {0};
    pid_t pid = 0;
    ssize_t written_bytes = 0;


    /* Open a pipe for the new child and fork*/
    pipe(fds);
    std::cout.flush();
    pid = fork();

    /* Child process, solve the instance and return the result */
    if (0 == pid) {
        close(fds[0]);
        close_all_fds(other_workers);
        struct problem_instance_result result = solve(problem, train_timeout_ms, exec_timeout_ms, episode_count,
                                                      max_steps);
        do {
            written_bytes += write(fds[1], &result, sizeof(result));
        } while (written_bytes < sizeof(result));
        close(fds[1]);
        exit(0);
    }

        /* Parent process, read the result after the child is finished */
    else {
        close(fds[1]);
        struct worker_data new_worker;
        strncpy(new_worker.env_name, problem.env_creator->name.c_str(), MAX_MAP_NAME);
        strncpy(new_worker.solver_name, problem.solver_creator->name.c_str(), MAX_SOLVER_NAME);
        new_worker.scen_id = problem.env_creator->scen_id;
        new_worker.n_agents = problem.env_creator->n_agents;
        new_worker.problem_id = problem.id;
        new_worker.pid = pid;
        new_worker.fd = fds[0];
        return new_worker;
    }
}

struct problem_instance_result read_result(struct worker_data worker_data) {
    struct problem_instance_result result;
    ssize_t read_bytes = 0;
    ssize_t read_result = 0;

    memset(&result, 0, sizeof(result));

    do {
        read_result = read(worker_data.fd, &result, sizeof(result));
        if (0 >= read_result) {
            result.status = PROBLEM_FAIL;
            result.id = worker_data.problem_id;
            strncpy(result.map_name, worker_data.env_name, MAX_MAP_NAME);
            strncpy(result.solver_name, worker_data.solver_name, MAX_SOLVER_NAME);
            result.scen_id = worker_data.scen_id;
            result.n_agents = worker_data.n_agents;

            return result;
        }


        read_bytes += read_result;
    } while (read_bytes < sizeof(result));

    return result;
}

void create_log_file(string log_file) {
    std::ostringstream file_name;
    file_name << log_file << ".log.csv";
    ofstream log_csv_file;

    log_csv_file.open(file_name.str(), ios::out);

    /* Write columns */
    log_csv_file << "map_name";
    log_csv_file << "," << "scen_id";
    log_csv_file << "," << "n_agents";
    log_csv_file << "," << "solver_name";
    log_csv_file << "," << "reward";
    log_csv_file << "," << "total_time";
    log_csv_file << "," << "exec_time";
    log_csv_file << "," << "train_time";
    log_csv_file << "," << "end_reason";
    /* Solver specific */
    log_csv_file << "," << "replans_max_size";
    log_csv_file << "," << "replans_count";
    log_csv_file << "," << "n_conflicts";
    log_csv_file << "," << "eval_time";
    log_csv_file << "," << "init_time";
    log_csv_file << "," << "conflicts_time";
    log_csv_file << "," << "n_iterations";
    log_csv_file << endl;

    log_csv_file.close();
}

string end_reason(struct episode_info info) {
    if (info.collision) {
        return "collision";
    }
    if (info.timeout) {
        return "timeout";
    }
    if (info.stuck) {
        return "stuck";
    }

    return "success";
}

void log_if_needed(string log_file, struct problem_instance_result result) {
    if (log_file == "") {
        return;
    }

    /* Open the file */
    std::ostringstream file_name;
    file_name << log_file << ".log.csv";
    ofstream log_csv_file;
    log_csv_file.open(file_name.str(), ios::app);

    /* For each episode, write its line */
    for (size_t i = 0; i < EPISODE_COUNT; ++i) {
        log_csv_file << result.map_name;
        log_csv_file << "," << result.scen_id;
        log_csv_file << "," << result.n_agents;
        log_csv_file << "," << result.solver_name;
        log_csv_file << "," << result.episodes_data[i].reward;
        log_csv_file << "," << result.train_time + result.episodes_data[i].time;
        log_csv_file << "," << result.episodes_data[i].time;
        log_csv_file << "," << result.train_time;
        log_csv_file << "," << end_reason(result.episodes_data[i]);
        /* Solver specific */
        log_csv_file << "," << result.episodes_data[i].replans_max_size;
        log_csv_file << "," << result.episodes_data[i].replans_count;
        log_csv_file << "," << result.n_conflicts;
        log_csv_file << "," << result.eval_time;
        log_csv_file << "," << result.init_time;
        log_csv_file << "," << result.conflicts_time;
        log_csv_file << "," << result.n_iterations;
        log_csv_file << endl;
    }
    log_csv_file.close();

}


void solve_problems(list<struct problem_instance> *problems, size_t workers_limit, ResultDatabase *db,
                    double train_timeout_ms, double episode_exec_timeout_ms, int eval_episodes_count, int max_steps,
                    string log_file) {
    list<struct worker_data> workers;
    size_t finished_count = 0;
    size_t problems_count = problems->size();

    create_log_file(log_file);

    while (problems->size() > 0 || workers.size() > 0) {
        while (problems->size() > 0 && workers.size() < workers_limit) {
            /* Add another worker */
            workers.push_back(
                    spawn_worker(workers,
                                 *problems->begin(),
                                 train_timeout_ms,
                                 episode_exec_timeout_ms,
                                 eval_episodes_count,
                                 max_steps));
            problems->erase(problems->begin());
        }

        /* Iterate over all workers, if one of them finished, clear it */
        std::list<struct worker_data>::iterator worker_iter = workers.begin();
        while (workers.end() != worker_iter) {

            /* Check if the worker is done */
            std::list<struct worker_data>::iterator worker_to_delete = workers.end();
            if (!is_worker_active(*worker_iter)) {
                /* The worker is not active anymore, get its result and erase it */
                worker_to_delete = worker_iter;
                struct problem_instance_result problem_result = read_result(*worker_iter);
                db->insert(problem_result);
                log_if_needed(log_file, problem_result);
                ++finished_count;
                cout << "finished " << finished_count << "/" << problems_count << endl;
                worker_to_delete = worker_iter;
            }

            /* Advance to the next worker, delete the previous worker if it was done */
            ++worker_iter;
            if (workers.end() != worker_to_delete) {
                workers.erase(worker_to_delete);
            }
        }

        usleep(POLL_SLEEP_TIME_nS);
    }

    cout << "Finished Running" << endl;
}
