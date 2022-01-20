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
                                     double timeout_ms,
                                     int episode_count,
                                     int max_steps) {
    struct problem_instance_result res;
    Policy *policy = nullptr;
    MapfEnv *env = nullptr;

    /* Create the policy */
    env = (*problem.env_creator)();
    policy = (*problem.solver_creator)(env, 1.0);

    /* Add the mountains to env */
    add_mountains_to_env(env);

    MEASURE_TIME;

    /* Train and evaluate */
    policy->train(timeout_ms);
    TrainInfo *train_info = policy->get_train_info();
    EvaluationInfo *eval_info = policy->evaluate(episode_count,
                                                 max_steps,
                                                 timeout_ms - ELAPSED_TIME_MS);
    /* Set res fields */
    res.status = PROBLEM_SUCCESS;
    res.id = problem.id;
    strncpy(res.env_name, problem.env_creator->name.c_str(), MAX_ENV_NAME);
    strncpy(res.solver_name, policy->name.c_str(), MAX_SOLVER_NAME);
    res.adr = eval_info->mdr;
    res.rate = eval_info->success_rate;
    res.total_time = eval_info->mean_episode_time + train_info->time;
    res.exec_time = eval_info->mean_episode_time;
    res.train_time = train_info->time;
    res.timeout_rate = eval_info->timeout_rate;
    res.stuck_rate = eval_info->stuck_rate;
    res.collision_rate = eval_info->collision_rate;

    return res;

}

struct worker_data spawn_worker(list<struct worker_data> other_workers,
                                struct problem_instance problem,
                                double timeout_ms,
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
        struct problem_instance_result result = solve(problem, timeout_ms, episode_count, max_steps);
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
        strncpy(new_worker.env_name, problem.env_creator->name.c_str(), MAX_ENV_NAME);
        strncpy(new_worker.solver_name, problem.solver_creator->name.c_str(), MAX_SOLVER_NAME);
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
            strncpy(result.env_name, worker_data.env_name, MAX_ENV_NAME);
            strncpy(result.solver_name, worker_data.solver_name, MAX_SOLVER_NAME);
            return result;
        }


        read_bytes += read_result;
    } while (read_bytes < sizeof(result));

    return result;
}

void solve_problems(list<struct problem_instance> *problems,
                    size_t workers_limit,
                    ResultDatabase *db,
                    double episode_timeout_ms,
                    int eval_episodes_count,
                    int max_steps) {
    list<struct worker_data> workers;
    size_t finished_count = 0;
    size_t problems_count = problems->size();

    while (problems->size() > 0 || workers.size() > 0) {
        while (problems->size() > 0 && workers.size() < workers_limit) {
            /* Add another worker */
            workers.push_back(
                    spawn_worker(workers, *problems->begin(), episode_timeout_ms, eval_episodes_count, max_steps));
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
