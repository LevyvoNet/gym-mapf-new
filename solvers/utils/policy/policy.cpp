//
// Created by levyvonet on 26/10/2021.
//

#include "policy.h"

TrainInfo::TrainInfo() {
    this->time = 0;
    this->additional_data = new std::unordered_map<std::string, std::string>;
}


Policy::Policy(MapfEnv *env, float gamma, const std::string &name) {
    this->env = env;
    this->gamma = gamma;
    this->name = name;


    this->train_info = new TrainInfo();
    this->reset();
}

Policy::~Policy() {
    if (nullptr != this->train_info) {
        delete this->train_info;
    }
}

void Policy::reset() {
    this->env->reset();
}

/** Evaluation ************************************************************************************************/
EvaluationInfo::EvaluationInfo() {
    this->mdr = 0;
    this->success_rate = 0;
    this->mean_episode_time = 0;
    this->collision_rate = 0;
    this->stuck_rate = 0;
    this->timeout_rate = 0;
    this->oom_rate = 0;
    this->additional_data = new std::unordered_map<std::string, std::string>;
}

void Policy::eval_episode_info_update(episode_info *episode_info) {}

void Policy::eval_episodes_info_process(EvaluationInfo *eval_info) {

}

episode_info Policy::evaluate_single_episode(std::size_t max_steps, double timeout_ms) {
    MEASURE_TIME;
    vector<Action> all_stay_vector(this->env->n_agents);
    for (size_t i = 0; i < this->env->n_agents; ++i) {
        all_stay_vector[i] = STAY;
    }
    MultiAgentAction all_stay(all_stay_vector, 0);
    size_t steps = 0;
    int episode_reward = 0;
    MultiAgentState next_state(this->env->start_state->locations, -1);
    int reward = 0;
    bool done = false;
    bool is_collision = false;
    MultiAgentAction *selected_action = nullptr;
    struct episode_info res;
    pid_t pid = 0;

    /* Initialize structs */
    memset(&res, 0, sizeof(res));

    /* Reset policy */
    this->reset();

    try {
        do {
            steps++;

            /* Select the best action */
            selected_action = this->act(*(this->env->s), timeout_ms - ELAPSED_TIME_MS);
            if (nullptr == selected_action) {
                res.time = ELAPSED_TIME_MS;
                res.end_reason = EPISODE_TIMEOUT;
                /* Give the inheriting policy a chance to collect inner data about the last episode */
                this->eval_episode_info_update(&res);
                return res;
            }
            /* Check if we are stuck */
            if (*selected_action == all_stay) {
//            cout << "chosen all stay" << endl;
                res.reward = episode_reward;
                res.time = ELAPSED_TIME_MS;
                res.end_reason = EPISODE_STUCK;
                /* Give the inheriting policy a chance to collect inner data about the last episode */
                this->eval_episode_info_update(&res);
                return res;
            }

            /* Step */
            this->env->step(*selected_action, &next_state, &reward, &done, &is_collision, false);
            episode_reward += reward;
            delete selected_action;

            /* If collision happened, the episode considered non-solved. Mark that the policy is not sound (illegal solution) */
            if (is_collision) {
                res.reward = episode_reward;
                res.time = ELAPSED_TIME_MS;
                res.end_reason = EPISODE_COLLISION;
                /* Give the inheriting policy a chance to collect inner data about the last episode */
                this->eval_episode_info_update(&res);
                return res;
            }

            /* The goal was reached, the episode is solved. Update its stats. */
            if (done) {
                res.reward = episode_reward;
                res.time = ELAPSED_TIME_MS;
                res.end_reason = EPISODE_SUCCESS;
                /* Give the inheriting policy a chance to collect inner data about the last episode */
                this->eval_episode_info_update(&res);
                return res;
            }

        } while (steps < max_steps);
    } catch (std::bad_alloc const &) {
        res.end_reason = EPISODE_OUT_OF_MEMORY;
        /* Give the inheriting policy a chance to collect inner data about the last episode */
        this->eval_episode_info_update(&res);
        return res;
    }

    res.reward = episode_reward;
    res.time = ELAPSED_TIME_MS;
    res.end_reason = EPISODE_STUCK;

    /* Give the inheriting policy a chance to collect inner data about the last episode */
    this->eval_episode_info_update(&res);

    this->reset();

    return res;
}

float calc_mdr_std(vector<episode_info> episodes, float adr) {
    float squared_distance_sum = 0;
    int count = 0;
    for (episode_info episode: episodes) {
        if (episode.end_reason != EPISODE_SUCCESS) {
            continue;
        }
        ++count;

        squared_distance_sum += pow(episode.reward - adr, 2);
    }

    return sqrt(squared_distance_sum / count);
}

float calc_time_std(vector<episode_info> episodes, float mean_time) {
    float squared_distance_sum = 0;
    int count = 0;
    for (episode_info episode: episodes) {
        if (episode.end_reason != EPISODE_SUCCESS) {
            continue;
        }

        ++count;

        squared_distance_sum += pow(episode.time - mean_time, 2);
    }

    return sqrt(squared_distance_sum / count);
}

EvaluationInfo *Policy::evaluate(size_t n_episodes, size_t max_steps, double episode_timeout_ms, bool forked) {
    EvaluationInfo *eval_info = new EvaluationInfo();


    if (episode_timeout_ms <= 0) {
        eval_info->timeout_rate = 100;
        return eval_info;
    }

    for (size_t episode = 1; episode <= n_episodes; ++episode) {
        struct episode_info episode_info;
        if (!forked) {
            episode_info = this->evaluate_single_episode(max_steps, episode_timeout_ms);

        } else {
            int fds[2] = {0};
            pid_t pid = 0;
            ssize_t written_bytes = 0;

            pipe(fds);
            std::cout.flush();
            pid = fork();

            if (pid == 0){
                close(fds[0]);
                struct episode_info episode_info_child = this->evaluate_single_episode(max_steps, episode_timeout_ms);
                do {
                    written_bytes += write(fds[1], &episode_info_child, sizeof(episode_info_child));
                } while (written_bytes < sizeof(episode_info_child));
                close(fds[1]);
                exit(0);

            } else {
                close(fds[1]);
                ssize_t read_result = 0;
                ssize_t read_bytes = 0;
                memset(&episode_info, 0, sizeof(episode_info));

                do {
                    read_result = read(fds[0], &episode_info, sizeof(episode_info));
                    if (0 >= read_result) {
                        episode_info.end_reason = EPISODE_UNKNOWN_FAILURE;
                        break;
                    }
                    read_bytes += read_result;
                } while (read_bytes < sizeof(episode_info));
            }
        }

        /* Insert the current episode info to the episodes raw data storage */
        eval_info->episodes_info.push_back(episode_info);

        /* If we don't have a chance, give up */
        if (eval_info->episodes_info.size() == EPISODES_TIMEOUT_LIMIT) {
            bool all_timeouts = true;
            for (size_t i = 0; i < EPISODES_TIMEOUT_LIMIT; ++i) {
                all_timeouts = all_timeouts && eval_info->episodes_info[i].end_reason == EPISODE_TIMEOUT;
            }
            if (all_timeouts) {
                break;
            }
        }
    }

    /* aggregate the information from all episodes */
    float episodes_success_count = 0;
    float reward_sum = 0;
    float time_sum_ms = 0;
    float timeout_count = 0;
    float collision_count = 0;
    float stuck_count = 0;
    float oom_count = 0;
    for (episode_info episode: eval_info->episodes_info) {
        if (episode.end_reason == EPISODE_COLLISION) {
            ++collision_count;
            continue;
        }
        if (episode.end_reason == EPISODE_TIMEOUT) {
            ++timeout_count;
            continue;
        }
        if (episode.end_reason == EPISODE_STUCK) {
            ++stuck_count;
            continue;
        }
        if (episode.end_reason == EPISODE_OUT_OF_MEMORY) {
            ++oom_count;
            continue;
        }

        /* Episode succeed */
        ++episodes_success_count;
        reward_sum += episode.reward;
        time_sum_ms += episode.time;
    }

    if (episodes_success_count > 0) {
        eval_info->mdr = reward_sum / episodes_success_count;
        eval_info->mean_episode_time = (time_sum_ms / 1000) / episodes_success_count;
        eval_info->success_rate = round((episodes_success_count / n_episodes) * 100);

        eval_info->mdr_stderr = calc_mdr_std(eval_info->episodes_info,
                                             eval_info->mdr) / sqrt(episodes_success_count);
        eval_info->mean_episode_time_stderr = calc_time_std(eval_info->episodes_info,
                                                            eval_info->mean_episode_time) /
                                              sqrt(episodes_success_count);

        eval_info->mdr = round(eval_info->mdr * 100) / 100;
        eval_info->mean_episode_time = round(eval_info->mean_episode_time * 100) / 100;
        eval_info->mdr_stderr = round(eval_info->mdr_stderr * 100) / 100;
        eval_info->mean_episode_time_stderr = round(eval_info->mean_episode_time_stderr * 100) / 100;
    }

    if (timeout_count > 0) {
        eval_info->timeout_rate = round((timeout_count / n_episodes) * 100);
    }

    if (collision_count > 0) {
        eval_info->collision_rate = round((collision_count / n_episodes) * 100);
    }

    if (stuck_count > 0) {
        eval_info->stuck_rate = round((stuck_count / n_episodes) * 100);
    }

    if (oom_count > 0) {
        eval_info->oom_rate = round((oom_count / n_episodes) * 100);
    }


    /* Give the inheriting policy a change to process data collected from all episodes */
    this->eval_episodes_info_process(eval_info);

    this->reset();

    return eval_info;
}

TrainInfo *Policy::get_train_info() {
    return this->train_info;
}



