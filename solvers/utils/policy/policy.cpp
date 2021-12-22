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
EpisodeInfo::EpisodeInfo(int reward, double time, bool collision, bool timeout) :
        reward(reward), time(time), collision(collision), timeout(timeout) {}

EvaluationInfo::EvaluationInfo() {
    this->mdr = 0;
    this->success_rate = 0;
    this->mean_episode_time = 0;
    this->collision_rate = 0;
    this->timeout_rate = 0;
    this->additional_data = new std::unordered_map<std::string, std::string>;
}

void Policy::eval_episode_info_update(EpisodeInfo episode_info) {}

void Policy::eval_episodes_info_process(EvaluationInfo *eval_info) {

}

EpisodeInfo Policy::evaluate_single_episode(std::size_t max_steps, double timeout_ms) {
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

    /* Reset policy */
    this->reset();

    do {
        steps++;

        /* Select the best action */
        selected_action = this->act(*(this->env->s), timeout_ms - ELAPSED_TIME_MS);
        if (nullptr == selected_action) {
            return EpisodeInfo(-999, ELAPSED_TIME_MS, false, true);
        }
        /* Check if we are stuck */
        if (*selected_action == all_stay) {
            return EpisodeInfo(episode_reward, ELAPSED_TIME_MS, false, true);
        }

        /* Step */
        this->env->step(*selected_action, &next_state, &reward, &done, &is_collision);
        episode_reward += reward;
        delete selected_action;

        /* If collision happened, the episode considered non-solved. Mark that the policy is not sound (illegal solution) */
        if (is_collision) {
            return EpisodeInfo(episode_reward, ELAPSED_TIME_MS, true, false);
        }

        /* The goal was reached, the episode is solved. Update its stats. */
        if (done) {
            return EpisodeInfo(episode_reward, ELAPSED_TIME_MS, false, false);
        }

    } while (steps < max_steps);

    return EpisodeInfo(episode_reward, ELAPSED_TIME_MS, false, true);
}

EvaluationInfo *Policy::evaluate(std::size_t n_episodes,
                                 std::size_t max_steps,
                                 double episode_timeout_ms,
                                 double min_success_rate) {
    EvaluationInfo *eval_info = new EvaluationInfo();

    if (episode_timeout_ms<=0){
        eval_info->timeout_rate = 1;
        return eval_info;
    }

    for (size_t episode = 1; episode <= n_episodes; ++episode) {
        EpisodeInfo episode_info = this->evaluate_single_episode(max_steps, episode_timeout_ms);
        eval_info->episodes_info.push_back(episode_info);

        /* Give the inheriting policy a chance to collect inner data about the last episode */
        this->eval_episode_info_update(episode_info);
    }

    /* aggregate the information from all episodes */
    float episodes_success_count = 0;
    float reward_sum = 0;
    float time_sum = 0;
    float timeout_count = 0;
    float collision_count = 0;
    for (EpisodeInfo episode: eval_info->episodes_info) {
        if (episode.collision) {
            ++collision_count;
            continue;
        }

        if (episode.timeout) {
            ++timeout_count;
            continue;
        }

        /* Episode succeed */
        ++episodes_success_count;
        reward_sum += episode.reward;
        time_sum += episode.time;
    }

    if (episodes_success_count > 0) {
        eval_info->mdr = reward_sum / episodes_success_count;
        eval_info->mean_episode_time = time_sum / episodes_success_count;
        eval_info->success_rate = n_episodes / episodes_success_count;

        eval_info->mdr = round(eval_info->mdr * 100) / 100;
        eval_info->mean_episode_time = round(eval_info->mean_episode_time * 100) / 100;
    }

    if (timeout_count > 0){
        eval_info->timeout_rate = n_episodes / timeout_count;
    }

    if (collision_count > 0) {
        eval_info->collision_rate = n_episodes / collision_count;
    }

    /* Give the inheriting policy a change to process data collected from all of the episodes */
    this->eval_episodes_info_process(eval_info);

    this->reset();

    return eval_info;
}

TrainInfo *Policy::get_train_info() {
    return this->train_info;
}



