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

    this->reset();
}

void Policy::reset() {
    this->env->reset();
    this->train_info = new TrainInfo();
}

/** Evaluation ************************************************************************************************/

EvaluationInfo::EvaluationInfo() {
    this->mdr = 0;
    this->success_rate = 0;
    this->mean_episode_time = 0;
    this->collision_happened = false;
}

void Policy::eval_episode_info_update() {

}

void Policy::eval_episode_info_process() {

}

void Policy::evaluate_single_episode(std::size_t max_steps, EvaluationInfo *eval_info) {
    vector<Action> all_stay_vector(this->env->n_agents);
    for (size_t i = 0; i < this->env->n_agents; ++i) {
        all_stay_vector[i] = STAY;
    }
    MultiAgentAction all_stay(all_stay_vector, 0);
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;
    size_t steps = 0;
    int episode_reward = 0;
    MultiAgentState next_state(this->env->start_state->locations, -1);
    int reward = 0;
    bool done = false;
    bool is_collision = false;
    MultiAgentAction *selected_action = NULL;

    /* Reset policy */
    this->reset();

    do {
        steps++;

        /* Select the best action */
        selected_action = this->act(*(this->env->s));
        /* Check if we are stuck */
        if (*selected_action == all_stay) {
            break;
        }

        /* Step */
        this->env->step(*selected_action, &next_state, &reward, &done, &is_collision);
        episode_reward += reward;

        /* If collision happened, the episode considered non-solved. Mark that the policy is not sound (illegal solution) */
        if (is_collision) {
            eval_info->collision_happened = true;
            break;
        }

        /* The goal was reached, the episode is solved. Update its stats. */
        if (done) {
            eval_info->episodes_rewards.push_back(episode_reward);
            end = std::chrono::steady_clock::now();
            auto elapsed_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            float elapsed_time_seconds = float(elapsed_time_milliseconds)/1000;
            eval_info->episodes_times.push_back(round(elapsed_time_seconds * 100) / 100);
        }

    } while (steps < max_steps);

}

EvaluationInfo *Policy::evaluate(std::size_t n_episodes, std::size_t max_steps, double min_success_rate) {
    EvaluationInfo *eval_info = new EvaluationInfo();

    for (size_t episode = 1; episode <= n_episodes; ++episode) {
        this->evaluate_single_episode(max_steps, eval_info);

        /* Give the inheriting policy a change to collect inner data about the last episode */
        this->eval_episode_info_update();
    }

    /* Calculate the general evaluation info */
    if (eval_info->episodes_rewards.size() > 0) {
        float reward_sum = 0;
        float time_sum = 0;
        for (size_t i = 0; i < eval_info->episodes_rewards.size(); ++i) {
            reward_sum += eval_info->episodes_rewards[i];
            time_sum += eval_info->episodes_times[i];
        }

        /* Calculate MDR */
        eval_info->mdr = reward_sum / eval_info->episodes_rewards.size();
        eval_info->mdr = round(eval_info->mdr * 100) / 100;

        /* Calculate mean execution time */
        eval_info->mean_episode_time = time_sum / eval_info->episodes_rewards.size();
        eval_info->mean_episode_time = round(eval_info->mean_episode_time * 100) / 100;

        /* Calculate success rate */
        eval_info->success_rate = round((float(eval_info->episodes_rewards.size())/float(n_episodes)) *100);
    }


    /* Give the inheriting policy a change to process data collected from all of the episodes */
    this->eval_episode_info_process();

    this->reset();

    return eval_info;
}




