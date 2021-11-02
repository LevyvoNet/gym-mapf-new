//
// Created by levyvonet on 26/10/2021.
//

#include "value_iteration.h"

/** Constants **************************************************************************************************/
#define MAX_ITERATIONS (1000)
#define EPSILON (0.01)

ValueIterationPolicy::ValueIterationPolicy(MapfEnv *env, float gamma, const string &name) : Policy(env, gamma, name) {
    this->default_value = 0;
    this->v = new double[this->env->nS];
    std::fill(this->v, this->v+this->env->nS, 0);
}

void ValueIterationPolicy::train() {
    size_t i = 0;
    MultiAgentStateIterator s = this->env->observation_space->begin();
    MultiAgentActionIterator a = this->env->action_space->begin();
    double q_sa = 0;
    double v_s = -std::numeric_limits<double>::max();
    list<Transition *> *transitions = NULL;
    double max_diff = 0;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;
    double *prev_v = NULL;


    for (i = 0; i < MAX_ITERATIONS; i++) {
        /* Perform a full iteration */
        max_diff = 0;
        if (NULL != prev_v){
            delete prev_v;
        }
        prev_v = this->v;
        this->v = new double[this->env->nS];
        /* Update the value of current state */
        for (s = this->env->observation_space->begin(); s != this->env->observation_space->end(); ++s) {
            v_s = -std::numeric_limits<double>::max();
            /* Calculate Q(s,a) and keep the maximum one */
            for (a = this->env->action_space->begin(); a != this->env->action_space->end(); ++a) {
                q_sa = 0;
                transitions = this->env->get_transitions(*s, *a);
                for (Transition *t: *transitions) {
                    if (t->is_collision) {
                        q_sa = -std::numeric_limits<double>::max();
                        break;
                    }

                    q_sa += t->p * (t->reward + this->gamma * prev_v[t->next_state->id]);
                }

                if (q_sa > v_s) {
                    v_s = q_sa;
                }
            }

            /* Update the value table and the diff */
            if (std::abs(prev_v[s->id] - v_s) > max_diff) {
                max_diff = std::abs(prev_v[s->id] - v_s);
            }
            this->v[s->id] = v_s;
        }

        if (max_diff <= EPSILON) {
            break;
        }
    }

    /* Update the training time in train_info */
    (*(this->train_info->additional_data))["n_iterations"] = std::to_string(i + 1);
    end = std::chrono::steady_clock::now();
    auto elapsed_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    float elapsed_time_seconds = float(elapsed_time_milliseconds) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;
}

MultiAgentAction *ValueIterationPolicy::act(const MultiAgentState &state) {
    double q_sa = 0;
    list<Transition *> *transitions = NULL;
    MultiAgentAction *best_action = NULL;
    double max_q = -std::numeric_limits<double>::max();

    for (MultiAgentActionIterator a = this->env->action_space->begin(); a != this->env->action_space->end(); ++a) {
        q_sa = 0;
        transitions = this->env->get_transitions(state, *a);
        for (Transition *t: *transitions) {
            if (t->is_collision) {
                q_sa = -std::numeric_limits<double>::max();
                break;
            }

            q_sa += t->p * (t->reward + this->gamma * this->v[t->next_state->id]);
        }

        if (q_sa > max_q) {
            /* TODO: this copies the actions, make everything a pointer */
            best_action = new MultiAgentAction(a->actions, a->id);
            max_q = q_sa;
        }
    }

    return best_action;
}

TrainInfo *ValueIterationPolicy::get_train_info() {
    return this->train_info;
}

