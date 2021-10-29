//
// Created by levyvonet on 26/10/2021.
//

#include "value_iteration.h"

/** Constants **************************************************************************************************/
#define MAX_ITERATIONS (100)
#define EPSILON (0.01)

ValueIterationPolicy::ValueIterationPolicy(MapfEnv *env, float gamma, const string &name) : Policy(env, gamma, name) {
    this->v = new ValueTable(0);
}

void ValueIterationPolicy::train() {
    size_t i = 0;
    MultiAgentStateIterator s = this->env->observation_space->begin();
    MultiAgentActionIterator a = this->env->action_space->begin();
    double q_sa = 0;
    double v_s = -std::numeric_limits<double>::min();
    list<Transition *> *transitions = NULL;
    double max_diff = 0;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;
    ValueTable *prev_v = NULL;


    for (i = 0; i < MAX_ITERATIONS; i++) {
        /* Perform a full iteration */
        max_diff = 0;
        prev_v = new ValueTable(*(this->v));
        /* Update the value of current state */
        for (s = this->env->observation_space->begin(); s != this->env->observation_space->end(); ++s) {
            v_s = -std::numeric_limits<double>::min();
            /* Calculate Q(s,a) and keep the maximum one */
            for (a = this->env->action_space->begin(); a != this->env->action_space->end(); ++a) {
                q_sa = 0;
                transitions = this->env->get_transitions(*s, *a);
                for (Transition *t: *transitions) {
                    if (t->is_collision) {
                        q_sa = -std::numeric_limits<double>::min();
                        break;
                    }

                    q_sa += t->p * (t->reward + this->gamma * ((*prev_v)[*t->next_state]));
                }

                if (q_sa > v_s) {
                    v_s = q_sa;
                }
            }

            /* Update the value table and the diff */
            if (std::abs((*(this->v))[*s] - v_s) > max_diff) {
                max_diff = std::abs((*(this->v))[*s] - v_s);
            }
            this->v->set(*s, v_s);
        }

        if (max_diff <= EPSILON) {
            (*(this->train_info->additional_data))["n_iterations"] = std::to_string(i + 1);
            break;
        }
    }

    /* Update the training time in train_info */
    end = std::chrono::steady_clock::now();
    auto elapsed_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    float elapsed_time_seconds = elapsed_time_milliseconds / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;
}

MultiAgentAction *ValueIterationPolicy::act(const MultiAgentState &state) {
    return nullptr;
}

TrainInfo *ValueIterationPolicy::get_train_info() {
    return this->train_info;
}

