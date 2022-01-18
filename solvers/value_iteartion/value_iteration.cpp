//
// Created by levyvonet on 26/10/2021.
//

#include "value_iteration.h"

/** Constants **************************************************************************************************/
#define MAX_ITERATIONS (1000)
#define MDR_EPSILON (0.01)

/** Private ***************************************************************************************************/
ValueIterationPolicy::ValueIterationPolicy(MapfEnv *env, float gamma, const string &name, Dictionary *const_vals)
        : ValueFunctionPolicy(env,
                              gamma,
                              name) {
    if (nullptr == const_vals) {
        this->const_vals = new Dictionary(0);
    } else {
        this->const_vals = const_vals;
    }

    this->v = this->const_vals->clone();
}


void ValueIterationPolicy::train(double timeout_milliseconds) {
    size_t i = 0;
    MultiAgentStateIterator *s = this->env->observation_space->begin();
    MultiAgentActionIterator a = this->env->action_space->begin();
    double q_sa = 0;
    double v_s = -std::numeric_limits<double>::max();
    list<Transition *> *transitions = NULL;
    double max_diff = 0;
    Dictionary *prev_v = NULL;
    MultiAgentActionIterator action_end = this->env->action_space->end();
    MultiAgentStateIterator *state_end = this->env->observation_space->end();
    int states_count = 0;
    double value = 0;

    MEASURE_TIME;

    for (i = 0; i < MAX_ITERATIONS; i++) {
        /* Perform a full iteration */
        max_diff = 0;
        if (NULL != prev_v) {
            delete prev_v;
        }
        prev_v = this->v;
        this->v = this->const_vals->clone();
        /* Update the value of current state */
        states_count = 0;
        for (s->reach_begin(); *s != *state_end; ++*s) {
            ++states_count;
            v_s = -std::numeric_limits<double>::max();
            /* Calculate Q(s,a) and keep the maximum one */
            for (a.reach_begin(); a != action_end; ++a) {
                q_sa = 0;
                transitions = this->env->get_transitions(**s, *a)->transitions;
                if (ELAPSED_TIME_MS >= timeout_milliseconds) {
                    /* timeout */
                    return;
                }
                for (Transition *t: *transitions) {
                    if (t->is_collision) {
                        q_sa = -std::numeric_limits<double>::max();
                        break;
                    }

                    q_sa += t->p * (t->reward + this->gamma * prev_v->get(t->next_state->id));
                }
                v_s = max(v_s, q_sa);

            }

            /* Update the value table and the diff */
            max_diff = max(abs(prev_v->get((*s)->id) - v_s), max_diff);
            this->v->set((*s)->id, v_s);
        }

        if (max_diff <= MDR_EPSILON) {
            break;
        }
    }

    /* Update the training time in train_info */
    (*(this->train_info->additional_data))["n_iterations"] = std::to_string(i + 1);
    float elapsed_time_seconds = float(ELAPSED_TIME_MS) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;

    /* NOTE: there are two pointers which are leaking at the end (s_ptr and state_end_ptr) */
}


double ValueIterationPolicy::get_value(MultiAgentState *s) {
    return this->v->get(s->id);
}

ValueIterationPolicy::~ValueIterationPolicy() {
    delete this->v;
}



