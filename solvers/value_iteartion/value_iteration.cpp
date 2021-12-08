//
// Created by levyvonet on 26/10/2021.
//

#include "value_iteration.h"

/** Constants **************************************************************************************************/
#define MAX_ITERATIONS (1000)
#define MDR_EPSILON (0.01)

ValueIterationPolicy::ValueIterationPolicy(MapfEnv *env, float gamma, const string &name) : ValueFunctionPolicy(env,
                                                                                                                gamma,
                                                                                                                name) {
//    this->default_value = 0;
    this->v = new Dictionary(0);
//    this->v = new double[this->env->nS];
//    this->v = new tsl::hopscotch_map<int64_t, double>();
//    std::fill(this->v, this->v + this->env->nS, 0);
}

void ValueIterationPolicy::train() {
    size_t i = 0;
    MultiAgentStateIterator *s = this->env->observation_space->begin();
    MultiAgentActionIterator a = this->env->action_space->begin();
    double q_sa = 0;
    double v_s = -std::numeric_limits<double>::max();
    list<Transition *> *transitions = NULL;
    double max_diff = 0;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;
    Dictionary *prev_v = NULL;
    MultiAgentActionIterator action_end = this->env->action_space->end();
    MultiAgentStateIterator *state_end = this->env->observation_space->end();
    int states_count = 0;
    double value = 0;

    for (i = 0; i < MAX_ITERATIONS; i++) {
        /* Perform a full iteration */
        max_diff = 0;
        if (NULL != prev_v) {
            delete prev_v;
        }
        prev_v = this->v;
        this->v = new Dictionary(0);
        /* Update the value of current state */
        states_count = 0;
        for (s->reach_begin(); *s != *state_end; ++*s) {
            ++states_count;
            v_s = -std::numeric_limits<double>::max();
            /* Calculate Q(s,a) and keep the maximum one */
            for (a.reach_begin(); a != action_end; ++a) {
                q_sa = 0;
                transitions = this->env->get_transitions(**s, *a)->transitions;
                for (Transition *t: *transitions) {
                    if (t->is_collision) {
                        q_sa = -std::numeric_limits<double>::max();
                        break;
                    }

                    q_sa += t->p * (t->reward + this->gamma * prev_v->get(t->next_state->id));
                }

                if (q_sa > v_s) {
                    v_s = q_sa;
                }
            }

            /* Update the value table and the diff */
            max_diff = max(abs(prev_v->get((*s)->id)- v_s), max_diff);
            this->v->set((*s)->id, v_s);
        }

//        for (s->reach_begin(); *s != *state_end; ++*s) {
//            cout << this->v->get((*s)->id) << " ";
//        }
//        cout << endl;

        if (max_diff <= MDR_EPSILON) {
            break;
        }
    }

//    for (s->reach_begin(); *s != *state_end; ++*s) {
//        cout << this->v->get((*s)->id) << " ";
//        if (this->v->get((*s)->id) < 0){
//            cout << "fuck" << endl;
//        }
//    }

    /* Update the training time in train_info */
    (*(this->train_info->additional_data))["n_iterations"] = std::to_string(i + 1);
    end = std::chrono::steady_clock::now();
    auto elapsed_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    float elapsed_time_seconds = float(elapsed_time_milliseconds) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;

    /* NOTE: there are two pointers which are leaking at the end (s_ptr and state_end_ptr) */
}


double ValueIterationPolicy::get_value(MultiAgentState *s) {
    return this->v->get(s->id);
}

ValueIterationPolicy::~ValueIterationPolicy() {
    delete this->v;
}



