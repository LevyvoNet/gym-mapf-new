//
// Created by levyvonet on 06/11/2021.
//

#include "value_function_policy.h"

void ValueFunctionPolicy::select_max_value_action(const MultiAgentState &s,
                                                  double *value,
                                                  MultiAgentAction *ret,
                                                  double timeout_ms) {
    MEASURE_TIME;
    double q_sa = 0;
    list<Transition *> *transitions = NULL;
    MultiAgentAction best_action = MultiAgentAction(this->env->n_agents);
    double max_q = -std::numeric_limits<double>::max();
    MultiAgentActionIterator *action_space_end = this->env->action_space->end();
    MultiAgentActionIterator *a = this->env->action_space->begin();

    for (a->reach_begin(); *a != *action_space_end; ++*a) {
        /* Skip all stay action */
//        if ((*a)->id == 0){
//            continue;
//        }

        q_sa = 0;
        transitions = this->env->get_transitions(s, **a)->transitions;
        if (ELAPSED_TIME_MS >= timeout_ms) {
            return;
        }
        for (Transition *t: *transitions) {
            if (t->is_collision) {
                q_sa = -std::numeric_limits<double>::max();
                break;
            }

            q_sa += t->p * (t->reward + this->gamma * this->get_value(t->next_state));
        }

        if (q_sa > max_q) {
            best_action = **a;
            max_q = q_sa;
        }
    }

    if (nullptr != value) {

        *value = max_q;
    }
    if (nullptr != ret) {

        *ret = best_action;
    }
}

MultiAgentAction *ValueFunctionPolicy::act(const MultiAgentState &state, double timeout_ms) {
    MEASURE_TIME;
    double d = 0;
    MultiAgentAction *a = new MultiAgentAction(this->env->n_agents);
    this->select_max_value_action(state, &d, a, timeout_ms);
    if (ELAPSED_TIME_MS > timeout_ms) {
        delete a;
        return nullptr;
    }

    return a;
}

ValueFunctionPolicy::ValueFunctionPolicy(MapfEnv *env, float gamma, const string &name) : Policy(env, gamma, name) {}
