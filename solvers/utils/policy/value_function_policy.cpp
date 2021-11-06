//
// Created by levyvonet on 06/11/2021.
//

#include "value_function_policy.h"

MultiAgentAction *ValueFunctionPolicy::select_max_value_action(const MultiAgentState& s) {
    double q_sa = 0;
    list<Transition *> *transitions = NULL;
    MultiAgentAction *best_action = NULL;
    double max_q = -std::numeric_limits<double>::max();

    for (MultiAgentActionIterator a = this->env->action_space->begin(); a != this->env->action_space->end(); ++a) {
        q_sa = 0;
        transitions = this->env->get_transitions(s, *a);
        for (Transition *t: *transitions) {
            if (t->is_collision) {
                q_sa = -std::numeric_limits<double>::max();
                break;
            }

            q_sa += t->p * (t->reward + this->gamma * this->get_value(t->next_state));
        }

        if (q_sa > max_q) {
            /* TODO: this copies the actions, make everything a pointer */
            best_action = new MultiAgentAction(a->actions, a->id);
            max_q = q_sa;
        }
    }

    return best_action;
}

MultiAgentAction *ValueFunctionPolicy::act(const MultiAgentState &state) {
    return this->select_max_value_action(state);
}

ValueFunctionPolicy::ValueFunctionPolicy(MapfEnv *env, float gamma, const string &name) : Policy(env, gamma, name) {}
