//
// Created by levyvonet on 04/03/2022.
//

#include "online_window.h"

#define DEBUG_PRINT (false)

/** Window ************************************************************************************************/
Window::Window(GridArea area, Policy *policy, AgentsGroup group, const MapfEnv *full_env) :
        area(area), policy(policy), group(group), steps_count(0), reached_count(0), expanded_count(0),
        full_env_(full_env) {
    this->max_steps = calc_max_steps();
}

int Window::calc_max_steps() {
    double total_cells =
            (this->area.bottom_row - this->area.top_row + 1) * (this->area.right_col - this->area.left_col + 1);
    double max_steps = total_cells;

    max_steps *= LIVE_LOCK_BUFFER;

    return max_steps;
}

MultiAgentState *Window::cast_to_window_local_state(const MultiAgentState &s) {
    vector<Location> casted_locations;
    MultiAgentState *group_state = nullptr;
    MultiAgentAction *a = nullptr;

    for (size_t agent: this->group) {
        casted_locations.push_back(s.locations[agent]);
    }
    MapfEnv *local_group_env = get_local_view(this->full_env_, this->group);
    group_state = local_group_env->locations_to_state(casted_locations);

    return group_state;
}

MultiAgentAction *Window::act(const MultiAgentState &s, double timeout_ms) {
    MEASURE_TIME;
    MultiAgentState *group_state = this->cast_to_window_local_state(s);
    MultiAgentAction *a = this->policy->act(*group_state, timeout_ms - ELAPSED_TIME_MS);

    ++this->steps_count;

    return a;
}

std::ostream &operator<<(ostream &os, const Window &w) {
    os << "group: ";
    for (size_t agent: w.group) {
        os << agent << ",";
    }


    os << " area: " << w.area;

    return os;
}


/** Utilities *********************************************************************************************/
int locations_distance(const Location &l1, const Location &l2) {
    return abs(l1.row - l2.row) + abs(l1.col - l2.col);
}

int distance(Window *w1, Window *w2, const MultiAgentState &state) {
    int min_agents_distance = 99999;

    for (size_t a1: w1->group) {
        for (size_t a2: w2->group) {
            min_agents_distance = min(min_agents_distance,
                                      locations_distance(state.locations[a1], state.locations[a2]));
        }
    }

    return min_agents_distance;
}

/** OnlineWindowPolicy ************************************************************************************/

/** internal **********************************************************************************************/
GridArea construct_conflict_area(Grid *grid, const AgentsGroup &group, const MultiAgentState &s) {
    int top_row = grid->max_row;
    int bottom_row = 0;
    int left_col = grid->max_col;
    int right_col = 0;
    for (size_t agent: group) {
        top_row = min(top_row, s.locations[agent].row);
        bottom_row = max(bottom_row, s.locations[agent].row);
        left_col = min(left_col, s.locations[agent].col);
        right_col = max(right_col, s.locations[agent].col);
    }

    return GridArea(top_row, bottom_row, left_col, right_col);
}

GridArea pad_area(Grid *grid, GridArea area, int k) {
    int extra_rows = max(k - (area.bottom_row - area.top_row + 1), 0);
    int extra_cols = max(k - (area.right_col - area.left_col + 1), 0);

    int top_row = max(0, (int) floor(area.top_row - extra_rows / 2.0));
    int left_col = max(0, (int) floor(area.left_col - extra_cols / 2.0));
    int bottom_row = min(grid->max_row, (size_t) ceil(area.bottom_row + extra_rows / 2.0));
    int right_col = min(grid->max_col, (size_t) ceil(area.right_col + extra_cols / 2.0));

    return GridArea(top_row, bottom_row, left_col, right_col);
}

tsl::hopscotch_set<Location> get_intended_locations(Policy *p, Location start, int k, double timeout_ms) {
    MEASURE_TIME;
    tsl::hopscotch_set<Location> res;
    Location curr_location = start;
    for (size_t i = 1; i <= k; ++i) {
        MultiAgentState *curr_state = p->env->locations_to_state({curr_location});
        MultiAgentAction *a = p->act(*curr_state, timeout_ms - ELAPSED_TIME_MS);
        if (ELAPSED_TIME_MS >= timeout_ms) {
            return res;
        }
        curr_location = p->env->grid->execute(curr_location, a->actions[0]);
        delete a;
        res.insert(curr_location);
    }


    return res;
}


Dictionary *
get_window_girth_values(MapfEnv *env, Window *w, const MultiAgentState &s, const vector<Policy *> &single_policies,
                        int d, double timeout_ms) {
    MEASURE_TIME;

    if (DEBUG_PRINT) {
        cout << "calculating girth values for " << w->group.size() << " agents on state" << s << endl;
    }

    /* Generate state space from the window area */
    AreaMultiAgentStateSpace *window_area_state_space = new AreaMultiAgentStateSpace(env->grid,
                                                                                     w->area,
                                                                                     w->group.size());

    /* Set the girth of the area with a fixed value composed of the single agents values */
    vector<ValueFunctionPolicy *> policies;
    vector<vector<size_t>> agents_groups;
    vector<tsl::hopscotch_set<Location>> intended_locations;
    for (size_t agent: w->group) {
        Policy *agent_policy = single_policies[agent];
        policies.push_back((ValueFunctionPolicy *) agent_policy);
        agents_groups.push_back({agent});
        intended_locations.push_back(
                get_intended_locations(agent_policy, s.locations[agent], 2 * d, timeout_ms - ELAPSED_TIME_MS));
        if (ELAPSED_TIME_MS >= timeout_ms) {
            return nullptr;
        }
    }
    SolutionSumHeuristic *h = new SolutionSumHeuristic(policies, agents_groups);
    h->init(env, timeout_ms - ELAPSED_TIME_MS);
    if (ELAPSED_TIME_MS >= timeout_ms) {
        return nullptr;
    }

    /* Set the values of the girth (=single agent on girth, the rest on the area).
     * Add a bonus values if the location of the agent on the girth was intended by its original, self policy. */
    // TODO: this is actually a minor bug, there might be multiple agents on the girth at the same time.
    GirthMultiAgentStateSpace *girth_space_single = new GirthMultiAgentStateSpace(env->grid, w->area, 1);
    GirthMultiAgentStateIterator *girth_space_single_end = girth_space_single->end();
    AreaMultiAgentStateIterator *area_iter = window_area_state_space->begin();
    AreaMultiAgentStateIterator *area_end = window_area_state_space->end();
    Dictionary *girth_values = new Dictionary(0);
    for (; *area_iter != *area_end; ++*area_iter) {
        for (size_t agent_idx = 0; agent_idx < w->group.size(); ++agent_idx) {
            GirthMultiAgentStateIterator *girth_iter = girth_space_single->begin();
            for (; *girth_iter != *girth_space_single_end; ++(*girth_iter)) {
                MultiAgentState temp_state = **area_iter;
                temp_state.locations[agent_idx] = girth_iter->ptr->locations[0];
                temp_state.id = env->grid->calculate_multi_locations_id(temp_state.locations);

                double value = (*h)(&temp_state);

                if (intended_locations[agent_idx].find(temp_state.locations[agent_idx]) !=
                    intended_locations[agent_idx].end()) {
                    value += BONUS_VALUE;
                }

                girth_values->set(temp_state.id, value);
//                if (DEBUG_PRINT) {
//                    cout << "girth state " << temp_state << "got value of " << value << endl;
//                }
            }
        }
    }

    return girth_values;

}

Location get_girth_goal_state_for_agent(MapfEnv *env, Window *w, const MultiAgentState &s,
                                        const vector<Policy *> &single_policies, size_t agent, double timeout_ms) {
    MEASURE_TIME;

    /* If this agents goal_definition location in on the window area, return it. */
    if (w->area.contains(env->goal_state->locations[agent])) {
        return env->goal_state->locations[agent];
    }


    /* Otherwise, the agent must have somewhere outside the window it wants to reach */
    Policy *p = single_policies[agent];
    Location curr_location = s.locations[agent];

    while (w->area.contains(curr_location)) {
        MultiAgentState *curr_state = p->env->locations_to_state({curr_location});
        MultiAgentAction *a = p->act(*curr_state, timeout_ms - ELAPSED_TIME_MS);
        if (ELAPSED_TIME_MS >= timeout_ms) {
            return curr_location;
        }
        curr_location = p->env->grid->execute(curr_location, a->actions[0]);
        delete a;
    }

    return curr_location;

}

class AllowedAreaWithGoalException : public Constraint {
public:
    AllowedAreaWithGoalException(GridArea allowed_area, MultiAgentState goal_state) :
            allowed_area(allowed_area), goal_state(goal_state) {}

    virtual bool is_violated(const MultiAgentState *prev_state, const MultiAgentState *next_state) {
        if (*next_state == this->goal_state) {
            return false;
        }

        /* If one of the agents exits the allow area it is a violation since the final state is not the permitted goal_definition
         * state */
        for (size_t agent_idx = 0; agent_idx < next_state->locations.size(); agent_idx++) {
            if (!this->allowed_area.contains(next_state->locations[agent_idx])) {
                return true;
            }
        }

        return false;
    }

private:
    GridArea allowed_area;
    MultiAgentState goal_state;
};


class SelectedAgentsInGoal : public GoalDefinition {
public:
    SelectedAgentsInGoal(vector<Location> agent_goal_locations, vector<bool> is_important) :
            agent_goal_locations_(agent_goal_locations), is_important_(is_important) {}

    GoalDecision is_goal(const MapfEnv *env, const MultiAgentState &s) override {
        for (size_t agent = 0; agent < this->agent_goal_locations_.size(); ++agent) {
            if (this->is_important_[agent] && this->agent_goal_locations_[agent] == s.locations[agent]) {
                return GoalDecision{true, env->reward_of_goal};
            }
        }

        return GoalDecision{false, 0};
    }

private:
    vector<bool> is_important_;
    vector<Location> agent_goal_locations_;
};

class GirthBonusInGoal : public GoalDefinition {
public:
    GirthBonusInGoal(vector<Location> agent_goal_locations, vector<bool> is_important, GridArea window_area,
                     int bonus) :
            window_area_(window_area), bonus_(bonus) {
        this->selected_in_goal_ = std::make_unique<SelectedAgentsInGoal>(agent_goal_locations, is_important);
    }

    GoalDecision is_goal(const MapfEnv *env, const MultiAgentState &s) override {
        GoalDecision bonus_goal_decision = this->selected_in_goal_->is_goal(env, s);
        if (bonus_goal_decision.is_goal) {
            return GoalDecision{true, this->bonus_};
        }

        for (size_t agent = 0; agent < env->n_agents; ++agent) {
            if (!this->window_area_.contains(s.locations[agent])) {
                return GoalDecision{true, 0}; /* TODO: should it be reward_of_goal? */
            }
        }

        return GoalDecision{false, 0};
    }

private:
    std::unique_ptr<SelectedAgentsInGoal> selected_in_goal_;
    int bonus_;
    GridArea window_area_;
};

class GirthBonusInGoalHeuristic : public Heuristic {
public:
    GirthBonusInGoalHeuristic(vector<bool> is_important, int bonus, GridArea window_area) :
            window_area_(window_area),
            bonus_(bonus),
            any_goal_heuristic_(std::make_unique<AnyGoalHeuristic>(is_important)) {}

    void init(MapfEnv *env, double timeout_milliseconds) override {
        this->any_goal_heuristic_->init(env, timeout_milliseconds);
    }

    virtual double operator()(MultiAgentState *s) override {
        double best_distance_to_bonus_state = (*this->any_goal_heuristic_)(s);
        double bonus_heurisitc_val = (double) this->bonus_ + best_distance_to_bonus_state;

        int min_distance_from_girth = std::numeric_limits<int>::infinity();

        for (size_t agent = 0; agent < this->any_goal_heuristic_->env->n_agents; ++agent) {
            Location &agent_location = s->locations[agent];
            int distance_from_top = agent_location.row - this->window_area_.top_row;
            int distance_from_bottom = this->window_area_.bottom_row - agent_location.row;
            int distance_from_left = agent_location.col - this->window_area_.left_col;
            int distance_from_right = this->window_area_.right_col - agent_location.col;
            vector<int> distances = {distance_from_top, distance_from_bottom, distance_from_left, distance_from_right};

            int distance_from_girth = *std::min_element(distances.begin(), distances.end());

            min_distance_from_girth = min(min_distance_from_girth, distance_from_girth);
        }

        return max((double) min_distance_from_girth, bonus_heurisitc_val);
    }

private:
    unique_ptr<AnyGoalHeuristic> any_goal_heuristic_;
    int bonus_;
    GridArea window_area_;
};

void window_planner_rtdp_only_bonus(MapfEnv *env, float gamma, Window *w, const MultiAgentState &s,
                                    vector<Policy *> single_policies,
                                    int d,
                                    double timeout_ms) {
    MEASURE_TIME;


    /* Create a local view of the agents in the window's group.
     * Allow the agents to only move in the window area. */
    MapfEnv *local_group_env = get_local_view(env, w->group);
    local_group_env->start_state = w->cast_to_window_local_state(s);


    /* Find goals for each agent in the local env */
    vector<Location> local_goals;
    for (size_t agent = 0; agent < local_group_env->n_agents; ++agent) {
        Location local_goal = get_girth_goal_state_for_agent(env, w, s, single_policies, agent,
                                                             timeout_ms - ELAPSED_TIME_MS);
        local_goals.push_back(local_goal);
    }


    /* Set the goal_definition state of the env to be the joint state of local goals, this is helpful for running dijkstra to find
     * the distance for each agent from its goal_definition. */
    local_group_env->goal_state = local_group_env->locations_to_state(local_goals);

    /* Just in case - make sure the goal_definition reward is zero. We don't want it here anyway */
    local_group_env->reward_of_goal = 0;

    /* Decide on the policy we are going to use to solve this window, that depends on edge cases where some/all of the
     * local env agents (original) goal_definition state is inside the current window area */
    RtdpPolicy *window_policy = nullptr;
    bool all_goals_in_window = true;
    for (size_t agent = 0; agent < local_group_env->n_agents; ++agent) {
        all_goals_in_window = all_goals_in_window && w->area.contains(local_goals[agent]);
    }

    if (all_goals_in_window) {
        window_policy = new RtdpPolicy(local_group_env, gamma, "",
                                       new RtdpDijkstraHeuristic(gamma));
    } else {
        /* Not all are in window, that means we need to get out one of the agents which its goal_definition is outside the window.
         * We are going to do this by de-prioritizing the agents which them goal_definition is inside the window */
        vector<bool> is_important;
        for (size_t agent = 0; agent < local_group_env->n_agents; ++agent) {
            is_important.push_back(!w->area.contains(local_goals[agent]));
        }

        /* Set the local env goal_definition to any agent - the first agent reaches its goal_definition means done */
        local_group_env->set_goal_definition(
                std::move(std::make_unique<SelectedAgentsInGoal>(local_goals, is_important)));

        /* Solve the new env */
        window_policy = new RtdpPolicy(local_group_env, gamma, "",
                                       new AnyGoalHeuristic(is_important));
    }


    /* Limit the agents not to go through the girth, meaning they should stay on the
     * window area. The only exception for this is when an agent should reach its goal_definition
     * which is on the girth (the midterm goal_definition is the best state on the window's girth). */
//    local_group_env->add_constraint(new AllowedAreaWithGoalException(w->area, *local_group_env->goal_state));



    /* Solve the selected policy */
    window_policy->train(timeout_ms - ELAPSED_TIME_MS);

    /* Transfer ownership */
    w->policy = window_policy;
}

void window_planner_rtdp(MapfEnv *env, float gamma, Window *w, const MultiAgentState &s,
                         vector<Policy *> single_policies,
                         int d,
                         double timeout_ms) {
    MEASURE_TIME;


    /* Create a local view of the agents in the window's group.
     * Set its start state to the current one. */
    MapfEnv *local_group_env = get_local_view(env, w->group);
    local_group_env->start_state = w->cast_to_window_local_state(s);


    /* Find goals for each agent in the local env */
    vector<Location> local_goals;
    for (size_t agent_idx = 0; agent_idx < local_group_env->n_agents; ++agent_idx) {
        size_t agent = w->group[agent_idx];
        Location local_goal = get_girth_goal_state_for_agent(env, w, s, single_policies, agent,
                                                             timeout_ms - ELAPSED_TIME_MS);
        local_goals.push_back(local_goal);
    }


    /* Set the goal_definition state of the env to be the joint state of local goals, this is helpful for running dijkstra to find
     * the distance for each agent from its goal_definition. */
    local_group_env->goal_state = local_group_env->locations_to_state(local_goals);

    /* Just in case - make sure the goal_definition reward is zero. We don't want it here anyway */
    local_group_env->reward_of_goal = 0;

    /* Decide on the policy we are going to use to solve this window, that depends on edge cases where some/all of the
     * local env agents (original) goal_definition state is inside the current window area */
    RtdpPolicy *window_policy = nullptr;
    bool all_goals_in_window = true;
    for (size_t agent_idx = 0; agent_idx < local_group_env->n_agents; ++agent_idx) {
        all_goals_in_window = all_goals_in_window && w->area.contains(local_goals[agent_idx]);
    }

    if (all_goals_in_window) {
        window_policy = new RtdpPolicy(local_group_env, gamma, "",
                                       new RtdpDijkstraHeuristic(gamma));
    } else {
        /* Not all are in window, that means we need to get out one of the agents which its goal_definition is outside the window.
         * We are going to do this by de-prioritizing the agents which them goal_definition is inside the window */
        vector<bool> is_important;
        for (size_t agent = 0; agent < local_group_env->n_agents; ++agent) {
            is_important.push_back(!w->area.contains(local_goals[agent]));
        }

        /* Set the local env goal_definition to any agent - the first agent reaches its goal_definition means done */
        local_group_env->set_goal_definition(
                std::move(std::make_unique<GirthBonusInGoal>(local_goals, is_important, w->area, BONUS_VALUE)));

        /* Solve the new env */
        window_policy = new RtdpPolicy(local_group_env, gamma, "",
                                       new GirthBonusInGoalHeuristic(is_important, BONUS_VALUE, w->area));
    }


    /* Limit the agents not to go through the girth, meaning they should stay on the
     * window area. The only exception for this is when an agent should reach its goal_definition
     * which is on the girth (the midterm goal_definition is the best state on the window's girth). */
//    local_group_env->add_constraint(new AllowedAreaWithGoalException(w->area, *local_group_env->goal_state));



    /* Solve the selected policy */
    window_policy->train(timeout_ms - ELAPSED_TIME_MS);

    /* Transfer ownership */
    w->policy = window_policy;
}


void window_planner_vi(MapfEnv *env, float gamma, Window *w, const MultiAgentState &s, vector<Policy *> single_policies,
                       int d,
                       double timeout_ms) {
    MEASURE_TIME;

    /* Create a local view of the agents in the window's group. */
    MapfEnv *area_env = get_local_view(env, w->group);

    /* Set the set space to be our subspace, this will cause value iteration to only iterate over the conflict
     * area instead of the whole grid. */
    AreaMultiAgentStateSpace *window_area_state_space = new AreaMultiAgentStateSpace(env->grid,
                                                                                     w->area,
                                                                                     w->group.size());
    area_env->observation_space = window_area_state_space;


    /* Calculate the area girth values, these will be set during the run of value iteration */
    Dictionary *girth_values = get_window_girth_values(env, w, s, single_policies, d, timeout_ms - ELAPSED_TIME_MS);
    if (ELAPSED_TIME_MS >= timeout_ms) {
        return;
    }

    /* Solve the env by value iteration */
    ValueIterationPolicy *policy = new ValueIterationPolicy(area_env, gamma, "", girth_values);
    policy->train(timeout_ms - ELAPSED_TIME_MS);
    delete girth_values;

    /* Transfer ownership */
    w->policy = policy;
}

AllStayExceptFirstActionSpaceIterator &AllStayExceptFirstActionSpaceIterator::operator++() {
    this->ptr->actions[0] = (Action) (int(this->ptr->actions[0]) + 1);
    if (this->ptr->actions[0] == LAST_INVALID_ACTION) {
        this->reach_end();
    }
    this->ptr->id++;
    return *this;
}

AllStayExceptFirstActionSpaceIterator::AllStayExceptFirstActionSpaceIterator(size_t n_agents)
        : MultiAgentActionIterator(n_agents) {}

AllStayExceptFirstActionSpace::AllStayExceptFirstActionSpace(size_t n_agents)
        : MultiAgentActionSpace(n_agents) {

}

MultiAgentActionIterator *AllStayExceptFirstActionSpace::begin() {
    return new AllStayExceptFirstActionSpaceIterator(this->n_agents);
}

MultiAgentActionIterator *AllStayExceptFirstActionSpace::end() {
    AllStayExceptFirstActionSpaceIterator *iter = new AllStayExceptFirstActionSpaceIterator(this->n_agents);
    iter->reach_end();

    return iter;
}

/** private **********************************************************************************************/
Window *OnlineWindowPolicy::merge_windows(Window *w1, Window *w2, const MultiAgentState &s) {
    vector<size_t> new_group;
    for (size_t agent: w1->group) {
        new_group.push_back(agent);
    }
    for (size_t agent: w2->group) {
        new_group.push_back(agent);
    }

    GridArea new_area = construct_conflict_area(this->env->grid, new_group, s);

    GridArea new_area_padded = pad_area(this->env->grid, new_area, this->d);

    /* Look for the desired window in our archive */
    for (Window *archived: *this->archived_windows) {
        if (archived->group == new_group && archived->area == new_area_padded) {
            this->archived_windows->erase(std::remove(this->archived_windows->begin(),
                                                      this->archived_windows->end(), archived));
            return archived;
        }
    }

    sort(new_group.begin(), new_group.end());
    return new Window(new_area_padded, nullptr, new_group, this->env);
}


vector<Window *> OnlineWindowPolicy::destruct_window(Window *w, const MultiAgentState &state) {
    vector<size_t> outside_agents;
    vector<size_t> remain_agents;
    vector<Window *> new_windows;
    GridArea all_grid = GridArea(0, this->env->grid->max_row, 0, this->env->grid->max_col);

    /* Calculate outside agents */
    for (size_t agent: w->group) {
        if (!w->area.contains(state.locations[agent])) {
            outside_agents.push_back(agent);
        } else {
            remain_agents.push_back(agent);
        }
    }

    if (outside_agents.empty()) {
        return {w};
    }

    /* Push all agents which are out of the old window */
    for (size_t single_agent: outside_agents) {
        new_windows.push_back((*this->singles_windows)[single_agent]);
    }

    /* Push the remaining agents in old window */
    if (!remain_agents.empty()) {
        sort(remain_agents.begin(), remain_agents.end());
        new_windows.push_back(new Window(w->area, nullptr, remain_agents, this->env));
    }

    return new_windows;
}


void OnlineWindowPolicy::clear_windows() {
    for (Window *w: *(this->curr_windows)) {
        if (w->group.size() != 1) {
            delete w->policy->env;
            delete w->policy;
        }
    }

    for (Window *w: *(this->archived_windows)) {
        if (w->group.size() != 1) {
            delete w->policy->env;
            delete w->policy;
        }
    }
    this->curr_windows = new vector<Window *>();
    this->archived_windows = new vector<Window *>();
}

bool should_merge(Window *w1, Window *w2, const MultiAgentState &state, int d) {
    if (distance(w1, w2, state) <= d) {
        return true;
    }

//    /* Check also for agents contained in the other window, might happen after expansions (after livelock or deadlock). */
//    for (size_t agent: w1->group) {
//        if (w2->area.contains(state.locations[agent])) {
//            return true;
//        }
//    }
//    for (size_t agent: w2->group) {
//        if (w1->area.contains(state.locations[agent])) {
//            return true;
//        }
//    }

    return false;
}

bool OnlineWindowPolicy::merge_current_windows(const MultiAgentState &state) {
    for (Window *w1: *this->curr_windows) {
        for (Window *w2: *this->curr_windows) {
            // TODO: FIX THIS BUG. This condition is required but not enough, An agent from window
            // w1 might be in the area of w2, even tough its distance from all of the agents in w2
            // is more than d. This is because of expansions. w2 might got expanded multiple times
            // because being in a deadlock/livelock.
            if (w1 != w2) {
                if (should_merge(w1, w2, state, this->d)) {
                    Window *new_window = this->merge_windows(w1, w2, state);
                    if (nullptr != w1->policy) {
                        this->archived_windows->push_back(w1);
                    }
                    if (nullptr != w2->policy) {
                        this->archived_windows->push_back(w2);
                    }
                    this->curr_windows->erase(std::remove(this->curr_windows->begin(), this->curr_windows->end(), w1));
                    this->curr_windows->erase(std::remove(this->curr_windows->begin(), this->curr_windows->end(), w2));
                    this->curr_windows->push_back(new_window);
                    return true;
                }
            }
        }
    }

    return false;
}

Window *OnlineWindowPolicy::try_fit_to_archive(AgentsGroup group, const MultiAgentState &state) {
    for (Window *w: *this->archived_windows) {
        if (w->group == group) {
            bool contains_all = true;
            for (size_t agent: group) {
                contains_all = contains_all && w->area.contains(state.locations[agent]);
            }

            if (contains_all) {
                return w;
            }
        }
    }

    return nullptr;
}


bool OnlineWindowPolicy::might_live_lock(Window *w) {
    return w->steps_count >= w->max_steps /* too many steps, means that we might be in a livelock */ ;
}


void OnlineWindowPolicy::expand_window(Window *w, const MultiAgentState &state, double timeout_ms) {
    MEASURE_TIME;

    int row_count = w->area.bottom_row - w->area.top_row + 1;
    int col_count = w->area.right_col - w->area.left_col + 1;

    int row_diff = row_count / 4 + 1;
    int col_diff = col_count / 4 + 1;

    int new_top = max(0, w->area.top_row - row_diff);
    int new_bottom = min((int) this->env->grid->max_row, w->area.bottom_row + row_diff);
    int new_left = max(0, w->area.left_col - col_diff);
    int new_right = min((int) this->env->grid->max_col, w->area.right_col + col_diff);

    GridArea new_area = GridArea(new_top, new_bottom, new_left, new_right);
    GridArea old_area = w->area;

    /* Set window attributes after expansion */
    w->area = new_area;
    w->steps_count = 0;
    w->max_steps = w->calc_max_steps();
    w->expanded_count++;

    if (!(new_area == old_area)) {
        if (DEBUG_PRINT) {
            cout << "expanding window  " << *w << endl;
        }
        this->plan_window(w, state, timeout_ms - ELAPSED_TIME_MS);
    }

}

void OnlineWindowPolicy::update_current_windows(const MultiAgentState &state, double timeout_ms) {
    /* DEBUG */
//    cout << "----------------------------updating windows------------------------------" << endl;
//    cout << "state is " << state << endl;
//    cout << "-------old windows: --------" << endl;
//    for (Window *w: *this->curr_windows) {
//        cout << *w << endl;
//    }

    MEASURE_TIME;
    vector<Window *> old_windows_copy;
    vector<Window *> *old_windows = this->curr_windows;
    for (Window *old_window: *old_windows) {
        old_windows_copy.push_back(old_window);
    }
    vector<Window *> new_windows;
    bool merge_possible = true;
    Window *archived_window = nullptr;

    /* Destruct required windows */
    for (Window *old_window: *old_windows) {
        int new_windows_count = 0;
        for (Window *new_window: this->destruct_window(old_window, state)) {
            ++new_windows_count;
            new_windows.push_back(new_window);
            for (Window *archived: *this->archived_windows) {
                if (archived->group == new_window->group and archived->area == new_window->area) {
                    this->archived_windows->erase(
                            std::remove(this->archived_windows->begin(), this->archived_windows->end(), archived));
                }
            }
        }
        if (new_windows_count > 1) {
//            if (old_window->policy == nullptr) {
//                cout << "OMG" << endl;
//            }
            this->archived_windows->push_back(old_window);
        }
    }


    this->curr_windows->clear();
    for (Window *new_window: new_windows) {
        this->curr_windows->push_back(new_window);
    }

//    /* DEBUG */
//    cout << "-------new windows after destruction: --------"  << endl;
//    for (Window *w: *this->curr_windows) {
//        cout << *w << endl;
//    }

    /* Merge required new windows */
    while (merge_possible) {
        merge_possible = this->merge_current_windows(state);

        /* Expand windows which might be in live or deadlock */
        for (Window *w: *this->curr_windows) {
            if (this->might_live_lock(w) || this->in_deadlock(w, state, timeout_ms - ELAPSED_TIME_MS)) {
//                cout << "expanding " << *w;
                if (ELAPSED_TIME_MS >= timeout_ms) {
                    return;
                }
                this->expand_window(w, state, timeout_ms - ELAPSED_TIME_MS);
                this->max_times_window_expanded_episode = max(w->expanded_count,
                                                              this->max_times_window_expanded_episode);
                this->livelocks_count_episode++;
//                cout << ", new window is " << *w << endl;
                merge_possible = true;
            }
        }
    }

//    /* DEBUG */
//    cout << "-------new windows after merge: --------" << endl;
//    for (Window *w: *this->curr_windows) {
//        cout << *w << endl;
//    }

    vector<Window *> final_windows;

    /* Plan windows which don't have a policy */
    for (Window *w: *this->curr_windows) {
//        cout << "iterated over window " << *w << endl;
        if (nullptr == w->policy) {
            /* There was not an archived window which fits to the current state */
            archived_window = this->try_fit_to_archive(w->group, state);
            if (nullptr != archived_window) {
//                this->curr_windows->erase(std::remove(this->curr_windows->begin(), this->curr_windows->end(), w));
//                this->curr_windows->push_back(archived_window);
                final_windows.push_back(archived_window);
                this->archived_windows->erase(std::remove(this->archived_windows->begin(),
                                                          this->archived_windows->end(),
                                                          archived_window));

//                cout << "pulled window " << *archived_window << " from archive instead of " << *w << endl;

            } else {
                this->plan_window(w, state, timeout_ms - ELAPSED_TIME_MS);
                final_windows.push_back(w);
//                cout << "planned for window " << *w << endl;
//                if (w->policy == nullptr) {
//                    cout << "OMG" << endl;
//                }
            }
        } else {
            final_windows.push_back(w);
        }
    }

    this->curr_windows->clear();
    for (Window *final_window: final_windows) {
        this->curr_windows->push_back(final_window);
    }

    /* Collect statistics about windows which were just reached during this update */
    for (Window *w: *this->curr_windows) {
        bool reached_window = true;
        for (Window *old_w: old_windows_copy) {
            if (w == old_w) {
                reached_window = false;
            }
        }
        if (reached_window && w->group.size() > 1) {
            w->reached_count++;
            this->max_times_window_reached_episode = max(this->max_times_window_reached_episode, w->reached_count);
        }
    }

//    for (Window *w: *this->curr_windows) {
//        cout << "settled on window " << *w << endl;
//    }
//
//    for (Window *w: *this->curr_windows) {
//        if (nullptr == w->policy) {
//            cout << *w << " had a NULL policy" << endl;
//        }
//    }
}


void OnlineWindowPolicy::plan_window(Window *w, const MultiAgentState &s, double timeout_ms) {
    if (DEBUG_PRINT) {
        cout << "planning window " << *w << ". ";
        cout << " Agents are in state: " << s;
        cout << endl;
    }

    MEASURE_TIME;

    /* Update statistics */
    ++this->replans_count;
    this->replans_max_size = max(w->group.size(), this->replans_max_size);
    int curr_area = (w->area.bottom_row - w->area.top_row + 1) * (w->area.right_col - w->area.left_col + 1);
    if (w->group.size() > this->replans_max_size_episode) {
        this->max_agents_replan_area_episode = curr_area;
    } else if (w->group.size() == this->replans_max_size_episode) {
        this->max_agents_replan_area_episode = max(curr_area, this->max_agents_replan_area_episode);
    }
    this->replans_max_size_episode = max(w->group.size(), this->replans_max_size_episode);


    /* Plan for the window */
    vector<Policy *> single_policies;
    for (int agent = 0; agent < this->env->n_agents; agent++) {
        single_policies.push_back((*this->singles_windows)[agent]->policy);
    }
    this->window_planner_func(this->env, this->gamma, w, s, single_policies, this->d, timeout_ms - ELAPSED_TIME_MS);
}

/** public **************************************************************************************************/

OnlineWindowPolicy::OnlineWindowPolicy(MapfEnv *env, float gamma, const string &name,
                                       SolverCreator *low_level_planner_creator, int d,
                                       window_planner window_planner_func) :
        Policy(env, gamma, name),
        d(d), low_level_planner_creator(low_level_planner_creator), window_planner_func(window_planner_func),
        replans_count(0), replans_sum(0), episodes_count(0), replans_max_size(0),
        max_steps_in_window_episode(0), max_times_window_reached_episode(0), max_times_window_expanded_episode(0),
        livelocks_count_episode(0), max_agents_replan_area_episode(0) {
    this->curr_windows = new vector<Window *>();
    this->archived_windows = new vector<Window *>();
    this->singles_windows = new vector<Window *>();
}

void OnlineWindowPolicy::train(double timeout_ms) {
    MEASURE_TIME;
    vector<AgentsGroup> groups(env->n_agents);

    /* Solve Independently for each agent */
    for (size_t i = 0; i < env->n_agents; ++i) {
        groups[i] = {i};
    }


    CrossedPolicy *singles_policy = solve_local_and_cross(this->env,
                                                          this->gamma,
                                                          timeout_ms - ELAPSED_TIME_MS,
                                                          this->low_level_planner_creator,
                                                          &groups);



    /* Initialize the current windows where each agent is in its own window which spans on all of the grid */
    GridArea all_grid = GridArea(0, this->env->grid->max_row, 0, this->env->grid->max_col);
    for (AgentsGroup group: groups) {
        this->curr_windows->push_back(new Window(all_grid, singles_policy->policies[group[0]], group, this->env));
        this->singles_windows->push_back(new Window(all_grid, singles_policy->policies[group[0]], group, this->env));
    }


    this->train_info->time = ELAPSED_TIME_MS;
}

OnlineWindowPolicy::~OnlineWindowPolicy() {
    this->clear_windows();
}


void OnlineWindowPolicy::eval_episodes_info_process(EvaluationInfo *eval_info) {
    float replans_mean = float(this->replans_sum) / this->episodes_count;
    (*eval_info->additional_data)["replans_mean"] = std::to_string(round(replans_mean * 100) / 100);
    (*eval_info->additional_data)["replans_max_size"] = std::to_string(this->replans_max_size);
}

void OnlineWindowPolicy::eval_episode_info_update(episode_info *episode_info) {
    this->replans_sum += this->replans_count;
    ++this->episodes_count;

    episode_info->replans_count = this->replans_count;
    episode_info->replans_max_size = this->replans_max_size_episode;
    episode_info->max_agents_replan_area = this->max_agents_replan_area_episode;
    episode_info->max_steps_window = this->max_steps_in_window_episode;
    episode_info->max_reached_window = this->max_times_window_reached_episode;
    episode_info->max_expanded_window = this->max_times_window_expanded_episode;
    episode_info->livelock_count = this->livelocks_count_episode;
}

void OnlineWindowPolicy::reset() {
    Policy::reset();

    this->replans_count = 0;
    this->replans_max_size_episode = 0;
    this->max_agents_replan_area_episode = 0;
    this->max_steps_in_window_episode = 0;
    this->max_times_window_reached_episode = 0;
    this->max_times_window_expanded_episode = 0;
    this->livelocks_count_episode = 0;
    this->clear_windows();

    for (Window *w: *this->singles_windows) {
        this->curr_windows->push_back(new Window(w->area, w->policy, w->group, this->env));
    }
}

MultiAgentAction *OnlineWindowPolicy::act(const MultiAgentState &state, double timeout_ms) {
    MEASURE_TIME;
    MultiAgentAction *window_action = nullptr;
    vector<Action> selected_actions(this->env->n_agents);

    this->update_current_windows(state, timeout_ms - ELAPSED_TIME_MS);
    if (ELAPSED_TIME_MS >= timeout_ms) {
        return nullptr;
    }


    for (Window *w: *this->curr_windows) {
        window_action = w->act(state, timeout_ms - ELAPSED_TIME_MS);
        if (!window_action) {
            /* timeout, propagate the null action all the way to evaluate() */
            return nullptr;
        }
        if (w->group.size() > 1) {
            this->max_steps_in_window_episode = max(this->max_steps_in_window_episode, w->steps_count);
        }
        for (size_t i = 0; i < w->group.size(); ++i) {
            selected_actions[w->group[i]] = window_action->actions[i];
        }
    }

    return actions_to_action(selected_actions);
}

bool OnlineWindowPolicy::in_deadlock(Window *w, const MultiAgentState &state, double timeout_ms) {
    MEASURE_TIME;

    /* No policy, this is a new window which cannot be in a deadlock */
    if (w->policy == nullptr) {
        return false;
    }

    MultiAgentAction *a = w->act(*this->env->s, timeout_ms - ELAPSED_TIME_MS);

    if (!a) {
        return false;
    }

    /* No chance for a deadlock if the action is not all stay */
    if (a->id != 0) {
        return false;
    }

    /* All stay action was chosen, we are in a deadlock unless every agent is in its goal_definition */
    bool all_in_goal = true;
    for (size_t agent: w->group) {
        all_in_goal = all_in_goal && this->env->goal_state->locations[agent] == state.locations[agent];
    }
    if (!all_in_goal && DEBUG_PRINT) {
        cout << *w << " was in a deadlock" << endl;
    }

    return !all_in_goal;
}



