//
// Created by levyvonet on 04/03/2022.
//

#include "online_window.h"

/** Window ************************************************************************************************/
Window::Window(GridArea area, Policy *policy, AgentsGroup group) :
        area(area), policy(policy), group(group), steps_count(0) {}

MultiAgentAction *Window::act(const MultiAgentState &s, double timeout_ms) {
    MEASURE_TIME;

    vector<Location> casted_locations;
    MultiAgentState *group_state = nullptr;
    MultiAgentAction *a = nullptr;

    for (size_t agent: this->group) {
        casted_locations.push_back(s.locations[agent]);
    }
    group_state = this->policy->env->locations_to_state(casted_locations);
    a = this->policy->act(*group_state, timeout_ms - ELAPSED_TIME_MS);

    return a;
}


/** Utilities *********************************************************************************************/
int locations_distance(const Location &l1, const Location &l2) {
    return abs(l1.row - l2.row) + abs(l1.col - l2.col);
}

int distance(Window *w1, Window *w2, const MultiAgentState &state) {
    int min_agents_distance = std::numeric_limits<int>::infinity();

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
//GridArea construct_conflict_area(Grid *grid, const AgentsGroup &group, const MultiAgentState &s) {
//    int top_row = grid->max_row;
//    int bottom_row = 0;
//    int left_col = grid->max_col;
//    int right_col = 0;
//    for (size_t agent: group) {
//        top_row = min(top_row, s.locations[agent].row);
//        bottom_row = max(bottom_row, s.locations[agent].row);
//        left_col = min(left_col, s.locations[agent].col);
//        right_col = max(right_col, s.locations[agent].col);
//    }
//
//    return GridArea(top_row, bottom_row, left_col, right_col);
//}
//
//GridArea pad_area(Grid *grid, GridArea area, int k) {
//    int extra_rows = max(k - (area.bottom_row - area.top_row + 1), 0);
//    int extra_cols = max(k - (area.right_col - area.left_col + 1), 0);
//
//    int top_row = max(0, (int) floor(area.top_row - extra_rows / 2.0));
//    int left_col = max(0, (int) floor(area.left_col - extra_cols / 2.0));
//    int bottom_row = min(grid->max_row, (size_t) ceil(area.bottom_row + extra_rows / 2.0));
//    int right_col = min(grid->max_col, (size_t) ceil(area.right_col + extra_cols / 2.0));
//
//    return GridArea(top_row, bottom_row, left_col, right_col);
//}
//tsl::hopscotch_set<Location> get_intended_locations(Policy *p, Location start, int k, double timeout_ms) {
//    MEASURE_TIME;
//    tsl::hopscotch_set<Location> res;
//    Location curr_location = start;
//    for (size_t i = 1; i <= k; ++i) {
//        MultiAgentState *curr_state = p->env->locations_to_state({curr_location});
//        MultiAgentAction *a = p->act(*curr_state, timeout_ms - ELAPSED_TIME_MS);
//        if (ELAPSED_TIME_MS >= timeout_ms) {
//            return res;
//        }
//        curr_location = p->env->grid->execute(curr_location, a->actions[0]);
//        delete a;
//        res.insert(curr_location);
//    }
//
//
//    return res;
//}
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

    return new Window(new_area_padded, nullptr, new_group);
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
        new_windows.push_back(new Window(w->area, nullptr, remain_agents));
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
    this->curr_windows = new vector<Window *>();
}

bool OnlineWindowPolicy::merge_current_windows(const MultiAgentState &state) {
    for (Window *w1: *this->curr_windows) {
        for (Window *w2: *this->curr_windows) {
            if (w1 != w2) {
                if (distance(w1, w2, state) <= this->d) {
                    Window *new_window = this->merge_windows(w1, w2, state);
                    std::remove(this->curr_windows->begin(), this->curr_windows->end(), w1);
                    std::remove(this->curr_windows->begin(), this->curr_windows->end(), w2);
                    this->curr_windows->push_back(new_window);
                    return true;
                }
            }
        }
    }

    return false;
}

void OnlineWindowPolicy::update_current_windows(const MultiAgentState &state, double timeout_ms) {
    MEASURE_TIME;

    vector<Window *> *old_windows = this->curr_windows;
    vector<Window *> new_windows;
    bool merged = true;

    /* Destruct required windows */
    for (Window *old_window: *old_windows) {
        for (Window *new_window: this->destruct_window(old_window, state)) {
            new_windows.push_back(new_window);
        }
    }
    this->curr_windows->clear();
    for (Window *new_window: new_windows) {
        this->curr_windows->push_back(new_window);
    }

    /* Merge required new windows */
    while (merged) {
        merged = this->merge_current_windows(state);
    }

    /* Plan windows which don't have a policy */
    for (Window *w: *this->curr_windows) {
        if (nullptr == w->policy) {
            this->plan_window(w, state, timeout_ms - ELAPSED_TIME_MS);
        }
    }
}


void OnlineWindowPolicy::plan_window(Window *w, const MultiAgentState &s, double timeout_ms) {
    MEASURE_TIME;

    /* Generate state space from the conflict area */
    AreaMultiAgentStateSpace *conflict_area_state_space = new AreaMultiAgentStateSpace(this->env->grid,
                                                                                       w->area,
                                                                                       w->group.size());

    /* Create an environment and set its state space to be our sub-space */
    MapfEnv *area_env = get_local_view(this->env, w->group);
    area_env->observation_space = conflict_area_state_space;

    /* Set the girth of the area with a fixed value` composed of the single agents values */
    vector<ValueFunctionPolicy *> policies;
    vector<vector<size_t>> agents_groups;
    vector<tsl::hopscotch_set<Location>> intended_locations;
    for (size_t agent: w->group) {
        Policy *agent_policy = (*this->singles_windows)[agent]->policy;
        policies.push_back((ValueFunctionPolicy *) agent_policy);
        agents_groups.push_back({agent});
        intended_locations.push_back(
                get_intended_locations(agent_policy, s.locations[agent], 2 * this->d, timeout_ms - ELAPSED_TIME_MS));
        if (ELAPSED_TIME_MS >= timeout_ms) {
            return;
        }
    }
    SolutionSumHeuristic *h = new SolutionSumHeuristic(policies, agents_groups);
    h->init(this->env, timeout_ms - ELAPSED_TIME_MS);

    /* Only a single one outside the area */
    GirthMultiAgentStateSpace *girth_space_single = new GirthMultiAgentStateSpace(this->env->grid, w->area, 1);
    GirthMultiAgentStateIterator *girth_space_single_end = girth_space_single->end();
    AreaMultiAgentStateIterator *area_iter = conflict_area_state_space->begin();
    AreaMultiAgentStateIterator *area_end = conflict_area_state_space->end();
    Dictionary *girth_values = new Dictionary(0);
    for (; *area_iter != *area_end; ++*area_iter) {
        for (size_t agent_idx = 0; agent_idx < w->group.size(); ++agent_idx) {
            GirthMultiAgentStateIterator *girth_iter = girth_space_single->begin();
            for (; *girth_iter != *girth_space_single_end; ++(*girth_iter)) {
                MultiAgentState temp_state = **area_iter;
                temp_state.locations[agent_idx] = girth_iter->ptr->locations[0];
                temp_state.id = this->env->grid->calculate_multi_locations_id(temp_state.locations);

                double value = (*h)(&temp_state);

                if (intended_locations[agent_idx].find(temp_state.locations[agent_idx]) !=
                    intended_locations[agent_idx].end()) {
                    value += BONUS_VALUE;
                }

                girth_values->set(temp_state.id, value);
            }
        }
    }

    /* Solve the env */
    Policy *policy = this->window_planner_func(area_env, girth_values, this->gamma, timeout_ms - ELAPSED_TIME_MS);
    delete girth_values;

    w->policy = policy;
}

/** public **************************************************************************************************/

OnlineWindowPolicy::OnlineWindowPolicy(MapfEnv *env, float gamma, const string &name,
                                       SolverCreator *low_level_planner_creator, int d,
                                       window_planner window_planner_func) :
        Policy(env, gamma, name),
        d(d), low_level_planner_creator(low_level_planner_creator), window_planner_func(window_planner_func),
        replans_count(0), replans_sum(0), episodes_count(0), replans_max_size(0) {
    this->curr_windows = new vector<Window *>();
    this->singles_windows = new vector<Window *>();
}

void OnlineWindowPolicy::train(double timeout_ms) {
    MEASURE_TIME;
    vector<vector<size_t>> groups(env->n_agents);

    /* Solve Independently for each agent */
    for (size_t i = 0; i < env->n_agents; ++i) {
        groups[i] = {i};
    }


    CrossedPolicy *singles_policy = solve_local_and_cross(this->env,
                                                          this->gamma,
                                                          timeout_ms - ELAPSED_TIME_MS,
                                                          this->low_level_planner_creator, &groups);



    /* Initialize the current windows where each agent is in its own window which spans an all of the grid */
    GridArea all_grid = GridArea(0, this->env->grid->max_row, 0, this->env->grid->max_col);
    for (AgentsGroup group: groups) {
        this->curr_windows->push_back(new Window(all_grid, singles_policy->policies[group[0]], group));
        this->singles_windows->push_back(new Window(all_grid, singles_policy->policies[group[0]], group));
    }


    float elapsed_time_seconds = float(ELAPSED_TIME_MS) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;
}

OnlineWindowPolicy::~OnlineWindowPolicy() {
    this->clear_windows();
}


void OnlineWindowPolicy::eval_episodes_info_process(EvaluationInfo *eval_info) {
    float replans_mean = float(this->replans_sum) / this->episodes_count;
    (*eval_info->additional_data)["replans_mean"] = std::to_string((int) round(replans_mean * 100) / 100);
    (*eval_info->additional_data)["replans_max_size"] = std::to_string(this->replans_max_size);
}

void OnlineWindowPolicy::eval_episode_info_update(episode_info *episode_info) {
    this->replans_sum += this->replans_count;
    ++this->episodes_count;

    episode_info->replans_count = this->replans_count;
    episode_info->replans_max_size = this->replans_max_size_episode;
}

void OnlineWindowPolicy::reset() {
    Policy::reset();

    this->replans_count = 0;
    this->replans_max_size_episode = 0;
    this->clear_windows();

    for (Window *w: *this->singles_windows) {
        this->curr_windows->push_back(new Window(w->area, w->policy, w->group));
    }
}

MultiAgentAction *OnlineWindowPolicy::act(const MultiAgentState &state, double timeout_ms) {
    MEASURE_TIME;
    MultiAgentAction *window_action = nullptr;
    vector<Action> selected_actions(this->env->n_agents);

    this->update_current_windows(state, timeout_ms - ELAPSED_TIME_MS);


    for (Window *w: *this->curr_windows) {
        window_action = w->act(state, timeout_ms - ELAPSED_TIME_MS);
        for (size_t i = 0; i < w->group.size(); ++i) {
            selected_actions[w->group[i]] = window_action->actions[i];
        }
    }

    return actions_to_action(selected_actions);
}











