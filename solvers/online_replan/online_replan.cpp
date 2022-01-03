//
// Created by levyvonet on 28/11/2021.
//

#include "online_replan.h"

#define BONUS_VALUE (100)
/** Utilities ***************************************************************************************************/
vector<size_t> PRIMES = {2, 3, 5, 7, 9, 11, 13, 17, 19};

size_t hash<vector<size_t>>::operator()(const vector<size_t> &v) const {
    size_t h = 0;
    for (
            size_t i = 0;
            i < v.

                    size();

            ++i) {
        h +=
                pow(PRIMES[(i + 1) % PRIMES.size()], v[i]
                );
    }

    return h;
}

/** Private methods *********************************************************************************************/
int OnlineReplanPolicy::calc_distance(const Location &l1, const Location &l2) {
    return abs(l1.row - l2.row) + abs(l1.col - l2.col);
}

vector<vector<size_t>> OnlineReplanPolicy::divide_to_groups(const MultiAgentState &s) {
    vector<vector<size_t>> direct_neighbours(this->env->n_agents);

    /* Calculate the direct neighbours */
    for (size_t i = 0; i < this->env->n_agents; ++i) {
        for (size_t j = 0; j < this->env->n_agents; ++j) {
            /* Don't need to check symmetrical cases */
            if (i >= j) {
                continue;
            }

            if (this->calc_distance(s.locations[i], s.locations[j]) <= this->k) {
                direct_neighbours[i].push_back(j);
                direct_neighbours[j].push_back(i);
            }
        }
    }

    /* Run DFS-like search and determine the connectivity components of the graph */
    vector<vector<size_t>> groups;
    for (size_t agent = 0; agent < this->env->n_agents; ++agent) {
        /* If the agent is already part of a group (black), continue to the next one */
        bool is_black = false;
        for (vector<size_t> g: groups) {
            for (size_t agent_in_group: g) {
                if (agent_in_group == agent) {
                    is_black = true;
                }
            }
        }
        if (is_black) {
            continue;
        }

        /* Calculate the connectivity component of the current agent */
        vector<size_t> new_group;
        std::stack<size_t> stack;
        stack.push(agent);
        while (stack.size() > 0) {
            size_t curr_agent = stack.top();
            stack.pop();
            for (size_t n: direct_neighbours[curr_agent]) {
                bool n_in_group = false;
                for (size_t agent_from_group: new_group) {
                    if (agent_from_group == n) {
                        n_in_group = true;
                        break;
                    }
                }
                if (!n_in_group) {
                    stack.push(n);
                }
            }
            new_group.push_back(curr_agent);
        }
        std::sort(new_group.begin(), new_group.end());
        groups.push_back(new_group);
    }

    return groups;
}

GridArea construct_conflict_area(Grid *grid, const vector<size_t> &group, const MultiAgentState &s) {
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
        if (ELAPSED_TIME_MS >= timeout_ms){
            return res;
        }
        curr_location = p->env->grid->execute(curr_location, a->actions[0]);
        delete a;
        res.insert(curr_location);
    }


    return res;
}

Policy *OnlineReplanPolicy::replan(const vector<size_t> &group,
                                   const MultiAgentState &s,
                                   double timeout_ms) {
    MEASURE_TIME;
    /* Calculate the conflict area */
    GridArea conflict_area = construct_conflict_area(this->env->grid, group, s);

    /* Pad the area TODO: is it good? */
    conflict_area = pad_area(this->env->grid, conflict_area, this->k);

    /* Generate state space from the conflict area */
    AreaMultiAgentStateSpace *conflict_area_state_space = new AreaMultiAgentStateSpace(this->env->grid,
                                                                                       conflict_area,
                                                                                       group.size());

    /* Create an environment and set its state space to be our sub-space */
    MapfEnv *area_env = get_local_view(this->env, group);
    area_env->observation_space = conflict_area_state_space;

    /* Set the girth of the area with a fixed value composed of the single agents values */
    vector<ValueFunctionPolicy *> policies;
    vector<vector<size_t>> agents_groups;
    vector<tsl::hopscotch_set<Location>> intended_locations;
    for (size_t agent: group) {
        Policy *agent_policy = this->local_policy->policies[agent];
        policies.push_back((ValueFunctionPolicy *) agent_policy);
        agents_groups.push_back({agent});
        intended_locations.push_back(get_intended_locations(agent_policy, s.locations[agent], 2 * this->k, timeout_ms - ELAPSED_TIME_MS));
        if (ELAPSED_TIME_MS >= timeout_ms){
            return nullptr;
        }
    }
    SolutionSumHeuristic *h = new SolutionSumHeuristic(policies, agents_groups);
    h->init(this->env, timeout_ms - ELAPSED_TIME_MS);

    /* Only a single one outside the area */
    GirthMultiAgentStateSpace *girth_space_single = new GirthMultiAgentStateSpace(this->env->grid, conflict_area, 1);
    GirthMultiAgentStateIterator *girth_space_single_end = girth_space_single->end();
    AreaMultiAgentStateIterator *area_iter = conflict_area_state_space->begin();
    AreaMultiAgentStateIterator *area_end = conflict_area_state_space->end();
    Dictionary *girth_values = new Dictionary(0);
    for (; *area_iter != *area_end; ++*area_iter) {
        for (size_t agent_idx = 0; agent_idx < group.size(); ++agent_idx) {
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



    /* Solve the env by value iteration */
    ValueIterationPolicy *policy = new ValueIterationPolicy(area_env, this->gamma, "", girth_values);
    policy->train(timeout_ms - ELAPSED_TIME_MS);
    delete girth_values;

    /* Save the new policy in replans cache */
    if (!this->replans->contains(group)) {
        (*this->replans)[group] = new tsl::hopscotch_map<GridArea, Policy *>();
    }
    (*(*this->replans)[group])[conflict_area] = policy;
    ++this->replans_count;
    this->replans_max_size = max(group.size(), this->replans_max_size);

//    /* debug print */
//    cout << "replanned for group [";
//    for (size_t agent: group) {
//        cout << agent << ", ";
//    }
//    cout << "]";
//    cout << " and conflict starts at " << conflict_area.top_row << ","
//         << conflict_area.left_col << " ends at " << conflict_area.bottom_row << "," << conflict_area.right_col << ". ";
//
//    cout << "full state is " << s << endl;

    return policy;
}

Policy *OnlineReplanPolicy::search_replan(const vector<size_t> &group, const MultiAgentState &s) {
    if (this->replans->contains(group)) {
        /* Search for an area which contains our state */
        for (auto item: *(*this->replans)[group]) {
            GridArea area = item.first;
            bool contains_all = true;
            for (size_t agent: group) {
                contains_all = contains_all && area.contains(s.locations[agent]);
            }
            if (contains_all) {
                return item.second;
            }
        }
    }
    return nullptr;
}

MultiAgentAction *OnlineReplanPolicy::select_action_for_group(vector<size_t> group,
                                                              const MultiAgentState &s,
                                                              double timeout_ms) {
    MEASURE_TIME;
    Policy *policy = nullptr;
    vector<Location> casted_locations;
    MultiAgentState *group_state = nullptr;
    MultiAgentAction *a = nullptr;


    /* In case of a single agent just return from the local policy */
    if (1 == group.size()) {
        policy = this->local_policy->policies[group[0]];
    } else {
        /* Search for an existing policy for that group and location */
        policy = this->search_replan(group, s);
        if (nullptr == policy) {
            /* Replan for this group */
            policy = this->replan(group, s, timeout_ms - ELAPSED_TIME_MS);
            if (ELAPSED_TIME_MS >= timeout_ms){
                return nullptr;
            }
        }

    }

    /* Extract the action from the retrieved policy */
    for (size_t agent: group) {
        casted_locations.push_back(s.locations[agent]);
    }
    group_state = policy->env->locations_to_state(casted_locations);
    a = policy->act(*group_state, timeout_ms - ELAPSED_TIME_MS);


l_cleanup:
    delete group_state;

    return a;
}

/** Public methods ***********************************************************************************************/


OnlineReplanPolicy::OnlineReplanPolicy(MapfEnv *env,
                                       float gamma,
                                       const string &name,
                                       SolverCreator *low_level_planner_creator,
                                       int k) :
        Policy(env, gamma, name),
        k(k), low_level_planner_creator(low_level_planner_creator), local_policy(nullptr),
        replans_count(0), replans_sum(0), episodes_count(0), replans_max_size(0) {
    this->replans = new tsl::hopscotch_map<vector<size_t>, tsl::hopscotch_map<GridArea, Policy *> *>();
}

OnlineReplanPolicy::~OnlineReplanPolicy() {
    delete this->local_policy;
    this->delete_replans();
}

void OnlineReplanPolicy::train(double timeout_ms) {
    MEASURE_TIME;
    vector<vector<size_t>> groups(env->n_agents);

    /* Solve Independently for each agent */
    for (size_t i = 0; i < env->n_agents; ++i) {
        groups[i] = {i};
    }
    this->local_policy = solve_local_and_cross(this->env,
                                               this->gamma,
                                               timeout_ms - ELAPSED_TIME_MS,
                                               this->low_level_planner_creator, &groups);

    float elapsed_time_seconds = float(ELAPSED_TIME_MS) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;
}


void OnlineReplanPolicy::reset() {
    Policy::reset();

    this->replans_count = 0;
    this->delete_replans();

    this->replans = new tsl::hopscotch_map<vector<size_t>, tsl::hopscotch_map<GridArea, Policy *> *>();

}

MultiAgentAction *OnlineReplanPolicy::act(const MultiAgentState &state, double timeout_ms) {
    MEASURE_TIME;
    vector<vector<size_t>> groups = this->divide_to_groups(state);
    vector<Action> selected_actions(this->env->n_agents);
    MultiAgentAction *group_action = nullptr;

    for (vector<size_t> group: groups) {
        group_action = this->select_action_for_group(group, state, timeout_ms - ELAPSED_TIME_MS);
        if (ELAPSED_TIME_MS >= timeout_ms) {
            return nullptr;
        }
        for (size_t i = 0; i < group.size(); ++i) {
            selected_actions[group[i]] = group_action->actions[i];
        }
        delete group_action;
    }

    return actions_to_action(selected_actions);
}

void OnlineReplanPolicy::eval_episodes_info_process(EvaluationInfo *eval_info) {
    float replans_mean = float(this->replans_sum) / this->episodes_count;
    (*eval_info->additional_data)["replans_mean"] = std::to_string(replans_mean);
    (*eval_info->additional_data)["replans_max_size"] = std::to_string(this->replans_max_size);
}

void OnlineReplanPolicy::eval_episode_info_update(EpisodeInfo episode_info) {
    this->replans_sum += this->replans_count;
    ++this->episodes_count;
}

void OnlineReplanPolicy::delete_replans() {
    /* Delete old replans */
    for (auto item: *this->replans) {
        for (auto nested_item: *item.second) {
            Policy *policy = nested_item.second;
            delete policy->env;
            delete policy;
        }

        delete item.second;
    }

//    cout << "---------------------------" << endl;

    delete this->replans;
}



