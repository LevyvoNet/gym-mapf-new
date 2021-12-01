//
// Created by levyvonet on 28/11/2021.
//

#include "online_replan.h"

/** Utilities ***************************************************************************************************/
vector<size_t> PRIMES = {2, 3, 5, 7, 9, 11, 13, 17, 19};

size_t hash<vector<size_t>>::operator()(const vector<size_t> &v) const {
    size_t h = 0;
    for (size_t i = 0; i < v.size(); ++i) {
        h += pow(v[i], PRIMES[i % PRIMES.size()]);
    }

    return h;
}

bool GridArea::contains(const Location &l) {
    return ((l.row <= this->bottom_row) &&
            (l.row >= this->top_row) &&
            (l.row <= this->right_col) &&
            (l.col >= this->left_col));
}

GridArea::GridArea(int top_row, int bottom_row, int left_col, int right_col) :
        top_row(top_row), bottom_row(bottom_row), left_col(left_col), right_col(right_col) {}


void AreaMultiAgentStateIterator::set_locations(vector<Location> locations) {
    int mul = 1;
    int sum = 0;
    int n_options = this->grid->id_to_loc.size();

    for (size_t i = 0; i < this->n_agents; ++i) {
        mul *= locations[i].id * mul;
    }

    this->ptr->locations = locations;
    this->ptr->id = sum;
}

AreaMultiAgentStateIterator::AreaMultiAgentStateIterator(const Grid *grid,
                                                         GridArea area,
                                                         size_t n_agents) :
        MultiAgentStateIterator(grid, n_agents), area(area) {
    /* Start from the top left corner of the area */
    Location top_left_location = this->grid->get_location(this->area.top_row,
                                                          this->area.left_col);
    vector<Location> locations;
    for (size_t i = 0; i < this->n_agents; ++i) {
        locations.push_back(top_left_location);
    }
    this->set_locations(locations);
}

void AreaMultiAgentStateIterator::reach_begin() {
    /* Start from the top left corner of the area */
    Location top_left_location = this->grid->get_location(this->area.top_row,
                                                          this->area.left_col);
    vector<Location> locations;
    for (size_t i = 0; i < this->n_agents; ++i) {
        locations.push_back(top_left_location);
    }
    this->set_locations(locations);
}

MultiAgentStateIterator &AreaMultiAgentStateIterator::operator++() {
    size_t agent_idx = 0;
    bool carry = false;
    bool carry_happened = false;

    /* Increment the first agent, then handle the "carry" */

    do {
        ++(this->iters[agent_idx]);
        if (!this->area.contains(*this->iters[agent_idx])) {
            *this->iters[agent_idx] = this->grid->get_location(this->area.top_row,
                                                               this->area.left_col);
            carry = true;
            carry_happened = true;
        } else {
            carry = false;
        }

        this->ptr->locations[agent_idx] = *(this->iters[agent_idx]);
        agent_idx++;

    } while ((agent_idx < this->n_agents) && carry);



    /* Check if we are out because of reaching the last state. If so, return the end */
    if (agent_idx == this->n_agents && carry) {
        this->reach_end();
    }

    if (!carry_happened) {
        this->ptr->id++;
    } else {
        this->set_locations(this->ptr->locations);
    }

    return *this;
}

AreaMultiAgentStateSpace::AreaMultiAgentStateSpace(const Grid *grid, GridArea area, size_t n_agents) :
        MultiAgentStateSpace(grid, n_agents), area(area) {}

AreaMultiAgentStateIterator *AreaMultiAgentStateSpace::begin() {
    return new AreaMultiAgentStateIterator(this->grid, this->area, this->n_agents);
}

AreaMultiAgentStateIterator *AreaMultiAgentStateSpace::end() {
    AreaMultiAgentStateIterator *iter = new AreaMultiAgentStateIterator(this->grid, this->area, this->n_agents);
    iter->reach_end();

    return iter;
}

/** Private methods *********************************************************************************************/
int OnlineReplanPolicy::calc_distance(const Location &l1, const Location &l2) {
    return abs(l1.row - l2.row) + abs(l1.col - l2.col);
}

vector<vector<size_t>> OnlineReplanPolicy::divide_to_groups(const MultiAgentState &s) {
    vector<vector<size_t>> direct_neighbours(this->env->n_agents);

    /* Calculate the direct neighbours */
    for (size_t i = 0; i < direct_neighbours.size(); ++i) {
        for (size_t j = 0; j < direct_neighbours.size(); ++j) {
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

Policy *OnlineReplanPolicy::replan(const vector<size_t> &group, const MultiAgentState &s) {



    /* Calculate the conflict area */
    int top_row = this->env->grid->max_row;
    int bottom_row = 0;
    int left_col = this->env->grid->max_col;
    int right_col = 0;
    for (size_t agent: group) {
        top_row = min(top_row, s.locations[agent].row);
        bottom_row = max(bottom_row, s.locations[agent].row);
        left_col = min(left_col, s.locations[agent].row);
        right_col = max(right_col, s.locations[agent].row);
    }
    GridArea conflict_area = GridArea(top_row, bottom_row, left_col, right_col);

    /* Generate state space from the conflict area */
    MultiAgentStateSpace *sub_state_space = new AreaMultiAgentStateSpace(this->env->grid, conflict_area, group.size());


    /* Create an environment and set its state space to be our sub-space */
    MapfEnv *area_env = get_local_view(this->env, group);
    area_env->observation_space = sub_state_space;

    /* Solve the environment using value iteration */
    ValueIterationPolicy *policy = new ValueIterationPolicy(area_env, this->gamma, "");
    policy->train();

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

MultiAgentAction *OnlineReplanPolicy::select_action_for_group(vector<size_t> group, const MultiAgentState &s) {
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
            policy = this->replan(group, s);
        }
    }

    /* Extract the action from the retrieved policy */
    for (size_t agent: group) {
        casted_locations.push_back(s.locations[agent]);
    }
    group_state = policy->env->locations_to_state(casted_locations);
    a = policy->act(*group_state);

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
        k(k), low_level_planner_creator(low_level_planner_creator), local_policy(nullptr), replans_count(0) {}

OnlineReplanPolicy::~OnlineReplanPolicy() {
    delete this->local_policy;
}

void OnlineReplanPolicy::train() {
    vector<vector<size_t>> groups(env->n_agents);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    /* Solve Independently for each agent */
    for (size_t i = 0; i < env->n_agents; ++i) {
        groups[i] = {i};
    }
    this->local_policy = solve_local_and_cross(this->env, this->gamma, this->low_level_planner_creator, &groups);

    auto elapsed_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - begin).count();
    float elapsed_time_seconds = float(elapsed_time_milliseconds) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;
}

void OnlineReplanPolicy::reset() {
    Policy::reset();

    this->replans_count = 0;
}

MultiAgentAction *OnlineReplanPolicy::act(const MultiAgentState &state) {
    vector<vector<size_t>> groups = this->divide_to_groups(state);
    vector<Action> selected_actions(this->env->n_agents);
    MultiAgentAction *group_action = nullptr;

    for (vector<size_t> group: groups) {
        group_action = this->select_action_for_group(group, state);
        for (size_t i = 0; i < group.size(); ++i) {
            selected_actions[group[i]] = group_action->actions[i];
        }
        delete group_action;
    }

    return actions_to_action(selected_actions);
}

void OnlineReplanPolicy::eval_episodes_info_process() {
    /* TODO: implement this */
    Policy::eval_episodes_info_process();
}

void OnlineReplanPolicy::eval_episode_info_update() {
    /* TODO: implement this */
    Policy::eval_episode_info_update();
}



