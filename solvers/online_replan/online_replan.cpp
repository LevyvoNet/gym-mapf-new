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


size_t hash<GridArea>::operator()(const GridArea &area) const {
    size_t h = 0;
    h += pow(PRIMES[1], area.top_row);
    h += pow(PRIMES[2], area.bottom_row);
    h += pow(PRIMES[3], area.left_col);
    h += pow(PRIMES[4], area.right_col);
    return h;
}

bool GridArea::contains(const Location &l) {
    return ((l.row <= this->bottom_row) &&
            (l.row >= this->top_row) &&
            (l.col <= this->right_col) &&
            (l.col >= this->left_col));
}

GridArea::GridArea(int top_row, int bottom_row, int left_col, int right_col) :
        top_row(top_row), bottom_row(bottom_row), left_col(left_col), right_col(right_col) {}

bool GridArea::operator==(const GridArea &other) const {
    return ((this->top_row == other.top_row) &&
            (this->bottom_row == other.bottom_row) &&
            (this->left_col == other.left_col) &&
            (this->right_col == other.right_col));
}

/** Area state iterator ******************************************************************************************/
#define END_OF_AREA_ID (-222)
#define IS_END_OF_AREA(l) (l.id == END_OF_AREA_ID)

AreaMultiAgentStateIterator::AreaMultiAgentStateIterator(const Grid *grid,
                                                         GridArea area,
                                                         size_t n_agents) :
        MultiAgentStateIterator(grid, n_agents), area(area) {
    this->_reach_begin();
}

Location inc_area_iterator(const Location *l, GridArea area, const Grid *grid) {
    int row = l->row;
    int col = l->col;

    do {
        ++col;
        if (col > area.right_col) {
            col = area.left_col;
            ++row;
        }
    } while (row <= area.bottom_row && grid->loc_to_id[row][col] == ILLEGAL_LOCATION);

    /* We have reached the end */
    if (row > area.bottom_row) {
        return Location(area.top_row, area.left_col, END_OF_AREA_ID);
    }

    return grid->get_location(row, col);
}

Location area_first_legal_location(const Grid *grid, GridArea area) {
    int first_location_row = area.top_row;
    int first_location_col = area.left_col;

    if (grid->loc_to_id[first_location_row][first_location_col] == ILLEGAL_LOCATION) {
        Location temp_loc = Location(first_location_row, first_location_col, -1);
        return inc_area_iterator(&temp_loc, area, grid);
    } else {

        return grid->get_location(first_location_row, first_location_col);
    }
}

void AreaMultiAgentStateIterator::_reach_begin() {
    /* Start from the first legal location of the area */
    Location first_location = area_first_legal_location(this->grid, this->area);

    vector<Location> locations;
    for (size_t i = 0; i < this->n_agents; ++i) {
        locations.push_back(first_location);
        this->iters[i] = GridIterator(this->grid, first_location.id);
    }
    this->set_locations(locations);
}

void AreaMultiAgentStateIterator::reach_begin() {
    this->_reach_begin();
}

MultiAgentStateIterator &AreaMultiAgentStateIterator::operator++() {
    size_t agent_idx = 0;
    bool carry = false;

    /* Increment the first agent, then handle the "carry" */
    do {
        Location new_location = inc_area_iterator(this->iters[agent_idx].ptr, this->area, this->grid);
        if IS_END_OF_AREA(new_location) {
            Location first_location = area_first_legal_location(this->grid, this->area);
            new_location = first_location;
            carry = true;
        } else {
            carry = false;
        }
        this->iters[agent_idx] = GridIterator(this->grid, new_location.id);

        this->ptr->locations[agent_idx] = *(this->iters[agent_idx]);
        agent_idx++;

    } while ((agent_idx < this->n_agents) && carry);

    /* Check if we are out because of reaching the last state. If so, return the end */
    if (agent_idx == this->n_agents && carry) {
        this->reach_end();
    }

    this->set_locations(this->ptr->locations);

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

/** Girth state iterator *****************************************************************************************/

Location inc_grid_iterator_by_girth_aux(const Location *l, GridArea area, const Grid *grid) {
    int orig_row = l->row;
    int orig_col = l->col;

    Location loc = *l;

    do {
        /* top row, move right */
        if (loc.row == area.top_row - 1 && loc.col < area.right_col + 1) {
            ++loc.col;
            continue;
        }

        /* top row right corner, move down */
        if (loc.row == area.top_row - 1 && loc.col == area.right_col + 1) {
            ++loc.row;
            continue;
        }

        /* right col, move down */
        if (loc.col == area.right_col + 1 && loc.row < area.bottom_row + 1) {
            ++loc.row;
            continue;
        }

        /* right col bottom corner, move left */
        if (loc.col == area.right_col + 1 && loc.row == area.bottom_row + 1) {
            --loc.col;
            continue;
        }

        /* bottom row, move left */
        if (loc.row == area.bottom_row + 1 && loc.col > area.left_col - 1) {
            --loc.col;
            continue;
        }

        /* bottom row left corner, move up */
        if (loc.row == area.bottom_row + 1 && loc.col == area.left_col - 1) {
            --loc.row;
            continue;
        }

        /* left col, move up */
        if (loc.col == area.left_col - 1 && loc.row > area.top_row - 1) {
            --loc.row;
            continue;
        }
    } while (!grid->is_legal(loc) && !(loc.row == orig_row && loc.col == orig_col));

    if (loc.row == orig_row && loc.col == orig_col && !grid->is_legal(loc)) {
        return Location(area.top_row - 1, area.left_col - 1, -1);
    }

    return grid->get_location(loc.row, loc.col);
}

Location girth_first_legal_location(const Grid *grid, GridArea area) {
    Location first_location = Location(area.top_row - 1, area.left_col - 1, -1);
    if (area.top_row - 1 < 0 || area.left_col - 1 < 0 || area.top_row - 1 > grid->max_row ||
        area.left_col - 1 > grid->max_col) {
        first_location = inc_grid_iterator_by_girth_aux(&first_location, area, grid);
    }

    return first_location;

}

void GirthMultiAgentStateIterator::_reach_begin() {
    Location first_location = girth_first_legal_location(this->grid, this->area);

    if ((first_location.row == this->area.top_row - 1) &&
        (first_location.col == this->area.left_col - 1) &&
        (!this->grid->is_legal(first_location))) {
        this->reach_end();
        return;
    }

    /* Set the ID for the first location properly */
    first_location = this->grid->get_location(first_location.row, first_location.col);

    vector<Location> locations;
    for (size_t i = 0; i < this->n_agents; ++i) {
        locations.push_back(first_location);
        this->iters[i] = GridIterator(this->grid, first_location.id);
    }
    this->set_locations(locations);
}

GirthMultiAgentStateIterator::GirthMultiAgentStateIterator(const Grid *grid, GridArea area, size_t n_agents) :
        MultiAgentStateIterator(grid, n_agents), area(area) {
    this->_reach_begin();
}

void GirthMultiAgentStateIterator::reach_begin() {
    this->_reach_begin();
}

MultiAgentStateIterator &GirthMultiAgentStateIterator::operator++() {
    size_t agent_idx = 0;
    bool carry = false;
    Location first_location = girth_first_legal_location(this->grid, this->area);
    GridIterator end = GridIterator(this->grid, &first_location);
    /* Increment the first agent, then handle the "carry" */

    do {
        Location new_location = inc_grid_iterator_by_girth_aux(this->iters[agent_idx].ptr, this->area, this->grid);
        this->iters[agent_idx] = GridIterator(this->grid, new_location.id);
        if (this->iters[agent_idx] == end) {
            carry = true;
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

    this->set_locations(this->ptr->locations);

    return *this;
}

GirthMultiAgentStateSpace::GirthMultiAgentStateSpace(const Grid *grid, GridArea area, size_t n_agents) :
        MultiAgentStateSpace(grid, n_agents), area(area) {

}

GirthMultiAgentStateIterator *GirthMultiAgentStateSpace::begin() {
    return new GirthMultiAgentStateIterator(this->grid, this->area, this->n_agents);
}

GirthMultiAgentStateIterator *GirthMultiAgentStateSpace::end() {
    GirthMultiAgentStateIterator *iter = new GirthMultiAgentStateIterator(this->grid, this->area, this->n_agents);
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
    for (size_t agent:group) {
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

tsl::hopscotch_set<Location> get_intended_locations(Policy *p, Location start, int k) {
    tsl::hopscotch_set<Location> res;
    Location curr_location = start;
    for (size_t i = 1; i <= k; ++i) {
        MultiAgentState *curr_state = p->env->locations_to_state({curr_location});
        MultiAgentAction *a = p->act(*curr_state);
        curr_location = p->env->grid->execute(curr_location, a->actions[0]);
        delete a;
        res.insert(curr_location);
    }


    return res;
}

Policy *OnlineReplanPolicy::replan(const vector<size_t> &group, const MultiAgentState &s) {
    /* Calculate the conflict area */
    GridArea conflict_area = construct_conflict_area(this->env->grid, group, s);

    /* Pad the area TODO: is it good? */
    conflict_area = pad_area(this->env->grid, conflict_area, this->k);

    /* Generate state space from the conflict area */
    AreaMultiAgentStateSpace *conflict_area_state_space = new AreaMultiAgentStateSpace(this->env->grid, conflict_area,
                                                                                       group.size());

    /* Create an environment and set its state space to be our sub-space */
    MapfEnv *area_env = get_local_view(this->env, group);
    area_env->observation_space = conflict_area_state_space;

    /* Set the girth of the area with a fixed value composed of the single agents values */
    vector<ValueFunctionPolicy *> policies;
    vector<vector<size_t>> agents_groups;
    vector<tsl::hopscotch_set<Location>> intended_locations;
    for (size_t agent:group) {
        Policy *agent_policy = this->local_policy->policies[agent];
        policies.push_back((ValueFunctionPolicy *) agent_policy);
        agents_groups.push_back({group[agent]});
        intended_locations.push_back(get_intended_locations(agent_policy, s.locations[agent], this->k + 1));
    }
    SolutionSumHeuristic *h = new SolutionSumHeuristic(policies, agents_groups);
    h->init(this->env);

    /* Only a single one outside the area */
    GirthMultiAgentStateSpace *girth_space_single = new GirthMultiAgentStateSpace(this->env->grid, conflict_area, 1);
    GirthMultiAgentStateIterator *girth_space_single_end = girth_space_single->end();
    AreaMultiAgentStateIterator *area_iter = conflict_area_state_space->begin();
    AreaMultiAgentStateIterator *area_end = conflict_area_state_space->end();
    Dictionary* girth_values = new Dictionary(0);
    for (; *area_iter != *area_end; ++*area_iter) {
        for (size_t agent_idx=0;agent_idx<group.size();++agent_idx) {
            GirthMultiAgentStateIterator *girth_iter = girth_space_single->begin();
            for (; *girth_iter != *girth_space_single_end; ++(*girth_iter)) {
                MultiAgentState temp_state = **area_iter;
                temp_state.locations[agent_idx] = girth_iter->ptr->locations[0];
                temp_state.id = this->env->grid->calculate_multi_locations_id(temp_state.locations);

                double value = (*h)(&temp_state);

                if (intended_locations[agent_idx].find(temp_state.locations[agent_idx]) != intended_locations[agent_idx].end()) {
                    value += BONUS_VALUE;
                }

                girth_values->set(temp_state.id, value);
            }
        }
    }



    /* Solve the env by value iteration */
    ValueIterationPolicy *policy = new ValueIterationPolicy(area_env, this->gamma, "", girth_values);
    policy->train();
    delete girth_values;

    /* Save the new policy in replans cache */
    if (!this->replans->contains(group)) {
        (*this->replans)[group] = new tsl::hopscotch_map<GridArea, Policy *>();
    }
    (*(*this->replans)[group])[conflict_area] = policy;
    ++this->replans_count;
    this->replans_max_size = max(group.size(), this->replans_max_size);

//    cout << "replanned for group sized " << group.size() << " and conflict starts at " << conflict_area.top_row << ","
//         << conflict_area.left_col << " ends at " << conflict_area.bottom_row << "," << conflict_area.right_col << ". ";
//    cout << "locations are" << s.locations[0] << ", " << s.locations[1] << endl;

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
        k(k), low_level_planner_creator(low_level_planner_creator), local_policy(nullptr),
        replans_count(0), replans_sum(0), episodes_count(0), replans_max_size(0) {
    this->replans = new tsl::hopscotch_map<vector<size_t>, tsl::hopscotch_map<GridArea, Policy *> *>();
}

OnlineReplanPolicy::~OnlineReplanPolicy() {
    delete this->local_policy;
    this->delete_replans();
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
    this->delete_replans();

    this->replans = new tsl::hopscotch_map<vector<size_t>, tsl::hopscotch_map<GridArea, Policy *> *>();

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

void OnlineReplanPolicy::eval_episodes_info_process(EvaluationInfo *eval_info) {
    float replans_mean = float(this->replans_sum) / this->episodes_count;
    (*eval_info->additional_data)["replans_mean"] = std::to_string(replans_mean);
    (*eval_info->additional_data)["replans_max_size"] = std::to_string(this->replans_max_size);
}

void OnlineReplanPolicy::eval_episode_info_update() {
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

    delete this->replans;
}



