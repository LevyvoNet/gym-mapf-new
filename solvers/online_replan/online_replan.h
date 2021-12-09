//
// Created by levyvonet on 28/11/2021.
//

#ifndef GYM_MAPF_ONLINE_REPLAN_H
#define GYM_MAPF_ONLINE_REPLAN_H

#include <gym_mapf/grid/grid.h>
#include <solvers/utils/policy/policy.h>
#include <solvers/utils/policy/crossed_policy.h>
#include <solvers/utils/utils.h>
#include <solvers/value_iteartion/value_iteration.h>
#include <solvers/heuristics/solution_sum_heuristic.h>
#include <stack>
#include <cmath>

/** Internal ********************************************************************/
class GridArea; /* Forward Declaration */
GridArea construct_conflict_area(Grid *grid, const vector<size_t> &group, const MultiAgentState &s);


template<>
class std::hash<vector<size_t>> {
public:
    size_t operator()(const vector<size_t> &v) const;
};

class GridArea {
public:
    int top_row;
    int bottom_row;
    int left_col;
    int right_col;

    GridArea(int top_row, int bottom_row, int left_col, int right_col);

    bool contains(const Location &l);

    bool operator==(const GridArea &other) const;
};

template<>
class std::hash<GridArea> {
public:
    size_t operator()(const GridArea &area) const;
};

class AreaMultiAgentStateIterator : public MultiAgentStateIterator {
public:
    GridArea area;

    void _reach_begin();

    AreaMultiAgentStateIterator(const Grid *grid, GridArea area, size_t n_agents);

    virtual void reach_begin() override;

    virtual MultiAgentStateIterator &operator++() override;
};

class AreaMultiAgentStateSpace : public MultiAgentStateSpace {
protected:
    GridArea area;
public:
    AreaMultiAgentStateSpace(const Grid *grid, GridArea area, size_t n_agents);

    virtual AreaMultiAgentStateIterator *begin() override;

    virtual AreaMultiAgentStateIterator *end() override;
};


class GirthMultiAgentStateIterator : public MultiAgentStateIterator {

public:
    GridArea area;

    GirthMultiAgentStateIterator(const Grid *grid, GridArea area, size_t n_agents);

    virtual MultiAgentStateIterator &operator++() override;

    void _reach_begin();

    virtual void reach_begin() override;
};

class GirthMultiAgentStateSpace : public MultiAgentStateSpace {
protected:
    GridArea area;

public:
    GirthMultiAgentStateSpace(const Grid *grid, GridArea area, size_t n_agents);

    virtual GirthMultiAgentStateIterator *begin() override;

    virtual GirthMultiAgentStateIterator *end() override;
};

class OnlineReplanPolicy : public Policy {
private:
    MultiAgentAction *select_action_for_group(vector<size_t> group, const MultiAgentState &s);

    vector<vector<size_t>> divide_to_groups(const MultiAgentState &s);

    Policy *search_replan(const vector<size_t> &group, const MultiAgentState &s);

    Policy *replan(const vector<size_t> &group, const MultiAgentState &s);

    int calc_distance(const Location &l1, const Location &l2);

    void delete_replans();

    tsl::hopscotch_map<vector<size_t>, tsl::hopscotch_map<GridArea, Policy *> *> *replans;

public:
    SolverCreator *low_level_planner_creator;
    int k;
    CrossedPolicy *local_policy;
    int replans_count;
    int episodes_count;
    int replans_sum;

    OnlineReplanPolicy(MapfEnv *env,
                       float gamma,
                       const string &name,
                       SolverCreator *low_level_planner_creator,
                       int k);

    virtual ~OnlineReplanPolicy();

    virtual void train() override;

    void reset() override;

    virtual MultiAgentAction *act(const MultiAgentState &state) override;

    virtual void eval_episodes_info_process(EvaluationInfo* eval_info) override;

    virtual void eval_episode_info_update() override;
};


#endif //GYM_MAPF_ONLINE_REPLAN_H
