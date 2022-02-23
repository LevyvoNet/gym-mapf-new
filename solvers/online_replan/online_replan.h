//
// Created by levyvonet on 28/11/2021.
//

#ifndef GYM_MAPF_ONLINE_REPLAN_H
#define GYM_MAPF_ONLINE_REPLAN_H

#include <set>
#include <stack>
#include <cmath>

#include <tsl/hopscotch_set.h>

#include <gym_mapf/grid/grid.h>
#include <solvers/utils/policy/policy.h>
#include <solvers/utils/policy/crossed_policy.h>
#include <solvers/utils/utils.h>
#include <solvers/value_iteartion/value_iteration.h>
#include <solvers/heuristics/solution_sum_heuristic.h>

/** Internal ********************************************************************/
class GridArea; /* Forward Declaration */
GridArea construct_conflict_area(Grid *grid, const vector<size_t> &group, const MultiAgentState &s);


template<>
class std::hash<vector<size_t>> {
public:
    size_t operator()(const vector<size_t> &v) const;
};

class Window {
    vector<size_t> agents;
    GridArea area;
};


class OnlineReplanPolicy : public Policy {
private:
    vector<vector<size_t>> prev_groups;

    MultiAgentAction *select_action_for_group(vector<size_t> group, const MultiAgentState &s, double timeout_ms);

    vector<vector<size_t>> divide_to_groups(const MultiAgentState &s);

    Policy *search_replan(const vector<size_t> &group, const MultiAgentState &s);

    Policy *replan(const vector<size_t> &group, const MultiAgentState &s, double timeout_ms);

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
    size_t replans_max_size;
    size_t replans_max_size_episode;

    OnlineReplanPolicy(MapfEnv *env,
                       float gamma,
                       const string &name,
                       SolverCreator *low_level_planner_creator,
                       int k);

    virtual ~OnlineReplanPolicy();

    virtual void train(double timeout_ms) override;

    void reset() override;

    virtual MultiAgentAction *act(const MultiAgentState &state, double timeout_ms) override;

    virtual void eval_episodes_info_process(EvaluationInfo* eval_info) override;

    virtual void eval_episode_info_update(episode_info *episode_info) override;
};


#endif //GYM_MAPF_ONLINE_REPLAN_H
