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

/** Constants ********************************************************************/
#define BONUS_VALUE (100)
#define LIVE_LOCK_THRESHOLD (100)

template<>
class std::hash<vector<size_t>> {
public:
    size_t operator()(const vector<size_t> &v) const;
};

typedef Policy *(*window_planner)(MapfEnv *, Dictionary *, float gamma, double timeout_ms);

Policy *window_planner_vi(MapfEnv *env, Dictionary *girth_values, float gamma, double timeout_ms);

Policy *window_planner_vi_deterministic_relaxation(MapfEnv *env, Dictionary *girth_values,float gamma,  double timeout_ms);

class WindowPolicy {
public:
    Policy *policy;
    GridArea area;
    int steps_count;
    int d;

    WindowPolicy(Policy *policy, GridArea area, int d);

    bool might_live_lock();
};

class OnlineReplanPolicy : public Policy {
private:
    vector<vector<size_t>> prev_groups;

    MultiAgentAction *select_action_for_group(vector<size_t> group, const MultiAgentState &s, double timeout_ms);

    vector<vector<size_t>> divide_to_groups(const MultiAgentState &s);

    Policy *plan_window(vector<size_t> group, GridArea window_area, const MultiAgentState &s, double timeout_ms);

    WindowPolicy *search_window_policy(const vector<size_t> &group, const MultiAgentState &s);

    WindowPolicy *replan(const vector<size_t> &group, const MultiAgentState &s, double timeout_ms);

    void extend_window(vector<size_t> group, WindowPolicy *window_policy, const MultiAgentState &s, double timeout_ms);

    int calc_distance(const Location &l1, const Location &l2);

    void delete_replans();

    tsl::hopscotch_map<vector<size_t>, vector<WindowPolicy *> *> *replans;

public:
    SolverCreator *low_level_planner_creator;
    window_planner window_planner_func;
    int d;
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
                       int d,
                       window_planner window_planner_func);

    virtual ~OnlineReplanPolicy();

    virtual void train(double timeout_ms) override;

    void reset() override;

    virtual MultiAgentAction *act(const MultiAgentState &state, double timeout_ms) override;

    virtual void eval_episodes_info_process(EvaluationInfo *eval_info) override;

    virtual void eval_episode_info_update(episode_info *episode_info) override;
};


#endif //GYM_MAPF_ONLINE_REPLAN_H
