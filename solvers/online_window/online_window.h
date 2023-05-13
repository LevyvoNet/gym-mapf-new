//
// Created by levyvonet on 04/03/2022.
//

#ifndef GYM_MAPF_ONLINE_WINDOW_H
#define GYM_MAPF_ONLINE_WINDOW_H


#include <set>
#include <stack>
#include <cmath>

#include <tsl/hopscotch_set.h>

#include <gym_mapf/gym_mapf.h>
#include <solvers/utils/policy/policy.h>
#include <solvers/utils/policy/crossed_policy.h>
#include <solvers/utils/utils.h>
#include <solvers/value_iteartion/value_iteration.h>
#include <solvers/rtdp/rtdp.h>
#include <solvers/heuristics/any_goal_heurisitc.h>
#include <solvers/heuristics/rtdp_dijkstra_heuristic.h>


/** Constants *****************************************************************************************/
#define BONUS_VALUE (100)
#define LIVE_LOCK_BUFFER (100)

class AllStayExceptFirstActionSpaceIterator : public MultiAgentActionIterator {
public:

    AllStayExceptFirstActionSpaceIterator(size_t nAgents);

    AllStayExceptFirstActionSpaceIterator &operator++();
};

class AllStayExceptFirstActionSpace : public MultiAgentActionSpace {
public:
    AllStayExceptFirstActionSpace(size_t nAgents);

    MultiAgentActionIterator *begin();

    MultiAgentActionIterator *end();
};

typedef vector<size_t> AgentsGroup;

/** Private functions ***************************************************************************************/
tsl::hopscotch_set<Location> get_intended_locations(Policy *p, Location start, int k, double timeout_ms);

GridArea pad_area(Grid *grid, GridArea area, int k);

GridArea construct_conflict_area(Grid *grid, const AgentsGroup &group, const MultiAgentState &s);

/** Utility Types ********************************************************************************************/

class Window {
public:
    GridArea area;
    Policy *policy;
    AgentsGroup group;

    int steps_count;
    int max_steps;
    int reached_count;
    int expanded_count;

    Window(GridArea area, Policy *policy, AgentsGroup group, const MapfEnv* full_env);

    MultiAgentAction *act(const MultiAgentState &state, double timeout_ms);

    int calc_max_steps();

    MultiAgentState *cast_to_window_local_state(const MultiAgentState &s);

    friend std::ostream &operator<<(std::ostream &os, const Window &w);

private:
    const MapfEnv* full_env_;
};

typedef void (*window_planner)(MapfEnv *env, float gamma, Window *w, const MultiAgentState &s,
                               vector<Policy *> single_policies,
                               int d,
                               double timeout_ms);
//typedef Policy *(*window_planner)(MapfEnv *, Dictionary *, float gamma, double timeout_ms);

void window_planner_vi(MapfEnv *env, float gamma, Window *w, const MultiAgentState &s, vector<Policy *> single_policies,
                       int d,
                       double timeout_ms);

void window_planner_rtdp(MapfEnv *env, float gamma, Window *w, const MultiAgentState &s,
                         vector<Policy *> single_policies,
                         int d,
                         double timeout_ms);

void window_planner_rtdp_only_bonus(MapfEnv *env, float gamma, Window *w, const MultiAgentState &s,
                                    vector<Policy *> single_policies,
                                    int d,
                                    double timeout_ms);

/** Policy ********************************************************************************************************/
class OnlineWindowPolicy : public Policy {
private:
    /* Parameters */
    SolverCreator *low_level_planner_creator;
    window_planner window_planner_func;
    int d;

    /* Consts */
    vector<Window *> *singles_windows;

    /* State */
    vector<Window *> *curr_windows;
    vector<Window *> *archived_windows;

    /* Metadata for statistics */
    int replans_count;
    int episodes_count;
    int replans_sum;
    size_t replans_max_size;
    size_t replans_max_size_episode;
    int max_steps_in_window_episode;
    int max_times_window_reached_episode;
    int max_times_window_expanded_episode;
    int livelocks_count_episode;
    int max_agents_replan_area_episode;


    /* Private Methods */
    void clear_windows();

    void update_current_windows(const MultiAgentState &state, double timeout_ms);

    vector<Window *> destruct_window(Window *w, const MultiAgentState &state);

    bool merge_current_windows(const MultiAgentState &state);

    Window *merge_windows(Window *w1, Window *w2, const MultiAgentState &state);

    void plan_window(Window *pWindow, const MultiAgentState &s, double d);

    Window *try_fit_to_archive(AgentsGroup, const MultiAgentState &state);

    bool in_deadlock(Window *w, const MultiAgentState &state, double timeout_ms);

    bool might_live_lock(Window *w);

    void expand_window(Window *w, const MultiAgentState &state, double timeout_ms);


public:

    OnlineWindowPolicy(MapfEnv *env,
                       float gamma,
                       const string &name,
                       SolverCreator *low_level_planner_creator,
                       int d,
                       window_planner window_planner_func);


    virtual ~OnlineWindowPolicy();

    virtual void train(double timeout_ms) override;

    void reset() override;

    virtual void eval_episodes_info_process(EvaluationInfo *eval_info) override;

    virtual void eval_episode_info_update(episode_info *episode_info) override;

    virtual MultiAgentAction *act(const MultiAgentState &state, double timeout_ms) override;

};


#endif //GYM_MAPF_ONLINE_WINDOW_H
