//
// Created by levyvonet on 10/1/22.
//

#ifndef GYM_MAPF_GOAL_PREDICATOR_H
#define GYM_MAPF_GOAL_PREDICATOR_H

#include <grid/grid.h>
#include <multiagent_state/multiagent_state.h>

class GoalPredicator {
public:
    virtual bool is_goal(const Location &goal_candidate) = 0;
};

class SingleLocationGoalPredicator : public GoalPredicator {
public:
    SingleLocationGoalPredicator(const Location &goal_candidate);

    virtual bool is_goal(const Location &l);

private:
    Location l;
};

class AreaGirthGoalPredicator : public GoalPredicator {
public:
    AreaGirthGoalPredicator(const Grid *grid, const GridArea &area);

    virtual bool is_goal(const Location &l);

private:
    const Grid* grid_;
    const GridArea area_;

};

#endif //GYM_MAPF_GOAL_PREDICATOR_H
