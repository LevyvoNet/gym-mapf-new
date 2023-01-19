//
// Created by levyvonet on 10/1/22.
//

#include "goal_predicator.h"

SingleLocationGoalPredicator::SingleLocationGoalPredicator(const Location &l) : l(l) {}

bool SingleLocationGoalPredicator::is_goal(const Location &goal_candidate) {
    return this->l == goal_candidate;
}

AreaGirthGoalPredicator::AreaGirthGoalPredicator(const Grid *grid, const GridArea &area) :
        grid_(grid), area_(area) {}

bool AreaGirthGoalPredicator::is_goal(const Location &l) {
    return l.row == area_.top_row - 1 ||
           l.row == area_.bottom_row + 1 ||
           l.row == area_.left_col - 1 ||
           l.row == area_.right_col + 1;
}