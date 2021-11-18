//
// Created by levyvonet on 17/11/2021.
//

#include <gtest/gtest.h>

#include <gym_mapf.h>


TEST(StateStorageTests, state2int_1_agent) {
    MultiAgentStateStorage<int *> *v = new MultiAgentStateStorage<int *>(1, nullptr);


    MultiAgentState s = MultiAgentState(
            {
                    Location(0, 0, 0),
            },
            0);

    int *result = nullptr;
    result = v->get(s);
    ASSERT_EQ(nullptr, result);

    int *new_value = new int;
    *new_value = 2;
    v->set(s, new_value);

    result = v->get(s);

    ASSERT_EQ(result, new_value);
    ASSERT_EQ(*result, 2);

    new_value = new int;
    *new_value = 3;
    v->set(s, new_value);

    result = v->get(s);

    ASSERT_EQ(result, new_value);
    ASSERT_EQ(*result, 3);

    delete v;

}

TEST(StateStorageTests, state2int_2_agents) {
    MultiAgentStateStorage<int *> *v = new MultiAgentStateStorage<int *>(2, nullptr);

    MultiAgentState s = MultiAgentState(
            {
                    Location(0, 0, 0),
                    Location(0, 0, 0),
            },
            0);

    int *result = nullptr;
    result = v->get(s);
    ASSERT_EQ(nullptr, result);

    int *new_value = new int;
    *new_value = 2;
    v->set(s, new_value);

    result = v->get(s);

    ASSERT_EQ(result, new_value);
    ASSERT_EQ(*result, 2);

    new_value = new int;
    *new_value = 3;
    v->set(s, new_value);

    result = v->get(s);

    ASSERT_EQ(result, new_value);
    ASSERT_EQ(*result, 3);

    delete v;
}

TEST(StateStorageTests, state2int_4_agents) {
    MultiAgentStateStorage<int *> *v = new MultiAgentStateStorage<int *>(4, nullptr);


    MultiAgentState s = MultiAgentState(
            {
                    Location(0, 0, 0),
                    Location(0, 0, 0),
                    Location(0, 0, 0),
                    Location(0, 0, 0)
            },
            0);

    int *result = nullptr;
    result = v->get(s);
    ASSERT_EQ(nullptr, result);

    int *new_value = new int;
    *new_value = 2;
    v->set(s, new_value);

    result = v->get(s);

    ASSERT_EQ(result, new_value);
    ASSERT_EQ(*result, 2);

    new_value = new int;
    *new_value = 3;
    v->set(s, new_value);

    result = v->get(s);

    ASSERT_EQ(result, new_value);
    ASSERT_EQ(*result, 3);

    delete v;
}

TEST(StateStorageTests, storage_4_agents_comlex_data) {
    MultiAgentStateStorage<ActionToTransitionStorage *> *v = new MultiAgentStateStorage<ActionToTransitionStorage *>(4,
                                                                                                                     nullptr);


    MultiAgentState s = MultiAgentState(
            {
                    Location(1, 2, 0),
                    Location(3, 4, 1),
                    Location(5, 6, 2),
                    Location(7, 8, 3)
            },
            0);

    ActionToTransitionStorage *result = nullptr;
    result = v->get(s);
    ASSERT_EQ(nullptr, result);

    MultiAgentAction all_stay = MultiAgentAction({STAY, STAY}, 0);
    ActionToTransitionStorage *new_value = new ActionToTransitionStorage();
    TransitionsList *transitions = new TransitionsList();
    transitions->transitions->push_back(new Transition(1, new MultiAgentState(s.locations, s.id), 0,
                                                       false, false));
    (*new_value->m)[all_stay] = transitions;
    v->set(s, new_value);

    result = v->get(s);

    for (Transition *t: *(*result->m)[all_stay]->transitions) {
        ASSERT_EQ(t->p, 1);
        ASSERT_EQ(*t->next_state, MultiAgentState(s.locations, s.id));
        ASSERT_EQ(t->done, false);
        ASSERT_EQ(t->is_collision, false);
    }

    delete v;
}

TEST(StateStorageTests, storage_4_agents_action) {
    MultiAgentStateStorage<MultiAgentAction *> *v = new MultiAgentStateStorage<MultiAgentAction *>(4,
                                                                                                   nullptr);


    MultiAgentState s = MultiAgentState(
            {
                    Location(1, 2, 0),
                    Location(3, 4, 1),
                    Location(5, 6, 2),
                    Location(7, 8, 3)
            },
            0);

    MultiAgentAction *result = nullptr;
    result = v->get(s);
    ASSERT_EQ(nullptr, result);

    v->set(s, new MultiAgentAction({UP, RIGHT, LEFT, DOWN}, 555));

    result = v->get(s);


    ASSERT_EQ(result->actions[0], UP);
    ASSERT_EQ(result->actions[1], RIGHT);
    ASSERT_EQ(result->actions[2], LEFT);
    ASSERT_EQ(result->actions[3], DOWN);
    ASSERT_EQ(result->id, 555);

    delete v;
}