//
// Created by levyvonet on 26/10/2021.
//

#include <gtest/gtest.h>

#include <utils/parsing/parsing_utils.h>


TEST(UtilsTests, EmptyGridCorrect) {
    MultiAgentState start({});
    MultiAgentState goal({});

    /* Create the file path */
    std::string file_path = __FILE__;
    std::string dir_path = file_path.substr(0, file_path.find_last_of("/\\"));
    std::ostringstream scen_file_path;
    scen_file_path << dir_path << "/../maps/" << "empty-8-8/empty-8-8-even-1.scen";

    /* Parse */
    parse_scen_file(scen_file_path.str(), 4, &start, &goal);

    ASSERT_EQ(start, MultiAgentState(
            {Location(0, 0, DONT_CARE_ID), Location(5, 3, DONT_CARE_ID), Location(1, 7, DONT_CARE_ID),
             Location(0, 5, DONT_CARE_ID)}));
    ASSERT_EQ(goal, MultiAgentState(
            {Location(1, 0, DONT_CARE_ID), Location(5, 6, DONT_CARE_ID), Location(6, 4, DONT_CARE_ID),
             Location(7, 4, DONT_CARE_ID)}));
}

TEST(UtilsTests, CreateMapfEnv) {
    MapfEnv *empty_8_8_1 = create_mapf_env("empty-8-8", 1, 2, 0.2, -1000, 100, -1);

    ASSERT_EQ(*empty_8_8_1->s,
              MultiAgentState({empty_8_8_1->grid_ptr->get_location(0, 0), empty_8_8_1->grid_ptr->get_location(5, 3)}));

    MapfEnv *empty_48_48_16 = create_mapf_env("empty-48-48", 16, 2, 0.2, -1000, 100, -1);

    ASSERT_EQ(*empty_48_48_16->s, MultiAgentState(
            { empty_48_48_16->grid_ptr->get_location(40, 42), empty_48_48_16->grid_ptr->get_location(17, 2) }));
}