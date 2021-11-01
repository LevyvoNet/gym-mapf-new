//
// Created by levyvonet on 26/10/2021.
//

#include <gtest/gtest.h>

#include <utils/parsing/parsing_utils.h>


TEST(UtilsTests, CreateMapfEnv) {
    MapfEnv *empty_8_8_1 = create_mapf_env("empty-8-8", 1, 2, 0.2, -1000, 100, -1);

    ASSERT_EQ(*empty_8_8_1->s,
              *empty_8_8_1->locations_to_state(
                      {empty_8_8_1->grid->get_location(0, 0), empty_8_8_1->grid->get_location(5, 3)}));

    MapfEnv *empty_48_48_16 = create_mapf_env("empty-48-48", 16, 2, 0.2, -1000, 100, -1);

    ASSERT_EQ(*empty_48_48_16->s,
              *empty_48_48_16->locations_to_state(
                      {empty_48_48_16->grid->get_location(40, 42), empty_48_48_16->grid->get_location(17, 2)}));
}