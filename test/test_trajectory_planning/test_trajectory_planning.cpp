#include <unity.h>
#include <stdio.h>
#include <TrajectoryPlanning.hpp>

using namespace std;
using namespace Trajectory;

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_planning(void) {

    static constexpr uint16_t position_error = 15;
    static constexpr uint16_t velocity_setpoint = 10;

    static constexpr Block expected_movement_plan[] = {
        {255, 1}, {202, 1}, {173, 1}, {154, 1}, {141, 1}, {130, 1}, {122, 1}, {115, 1}, {109, 1}, {104, 1}, {100, 1}, {96, 1}, {92, 1}, {89, 1}, {87, 1}, {84, 1}, {82, 1}, {79, 1}, {78, 1}, {76, 1}, {74, 1}, {72, 1}, {71, 1}, {69, 1}, {68, 1}, {67, 1}, {66, 1}, {65, 1}, {64, 1}, {63, 1}, {62, 1}, {61, 1}, {60, 1}, {59, 1}, {58, 1}, {57, 2}, {56, 1}, {55, 1}, {54, 2}, {53, 2}, {52, 1}, {51, 2}, {50, 2}, {49, 2}, {48, 3}, {47, 2}, {46, 3}, {45, 2}, {44, 3}, {43, 3}, {42, 4}, {41, 3}, {40, 4}, {39, 4}, {38, 5}, {37, 5}, {36, 5}, {35, 6}, {34, 7}, {33, 7}, {32, 7}, {31, 9}, {30, 9}, {29, 10}, {28, 12}, {27, 13}, {26, 14}, {25, 17}, {24, 18}, {23, 21}, {22, 24}, {21, 27}, {20, 33}, {19, 37}, {18, 44}, {17, 53}, {16, 64}, {15, 77}, {14, 96}, {13, 120}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 255}, {12, 248}, {13, 120}, {14, 96}, {15, 77}, {16, 64}, {17, 53}, {18, 44}, {19, 38}, {20, 32}, {21, 28}, {22, 24}, {23, 21}, {24, 18}, {25, 16}, {26, 15}, {27, 13}, {28, 11}, {29, 11}, {30, 9}, {31, 9}, {32, 7}, {33, 7}, {34, 7}, {35, 6}, {36, 5}, {37, 5}, {38, 5}, {39, 4}, {40, 4}, {41, 3}, {42, 4}, {43, 3}, {44, 3}, {45, 3}, {46, 2}, {47, 3}, {48, 2}, {49, 2}, {50, 2}, {51, 2}, {52, 2}, {53, 1}, {54, 2}, {55, 2}, {56, 1}, {57, 1}, {58, 2}, {59, 1}, {60, 1}, {61, 1}, {62, 1}, {63, 1}, {64, 1}, {65, 1}, {66, 1}, {67, 1}, {68, 1}, {70, 1}, {71, 1}, {73, 1}, {74, 1}, {76, 1}, {78, 1}, {80, 1}, {82, 1}, {84, 1}, {87, 1}, {89, 1}, {92, 1}, {96, 1}, {99, 1}, {103, 1}, {108, 1}, {113, 1}, {120, 1}, {127, 1}, {136, 1}, {147, 1}, {161, 1}, {180, 1}
    };

    Planner planner = Planner(
        position_error,
        velocity_setpoint
    );

    for (const auto &elem : expected_movement_plan) {
        const auto block = planner.calculate_next_block();

        TEST_ASSERT_EQUAL(elem.k, block.k);
        TEST_ASSERT_EQUAL(elem.cnts, block.cnts);
    }

    TEST_ASSERT(planner.is_done());

}

void test_calculate_cycleupdate_steps(void) {

    static constexpr auto &arr = Planner::precalculated_cycleupdate_steps;

    TEST_ASSERT_EQUAL(4, arr[255]);
    TEST_ASSERT_EQUAL(6, arr[200]);
    TEST_ASSERT_EQUAL(11, arr[150]);
    TEST_ASSERT_EQUAL(19, arr[113]);
}

int main( int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_planning);
    RUN_TEST(test_calculate_cycleupdate_steps);
    UNITY_END();
}
