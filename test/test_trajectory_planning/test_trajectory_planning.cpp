#include <vector>
#include <numeric>
#include <unity.h>
#include <stdio.h>
#include <TrajectoryPlanning.hpp>

using namespace std;

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_planning(void) {

    static constexpr uint16_t position_error = 15;
    static constexpr uint16_t velocity_setpoint = 10;
    static constexpr uint16_t acceleration_setpoint = 30;


    TrajectoryPlanner planner = TrajectoryPlanner(
        position_error,
        velocity_setpoint,
        acceleration_setpoint
    );

    auto movement_plan = vector<TrajectoryBlock>();

    while(planner.is_done() == false) {
        auto block = planner.calculate_next_block();

        movement_plan.push_back(block);
    }

    auto overall_cnts = accumulate(
        movement_plan.begin(),
        movement_plan.end(),
        0,
        [&](int acc, TrajectoryBlock &elem){
            return acc + elem.cnts;
        }
    );

    // TODO: improve accuracy
    TEST_ASSERT_INT_WITHIN_MESSAGE(1000, 180827, overall_cnts, "Way off in overall steps :(");
}

void test_calculate_cycleupdate_steps(void) {

    static constexpr auto arr = TrajectoryPlanner::calculate_cycleupdate_steps(16e6 / 256, 3.0 / 1600, 30);

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
