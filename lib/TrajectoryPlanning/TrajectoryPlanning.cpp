#include "TrajectoryPlanning.hpp"
#include "stdio.h"

namespace Trajectory
{

constexpr uint32_t as_u32(float value) {
    return static_cast<uint32_t>(value);
}

constexpr uint16_t as_u16(float value) {
    return static_cast<uint32_t>(value);
}

static_assert(as_u32(166.66666) == 166);

constexpr uint16_t Planner::precalculated_cycleupdate_steps[];

Block Planner::calculate_next_block()  {

    //printf("calculate_next_block %d\n(k_set: %d)\n", state, k_set);

    Block blk (k, 0);

    if (state == State::acceleration) {

        if (k > k_set) {
            while (blk.k == k && position.get_upper_cnt() < p_set && blk.cnts < 255) {
                if (dk > k) {
                    dk -= k;
                } else {
                    uint8_t r = k - dk;
                    k--;

                    while (r > precalculated_cycleupdate_steps[k]) {
                        r -= precalculated_cycleupdate_steps[k];
                        k--;
                    }

                    dk = precalculated_cycleupdate_steps[k] - r;
                }

                blk.cnts++;
                position.lower_increment();
            }
        }

        if (blk.k == k_set) {
            steady_end_position = Position(p_set, 0) - position;
            k = k_set;
            state = State::steady_state;

            /*printf("Going into steady with k=%d, steady_end %d.%d (pos: %d.%d)\n",
                k,
                steady_end_position.get_upper_cnt(), steady_end_position.get_lower_cnt(),
                position.get_upper_cnt(), position.get_lower_cnt()
            );*/
        }
    }

    // Steady State

    if (state == State::steady_state) {

        while (blk.k == k_set && blk.cnts < 255 && position < steady_end_position) {
            blk.cnts++;
            position.lower_increment();
        }

        if (position >= steady_end_position) {
            //printf("going further ..deacceleration, %d.%d\n", position.get_upper_cnt(), position.get_lower_cnt());
            dk = r+1; // TODO: get rid of this number
            state = State::deceleration;
        }

    }

    // Deacceleration

    if (state == State::deceleration) {
        if (k < 255) {
            while (blk.k == k && position <= Position(p_set, 0) && blk.cnts < 255) {
                if (dk > k) {
                    dk -= k;
                } else {
                    r = k - dk;
                    k++;

                    while (r > precalculated_cycleupdate_steps[k]) {
                        r -= precalculated_cycleupdate_steps[k];
                        k++;
                    }

                    dk = precalculated_cycleupdate_steps[k] - r;
                }

                //printf("deacceleration %d, %d\n", k, dk);

                blk.cnts++;
                position.lower_increment();
            }
        }

        if (position >= Position(p_set, 0)) {

            //printf("State::done :)");
            state = State::done;

            k = 0;
        }
    }


    //printf("block: %d, %d (k_new=%d, dk=%d)\n", blk.k, blk.cnts, k, dk);
    return blk;

}

}
