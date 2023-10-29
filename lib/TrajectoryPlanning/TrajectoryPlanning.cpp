#include "TrajectoryPlanning.hpp"
#include "stdio.h"

static constexpr auto initial_k = 256U;

static constexpr auto prescaler = 8U;

static constexpr uint16_t a_set = 30;

static constexpr float f_clk = 16000000.0f / prescaler;

static constexpr float m = 3.0f / (8.0f * 4500.0f);

constexpr uint32_t as_u32(float value) {
    return static_cast<uint32_t>(value);
}

constexpr uint16_t as_u16(float value) {
    return static_cast<uint32_t>(value);
}

static_assert(as_u32(166.66666) == 166);

static inline uint16_t calculate_initial_time_ms(uint16_t a_set) {
    return as_u32(1000 * f_clk * m) / ( a_set * initial_k);
}

enum class TrajectoryPhase{
    init,
    acceleration,
    stationary,
    decelleration
};

TrajectoryPlanner::TrajectoryPlanner(uint16_t position_error, uint16_t set_velocity, uint16_t set_acceleration)
    :p_set(position_error), v_set(set_velocity),  t_0(calculate_initial_time_ms(set_acceleration))
{
    state = State::acceleration;

    // TODO adapt v_set if triangular..
}

static constexpr uint16_t calculate_acceleration_k(uint16_t t_0, uint32_t k_sum) {
    return 55555L / (217L + 10L * k_sum / 2000L);
}

static_assert(calculate_acceleration_k(21, 256) == 254);

static uint16_t calculate_v(uint16_t k) {
    return as_u16(f_clk * m) / k;
}

static uint16_t calculate_p(uint32_t cnts) {
    return cnts * as_u32(1e6 * m) / 1000000;
}

static constexpr uint16_t calculate_deacceleration_k(uint16_t v_set, uint32_t k_sum_deacceleration) {
    return as_u16(100 * f_clk * m) / (as_u32(100 * v_set) - a_set * as_u32(1e7/ f_clk) * k_sum_deacceleration / as_u32(1e5));
}

static_assert(calculate_deacceleration_k(10, 256) == 16);
static_assert(calculate_deacceleration_k(10, 0) == 16);


TrajectoryBlock TrajectoryPlanner::calculate_next_block()  {

    printf("calculate_next_block %d\n", state);
    
    uint16_t k = 0;
    uint16_t cnt = 0;

    if (state == State::acceleration) {

        k = calculate_acceleration_k(t_0, k_sum);

        while (cnt < 255 && calculate_v(k) < v_set && calculate_p(cnt_sum+cnt) < (p_set / 2)) {
            cnt++;
            k_sum += k;
            
            // next iterations k
            if(calculate_acceleration_k(t_0, k_sum) != k) {
                // next block..
                break;
            }
        }

        if (calculate_v(k) >= v_set) {
            printf("Entering Steady\n");

            state = State::steady_state;

            p_entering_steady = calculate_p(cnt_sum);

        } else if (calculate_p(cnt_sum) >= (p_set / 2)) {
            printf("Entering Deacceleration (p: %d >= p_set: %d, k_sum was %d)\n", calculate_p(cnt_sum), p_set, cnt_sum);

            state = State::deceleration;

            k_sum_entering_deacceleration = k_sum;
        }
    }

    // Steady State

    if (state == State::steady_state) {
        
        static constexpr uint32_t p_blockmaxx100 = as_u32(100 * 255 * m);
        const uint32_t p_leftx100 = 100 *(p_set - p_entering_steady - calculate_p(cnt_sum));
        k = as_u16(100 * f_clk * m) / v_set / 100;
        

        if (p_leftx100 <= 0) {
            cnt = 0;
        } else if (p_leftx100 > p_blockmaxx100) {
            cnt = 255;
        }  else {
            cnt = 255 * p_leftx100 / p_blockmaxx100;
        }

        k_sum += k * cnt;

        if (p_leftx100 <= 0) {
            printf("Entering Deacceleration\n");
            state = State::deceleration;
            k_sum_entering_deacceleration = k_sum;
        }
    }

    // Deacceleration

    if (state == State::deceleration) {

        printf("Deacceleration\n");

        k = calculate_deacceleration_k(v_set, k_sum - k_sum_entering_deacceleration);

        printf("%d %d\n", v_set, k);

        while (cnt < 255 && calculate_p(cnt_sum+cnt) < p_set) {
            cnt++;
            k_sum += k;
        }

        printf("Deacceleration round over\n");

        if (calculate_p(cnt_sum) >= p_set) {
            printf("Leaving Deacceleration\n");

            state = State::deceleration;

            k_sum_entering_deacceleration = k_sum;

            state = State::done;
        }

        
    }

    if (state == State::done) {
        return TrajectoryBlock::empty_block();
    } else {
        cnt_sum += cnt;
        printf("block: %d, %d\n", k, cnt);
        return TrajectoryBlock(k, cnt);
    }
    
}
