#include <stdint.h>
#include <array>
#include <cmath>

class TrajectoryBlock {
public:
    TrajectoryBlock() = default;

    TrajectoryBlock (uint8_t k, uint8_t cnts)
        :k(k), cnts(cnts) {};

    TrajectoryBlock (const TrajectoryBlock &other)
        :k(other.k), cnts(other.cnts) {};

    static TrajectoryBlock empty_block() {
        return TrajectoryBlock(0, 0);
    }
    inline bool is_empty() {
        return cnts == 0;
    }

    TrajectoryBlock& operator=(const TrajectoryBlock& other) {
        k = other.k;
        cnts = other.cnts;

        return *this;
    }

    // TODO no protection against manipulation here..
    uint8_t k = 0;
    uint8_t cnts = 0;
};

class TrajectoryPlanner {
public:
    TrajectoryPlanner(uint16_t position_error, uint16_t set_velocity, uint16_t set_acceleration);

    inline uint16_t get_t_0() {
        return t_0;
    }

    inline uint32_t get_k_sum() {
        return k_sum;
    }

    inline bool is_done() {
        return state == State::done;
    };

    TrajectoryBlock calculate_next_block();


    static constexpr std::array<uint16_t, 256> calculate_cycleupdate_steps(float f_timer, float m, float a_set) {
        std::array<uint16_t, 256> ret{};

        for (int i=255; i>2; i--) {
            ret[i] = static_cast<uint16_t>(
                std::roundf(m * f_timer * f_timer / a_set * (1.0f/(i-1) - 1.0f/i))
            );
        }

        return ret;
    }

private:
    const uint16_t p_set = 0;
    const uint16_t v_set = 0;
    const uint16_t t_0 = 0;

    uint32_t k_sum = 0;
    uint32_t cnt_sum = 0;

    enum class State {
        acceleration,
        steady_state,
        deceleration,
        done
    };

    State state;
    uint16_t p_entering_steady = 0;
    uint16_t k_sum_entering_deacceleration = 0;

};
