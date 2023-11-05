#include <stdint.h>
//#include <array>
#include <math.h>

namespace Trajectory {

static constexpr auto prescaler = 256U;

static constexpr uint16_t a_set = 30;

static constexpr float f_clk = 16e6f;

static constexpr float f_timer = f_clk / prescaler;

static constexpr float m = 3.0f / (1600.0f);

class Block {
public:
    Block() = default;

    constexpr Block (uint8_t k, uint8_t cnts)
        :k(k), cnts(cnts) {};

    Block (const Block &other)
        :k(other.k), cnts(other.cnts) {};

    static Block empty_block() {
        return Block(0, 0);
    }
    inline bool is_empty() {
        return cnts == 0;
    }

    Block& operator=(const Block& other) {
        k = other.k;
        cnts = other.cnts;

        return *this;
    }

    // TODO no protection against manipulation here..
    uint8_t k = 0;
    uint8_t cnts = 0;
};

template<class T1, class T2, T2 conversion_factor>
class TwoLevelCounter {
public:
    constexpr TwoLevelCounter(T1 upper_init_value, T2 lower_init_value)
        :upper_cnt(upper_init_value), lower_cnt(lower_init_value) {};

    constexpr TwoLevelCounter(const TwoLevelCounter &other)
        :upper_cnt(other.upper_cnt), lower_cnt(other.lower_cnt) {}

    inline void lower_increment() {
        lower_cnt++;

        if (lower_cnt > conversion_factor) {
            lower_cnt = 0;
            upper_cnt ++;
        }
    }

    inline T1 get_upper_cnt() { return upper_cnt; }
    inline T2 get_lower_cnt() { return lower_cnt; }

    inline friend bool operator< (const TwoLevelCounter& a, const TwoLevelCounter& b) {
        if (a.upper_cnt < b.upper_cnt) {
            return true;
        } else if (a.upper_cnt == b.upper_cnt) {
            return a.lower_cnt < b.lower_cnt;
        }

        return false;
    }

    inline friend bool operator>= (const TwoLevelCounter& a, const TwoLevelCounter& b) {
        return !(a < b);
    }

    inline friend bool operator> (const TwoLevelCounter& a, const TwoLevelCounter& b) {
        if (a.upper_cnt > b.upper_cnt) {
            return true;
        } else if (a.upper_cnt == b.upper_cnt) {
            return a.lower_cnt > b.lower_cnt;
        }

        return false;
    }

    inline friend bool operator<= (const TwoLevelCounter& a, const TwoLevelCounter& b) {
        return !(a > b);
    }

    inline friend TwoLevelCounter operator- (const TwoLevelCounter& a, const TwoLevelCounter& b) {
        const auto lower = a.lower_cnt > b.lower_cnt ? a.lower_cnt - b.lower_cnt : 533 - b.lower_cnt + a.lower_cnt;

        const auto upper = a.lower_cnt > b.lower_cnt ? a.upper_cnt - b.upper_cnt : a.upper_cnt -b.upper_cnt - 1;

        return TwoLevelCounter(upper, lower);
    }

private:
    T1 upper_cnt;
    T2 lower_cnt;

};

using Position = TwoLevelCounter<uint8_t, uint16_t, 533>;

class Planner {
public:
    constexpr Planner(uint16_t position_error, uint16_t set_velocity)
        :p_set(position_error), k_set(roundf(m * f_timer / set_velocity)) {}


    inline bool is_done() {
        return state == State::done;
    };

    Block calculate_next_block();

    static constexpr uint8_t minimal_speed_k = 255;
    static constexpr uint16_t precalculated_cycleupdate_steps[] = {
	    0, 0, 0, 40690, 20345, 12207, 8138, 5813, 4360, 3391, 2713, 2219, 1850, 1565, 1341, 1163, 1017, 898, 798, 714, 642, 581, 528, 482, 442, 407, 376, 348, 323, 301, 281, 263, 246, 231, 218, 205, 194, 183, 174, 165, 157, 149, 142, 135, 129, 123, 118, 113, 108, 104, 100, 96, 92, 89, 85, 82, 79, 76, 74, 71, 69, 67, 65, 63, 61, 59, 57, 55, 54, 52, 51, 49, 48, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 33, 32, 31, 30, 30, 29, 29, 28, 27, 27, 26, 26, 25, 25, 24, 24, 23, 23, 22, 22, 22, 21, 21, 20, 20, 20, 19, 19, 19, 18, 18, 18, 17, 17, 17, 17, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4
    };

private:
    const uint16_t p_set = 0;
    const uint16_t k_set = 0;

    Position position {0, 0};
    Position steady_end_position {0, 0};
    uint8_t k = minimal_speed_k;
    uint16_t dk = precalculated_cycleupdate_steps[minimal_speed_k];
    uint8_t r = 0;

    enum class State {
        acceleration,
        steady_state,
        deceleration,
        done
    };

    State state = State::acceleration;
};

}
