#include <stddef.h>
#include <stdint.h>
#include <result.hpp>

namespace uart_protocol {

enum class Direction {
    forwards,
    backwards
};

struct Command {
    Direction dir;
    uint8_t length;
};

using ParseCommandResult = result::Result<Command, int>;

inline uint8_t ascii2uint(char c) {
    return static_cast<uint8_t>(c - '0');
}

inline bool is_ascii_number(char c) {
    return (c >= '0') && (c <= '9');
}

ParseCommandResult parse_command(char *str, size_t str_len) {

    Command cmd;

    if (str_len < 4) {
        return ParseCommandResult::Error(-1);
    }

    if (str[0] != 'g') {
        return ParseCommandResult::Error(-2);
    }

    if (str[1] == '+') {
        cmd.dir = Direction::forwards;
    } else if (str[1] == '-') {
        cmd.dir = Direction::backwards;
    } else {
        return ParseCommandResult::Error(-3);
    }

    cmd.length = 0U;

    for (uint8_t i=2; i<str_len; i++) {
        if (is_ascii_number(str[i]) == false) {
            break;
        }
        cmd.length = 10 * cmd.length + ascii2uint(str[i]);
    }

    if (cmd.length == 0) {
        return ParseCommandResult::Error(-4);
    }

    return ParseCommandResult::Ok(cmd);
}

}
