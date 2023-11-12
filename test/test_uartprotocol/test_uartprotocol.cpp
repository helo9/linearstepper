#include <stdio.h>
#include <string.h>
#include <unity.h>
#include <UartProtocol.hpp>

using namespace std;
using namespace uart_protocol;

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_valid_command(void) {

    char str[] = "g+15\n";

    auto res = parse_command(str, strlen(str));

    TEST_ASSERT(res.is_ok);
    TEST_ASSERT_EQUAL(Direction::forwards, res.value().dir);
    TEST_ASSERT_EQUAL(15, res.value().length);
}

void test_invalid_command(void) {

    char str[] = "g15\n";

    auto res = parse_command(str, strlen(str));

    TEST_ASSERT(res.is_ok == false);
}

void test_invalid_command2(void) {

    char str[] = "g++\n";

    auto res = parse_command(str, strlen(str));

    TEST_ASSERT(res.is_ok == false);
}


int main( int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_valid_command);
    RUN_TEST(test_invalid_command);
    RUN_TEST(test_invalid_command2);
    UNITY_END();
}
