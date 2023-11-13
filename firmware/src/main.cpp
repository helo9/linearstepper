#include <stdlib.h>
#include <Arduino.h>
#include "ardu_algorithm.hpp"
#include "TrajectoryPlanning.hpp"
#include "UartProtocol.hpp"

using namespace Trajectory;

static constexpr size_t movement_plan_length = 256U;
static volatile Block movement_plan[movement_plan_length] = {};
volatile static uint16_t movement_index = 0;
volatile static bool movement_ready = false;

char input_str[25] = {};      // a String to hold incoming data
uint8_t input_str_index = 0;
bool input_str_complete = false;  // whether the string is complete

void plan_movement(uint8_t distance);

enum class State {
    IDLE,
    MOVING,
};

void setup(){

    // Enable LED
    DDRB |= (1<<PB5);

    // Enable status pin / Active during caculation
    DDRD |= (1<<PD7) | (1<<PD6) | (1<<PD5);
    PORTD = 0x00;

    // Configure Timer 2
    {
        // Wave Form Generation Mode 2: CTC, OC2A disconnected
        TCCR2A = (1<<COM2A0) | (1<<WGM21);

        TCCR2B = 0;
        // interrupt when Compare Match with OCR2A
        TIMSK2 = (1<<OCIE2A);
    }

    {
        TCCR1A = 0x00;
        TIMSK1 = (1<<TOIE1);

    }

    OCR2A = 255;

    PORTD = 0;

    Serial.begin(19200);

    Serial.print("Booted\n\r");
}

void loop() {

    static State main_state = State::IDLE;

    switch (main_state) {
        case State::IDLE: {
            if (!input_str_complete) {
                // nothing todo
                return;
            }

            input_str_complete = false;

            const auto cmd_res = uart_protocol::parse_command(input_str, input_str_index+1);

            input_str_index = 0;

            if (cmd_res.is_ok == false) {
                return;
            }

            if (cmd_res.value().dir == uart_protocol::Direction::backwards) {
                PORTD |= (1<<PD5);
            } else {
                PORTD &= ~(1<<PD5);
            }

            PORTB |= (1<<PB5);
            PORTD |= (1<<PD6);

            const uint8_t distance = clamp(
                cmd_res.value().length,
                static_cast<uint8_t>(4U), // TODO: fix triangular movement planning to lower this
                static_cast<uint8_t>(35U)
            );

            Serial.print("Starting movement planning...");

            plan_movement(distance);

            Serial.print("done\n");

            PORTD &= ~(1<<PD6);
            PORTB ^= (1<<PB5);

            main_state = State::MOVING;

            break;
        }
        case State::MOVING: {

            Serial.print("Moving..");

            movement_index = 0;
            movement_ready = false;

            // prescaler = 256
            TCCR2B = (1<<CS22) | (1<<CS21);
            PORTB &= ~(1<<PB5);

            while(movement_ready == false) {
                _NOP();
            }

            main_state = State::IDLE;

            Serial.print("done\n");

            break;
        }

    }
}

inline void plan_movement(uint8_t distance) {

    auto planner = Planner(distance, 15);

    for (size_t i=0; i<movement_plan_length-1; i++) {
        if (planner.is_done()) {
            movement_plan[i] = Block(0, 0);
            break;
        }

        movement_plan[i] = planner.calculate_next_block();

        /*Serial.print(movement_plan[i].k);
        Serial.print(' ');
        Serial.print(' ');
        Serial.print(movement_plan[i].cnts);
        Serial.print(' ');
        Serial.print(i);
        Serial.print("\n\r");*/
    }

}

void serialEvent() {
    while (Serial.available()) {
        // get the new byte:
        input_str[input_str_index] = (char)Serial.read();

        Serial.print(input_str[input_str_index]);

        // if the incoming character is a newline, set a flag so the main loop can
        // do something about it:
        if (input_str[input_str_index] == '\n' || input_str[input_str_index] == '\r') {
            Serial.print("in complete\n\r");
            input_str_complete = true;
        }

        input_str_index = (input_str_index + 1) % sizeof(input_str);
    }
}

ISR(TIMER2_COMPA_vect){  // Interrupt Service Routine
    if (movement_ready) {
        TCCR2B = 0;
        return;
    }

    PORTD |= (1<<PD7);
    TCNT1 = 0xFF80;
    TCCR1B = (1<<CS11);

    if (movement_plan[movement_index].cnts==0) {
        if (movement_index<512) {
            movement_index++;
            if (movement_plan[movement_index].k == 0) {
                movement_ready = true;
            }
        }
        else {
            TCCR2B = 0;
            movement_ready = true;
            return;
        }
    }


    auto &current_block = movement_plan[movement_index];

    OCR2A = current_block.k;
    current_block.cnts--;

    PORTD ^= (1<<PD6);
}

ISR(TIMER1_OVF_vect) {
    PORTD &= ~(1<<PD7);
    TCCR1B = 0x00;
}
