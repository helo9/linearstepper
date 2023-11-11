#include "Arduino.h"
#include "TrajectoryPlanning.hpp"

using namespace Trajectory;

static volatile Block movement_plan[256] = {};
volatile static uint16_t movement_index = 0;
volatile static bool movement_ready = false;
volatile static bool is_moving = false;

String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void plan_movement();

enum class State {
    IDLE,
    MOVING,
};

State main_state = State::IDLE;

void setup(){

    // Enable LED
    DDRB |= (1<<PB5);

    // Enable status pin / Active during caculation
    DDRD |= (1<<PD7) | (1<<PD6);
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

    Serial.print(roundf(m * f_timer / 15));
    Serial.print('\n');

    inputString.reserve(200);
}

void loop() {

    switch (main_state) {
        case State::IDLE: {
            if (!stringComplete) {
                // nothing todo
                return;
            }

            stringComplete = false;

            Serial.print("Got String\n");

            if(inputString.startsWith("g") == false) {
                inputString = "";
                return;
            }

            // TODO: parse
            inputString = "";

            Serial.print("String started with g\n");

            PORTB |= (1<<PB5);
            PORTD |= (1<<PD6);

            plan_movement();

            PORTD &= ~(1<<PD6);
            PORTB ^= (1<<PB5);

            Serial.print("Starting over..");

            main_state = State::MOVING;

            break;
        }
        case State::MOVING: {

            movement_index = 0;
            movement_ready = false;

            // prescaler = 256
            TCCR2B = (1<<CS22) | (1<<CS21);
            PORTB &= ~(1<<PB5);

            while(movement_ready == false) {
                _NOP();
            }

            main_state = State::IDLE;

            break;
        }

    }
}

void plan_movement() {

    auto planner = Planner(20, 15);

    for (int i=0; i<2800; i++) {
        if (i < 512 && planner.is_done() == false) {
            movement_plan[i] = planner.calculate_next_block();
            Serial.print(movement_plan[i].k);
            Serial.print(' ');
            Serial.print(movement_plan[i].cnts);
            Serial.print(' ');
            Serial.print(i);
            Serial.print('\n');
        } else {
            Serial.print(i);
            Serial.print('\n');
            planner.calculate_next_block();
            break;
        }
    }

}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    Serial.print(inChar);
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == 'g') {
      stringComplete = true;
      Serial.print("Got \\n\n");
    }
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
