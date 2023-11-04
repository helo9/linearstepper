#include "Arduino.h"
#include "TrajectoryPlanning.hpp"

volatile static TrajectoryBlock movement_plan[512];
volatile static uint16_t movement_index = 0;
volatile static bool movement_ready = false;

void setup(){

    // Enable LED
    DDRB |= (1<<PB5);

    // Enable status pin / Active during caculation
    DDRD |= (1<<PD7) | (1<<PD6);

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

}

void loop() {
    auto planner = TrajectoryPlanner(15, 10, 30);

    PORTB ^= (1<<PB5);
    PORTD &= ~(1<<PD6);

    for (int i=0; i<2800; i++) {
        if (i < 512) {
            movement_plan[i] = planner.calculate_next_block();
        } else {
            planner.calculate_next_block();
        }
    }

    //PORTD |= (1<<PD6);

    PORTB |= (1<<PB5);

    // prescaler = 8
    TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);

    while(movement_ready == false) {
        _NOP();
    }

    PORTB &= ~(1<<PB5);

}

ISR(TIMER2_COMPA_vect){  // Interrupt Service Routine
    PORTD |= (1<<PD7);
    TCNT1 = 0xFF00;
    TCCR1B = (1<<CS11);

    if (movement_plan[movement_index].cnts==0) {
        if (movement_index<512) {
            movement_index++;
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