#include "Arduino.h"
#include "TrajectoryPlanning.hpp"


static volatile TrajectoryBlock movement_plan[] = {
	{255, 1},
	{202, 1},
	{173, 1},
	{154, 1},
	{141, 1},
	{130, 1},
	{122, 1},
	{115, 1},
	{109, 1},
	{104, 1},
	{100, 1},
	{96, 1},
	{92, 1},
	{89, 1},
	{87, 1},
	{84, 1},
	{82, 1},
	{79, 1},
	{78, 1},
	{76, 1},
	{74, 1},
	{72, 1},
	{71, 1},
	{69, 1},
	{68, 1},
	{67, 1},
	{66, 1},
	{65, 1},
	{64, 1},
	{63, 1},
	{62, 1},
	{61, 1},
	{60, 1},
	{59, 1},
	{58, 1},
	{57, 2},
	{56, 1},
	{55, 1},
	{54, 2},
	{53, 2},
	{52, 1},
	{51, 2},
	{50, 2},
	{49, 2},
	{48, 3},
	{47, 2},
	{46, 3},
	{45, 2},
	{44, 3},
	{43, 3},
	{42, 4},
	{41, 3},
	{40, 4},
	{39, 4},
	{38, 5},
	{37, 5},
	{36, 5},
	{35, 6},
	{34, 7},
	{33, 7},
	{32, 7},
	{31, 9},
	{30, 9},
	{29, 10},
	{28, 12},
	{27, 13},
	{26, 14},
	{25, 17},
	{24, 18},
	{23, 21},
	{22, 24},
	{21, 27},
	{20, 33},
	{19, 37},
	{18, 44},
	{17, 53},
	{16, 64},
	{15, 77},
	{14, 96},
	{13, 120},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 255},
	{12, 248},
	{13, 120},
	{14, 96},
	{15, 77},
	{16, 64},
	{17, 53},
	{18, 44},
	{19, 38},
	{20, 32},
	{21, 28},
	{22, 24},
	{23, 21},
	{24, 18},
	{25, 16},
	{26, 15},
	{27, 13},
	{28, 11},
	{29, 11},
	{30, 9},
	{31, 9},
	{32, 7},
	{33, 7},
	{34, 7},
	{35, 6},
	{36, 5},
	{37, 5},
	{38, 5},
	{39, 4},
	{40, 4},
	{41, 3},
	{42, 4},
	{43, 3},
	{44, 3},
	{45, 3},
	{46, 2},
	{47, 3},
	{48, 2},
	{49, 2},
	{50, 2},
	{51, 2},
	{52, 2},
	{53, 1},
	{54, 2},
	{55, 2},
	{56, 1},
	{57, 1},
	{58, 2},
	{59, 1},
	{60, 1},
	{61, 1},
	{62, 1},
	{63, 1},
	{64, 1},
	{65, 1},
	{66, 1},
	{67, 1},
	{68, 1},
	{70, 1},
	{71, 1},
	{73, 1},
	{74, 1},
	{76, 1},
	{78, 1},
	{80, 1},
	{82, 1},
	{84, 1},
	{87, 1},
	{89, 1},
	{92, 1},
	{96, 1},
	{99, 1},
	{103, 1},
	{108, 1},
	{113, 1},
	{120, 1},
	{127, 1},
	{136, 1},
	{147, 1},
	{161, 1},
	{180, 1}
};


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
    /*auto planner = TrajectoryPlanner(5, 15, 3000);

    PORTB ^= (1<<PB5);
    PORTD &= ~(1<<PD6);

    for (int i=0; i<2800; i++) {
        if (i < 512) {
            movement_plan[i] = planner.calculate_next_block();
        } else {
            planner.calculate_next_block();
        }
    }*/

    //PORTD |= (1<<PD6);

    PORTB |= (1<<PB5);

    // prescaler = 256
    TCCR2B = (1<<CS22) | (1<<CS21);

    while(movement_ready == false) {
        _NOP();
    }

    PORTB &= ~(1<<PB5);

}

ISR(TIMER2_COMPA_vect){  // Interrupt Service Routine
    if (movement_ready) {
        TCCR2B = 0;
        return;
    }

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
