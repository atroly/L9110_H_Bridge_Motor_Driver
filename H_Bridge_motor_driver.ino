/*
 * L9110 motor driver for automatic watch winder
 * Copyright (C) 2017 Chris Taylor
 *
 * This program is free software: you can redistribute it and/or modify
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.  
 */

#define SECONDS 10      // seconds to run for each step in sequence
#define DEFAULT_PWM 192 // default duty cycle (255=full speed)
#define STOP_DELAY 1000 // motor stop time in ms before reversing
#define MOTOR_COUNT 2   // number of motors to control

// Motor control pins
#define A_DIR 2   // A_IB
#define A_PWM 3   // A_IA
#define B_DIR 4   // B_IB
#define B_PWM 5   // B_IA

// Action input pins
#define A_RUN 6 // connect to Gnd to stop motor A
#define B_RUN 7 // connect to Gnd to stop motor B

enum action { FORWARD, REVERSE, STOP };

// Sequence of actions for motors
action const sequence[] = { FORWARD, STOP, REVERSE, STOP };

int const seq_count = sizeof(sequence)/sizeof(sequence[0]);

struct motors {
    byte dir_pin;
    byte pwm_pin;
    byte run_pin;
    boolean state;
    boolean prev_state;
};

motors motor[] = {
    {A_DIR, A_PWM, A_RUN, HIGH, LOW}, 
    {B_DIR, B_PWM, B_RUN, HIGH, LOW}
};

void setup() {
    for (int m=0; m<MOTOR_COUNT; m++) {
        pinMode(motor[m].dir_pin, OUTPUT);
        pinMode(motor[m].pwm_pin, OUTPUT);
        pinMode(motor[m].run_pin, INPUT_PULLUP);
    }
 }
 
void loop() {
    static action current_action = STOP;
    // acted on changed motor run pin
    for (int m=0; m<MOTOR_COUNT; m++) {
        motor[m].state = digitalRead(motor[m].run_pin);
        if (motor[m].prev_state != motor[m].state) {
            motor[m].prev_state = motor[m].state;
            motorCtrl(m, current_action); 
        }
    }
    // if defined time has elapsed, advance to the next action
    static unsigned long previousTime = millis() - 1000 * SECONDS;
    static int current_seq = 0;
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= 1000*SECONDS) {
        previousTime = currentTime;
        current_action = sequence[current_seq];
        if (++current_seq == seq_count) current_seq = 0;
        for (int m=0; m<MOTOR_COUNT; m++) {
            motorCtrl(m, current_action);
        }
    }
}
   
void motorCtrl(int m, action a) {
    if (motor[m].state == LOW) a = STOP;
    switch(a) {
        case STOP:
            digitalWrite(motor[m].dir_pin, LOW);
            digitalWrite(motor[m].pwm_pin, LOW);
            delay(STOP_DELAY);
            break; 
        case FORWARD:
            motorCtrl(m, STOP);
            digitalWrite(motor[m].dir_pin, HIGH);
            analogWrite(motor[m].pwm_pin, 255-DEFAULT_PWM);
            break;
        case REVERSE:
            motorCtrl(m, STOP);
            digitalWrite(motor[m].dir_pin, LOW);
            analogWrite(motor[m].pwm_pin, DEFAULT_PWM);
            break;       
    }
}
