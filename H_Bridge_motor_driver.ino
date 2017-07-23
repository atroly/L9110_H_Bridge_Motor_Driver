/*
  L9110 motor driver for automatic watch winder
  Chris Taylor
  21 July 2017
*/

#define SECONDS 10      // seconds to run for each step in sequence
#define DEFAULT_PWM 196 // default duty cycle (255=full speed)
#define STOP_DELAY 1000 // motor stop time in ms before reversing

// Motor control pins
#define A_DIR 2   // A_IB
#define A_PWM 3   // A_IA
#define B_DIR 4   // B_IB
#define B_PWM 5   // B_IA

// Action input pins
#define A_RUN 6 // connect to Gnd to stop motor A
#define B_RUN 7 // connect to Gnd to stop motor B

enum motor  { A, B };
enum action { FORWARD, REVERSE, STOP };

action sequence[] = { FORWARD, STOP, REVERSE, STOP };
action current_action = STOP;

int seq_count = sizeof(sequence)/sizeof(sequence[0]);
int current_seq = 0;
// ensure if() condition is true on startup
unsigned long previousTime = millis() - 1000 * SECONDS; 
boolean A_prev_state = LOW;
boolean B_prev_state = LOW;

void setup() {
    pinMode(A_DIR, OUTPUT);
    pinMode(A_PWM, OUTPUT);
    pinMode(B_DIR, OUTPUT);
    pinMode(B_PWM, OUTPUT);
    pinMode(A_RUN, INPUT_PULLUP);
    pinMode(B_RUN, INPUT_PULLUP);
}
 
void loop() {
    boolean A_state = digitalRead(A_RUN);
    boolean B_state = digitalRead(B_RUN);
    // act on changed motor A control input pin
    if (A_prev_state != A_state) {
        A_prev_state = A_state;
        motorCtrl(A, current_action, A_state);
    }
    // act on changed motor B control input pin
    if (B_prev_state != B_state) {
        B_prev_state = B_state;
        motorCtrl(B, current_action, B_state);
    }
    // if defined time has elapsed, advance to the next action
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= 1000*SECONDS) {
        previousTime = currentTime;
        current_action = sequence[current_seq];
        if (++current_seq == seq_count) current_seq = 0;
        motorCtrl(A, current_action, A_state);
        motorCtrl(B, current_action, B_state);
    }
}
   
void motorCtrl(motor m, action a, boolean state) {
    byte dir = (m==A) ? A_DIR : B_DIR;
    byte pwm = (m==A) ? A_PWM : B_PWM;
    if (state == LOW) a = STOP;
    switch(a) {
        case STOP:
            digitalWrite(dir, LOW);
            digitalWrite(pwm, LOW);
            delay(STOP_DELAY);
            break; 
        case FORWARD:
            motorCtrl(m, STOP, LOW);
            digitalWrite(dir, HIGH);
            analogWrite(pwm, 255-DEFAULT_PWM);
            break;
        case REVERSE:
            motorCtrl(m, STOP, LOW);
            digitalWrite(dir, LOW);
            analogWrite(pwm, DEFAULT_PWM);
            break;       
    }
}
