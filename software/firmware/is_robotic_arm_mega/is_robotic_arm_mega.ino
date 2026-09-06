/**
 * is-robotic-arm - Mega Firmware
 * ------------------------------------------------
 * Custom hardware controller for a 5-DOF robotic arm using Arduino Mega + RAMPS 1.4.
 * Features:
 *  - EMI-resistant asynchronous serial communication protocol (<T,j1,j2,j3,j4,j5>).
 *  - Multi-axis spatial synchronization using the extended Bresenham algorithm.
 *  - Trapezoidal velocity profile (acceleration/deceleration) using David Austin's algorithm.
 *  - Hardware-level real-time execution via Timer1 interrupts (Non-blocking architecture).
 */

#include <avr/interrupt.h>

// ==============================================================================
// 1. HARDWARE CONFIGURATION & PINOUT (RAMPS 1.4 Standard)
// ==============================================================================

// Step and Direction pins for 5 axes mapped to X, Y, Z, E0, E1 on RAMPS
const int STEP_PINS[5] = {54, 60, 46, 26, 36}; // J1(X), J2(Y), J3(Z), J4(E0), J5(E1)
const int DIR_PINS[5]  = {55, 61, 48, 28, 34};
const int ENABLE_PIN   = 8;                    // Stepper enable pin (Active LOW)

struct JointConfig {
    float gear_ratio;       // Mechanical reduction ratio
    float steps_per_degree; // Calculated for 1/16 microstepping (TMC2209)
};

// Robot kinematics translation (Assumes NEMA 17: 200 steps/rev, Microstepping: 16)
// Formula: (200 * 16 * gear_ratio) / 360
const JointConfig ROBOT_JOINTS[5] = {
    {16.0, 142.222f},  // J1: Base
    {64.0, 568.889f},  // J2: Shoulder
    {64.0, 568.889f},  // J3: Elbow
    {16.0, 142.222f},  // J4: Wrist Pitch
    {1.0,    8.889f}   // J5: Wrist Roll
};

// ==============================================================================
// 2. GLOBAL VOLATILE VARIABLES (Shared between main loop and ISR)
// ==============================================================================

// Position tracking (in discrete steps)
volatile long current_pos[5] = {0, 0, 0, 0, 0};
volatile long target_pos[5]  = {0, 0, 0, 0, 0};

// Bresenham algorithm variables
volatile long delta[5];
volatile int  dir[5];
volatile long accumulator[5];
volatile long S_max = 0;
volatile long step_count = 0;

// Velocity profile variables (Austin algorithm)
volatile float current_delay = 0;
volatile long  n = 1;
volatile long  accel_steps = 0;
volatile long  decel_steps = 0;

enum ProfilState { IDLE, ACCEL, CRUISE, DECEL };
volatile ProfilState state = IDLE;

// ==============================================================================
// 3. SERIAL COMMUNICATION VARIABLES
// ==============================================================================

const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;

// ==============================================================================
// 4. SETUP & INITIALIZATION
// ==============================================================================

void setup() {
    Serial.begin(115200);

    // Configure RAMPS pins
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); // Enable all drivers

    for (int i = 0; i < 5; i++) {
        pinMode(STEP_PINS[i], OUTPUT);
        pinMode(DIR_PINS[i], OUTPUT);
        digitalWrite(STEP_PINS[i], LOW);
    }

    // Configure Hardware Timer1 for real-time interrupts
    noInterrupts();           // Disable interrupts during setup
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    
    // Set CTC mode (Clear Timer on Compare Match) and Prescaler to 8
    // 16 MHz / 8 = 2 MHz (1 tick = 0.5 microsecond)
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    
    // Enable Timer1 compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    interrupts();             // Re-enable interrupts

    Serial.println("System Ready. Waiting for target <T,j1,j2,j3,j4,j5>...");
}

// ==============================================================================
// 5. MAIN LOOP (High-Level Brain)
// ==============================================================================

void loop() {
    // Non-blocking serial reading
    recvWithStartEndMarkers();
    
    // Process incoming command frame
    if (newData == true) {
        strcpy(tempChars, receivedChars);
        parseData();
        newData = false;
    }
    
    // Main loop remains extremely fast and is never blocked by motion delays.
}

// ==============================================================================
// 6. SERIAL PARSING LOGIC
// ==============================================================================

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) ndx = numChars - 1; // Prevent buffer overflow
            } else {
                receivedChars[ndx] = '\0'; // Terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {
    char * strtokIndx;
    char command_type;
    long targets[5];

    // Read the command header
    strtokIndx = strtok(tempChars, ",");
    if (strtokIndx == NULL) return;
    command_type = strtokIndx[0];

    // Trajectory command logic
    if (command_type == 'T') {
        for (int i = 0; i < 5; i++) {
            strtokIndx = strtok(NULL, ",");
            if (strtokIndx != NULL) {
                targets[i] = atol(strtokIndx); // Convert string to long
            }
        }
        
        // Execute motion only if the robot has finished the previous trajectory
        if (state == IDLE) {
            prepare_movement(targets[0], targets[1], targets[2], targets[3], targets[4]);
            Serial.println("ACK:MOVING");
        } else {
            Serial.println("ERR:BUSY"); // Reject command, robot is currently moving
        }
    }
}

// ==============================================================================
// 7. MOTION PLANNER (Pre-calculation)
// ==============================================================================

void prepare_movement(long t1, long t2, long t3, long t4, long t5) {
    noInterrupts(); // Pause timer while updating variables
    
    target_pos[0] = t1; target_pos[1] = t2; target_pos[2] = t3; 
    target_pos[3] = t4; target_pos[4] = t5;
    
    S_max = 0;
    
    // Calculate deltas and set hardware directions
    for(int i = 0; i < 5; i++) {
        delta[i] = target_pos[i] - current_pos[i];
        
        dir[i] = (delta[i] > 0) ? 1 : -1;
        digitalWrite(DIR_PINS[i], (dir[i] == 1) ? HIGH : LOW);
        
        delta[i] = abs(delta[i]);
        if(delta[i] > S_max) S_max = delta[i];
    }

    // Only configure profile if actual movement is required
    if (S_max > 0) {
        for(int i = 0; i < 5; i++) accumulator[i] = S_max / 2;
        
        // Profile shape: 25% Acceleration, 50% Cruise, 25% Deceleration
        accel_steps = S_max / 4;
        decel_steps = S_max / 4;
        
        step_count = 0;
        n = 1;
        current_delay = 2000.0; // Start delay: 2000 us (slow startup)
        
        // Prime the hardware timer
        // OCR1A = delay_us * 2 (since 1 tick = 0.5 us with Prescaler 8)
        OCR1A = (unsigned int)(current_delay * 2.0); 
        state = ACCEL;
    }
    
    interrupts(); // Resume timer, hardware will take over from here
}

// ==============================================================================
// 8. HARDWARE TIMERS ISR (Real-Time Execution)
// ==============================================================================

ISR(TIMER1_COMPA_vect) {
    if (state == IDLE) return; // Halt execution if not actively moving
    
    // 1. Spatial Synchronizer (Extended Bresenham Algorithm)
    for(int i = 0; i < 5; i++) {
        accumulator[i] += delta[i];
        if(accumulator[i] >= S_max) {
            accumulator[i] -= S_max;
            
            // Physical pulse generation to TMC2209 (Requires minimum ~100ns pulse width)
            // Standard digitalWrite takes ~3-4us, acting as a natural, safe pulse delay.
            digitalWrite(STEP_PINS[i], HIGH);
            digitalWrite(STEP_PINS[i], LOW);
            
            current_pos[i] += dir[i];
        }
    }
    
    step_count++;
    
    // 2. Movement Completion Check
    if (step_count >= S_max) {
        state = IDLE;
        return;
    }
    
    // 3. Trapezoidal Velocity Profile (David Austin Algorithm)
    float min_delay = 300.0; // Maximum allowed speed (lower delay = faster)
    long decelerate_after = S_max - decel_steps;
    
    switch(state) {
        case ACCEL:
            current_delay = current_delay - ((2.0 * current_delay) / (4.0 * n + 1.0));
            n++;
            if (step_count >= accel_steps || current_delay <= min_delay) {
                if (current_delay < min_delay) current_delay = min_delay;
                state = CRUISE;
            }
            break;
            
        case CRUISE:
            if (step_count >= decelerate_after) {
                state = DECEL;
            }
            break;
            
        case DECEL:
            current_delay = current_delay - ((2.0 * current_delay) / (4.0 * -n + 1.0));
            n--;
            break;
    }
    
    // 4. Program the timer for the next hardware interrupt
    OCR1A = (unsigned int)(current_delay * 2.0);
}