#include "mbed.h"
#include "nRF24L01P.h"
#include <cstring>
#include <cmath>
#include <cstdarg>

// ---------------------------
// CONFIG / HARDWARE PINS
// ---------------------------
// nRF24L01P pins (mosi, miso, sck, csn, ce, irq)
BufferedSerial pc(USBTX, USBRX, 9600);

nRF24L01P my_nrf24l01p(PTD2, PTD3, PTC5, PTD0, PTD5, PTA13);    // mosi, miso, sck, csn, ce, irq

#define TRANSFER_SIZE 5
char txData[TRANSFER_SIZE] = {0};
int txDataCnt = 0;

const long long RX_ADDR = 0xE7E7E7E7EA;
const long long TX_ADDR = 0xE7E7E7E7E9;

// Motor H-bridge (L298N) pins
DigitalOut IN1(PTB0);
DigitalOut IN2(PTB1);
DigitalOut IN3(PTB2);
DigitalOut IN4(PTB3);

// Ultrasonic HC-SR04 pins
DigitalOut trigPin(PTD7);
InterruptIn echoPin(PTD6);

// Encoders (IR sensors)
InterruptIn encRight(PTA4);
InterruptIn encLeft(PTA5);

// LED
DigitalOut myled1(LED1);

// ---------------------------
// STATE & NAV DATA STRUCTURES
// ---------------------------
struct Point { float x; float y; }; // coordinates in meters (by convention)
Point finalDestination = {0.0f, 0.0f};
Point actualPosition = {0.0f, 0.0f};

struct Coordinate { char axis; float value; };
enum class State { Initialize, WaitForCoordinates, ReceivePosition, MoveForward, Idle };
State currentState = State::Initialize;
State stateBeforeObstacle = State::Initialize;

volatile bool obstacleDetected = false;

// RF receive helpers
bool haveX = false, haveY = false;

// ---------------------------
// ENCODER VARIABLES & DEBOUNCE
// ---------------------------
volatile int32_t pulse_right = 0;
volatile int32_t pulse_left  = 0;

static const float CM_PER_PULSE_RIGHT = 1.37f; // as provided (cm per encoder pulse)
static const float CM_PER_PULSE_LEFT  = 1.37f;

Timer debounceTimer;
volatile uint32_t last_right_us = 0;
volatile uint32_t last_left_us  = 0;
static const uint32_t DEBOUNCE_US = 300;  // microseconds

void on_pulse_right()
{
    uint32_t now = debounceTimer.read_us();
    if ((now - last_right_us) > DEBOUNCE_US) {
        pulse_right++;
        last_right_us = now;
    }
}

void on_pulse_left()
{
    uint32_t now = debounceTimer.read_us();
    if ((now - last_left_us) > DEBOUNCE_US) {
        pulse_left++;
        last_left_us = now;
    }
}

void float_to_str1(char *out, float v)
{
    int32_t scaled = (int32_t)(v * 10.0f + (v >= 0 ? 0.5f : -0.5f));
    if (scaled < 0) scaled = -scaled;

    int32_t inteiro = scaled / 10;
    int32_t frac    = scaled % 10;

    snprintf(out, 16, "%ld.%01ld", (long)inteiro, (long)frac);
}

void resetPulses() {
    __disable_irq();
    pulse_right = 0;
    pulse_left = 0;
    __enable_irq();
}

float getAverageDistanceCm() {
    int32_t pr, pl;
    __disable_irq();
    pr = pulse_right;
    pl = pulse_left;
    __enable_irq();
    float dR = pr * CM_PER_PULSE_RIGHT;
    float dL = pl * CM_PER_PULSE_LEFT;
    return (dR + dL) / 2.0f;
}

// ---------------------------
// HC-SR04 (trigger + echo ISRs)
// ---------------------------
Timer usEchoTimer;      // used to timestamp echo edges
volatile uint32_t echo_start_us = 0;
volatile uint32_t echo_pulse_us = 0;
volatile bool new_measure = false;

void echo_rise() {
    echo_start_us = usEchoTimer.read_us();
}
void echo_fall() {
    uint32_t end_us = usEchoTimer.read_us();
    // protect against wrap (mbed timers are large; still safe)
    echo_pulse_us = end_us - echo_start_us;
    new_measure = true;
}

void send_trigger_pulse()
{
    trigPin = 0;
    wait_us(2);
    trigPin = 1;
    wait_us(10);
    trigPin = 0;
}

// ---------------------------
// nRF24 helpers (Option A receiver)
// ---------------------------
char rxBuffer[TRANSFER_SIZE];

void initializeRF(){
    my_nrf24l01p.setRxAddress(RX_ADDR);
    my_nrf24l01p.setTxAddress(TX_ADDR);

    pc.write("Powering up the nRF24L01+...\r\n", 31);
    my_nrf24l01p.powerUp();

    my_nrf24l01p.setTransferSize(TRANSFER_SIZE);
    my_nrf24l01p.setReceiveMode();
    my_nrf24l01p.enable();

    pc.write("Waiting for coordinates (send 'x' + float, 'y' + float)...\r\n", 55);
}

// coordinate parser (5-byte: axis + float)
Coordinate parseCoordinate(const char* msg)
{
    Coordinate c;
    c.axis = msg[0];
    memcpy(&c.value, msg + 1, sizeof(float));
    return c;
}

void checkRFReceive() {
    if (my_nrf24l01p.readable()) {
        char msg[TRANSFER_SIZE];
        int n = my_nrf24l01p.read(NRF24L01P_PIPE_P0, msg, TRANSFER_SIZE);
        if (n >= 1) {
            Coordinate c = parseCoordinate(msg);
            if (c.axis == 'x' || c.axis == 'X') {
                finalDestination.x = c.value;
                haveX = true;
                char buf[64];
                int l = snprintf(buf, sizeof(buf), "Received X=%.2f\r\n", c.value);
                pc.write(buf, l);
            } else if (c.axis == 'y' || c.axis == 'Y') {
                finalDestination.y = c.value;
                haveY = true;
                char buf[64];
                int l = snprintf(buf, sizeof(buf), "Received Y=%.2f\r\n", c.value);
                pc.write(buf, l);
            } else {
                char buf[64];
                int l = snprintf(buf, sizeof(buf), "Unknown axis received: %c\r\n", c.axis);
                pc.write(buf, l);
            }
        }
    }
}

// ---------------------------
// Motor helpers
// ---------------------------
void motorStop() {
    IN1 = 0; IN2 = 0; IN3 = 0; IN4 = 0;
    pc.write("Motors stopped\r\n", 16);
}

void motorForward() {
    // IN1/IN2 control right motor, IN3/IN4 left motor (match your wiring)
    IN1 = 0; IN2 = 1; IN3 = 1; IN4 = 0;
    myled1 = 1;
}

void motorBackward() {
    IN1 = 1; IN2 = 0; IN3 = 0; IN4 = 1;
    myled1 = 0;
}

void motorTurnLeftTimed(int ms) {
    // pivot/arc left: right wheel forward, left wheel stopped (simple)
    IN1 = 0; IN2 = 1; IN3 = 0; IN4 = 0;
    thread_sleep_for(ms);
    motorStop();
}

void motorTurnRightTimed(int ms) {
    IN1 = 0; IN2 = 0; IN3 = 0; IN4 = 1;
    thread_sleep_for(ms);
    motorStop();
}

// simple contour behavior (keeps your original logic)
void contour(Point finalDest) {
    if (finalDest.x > 0) {
        motorTurnRightTimed(5500);
        motorForward();
        thread_sleep_for(2000);
        motorStop();
    } else if (finalDest.x < 0) {
        motorTurnLeftTimed(5500);
        motorForward();
        thread_sleep_for(2000);
        motorStop();
    } else {
        motorStop();
        thread_sleep_for(500);
    }
}

void handleObstacle() {
    motorStop();
    thread_sleep_for(200);
    contour(finalDestination);
    obstacleDetected = false;
    currentState = stateBeforeObstacle;
}

// ---------------------------
// NAVIGATION / STATE MACHINE
// ---------------------------
const int TURN_TIME_MS = 2500;          // calibrate so each is ~90 deg
const float POSITION_TOLERANCE_CM = 2; // tolerance when reaching target
int navPhase = 0; // 0 -> move X, 1 -> turn to Y, 2 -> move Y, 3 -> done

// convert pulses->cm helpers (given constants are CM_PER_PULSE)
float pulsesRightToCm(int32_t p) { return p * CM_PER_PULSE_RIGHT; }
float pulsesLeftToCm(int32_t p)  { return p * CM_PER_PULSE_LEFT;  }

void runStateMachineStep() {
    switch (currentState) {
        case State::Initialize:
            pc.write("Initializing system...\r\n", 24);
            initializeRF();
            // attach encoders
            encRight.mode(PullUp);
            encLeft.mode(PullUp);
            encRight.rise(&on_pulse_right);
            encLeft.rise(&on_pulse_left);
            // ultrasonic
            trigPin = 0;
            echoPin.rise(&echo_rise);
            echoPin.fall(&echo_fall);
            usEchoTimer.start();
            debounceTimer.start();
            resetPulses();
            motorStop();
            navPhase = 0;
            haveX = haveY = false;
            // finalDestination.x = 1;
            // finalDestination.y = 1;
            // NEW: go wait for coordinates
            currentState = State::WaitForCoordinates;
            break;

        case State::WaitForCoordinates:
            // wait until at least one coordinate arrives; then go to ReceivePosition
            if (haveX || haveY) {
                pc.write("First coordinate received. Waiting for the second...\r\n", 58);
                currentState = State::ReceivePosition;
            } 
            // else just keep waiting; checkRFReceive() is called in main loop
            break;

        case State::ReceivePosition:
            // Wait until BOTH X and Y are received
            if (haveX && haveY) {
                char buf[80];
                int l = snprintf(buf, sizeof(buf), "Target set: (%.2f, %.2f)\r\n", finalDestination.x, finalDestination.y);
                pc.write(buf, l);
                // prepare for navigation
                resetPulses();
                actualPosition.x = 0.0f;
                actualPosition.y = 0.0f;
                navPhase = 0;
                currentState = State::MoveForward;
            }
            break;

        case State::MoveForward:
            // process new ultrasonic measurement (set by echo ISRs)
            if (new_measure) {
                new_measure = false;
                uint32_t dur = echo_pulse_us; // microseconds
                float dist_cm = dur * 0.01715f; // sound speed: 343 m/s -> /2 and convert
                if (dist_cm > 0 && dist_cm < 25.0f) {
                    obstacleDetected = true;
                }
            }

            // handle obstacle immediately
            if (obstacleDetected) {
                stateBeforeObstacle = currentState;
                pc.write("Obstacle detected! Performing contour...\r\n", 40);
                handleObstacle();
            }

            if (navPhase == 0) {
                // move along X: assume starting heading is +X
                float dx = finalDestination.x - actualPosition.x;
                if (fabsf(dx) < 1e-3f) {
                    navPhase = 1;
                    motorStop();
                    break;
                }
                float targetCm = fabsf(dx) * 100.0f; // expects coordinates in meters -> convert to cm
                resetPulses();
                if (dx > 0) {
                    motorForward();
                } else {
                    // turn 180 (two 90s) and go forward
                    motorTurnLeftTimed(TURN_TIME_MS);
                    motorTurnLeftTimed(TURN_TIME_MS);
                    motorForward();
                }

                // blocking loop but checks obstacleDetected/new_measure and updates serial
                while (true) {
                    if (obstacleDetected) {
                        motorStop();
                        stateBeforeObstacle = currentState;
                        return; // back to main loop for obstacle handling
                    }
                    // compute distance moved using average pulses
                    float movedCm = getAverageDistanceCm(); // cm
                    if (movedCm >= targetCm - POSITION_TOLERANCE_CM) {
                        motorStop();
                        // update actual position (meters)
                        float movedM = movedCm / 100.0f;
                        if (dx > 0) actualPosition.x += movedM;
                        else actualPosition.x -= movedM;
                        break;
                    }
                    // also keep processing RF so new coordinates won't be missed (non-critical)
                    checkRFReceive();
                    thread_sleep_for(15);
                }
                // if we had turned for negative dx, turn back to original heading
                if (dx < 0) {
                    motorTurnLeftTimed(TURN_TIME_MS);
                    motorTurnLeftTimed(TURN_TIME_MS);
                }
                navPhase = 1;
                thread_sleep_for(50);
            } else if (navPhase == 1) {
                // face +Y or -Y
                float dy = finalDestination.y - actualPosition.y;
                if (fabsf(dy) < 1e-3f) {
                    navPhase = 3; // nothing to do in Y
                } else {
                    if (dy > 0) motorTurnLeftTimed(TURN_TIME_MS); // assumption: left turn -> +Y
                    else motorTurnRightTimed(TURN_TIME_MS);
                    navPhase = 2;
                    resetPulses();
                    thread_sleep_for(50);
                }
            } else if (navPhase == 2) {
                float dy = finalDestination.y - actualPosition.y;
                if (fabsf(dy) < 1e-3f) {
                    navPhase = 3;
                } else {
                    float targetCm = fabsf(dy) * 100.0f;
                    resetPulses();
                    motorForward();
                    while (true) {
                        if (obstacleDetected) {
                            motorStop();
                            stateBeforeObstacle = currentState;
                            return;
                        }
                        float movedCm = getAverageDistanceCm();
                        if (movedCm >= targetCm - POSITION_TOLERANCE_CM) {
                            motorStop();
                            float movedM = movedCm / 100.0f;
                            if (dy > 0) actualPosition.y += movedM;
                            else actualPosition.y -= movedM;
                            break;
                        }
                        checkRFReceive();
                        thread_sleep_for(15);
                    }
                    navPhase = 3;
                }
            } else {
                // navPhase == 3 -> arrived
                char buf[80];
                int l = snprintf(buf, sizeof(buf), "Arrived approx at (%.2f, %.2f)\r\n", actualPosition.x, actualPosition.y);
                pc.write(buf, l);
                // clear received flags to accept new target later
                haveX = haveY = false;
                currentState = State::Idle;
            }
            break;

        case State::Idle:
            // wait for next coordinates
            if (haveX || haveY) {
                // if one coordinate arrives, go back to ReceivePosition to wait for both
                currentState = State::ReceivePosition;
            }
            break;
    }
}

// ---------------------------
// MAIN
// ---------------------------
int main() {
    // small startup messages
    pc.write("Robot booting...\r\n", 18);

    // set initial motor state
    IN1 = 0; IN2 = 0; IN3 = 0; IN4 = 0;
    trigPin = 0;

    // ultrasonic trigger ticker (100 ms)
    Ticker trigTicker;
    trigTicker.attach(&send_trigger_pulse, 100ms);

    // set initial state
    currentState = State::Initialize;

    // loop
    while (true) {
        // nonblocking: check for RF packets
        checkRFReceive();

        // also allow user to type characters to send via nRF (optional debug)
        if (pc.readable()) {
            char c;
            if (pc.read(&c, 1)) {
                // accumulate and send if you want (keeps previous feature)
                txData[txDataCnt++] = c;
                if (txDataCnt >= (int)sizeof(txData)) {
                    my_nrf24l01p.write(NRF24L01P_PIPE_P0, txData, txDataCnt);
                    txDataCnt = 0;
                }
            }
        }

        // run one state machine step
        runStateMachineStep();

        ThisThread::sleep_for(20ms);
    }
}
