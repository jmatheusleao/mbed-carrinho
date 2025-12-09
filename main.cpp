#include "mbed.h"
#include "nRF24L01P.h"
#include <cmath>
#include <cstring>

//
// HARDWARE
//

BufferedSerial pc(USBTX, USBRX, 9600);

// nRF24L01
nRF24L01P rf(PTD2, PTD3, PTC5, PTD0, PTD5, PTA13);

#define TRANSFER_SIZE 5
char rxBuf[TRANSFER_SIZE];

const long long RX_ADDR = 0xE7E7E7E7EA;
const long long TX_ADDR = 0xE7E7E7E7E9;

// Motors (L298N)
DigitalOut IN1(PTB0);
DigitalOut IN2(PTB1);
DigitalOut IN3(PTB2);
DigitalOut IN4(PTB3);

// Ultrasonic
DigitalOut trigPin(PTD7);
InterruptIn echoPin(PTD6);

// Encoders
InterruptIn encR(PTA4);
InterruptIn encL(PTA5);


// LED
DigitalOut led(LED1);


//
// NAVIGATION DATA
//

struct Point { float x; float y; };
Point finalDestination = {0,0};
Point actualPosition   = {0,0};

enum class Axis { X, Y };
enum class State {
    IdleInitializing,
    ReceivingCoordinates,
    MoveForward,
    Turn,
    EndState
};

State currentState = State::IdleInitializing;
Axis currentAxis   = Axis::X;

volatile bool haveX = false;
volatile bool haveY = false;
volatile bool done_x = false;
volatile bool done_y = false;
volatile bool obstacleDetected = false;



//
// ENCODERS
//

volatile int32_t pulsesR = 0;
volatile int32_t pulsesL = 0;

Timer debounceTimer;
uint32_t lastR = 0, lastL = 0;

static const uint32_t DEBOUNCE = 500;     
static const float CM_PER_PULSE_R = 1.25; 
static const float CM_PER_PULSE_L = 1.25;

void encR_isr() {
    uint32_t t = debounceTimer.read_us();
    if (t - lastR > DEBOUNCE) {
        pulsesR++;
        lastR = t;
    }
}

void encL_isr() {
    uint32_t t = debounceTimer.read_us();
    if (t - lastL > DEBOUNCE) {
        pulsesL++;
        lastL = t;
    }
}

void resetPulses() {
    __disable_irq();
    pulsesR = 0;
    pulsesL = 0;
    __enable_irq();
    pc.write("Encoders reset.\r\n", 18);
}

float getDistanceCm() {
    __disable_irq();
    int32_t pr = pulsesR;
    int32_t pl = pulsesL;
    __enable_irq();

    float dR = pr * CM_PER_PULSE_R;
    float dL = pl * CM_PER_PULSE_L;

    float avg = (dR + dL) * 0.5f;

    char msg[64];
    int n = sprintf(msg, "Encoder distance = %.2f cm\r\n", avg);
    pc.write(msg, n);

    return avg;
}



//
// ULTRASONIC
//

Timer echoTimer;
volatile uint32_t echoStart = 0;
volatile uint32_t echoWidth = 0;
volatile bool newEcho = false;

void echoRise() {
    echoStart = echoTimer.read_us();
}
void echoFall() {
    echoWidth = echoTimer.read_us() - echoStart;
    newEcho = true;
}

void sendPing() {
    trigPin = 0;
    wait_us(2);
    trigPin = 1;
    wait_us(10);
    trigPin = 0;
}

float readUltrasonicCm() {
    if (!newEcho) return 999;
    newEcho = false;

    float d = echoWidth * 0.01715f;

    char msg[64];
    sprintf(msg, "Ultrasonic: %.2f cm\r\n", d);
    pc.write(msg, strlen(msg));

    return d;
}

bool checkObstacle() {
    float d = readUltrasonicCm();
    return (d < 20.0f);
}



//
// MOTOR CONTROL
//

void motorStop() {
    IN1 = IN2 = IN3 = IN4 = 0;
    pc.write("Motor stop\r\n", 12);
}

void motorForward() {
    IN1 = 0; IN2 = 1; 
    IN3 = 1; IN4 = 0;
    pc.write("Motor forward\r\n", 15);
}

void motorBackward() {
    IN1 = 1; IN2 = 0;
    IN3 = 0; IN4 = 1;
    pc.write("Motor backward\r\n", 16);
}

void motorTurnLeft90() {
    pc.write("Turning left 90 deg...\r\n", 23);
    IN1 = 0; IN2 = 1;
    IN3 = 0; IN4 = 0;
    thread_sleep_for(3000);
    motorStop();
}

void motorTurnRight90() {
    pc.write("Turning right 90 deg...\r\n", 24);
    IN1 = 0; IN2 = 0;
    IN3 = 1; IN4 = 0;
    thread_sleep_for(3000);
    motorStop();
}



//
// RF RECEIVE
//

struct Coordinate {
    char axis;
    float value;
};

Coordinate parseRF(char* msg) {
    Coordinate c;
    c.axis = msg[0];
    memcpy(&c.value, msg+1, sizeof(float));
    return c;
}

void initRF() {
    pc.write("Initializing RF...\r\n", 20);
    rf.setTxAddress(TX_ADDR);
    rf.setRxAddress(RX_ADDR);
    rf.setTransferSize(TRANSFER_SIZE);
    rf.setRfFrequency(2450); // 2.450 GHz
    rf.setAirDataRate(NRF24L01P_DATARATE_1_MBPS); // 1 Mbps
    rf.setRfOutputPower(NRF24L01P_TX_PWR_ZERO_DB);  // 0 dBm

    rf.powerUp();
    rf.setReceiveMode();
    rf.enable();

    pc.write("RF Ready. Waiting for packets...\r\n", 34);
}

void checkRF() {
    if (rf.readable()) {

        pc.write("RF readable! Receiving...\r\n", 28);

        char buf[TRANSFER_SIZE];
        rf.read(NRF24L01P_PIPE_P0, buf, TRANSFER_SIZE);

        Coordinate c = parseRF(buf);

        char msg[64];

        if (c.axis == 'x' || c.axis == 'X') {
            finalDestination.x = c.value;
            haveX = true;

            sprintf(msg, "Received X = %.2f\r\n", c.value);
            pc.write(msg, strlen(msg));
        }
        else if (c.axis == 'y' || c.axis == 'Y') {
            finalDestination.y = c.value;
            haveY = true;

            sprintf(msg, "Received Y = %.2f\r\n", c.value);
            pc.write(msg, strlen(msg));
        }
        else {
            sprintf(msg, "Unknown axis: %c\r\n", c.axis);
            pc.write(msg, strlen(msg));
        }
    }
}



//
// MOVE FORWARD ON CURRENT AXIS
//

bool moveOnAxis(Axis axis) {

    float target = (axis == Axis::X ? finalDestination.x : finalDestination.y);
    float current = (axis == Axis::X ? actualPosition.x : actualPosition.y);

    char msg[80];
    sprintf(msg, "Moving on axis %s: target=%.2f  current=%.2f\r\n",
            (axis == Axis::X ? "X" : "Y"), target, current);
    pc.write(msg, strlen(msg));

    float remaining_cm = (target - current) * 100.0f;

    sprintf(msg, "Remaining distance = %.2f cm\r\n", remaining_cm);
    pc.write(msg, strlen(msg));

    if (fabs(remaining_cm) <= 2.0f) return true;

    if (remaining_cm > 0) motorForward();
    else motorBackward();

    float moved_cm = getDistanceCm();

    if (fabs(moved_cm) >= fabs(remaining_cm)) {

        sprintf(msg, "Axis %s reached.\r\n", (axis == Axis::X ? "X" : "Y"));
        pc.write(msg, strlen(msg));

        motorStop();
        float moved_m = moved_cm / 100.0f;

        if (axis == Axis::X)
            actualPosition.x += (remaining_cm > 0 ? moved_m : -moved_m);
        else
            actualPosition.y += (remaining_cm > 0 ? moved_m : -moved_m);

        resetPulses();

        return true;
    }

    return false;
}



//
// TURN LOGIC
//


void updateAxis() {
    currentAxis = (currentAxis == Axis::X ? Axis::Y : Axis::X);

    char msg[32];
    sprintf(msg, "Axis changed to %s\r\n",
            (currentAxis == Axis::X ? "X" : "Y"));
    pc.write(msg, strlen(msg));
}


//
// STATE MACHINE
//

void initializeSystem() {
    pc.write("System initialization\r\n", 23);

    initRF();
    echoTimer.start();
    debounceTimer.start();
    resetPulses();

    haveX = haveY = false;
    done_x = done_y = false;
    actualPosition = {0,0};
    currentAxis = Axis::X;

    pc.write("Initialization complete.\r\n", 26);
}

void runStateMachine() {

    char msg[64];

    switch(currentState) {

        case State::IdleInitializing:
            pc.write("[STATE] IdleInitializing\r\n", 28);
            initializeSystem();
            currentState = State::ReceivingCoordinates;
            break;


        case State::ReceivingCoordinates:
            pc.write("[STATE] ReceivingCoordinates\r\n", 31);

            if (haveX && haveY) {
                sprintf(msg, "Received both coordinates: X=%.2f  Y=%.2f\r\n",
                        finalDestination.x, finalDestination.y);
                pc.write(msg, strlen(msg));

                resetPulses();
                currentState = State::MoveForward;
            }
            break;


        case State::MoveForward:

            pc.write("[STATE] MoveForward\r\n", 23);

            if (checkObstacle()) {
                pc.write("Obstacle detected! Switching to TURN\r\n", 39);
                obstacleDetected = true;
                currentState = State::Turn;
                break;
            }

            if (moveOnAxis(currentAxis)) {
                if (currentAxis == Axis::X) done_x = true;
                if (currentAxis == Axis::Y) done_y = true;

                if (done_x && done_y) {
                    currentState = State::EndState;
                } else {
                    currentState = State::Turn;
                }
            }

            break;


        case State::Turn:
            pc.write("[STATE] Turn\r\n", 16);

            if(currentAxis == Axis::X) motorTurnRight90();
            if(currentAxis == Axis::Y) motorTurnLeft90();
            updateAxis();
            resetPulses();
            obstacleDetected = false;

            currentState = State::MoveForward;
            break;


        case State::EndState:
            pc.write("[STATE] EndState\r\n", 19);
            motorStop();
            pc.write("Destination reached.\r\n", 23);
            haveX = haveY = false;
            done_x = done_y = false;

            currentState = State::ReceivingCoordinates;
            break;
    }
}



//
// MAIN
//

int main() {

    pc.write("Robot booting...\r\n", 18);

    IN1 = IN2 = IN3 = IN4 = 0;
    trigPin = 0;

    encR.rise(&encR_isr);
    encL.rise(&encL_isr);

    echoPin.rise(&echoRise);
    echoPin.fall(&echoFall);

    Ticker t;
    t.attach(&sendPing, 100ms);

    while (true) {
        checkRF();
        runStateMachine();
        thread_sleep_for(20);
    }
}
