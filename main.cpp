#include "mbed.h"
#include "nRF24L01P.h"
#include <algorithm>

// IN1 -> PTB0
// IN2 -> PTB1
// IN3 -> PTB2
// IN4 -> PTB3
#define TURN_ON 1
#define TURN_OFF 0

#define TRANSFER_SIZE 5

// Initialize BufferedSerial object for PC communication
BufferedSerial pc(USBTX, USBRX, 9600); // USBTX and USBRX are default pins for serial communication

// nRF24L01+ module setup
nRF24L01P my_nrf24l01p(PTD2, PTD3, PTC5, PTD0, PTD5, PTA13);    // mosi, miso, sck, csn, ce, irq

// LED indicators
DigitalOut myled1(LED1);

// H-Bridge 
DigitalOut IN1(PTB0);
DigitalOut IN2(PTB1);
DigitalOut IN3(PTB2);
DigitalOut IN4(PTB3);

// HC-SR04 Ultrasonic
DigitalOut trigPin(PTD0);
DigitalIn echoPin(PTD1);
Timer echoTimer;

// HW201 IR speed sensors
InterruptIn irLeft(PTA4);
InterruptIn irRight(PTA5);

// Point position
struct Point {
        float x;
        float y;
};

Point finalDestination;
Point obstaclePosition;
Point actualPosition;

struct Coordinate {
    char axis;
    float value;
};

Coordinate xAxis;
Coordinate yAxis;

enum class State {
    Initialize,
    ReceivePosition,
    MoveForward,
    Turn
};


volatile bool obstacleDetected = false; // define object can be altered 
State currentState = State::Initialize;
State stateBeforeObstacle;

// ---------------------------
// Ultrasonic Measurement
// ---------------------------
float measureDistanceCM() {
    trigPin = 1;
    wait_us(10);
    trigPin = 0;

    while (echoPin.read() == 0);
    echoTimer.reset();
    echoTimer.start();

    while (echoPin.read() == 1);
    echoTimer.stop();

    float duration = echoTimer.read_us();
    float distance = duration * 0.0343f / 2.0f;  // cm
    return distance;
}

// ---------------------------
// Obstacle Detection (Async)
// ---------------------------
Ticker ultrasonicTicker;

void detectObjectISR() {
    float d = measureDistanceCM();
    if (d > 0 && d < 15.0f) {      // 15 cm threshold
        obstacleDetected = true;
    }
}

void motorStop() {
    IN1 = TURN_OFF;
    IN2 = TURN_OFF;
    IN3 = TURN_OFF;
    IN4 = TURN_OFF;
    len = sprintf(buffer, "Motores desligados!\r\n");
    pc.write(buffer, len);
}

void motorForward() {
    // later to pass pwm signal
    IN1 = TURN_ON;
    IN2 = TURN_OFF;
    IN3 = TURN_ON;
    IN4 = TURN_OFF;
    myled1 = 0;
    len = sprintf(buffer, "Andando para frente!\r\n");
    pc.write(buffer, len);
}

void motorTurnLeft() {
    // later to pass pwm signal
    IN1 = TURN_ON;
    IN2 = TURN_OFF;
    IN3 = TURN_OFF;
    IN4 = TURN_OFF;
    len = sprintf(buffer, "Virando para esquerda!\r\n");
    pc.write(buffer, len);
}

void motorTurnRight() {
    // later to pass pwm signal
    IN1 = TURN_OFF;
    IN2 = TURN_OFF;
    IN3 = TURN_ON;
    IN4 = TURN_OFF;
    len = sprintf(buffer, "Virando para direita!\r\n");
    pc.write(buffer, len);
}

void contour(Point final_destination) {
    if (final_destination.x > 0){
        motorTurnRight();
        thread_sleep_for(400);
        motorForward();
        thread_sleep_for(500);
    } else if (final_destination.x < 0){
        motorTurnLeft();
        thread_sleep_for(400);
        motorForward();
        thread_sleep_for(500);
    } else {
        motorStop();
        thread_sleep_for(500);
    }
}

// ---------------------------
// Obstacle Avoidance
// ---------------------------
void handleObstacle(Point final_destination) {
    len = sprintf(buffer, "Obstáculo detectado! Contornar-lo-ei!\r\n");
    pc.write(buffer, len);

    motorStop();
    thread_sleep_for(200);

    contour(final_destination);
    motorStop();

    obstacleDetected = false;
    currentState = stateBeforeObstacle;   // resume previous state
}

char* receiveMessage(){
    // Check if data is available in the nRF24L01+
    if (my_nrf24l01p.readable()) {
        rxDataCnt = my_nrf24l01p.read(NRF24L01P_PIPE_P0, rxData, sizeof(rxData));
        pc.write(rxData, rxDataCnt);
    }
    return rxData;
}


void initializeRF(){
    char txData[TRANSFER_SIZE] = {0}, rxData[TRANSFER_SIZE] = {0};
    int txDataCnt = 0;
    int rxDataCnt = 0;

    const long long RX_ADDR = 0xE7E7E7E7EA;
    const long long TX_ADDR = 0xE7E7E7E7E9;

    my_nrf24l01p.setRxAddress(RX_ADDR);
    my_nrf24l01p.setTxAddress(TX_ADDR);

    // Initialize the nRF24L01+ module
    pc.write("Powering up the nRF24L01+...\r\n", 31);
    my_nrf24l01p.powerUp();

    // Display the setup of the nRF24L01+ chip
    char buffer[100];
    int len = sprintf(buffer, "nRF24L01+ Frequency    : %d MHz\r\n", my_nrf24l01p.getRfFrequency());
    pc.write(buffer, len);
    len = sprintf(buffer, "nRF24L01+ Output power : %d dBm\r\n", my_nrf24l01p.getRfOutputPower());
    pc.write(buffer, len);
    len = sprintf(buffer, "nRF24L01+ Data Rate    : %d kbps\r\n", my_nrf24l01p.getAirDataRate());
    pc.write(buffer, len);
    len = sprintf(buffer, "nRF24L01+ TX Address   : 0x%010llX\r\n", my_nrf24l01p.getTxAddress());
    pc.write(buffer, len);
    len = sprintf(buffer, "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress());
    pc.write(buffer, len);

    pc.write("Type keys to test transfers:\r\n  (transfers are grouped into 4 characters)\r\n", 69);

    my_nrf24l01p.setTransferSize(TRANSFER_SIZE);
    my_nrf24l01p.setReceiveMode();

    my_nrf24l01p.enable();
}

void prepareCoordinates(char axis, float value, char* outBuffer)
{
    outBuffer[0] = axis;
    std::memcpy(outBuffer + 1, &value, sizeof(float));
}

Coordinate parseCoordinate(const char* msg)
{
    Coordinate c;
    c.axis = msg[0];
    std::memcpy(&c.value, msg + 1, sizeof(float));
    return c;
}

/*
    char* coor = "x";
    
    float f = 2.403893f;
    char float_str[sizeof(float)];
    
    memcpy(float_str, &f, sizeof(float));
    
    char buf[strlen(float_str) + strlen(coor)];
    
    strcpy(buf, coor);
    strcat(buf, float_str);
    
    std::cout << "Converted String: " << buf << std::endl; // Output: 1
    
    
    float decoded_float;
    char decoded_coor;
    
    // Copy the 4 bytes into the memory location of the float
    char* bytes = buf + 1; 
    std::memcpy(&decoded_float, bytes, sizeof(decoded_float));
    
    char bytesC = buf[0]; 
    
    std::cout << "Decoded cord: " << bytesC << std::endl; // Output: 1
    std::cout << "Decoded float: " << decoded_float << std::endl; // Output: 1
    return 0;

*/

// ---------------------------
// Main FSM Step
// ---------------------------
void runStateMachine() {

    switch (currentState) {
        case State::Initialize:
            printf("Initializing...\n");
            initializeRF();
            motorStop();
            currentState = State::ReceivePosition;
            break;

        case State::ReceivePosition:
            printf("Receiving target position...\n");
            char *msg;
            coordinate = msg[0];
            
            thread_sleep_for(500);
            msg = receiveMessage();
            currentState = State::MoveForward;
            break;

        case State::MoveForward:
            motorForward();
            len = sprintf(buffer, "Movendo para frente!\r\n");
            pc.write(buffer, len);
            // example condition for turn
            if (/* some condition */ false) {
                currentState = State::Turn;
            }
            break;

        case State::Turn:
            len = sprintf(buffer, "Virando!\r\n");
            pc.write(buffer, len);

            contour(final_destination);
            motorStop();
            currentState = State::MoveForward;
            break;
    }
}

/* main
// ---------------------------
// Main
// ---------------------------
int main() {

    ultrasonicTicker.attach(&detectObjectISR, 100ms);

    while (true) {

        if (obstacleDetected) {
            stateBeforeObstacle = currentState;
            handleObstacle();
        }

        runStateMachine();

        thread_sleep_for(50);  // small delay
    }
}
*/

int main() {
    while (1) {
        // Check if data is available on the serial interface
        if (pc.readable()) {
            char c;
            pc.read(&c, 1);
            txData[txDataCnt++] = c;

            // Transmit buffer full
            if (txDataCnt >= sizeof(txData)) {
                my_nrf24l01p.write(NRF24L01P_PIPE_P0, txData, txDataCnt);
                txDataCnt = 0;
            }
        }
    }
}