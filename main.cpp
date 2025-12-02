#include "mbed.h"
#include "nRF24L01P.h"

// IN1 -> PTB0
// IN2 -> PTB1
// IN3 -> PTB2
// IN4 -> PTB3
#define TURN_ON 1
#define TURN_OFF 0

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

int main() {
    #define TRANSFER_SIZE 4

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

        // Check if data is available in the nRF24L01+
        if (my_nrf24l01p.readable()) {
            rxDataCnt = my_nrf24l01p.read(NRF24L01P_PIPE_P0, rxData, sizeof(rxData));
            pc.write(rxData, rxDataCnt);
            if (rxData[0] == 'l') {
                IN1 = TURN_ON;
                IN2 = TURN_OFF;
                IN3 = TURN_OFF;
                IN4 = TURN_OFF;
                len = sprintf(buffer, "Ligando Motor A!\r\n");
                pc.write(buffer, len);
            } else if (rxData[0] == 'r') {
                IN1 = TURN_OFF;
                IN2 = TURN_OFF;
                IN3 = TURN_ON;
                IN4 = TURN_OFF;
                len = sprintf(buffer, "Ligando Motor B!\r\n");
                pc.write(buffer, len);
            } else if (rxData[0] == 'b') {
                IN1 = TURN_ON;
                IN2 = TURN_OFF;
                IN3 = TURN_ON;
                IN4 = TURN_OFF;
                myled1 = 0;
                len = sprintf(buffer, "Ligando Ambos!\r\n");
                pc.write(buffer, len);
            } else {
                IN1 = TURN_OFF;
                IN2 = TURN_OFF;
                IN3 = TURN_OFF;
                IN4 = TURN_OFF;
                myled1 = !myled1;
                len = sprintf(buffer, "Nada.\r\n");
                pc.write(buffer, len);
            }
        }
    }
}