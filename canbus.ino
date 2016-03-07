#include <avr/wdt.h>
#include <SPI.h>
#include "mcp_can.h"

/* On some shields, the CS pin should be set to 9 */
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);


volatile unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];

void MCP2515_ISR() {
    flagRecv = 1;
}

void setup() {
    Serial.begin(19200);
    Serial.println("Startup");

    while(1) {
        if (CAN_OK == CAN.begin(CAN_250KBPS)){
            Serial.println("CAN BUS Shield init ok!");
            break;
        }
        Serial.println("CAN BUS Shield init fail");
        delay(100);
    }

    /* Set filter for PGN 127508 */
    CAN.init_Mask(0, 1, 0x1ffff00);
    CAN.init_Mask(1, 1, 0x1ffff00);
    CAN.init_Filt(0, 1, 127508 << 8);

    /* Disable other filters */
    for (int i=1; i<6; i++){
        CAN.init_Filt(i, 1, 0x01);
    }

    wdt_enable(WDTO_2S); // Watchdog, 2 seconds

    attachInterrupt(0, MCP2515_ISR, FALLING); // Install interrupt handler
}

void loop() {
    //INT32U id = 0;
    // Uncomment these lines, and set the MCP2515 mode to loopback
    // if you want to test.
    //unsigned char msg[8] = { 0, 46, 10, 232, 3, 255, 255, 37};
    //unsigned char msg2[8] = { 1, 182, 6, 141, 2, 255, 255, 37};
    //CAN.sendMsgBuf(435295268, 1, 8, msg);
    //CAN.sendMsgBuf(435295268, 1, 8, msg2);
    //delay(1000);
    if(flagRecv) {

        flagRecv = 0;

        // iterate over all pending messages
        // If either the bus is saturated or the MCU is busy,
        // both RX buffers may be in use and reading a single
        // message will not clear the IRQ conditon.
        while (CAN_MSGAVAIL == CAN.checkReceive()) {
            // read data,  len: data length, buf: data buf
            //CAN.readMsgBufID(&id, &len, buf);
            CAN.readMsgBuf(&len, buf);

            // print the data
            if (len >= 8) {
                unsigned long int v = (buf[2]*256+buf[1]);
                unsigned long int i = (buf[4]*256+buf[3]);
                if (buf[0] % 2){
                    Serial.print("\nVPV\t"); Serial.print(v*10);
                    Serial.print("\nIPV\t"); Serial.print(i*100);
                } else {
                    unsigned long int t = (buf[6]*256+buf[5]);
                    Serial.print("\nV\t"); Serial.print(v*10);
                    Serial.print("\nI\t"); Serial.print(i*100);
                    Serial.print("\nP\t"); Serial.print((i*v)/1000); // Integer division!
                    Serial.print("\nT\t"); Serial.print((t+500)/1000); // Integer rounding
                }
            }
        }
        Serial.print("\n");
    } else {
        delay(10); // TODO use sleep mode so we wake up on the next interrupt
    }
    wdt_reset(); // Pat the watchdog
}

/*
NOTES

packets with id: 435295268, pgn = 127508
First data field (8 bits) is battery instance, followed by 16 bits voltage
followed by 16 bits ampere. Eg:

435295268: 0    46  10  0   0

Voltage = 10 * 256 + 46 = 2606 == 26.06V

Filter:
In this application, the data page bit along with the PDU fields form a 17 bit
PGN:

priority (3 bits), extended data (1 bit), pgn (17 bits), src (8 bits)
mask = 0x1ffff00

*/
