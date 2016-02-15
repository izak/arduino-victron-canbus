// d#emo: CAN-BUS Shield, receive data with interrupt mode
// when in interrupt mode, the data coming can't be too fast, must >20ms, or else you can use check mode
// loovee, 2014-6-13

#include <SPI.h>
#include "mcp_can.h"

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin


volatile unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
unsigned char output[16];
char str[20];

void MCP2515_ISR() {
    flagRecv = 1;
    digitalWrite(13, HIGH); //Flash LED
}

void setup() {
    Serial.begin(115200);
    pinMode(13, OUTPUT);     
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

    attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt
}

void loop() {
    //INT32U id = 0;
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
        // message does not clear the IRQ conditon.
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
                    Serial.print("\nV\t"); Serial.print(v*10);
                    Serial.print("\nI\t"); Serial.print(i*100);
                    Serial.print("\nP\t"); Serial.print((i*v)/1000); // Integer division!
                    Serial.print("\nT\t"); Serial.print(buf[7]);
                }
            }
        }
        Serial.print("\n");
        digitalWrite(13, LOW); // Turn LED off after handling data
    } else {
        delay(10);
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
/*

packets with id: 435295268, pgn = 127508
First data field (8 bits) is battery instance, followed by 16 bits voltage
followed by 16 bits ampere. Eg:

435295268: 0    46  10  0   0

Voltage = 10 * 256 + 46 = 2606 == 26.06V

Filter:
priority (3 bits), reserved (2 bits), pgn (24 bits), src (8 bits)
mask = 0x1ffff00, match the 17 bits for PGN

XXX1100 1111100100001010000100100
0000000 11111001000010100XXXXXXXX

*/
