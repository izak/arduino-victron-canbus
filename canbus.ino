#include <avr/wdt.h>
#include <SPI.h>
#include "mcp_can.h"

#define BATTERY_MASK 0x1ffff00
#define BATTERY_FILTER (127508 << 8)

#define VICTRON_MASK 0x1fffff00
#define VICTRON_FILTER 0x1cefff00 //7.0.0.EF.tg=FF.src

/*
const char* message = 
"\r\nV\t%ld" // Battery voltage
"\r\nVPV\t%ld" // Panel Voltage
"\r\nPPV\t%ld" // Panel power
"\r\nI\t%ld" // Battery current
"\r\nIL\t0"
"\r\nLOAD\tOFF"
"\r\nT\t%d" // Temperature
"\r\nH19\t%ld" // Yield Total
"\r\nH20\t%ld" //Yield Today
"\r\nH21\t%ld" // Max power today
"\r\nH22\t%ld" // Yield Yesterday
"\r\nH23\t%ld" // Max power yesterday
"\r\nERR\t0"
"\r\nCS\t%d" // Charger state
"\r\nFW\t2.03" // Hard coded, should be fetched
"\r\nPID\t0xA046" // Pretend to be a 150/70
"\r\nSER#\tHQ1437WAFP8" // Hard coded, look inside case of unit
"\r\nHSDS\t0"
"\r\nChecksum\t";*/

/* Response to vedirect serial number will be:
   0048513134343044345135522C48513134333757414650382C0D0A00000000000054
*/


/* On some shields, the CS pin should be set to 9 */
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

typedef struct {
    unsigned long int yield_total;
    unsigned long int yield_today;
    unsigned long int max_today;
    unsigned long int yield_yesterday;
    unsigned long int max_yesterday;
    unsigned char device_state;
    unsigned long int pv_voltage;
    unsigned long int pv_current;
    char checksum;
} DeviceData;

volatile unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
unsigned char count = 0;
unsigned char vregs[] = {0xD0, 0xD1, 0xD2, 0xD3, 0xDC};
DeviceData devicedata;

void MCP2515_ISR() {
    flagRecv = 1;
}

unsigned long int extract_value(unsigned char *b){
    unsigned long int r = 0;
    r = b[3];
    r = (r << 8) + b[2];
    r = (r << 8) + b[1];
    r = (r << 8) + b[0];
    return r;
}

uint8_t checksum(const char *msg){
    uint8_t chr = 0;
    for (int i=0; msg[i]!='\0'; i++) {
        chr = (chr + msg[i]);
    }
    return chr;
    //return (char)(256 - chr);
}

size_t serial_print(const char* s){
    devicedata.checksum += checksum(s);
    return Serial.print(s);
}

size_t serial_print(unsigned long i){
    unsigned long t = i;
    // If the long int was to be converted to a string, it would end
    // up ad chars between 0x30 and 0x39. So simply add them up in the
    // same manner.
    while (t>0) {
        devicedata.checksum += (0x30 + (t % 10));
        t /= 10;
    }
    return Serial.print(i);
}

void setup() {
    // Initialise pointer with device data
    memset((char*)&devicedata, 0, sizeof(devicedata));

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

    /* Set filter for the proprietary Victron messages */
    CAN.init_Mask(0, 1, VICTRON_MASK);
    CAN.init_Filt(0, 1, VICTRON_FILTER);
    CAN.init_Filt(1, 1, VICTRON_FILTER);

    /* Set filter for PGN 127508 */
    CAN.init_Mask(1, 1, BATTERY_MASK);
    CAN.init_Filt(2, 1, BATTERY_FILTER);
    CAN.init_Filt(3, 1, BATTERY_FILTER);
    CAN.init_Filt(4, 1, BATTERY_FILTER);
    CAN.init_Filt(5, 1, BATTERY_FILTER);

    wdt_enable(WDTO_8S); // Watchdog, 8 seconds

    attachInterrupt(0, MCP2515_ISR, FALLING); // Install interrupt handler
}

void loop() {
    INT32U id = 0;
    // Uncomment these lines, and set the MCP2515 mode to loopback
    // if you want to test.
    //unsigned char msg[8] = { 0, 46, 10, 232, 3, 255, 255, 37};
    //unsigned char msg2[8] = { 1, 182, 6, 141, 2, 255, 255, 37};
    //CAN.sendMsgBuf(435295268, 1, 8, msg);
    //CAN.sendMsgBuf(435295268, 1, 8, msg2);
    //delay(1000);
    if(flagRecv) {
        wdt_reset(); // Pat the watchdog
        flagRecv = 0;

        // iterate over all pending messages
        while (CAN_MSGAVAIL == CAN.checkReceive()) {
            CAN.readMsgBufID(&id, &len, buf);

            if ((id & BATTERY_MASK) == BATTERY_FILTER) {
                if (len >= 8) {
                    unsigned long int v = (buf[2]*256+buf[1]);
                    unsigned long int i = (buf[4]*256+buf[3]);
                    if (buf[0] % 2){
                        // Assume odd numbered bank info is pv, other is
                        // battery side.
                        devicedata.pv_voltage = v;
                        devicedata.pv_current = i;
                    } else {
                        unsigned long int t = (buf[6]*256+buf[5]);
                        devicedata.checksum = 0;
                        serial_print("\r\nV\t"); serial_print(v*10);
                        serial_print("\r\nVPV\t"); serial_print(devicedata.pv_voltage*10);
                        serial_print("\r\nPPV\t"); serial_print((devicedata.pv_voltage * devicedata.pv_current)/1000);
                        serial_print("\r\nI\t"); serial_print(i*100);
                        serial_print("\r\nIL\t0");
                        serial_print("\r\nLOAD\tOFF");
                        serial_print("\r\nT\t"); serial_print((t+500)/1000); // Integer rounding
                        serial_print("\r\nH19\t"); serial_print(devicedata.yield_total);
                        serial_print("\r\nH20\t"); serial_print(devicedata.yield_today);
                        serial_print("\r\nH21\t"); serial_print(devicedata.max_today);
                        serial_print("\r\nH22\t"); serial_print(devicedata.yield_yesterday);
                        serial_print("\r\nH23\t"); serial_print(devicedata.max_yesterday);
                        serial_print("\r\nCS\t"); serial_print(devicedata.device_state);
                        serial_print("\r\nChecksum\t");
                        Serial.print((char)(256 - devicedata.checksum));
                    }
                }

                // Send a request for the proprietary registers, ask for one
                // at a time, otherwise bus gets a little busy on the reception
                // side. Hard code our address as 0x40.
                unsigned char request_vregs[8] = {0x66, 0x99, 0x01, 0x00, vregs[count++], 0xED, 0xFF, 0xFF};
                CAN.sendMsgBuf(0x1cef0000 | ((id & 0xFF)<<8) | 0x40, 1, 8,
                    request_vregs);
                count %= sizeof(vregs);

            } else if ((id & VICTRON_MASK) == VICTRON_FILTER) {
                //unsigned long int vreg = (buf[3]<<8) + buf[2];
                unsigned long int vreg = buf[3];
                vreg = (vreg << 8) + buf[2];
                switch(vreg){
                    case 0xEDDC: // Total yield
                        devicedata.yield_total = extract_value(buf+4);
                        break;
                    case 0xEDD3: // Yield today
                        devicedata.yield_today = extract_value(buf+4);
                        break;
                    case 0xEDD2: // Max power today
                        devicedata.max_today = extract_value(buf+4);
                        break;
                    case 0xEDD1: // Yield yesterday
                        devicedata.yield_yesterday = extract_value(buf+4);
                        break;
                    case 0xEDD0: // Max power yesterday
                        devicedata.max_yesterday = extract_value(buf+4);
                        break;
                    case 0x0201: // Device state
                        devicedata.device_state = extract_value(buf+4);
                        break;
                }
            }
        }
    } else {
        delay(10); // TODO use sleep mode so we wake up on the next interrupt
    }
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

Modification to also accept proprietary responses
-------------------------------------------------
Sending proprietary message
priority = 7 (3 bits, 111)
two zero bits
EF = Addressable Single Frame, 8 bits
Target = 8 bits
Source = 8 bits
29 bits total

       pr     
mask = 111 11 11111111 00000000 00000000
     =   0x1F     0xFF     0x00     0x00 = 0x1fff0000
pack =   7 00       EF 00000000 00000000

MCP2515 has two receive buffers. Buffer0 has higher priority than Buffer1.
Buffer 0 has 1 mask and 2 filters.
Buffer 1 has 1 mask and 4 filters.

Plan:
Set buffer 0 to receive broadcasted proprietary messages

    mask = 0x1fffff00
  filter = 0x1cefff00 (7.0.0.EF.tg.src, and tg == ff, broadcast))

Set buffer 1 to receive pgn 127508
    mask = 0x1ffff00
  filter = (127508 << 8)


Proprietary messages we might care about:
102 153 1 2 0 0 0 0 (0x0201 == Device State, put on CS field)
102 153 188 237 0 0 0 0 (0xEDBC == Input Power)
102 153 0 2 4 0 0 0  (0x0200 == Device mode, 4 == off)
102 153 187 237 159 6 0 0 (0xEDBB == Input Voltage, 6*256+159 = 16.95V)
102 153 219 237 188 7 0 0 (0xEDDB == charger temp, 7*256+188 = 19.8 C)

We have to ask for these
102 153 2 1 0 3 2 0  (0x0102, firmware version, v3.02.00)
102 153 211 237 129 0 0 0 (0xEDD3, yield today, 1.29kwh)
102 153 220 237 24 109 1 0 (0xEDDC, user system yield, 934.64kwh)
102 153 209 237 34 1 0 0 (0xEDD1, yield yesterday, 2.9kwh)
102 153 210 237 85 2 0 0 (0xEDD2, max power today, 597W)
102 153 208 237 139 2 0 0 (0xEDD0, max power yesterday, 651W)

102 153 10 1 ... (0x010A, serial number)

*/
