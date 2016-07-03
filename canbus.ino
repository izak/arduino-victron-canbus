#include <stdlib.h>
#include <avr/wdt.h>
#include <SPI.h>
#include "mcp_can.h"

#define SWAP_UINT16(x) (((x) >> 8) | ((x) << 8))
#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))

#define CAN_ADDRESS 0x40 // Hard code our address
#define CAN_QUEUE_SIZE 8
#define SER_QUEUE_SIZE 16
#define BATTERY_MASK 0x1ffff00
#define BATTERY_FILTER (127508 << 8)

#define VICTRON_MASK 0x1fffff00
#define VICTRON_FILTER 0x1cefff00 //7.0.0.EF.tg=FF.src

/* Response to vedirect serial number will be:

serial = 'HQ1437WAFP8'
r = '070A0100'+''.join(["{:02X}".format(ord(x)) for x in serial])
s = sum([int(x, 16) for x in iter(
    lambda i=iter(r): "".join(islice(i, 2)), "")]) % 256
c = "{:02X}".format((0x55-s)%256)
r = r + c

# Verify
s = sum([int(x, 16) for x in iter(
    lambda i=iter(r): "".join(islice(i, 2)), "")]) % 256
s == 0x55

# Result
:70A0100485131343337574146503875
*/


/* On some shields, the CS pin should be set to 9 */
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

// Data structure for storing info that goes out as Text
typedef struct {
    unsigned long int yield_total;
    unsigned long int yield_today;
    unsigned long int max_today;
    unsigned long int yield_yesterday;
    unsigned long int max_yesterday;
    unsigned char device_state;
    unsigned long int pv_voltage;
    unsigned long int pv_current;
    uint8_t checksum;
} DeviceData;
DeviceData devicedata;

// Data structures for quering and caching vregs
unsigned char count = 0;
unsigned int vregs[] = {0x0201, 0xEDDC, 0xEDD3, 0xEDD2, 0xEDD1, 0xEDD0,
    0xEDBC, 0xEDEC, 0xED8D, 0xEDBB, 0xEDDB, 0xEDF0};
typedef struct {
    unsigned int vreg;
    unsigned long value;
} VRegCache;
VRegCache vreg_cache[sizeof(vregs)/2];

// Data structure for queueing CAN messages
typedef struct {
    INT32U id;
    unsigned char len;
    unsigned char buf[8];
} CanMessage;
CanMessage can_data[CAN_QUEUE_SIZE];
unsigned int can_head = 0;
unsigned int can_tail = 0;

// Variables for serial input side
String inputSerString = "";
typedef struct {
    unsigned int vreg;
} VeRequest;
VeRequest ser_data[SER_QUEUE_SIZE];
unsigned int ser_head = 0;
unsigned int ser_tail = 0;

VRegCache* get_vreg_cache(unsigned int vreg) {
    for (unsigned int i=0; i < sizeof(vregs)/2; i++) {
        if (vreg_cache[i].vreg == vreg) {
            return &vreg_cache[i];
        }
    }
    return NULL;
}

void set_vreg_cache(unsigned int vreg, unsigned long value) {
    VRegCache *ptr = get_vreg_cache(vreg);
    if (ptr) ptr->value = value;
}

unsigned long extract_value(unsigned char *b) {
    unsigned long int r = 0;
    r = b[3];
    r = (r << 8) + b[2];
    r = (r << 8) + b[1];
    r = (r << 8) + b[0];
    return r;
}

uint8_t vetext_checksum(const char *msg) {
    uint8_t chr = 0;
    for (int i=0; msg[i]!='\0'; i++) {
        chr = (chr + msg[i]);
    }
    return chr;
}

size_t serial_print(const char* s) {
    devicedata.checksum += vetext_checksum(s);
    return Serial.print(s);
}

size_t serial_print(unsigned long i) {
    unsigned long t = i;
    // If the long int was to be converted to a string, it would end
    // up as chars between 0x30 and 0x39. So simply add them up in the
    // same manner.
    do {
        devicedata.checksum += (0x30 + (t % 10));
        t /= 10;
    } while (t>0);
    return Serial.print(i);
}

void vehex_reply(unsigned int vreg, unsigned char flags, unsigned long v) {
    unsigned int lo = vreg & 0xFF;
    unsigned int hi = (vreg & 0xFF00)>>8;
    unsigned char *p = (unsigned char*)&v;
    uint8_t ck = (0x55-(7+lo+hi+p[0]+p[1]+p[2]+p[3])) % 0x100;
    char out[16];

    if (v > 0xFFFF) {
        snprintf(out, sizeof(out), ":7%02X%02X%02X%04lX%02X\n",
            lo, hi, flags, SWAP_UINT32(v), ck);
    } else {
        snprintf(out, sizeof(out), ":7%02X%02X%02X%02lX%02X\n",
            lo, hi, flags, SWAP_UINT16(v), ck);
    }
    Serial.print(out);
}

void MCP2515_ISR() {
    CanMessage *rec;
    while (CAN_MSGAVAIL == CAN.checkReceive()) {
        rec = &can_data[can_tail++];
        can_tail %= CAN_QUEUE_SIZE;
        CAN.readMsgBufID(&rec->id, &rec->len, rec->buf);
    }
}

void serialEvent() {
    int idx;
    while (Serial.available()) {
        // get the new byte:
        char ch = (char)Serial.read();
        // add it to the inputSerString:
        inputSerString += ch;
        idx = inputSerString.indexOf('\n');
        if (idx >= 0) {
            if ((idx > 6) && (inputSerString.startsWith(":7"))) {
                char id[5];
                inputSerString.substring(2, 6).toCharArray(id, sizeof(id));
                unsigned long vreg = strtol(id+2, NULL, 16);
                id[2] = '\0';
                vreg = (vreg << 8) + strtol(id, NULL, 16);
                ser_data[ser_tail++].vreg = (vreg & 0xFFFF);
                ser_tail %= SER_QUEUE_SIZE;
            }
            inputSerString = "";
        }
    }
}

void setup() {
    // Initialise pointer with device data
    memset((char*)&devicedata, 0, sizeof(devicedata));

    // Initialise vreg_cache
    for (unsigned int i=0; i < sizeof(vregs)/2; i++) {
        vreg_cache[i].vreg = vregs[i];
        vreg_cache[i].value = 0;
    }

    Serial.begin(19200);
    Serial.println("Startup");

    while(1) {
        if (CAN_OK == CAN.begin(CAN_250KBPS)) {
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
    // Uncomment these lines, and set the MCP2515 mode to loopback
    // if you want to test.
    //unsigned char msg[8] = { 0, 46, 10, 232, 3, 255, 255, 37};
    //unsigned char msg2[8] = { 1, 182, 6, 141, 2, 255, 255, 37};
    //CAN.sendMsgBuf(435295268, 1, 8, msg);
    //CAN.sendMsgBuf(435295268, 1, 8, msg2);
    //delay(1000);

    wdt_reset(); // Pat the watchdog

    CanMessage rec;

    // iterate over all pending CAN messages
    while (can_head != can_tail) {
        rec = can_data[can_head++];
        can_head %= CAN_QUEUE_SIZE;

        if ((rec.id & BATTERY_MASK) == BATTERY_FILTER) {
            if (rec.len >= 8) {
                unsigned long int v = (rec.buf[2]*256+rec.buf[1]);
                unsigned long int i = (rec.buf[4]*256+rec.buf[3]);
                if (rec.buf[0] % 2) {
                    // Assume odd numbered bank info is pv, other is
                    // battery side.
                    devicedata.pv_voltage = v;
                    devicedata.pv_current = i;
                } else {
                    unsigned long int t = (rec.buf[6]*256+rec.buf[5]);
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
                    serial_print("\r\nFW\t2.03");
                    serial_print("\r\nPID\t0xA046");
                    serial_print("\r\nSER#\tHQ1437WAFP8");
                    serial_print("\r\nChecksum\t");
                    Serial.print((char)(256 - devicedata.checksum));
                }
            }

            // Send a request for the proprietary registers, ask for one
            // at a time, otherwise bus gets a little busy on the reception
            // side.
            unsigned char request_vreg[8] = {0x66, 0x99, 0x01, 0x00,
                (unsigned char)(vregs[count] & 0xFF),
                (unsigned char)((vregs[count] & 0xFF00) >> 8), 0xFF, 0xFF};
            CAN.sendMsgBuf(0x1cef0000 | ((rec.id & 0xFF)<<8) | CAN_ADDRESS, 1, 8,
                request_vreg);
            count = (count + 1) % sizeof(vregs)/2;

        } else if ((rec.id & VICTRON_MASK) == VICTRON_FILTER) {
            unsigned int vreg = rec.buf[3];
            vreg = (vreg << 8) + rec.buf[2];
            switch(vreg) {
                case 0xEDDC: // Total yield
                    devicedata.yield_total = extract_value(rec.buf+4);
                    break;
                case 0xEDD3: // Yield today
                    devicedata.yield_today = extract_value(rec.buf+4);
                    break;
                case 0xEDD2: // Max power today
                    devicedata.max_today = extract_value(rec.buf+4);
                    break;
                case 0xEDD1: // Yield yesterday
                    devicedata.yield_yesterday = extract_value(rec.buf+4);
                    break;
                case 0xEDD0: // Max power yesterday
                    devicedata.max_yesterday = extract_value(rec.buf+4);
                    break;
                case 0x0201: // Device state
                    devicedata.device_state = extract_value(rec.buf+4);
                    break;
            }
            set_vreg_cache(vreg, extract_value(rec.buf+4));
        }
    }

    // iterate over all pending serial messages
    VeRequest *req;
    while (ser_head != ser_tail) {
        req = &ser_data[ser_head++];
        ser_head %= SER_QUEUE_SIZE;

        switch (req->vreg) {
            case 0x010A:
                // HQ1437WAFP8
                Serial.print(":70A0100485131343337574146503875\n");
                break;
            default:
                VRegCache *ptr = get_vreg_cache(req->vreg);
                if (ptr){
                    vehex_reply(req->vreg, '\x00', ptr->value);
                } else {
                    vehex_reply(req->vreg, '\x01', 0);
                }
        }
    }

    if ((ser_head == ser_tail) && (can_head == can_tail)) {
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
102 153 187 237 159 6 0 0 (0xEDBB, 0xEDBB == Input Voltage, 6*256+159 = 16.95V)
102 153 219 237 188 7 0 0 (0xEDDB == charger temp, 7*256+188 = 19.8 C)

We have to ask for these
102 153 2 1 0 3 2 0  (0x0102, firmware version, v2.03.00)
102 153 211 237 129 0 0 0 (0xEDD3, yield today, 1.29kwh)
102 153 220 237 24 109 1 0 (0xEDDC, user system yield, 934.64kwh)
102 153 209 237 34 1 0 0 (0xEDD1, yield yesterday, 2.9kwh)
102 153 210 237 85 2 0 0 (0xEDD2, max power today, 597W)
102 153 208 237 139 2 0 0 (0xEDD0, max power yesterday, 651W)

102 153 10 1 ... (0x010A, serial number)

*/
