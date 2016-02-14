# Convert Victron VE.CAN to VE.Direct.

This is code for the arduino that reads the broadcast messages from the
BlueSolar MPPT unit and converts them to a subset of the VE.Direct text
protocol used by the lower end MPPT controllers.

## Requirements

1. A canbus shield, such as [this one][shield] by [seeedstudio][seeed]. Try
[alice1101982][alice] on Ebay. If you build your own, consider using the TI
ISO1050 canbus transceiver for galvanic isolation.
2. The [canbus shield library][canbus-lib]. You need to change the CS pin
to 10 to work with this shield.
3. I use [Arduino-Makefile][arduino-make] for building, although it is easy
enough to build this in the arduino IDE.
4. Remember to terminate the other end of the bus inside your BlueSolar MPPT
controller. You will also need a 120 ohm terminator on the shield end, as not
all shields incorporate one (The elecfreaks one I used does not, but some do).

## Use

Once it's running, it simply sends voltage, current and power values over
the Arduino's serial link in the same text format as the lower-end MPPT charge
controllers. This is easier to parse and use in other hobbyist projects.

## How it works

It configures the MCP2515 can controller on the shield to filter for the PGN
127508, which contains all the information we need about our battery bank.
This information is broadcasted, so all we have to do is wait for it and
reformat the data.

## Useful resources

* The [canboat][canboat] project is absolutely indispensible for decoding
NMEA2000 canbus data.
* Also consider using [ttlappalainen][ttlappalainen]'s NMEA code for more
complex applications or for guidance.

## Purpose

I own a BlueSolar 150/70 MPPT controller, and I want to intgegrate it with
[Blue Lantern][bluelantern].

[shield]: http://www.seeedstudio.com/wiki/CAN-BUS_Shield
[seeed]: http://www.seeedstudio.com/
[alice]: http://www.ebay.com/usr/alice1101983
[canbus-lib]: https://github.com/Seeed-Studio/CAN_BUS_Shield
[arduino-make]: https://github.com/sudar/Arduino-Makefile
[canboat]: https://github.com/canboat/canboat
[ttlappalainen]: https://github.com/ttlappalainen
[bluelantern]: https://github.com/izak/ib.bluelantern
