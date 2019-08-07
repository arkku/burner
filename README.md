# KKBurner

An EEPROM programmer and PROM reader using an AVR ATmega328p (or similar)
microcontroller. I have successfully replaced the ROMs in various Commodore
C64 and C128D computers and the Commodore 1571 floppy drive with new
EEPROMs burned using this device, as well as read the original ROMs of
these and other vintage computers.

This is a random DIY project for my very limited personal use, and
might not even be worth the time to build (given that you can buy
superior devices quite cheaply). However, it was an interesting
hobby project, and it works for me. Maybe someone else will also
find this (or parts of it) useful in some way. It can probably be
built on an Arduino, although care needs to be taken with pin
assignments and having the correct crystal frequency set. See the
`Makefile` for build options.

~ [Kimmo Kulovesi](https://arkku.com/), 2019-06-16

## Compatible EPROMs

The following EPROMs have been tested to be writeable with this device:

* 27SF512
* W27C512
* AT28C256
* AT29C256
* M27C256 (UV-erasable)

It is reasonable to assume that other 28-pin variants of the same will also
work, i.e., these are untested but extremely likely to be compatible:

* 27SF256
* 27C512
* 27C256
* 27C128
* 27C64
* W27E512
* W27E257

_Reading_ (but not writing) has also been tested on a number of vintage
28-pin ROMs from old computers and their peripherals.

Note that the 29C family of EEPROMs requires writing whole 64-byte pages
at a time, whereas only a single complete Intel HEX input line is buffered
by the programmer, and is flashed immediately upon being received. Normally
IHEX lines contain 32 bytes of data (when decoded), which means that every
64-byte page will be written twice (since it is split across two IHEX lines).
This is not harmful as such, but halves the write speed to these devices and
uses up twice the write cycles necessary. This can be avoided by encoding 64
bytes per IHEX line, e.g., as follows with `bin2ihex`:

    ihex/bin2ihex -b 64 <rom.bin >rom.hex

Note that the executable `bin2ihex` is not compiled by the EEPROM programmer's
`Makefile` (since it runs on the host computer), but you can simply type `make`
inside the `ihex` submodule's directory to build the `bin2ihex` and `ihex2bin`
utilities.

## Hardware

There is now a [schematic](burner_schematic.pdf) available, but the
following is an informal description of the circuit (before I figured
out how to draw schematics on a computer).

In addition to the ATmega microcontroller and the EEPROM, this uses two
chained 74HC595 shift registers for the address bus, controlled via SPI.
The 16 outputs of the shift registers are all connected to the address
bus, but `A15` and `A9` also support high voltage (12-15 V).

To support the high programming/erase/id voltage on pins 1 (`Vpp/A15`),
22 (`!OE`), and 24 (`A9`) of the EEPROM, the following circuit is used
(three copies):

                                 +--------+--> Vpp supply
                                 |        |
                               [22K]   +--+
                                 |   |/e
                        +--[22K]-+---|  PN2907 (PNP)
                      |/             |\
     HV Ctl <--[22K]--|  PN2222 (NPN)  +
                      |\e              |
                       |               |
                       +------+->Gnd   |
                              |        |
                            [22K]      |
                  Schottky    |        |
     LV Ctl <----->|----------+--------+------> EEPROM pin

In the above schematic, `HV Ctl` is the output on the AVR that controls
whether the programming voltage `Vpp` (e.g., 12V) is applied. Meanwhile
`LV Ctl` is the regular logic level output (`!OE` on the AVR, `A9` and
`A15` on the second 74HC595 as `QB` and `QH`, respectively). The
Schottky diode protects the logic level output from `Vpp`, and the 22K
pull-down resistor pulls the EEPROM pin low when both `HV Ctl` and
`LV Ctl` are low.

The voltage `Vpp` is the same for every pin and is not controlled –
personally I just used 12V input, which is then regulated to 5V using
a 7805. Many EEPROMs can be programmed with 12V but some require a
higher voltage (such as 14V) for erasing. The way to do this is to
supply 14V as `Vpp`, erase the chip, and then change the `Vpp`
supply to 12V for writing (e.g., for the W27C512 EEPROM). (The
erasing and programming need not be done during the same session,
you can write anytime to an erased chip.)

The pin assignments on the microcontroller can be seen in `burner.c`,
near the top of the file. For quick reference, they are:

* `PC0`–`PC5` – EEPROM data `D0`–`D5`
* `PD6`–`PD7` – EEPROM data `D6`–`D7`
* `PB0` – EEPROM `!OE` `LV Ctl` (logic level) output enable
* `PB1` – EEPROM `!CE` chip enable
* `PD2` – `A9` `HV Ctl` identify/erase voltage control
* `PD3` – `!OE` `HV Ctl` programming voltage control
* `PD4` – `A15` `HV Ctl` programming voltage control
* `PB3` – low 74HC595 `SI` (high 74HC595 `SI` is chained from the
  `QH'` output of the low 74HC595)
* `PB5` – both 74HC595 `SCK`
* `PD5` – both 74HC595 `RCK` latch
* `PB2` – both 74HC595 `!SCLR` (optional, can also just tie high)
* `PD0` – UART `RX`
* `PD1` – UART `TX`
* `PB4` – ISP header MISO (don't use for other purposes)

Various crystal oscillator frequencies are supported, but it is
crucial to set the matching `F_CPU` in the arguments to `make`,
and to select a baud rate that is reliable with that frequency.
For example, a 16 MHz crystal at 19200 baud would be:

    make F_CPU=16000000UL BAUD=19200

This (16 MHz, 19200 baud) is also the default since it has proven the
highest reliable speed with that crystal and cheap USB to serial
adapters. Using a 14.746 MHz crystal allows higher baud rates.

In addition to the mentioned components, you need some kind of serial
port chip or cable that uses TTL levels. Personally I used an FTDI chip
for an USB serial port connected to the `RX` and `TX` pins.

I also built mine with an LED connected to the `!CE` pin of the EEPROM
(it will blink during read and write), as well as an ISP header for
simple programming without having to use a bootloader (there is plenty
of space left on the microcontroller for one, though, e.g., you can
probably build this on an Arduino if you so wish).  You probably want
to socket the microcontroller anyway in case something goes wrong.
And obviously the EEPROM needs to be socketed to be useful (a ZIF
socket is preferable, but I used a high quality milled socket
instead, since I had it around).

## Software

The main program with the simple command parser is in `burner.c`,
and the UART (serial port) helper functions in `kk_uart.c`. The Intel
HEX reading and writing is implemented using my own library,
[kk_ihex](https://github.com/arkku/ihex). It is added as a Git submodule,
so you need to run:

    git submodule update --init

Otherwise everything can be built to `burner.hex` by just running
`make`. See the `Makefile` for configurable options (such as the crystal
frequency, `F_CPU`, and UART baud rate, `BAUD`). If you use the AVR
Dragon in ISP mode, the microcontroller can be programmed with just
`make burn` and its fuses set with `make fuses`. You may be able to
upload to an Arduino-compatible bootloader with something like:

    make F_CPU=16000000 BAUD=115200
    make upload BURNER=arduino PORT=/dev/ttyUSB0 BPS=115200

However, anything other than AVR Dragon is untested.

## Usage

Use of the programmer can be done via a terminal program, it just needs
to be set to the correct baud rate (`BAUD` defined at compile-time, e.g.,
19200) and 8N1. The following commands (each followed by CRLF) are
recognized:

* `I` – **Identify**: read the two-byte identifier from the chip. This
  applies higher voltage to A9, so beware using on random chips.
* `C` – List known **chip types**. Note that not every variant
  is listed separately, try using the closest match.
* `C name` – Choose **chip type** (e.g., `C 27SF512`).
* `R` – **Read** the entire ROM in Intel HEX (IHEX) format.
* `R $start $end` – **Read** from the address `start` to the address
  `end` (inclusive). The addresses may be given in hexadecimal with `$`
  or `0x` prefix (e.g., `R $0000 $1FFF`), or decimal (no prefix).
* `E` – **Erase** the chip and verify that it now reads blank. This
  must be done before writing to a chip that has previously been
  written (new chips tend to be already blank).
* `V` – **Verify** that the chip is empty (all bytes read `$FF`). This
  can be used on a fresh chip to check whether there is a need to
  erase it.
* `S` - Determine the **size** of data on the ROM. Displays the address
  range that contains unique data - any further data present is just
  cyclically repeating. This can be used to determine the size of an
  unknown ROM chip, although it may be incorrect in case the same data
  is written multiple times.
* `N` – Count the **number** of ones and zeroes in each bit position,
  as well as the distribution of distinct nybbles. This was inspired by
  the idea of observing the erase progress of an UV-erasable EPROM, but
  may also be used to detect faults in specific bit positions, as well
  as find out if there is a lot of blank space or a test pattern
  on the ROM.
* `W` – **Write** enable. In write mode, further input is expected
  in the Intel Hex (IHEX) format. Each complete line of IHEX causes
  that line to be written to the EEPROM immediately. Write mode is
  exited when the IHEX end of file is sent. The burner may
  also exit write mode in case there are several errors. If this is a
  28C device, write protection is disabled automatically.
* `P` – **Protect** the device by enabling software write protection.
  This is only supported on the 28C family of EEPROMs.
* `!` – **Breaks** out of write and read modes, otherwise does nothing.
* `?` – **Help**  – print the list of commands.

In addition to the main commands listed above, several "debug" commands are
supported for testing purposes:

* `A $address` – Set the **address** bus to `$address` (e.g. `A $7FFF`).
* `D` – Read the **data** bus (a single byte).
* `O $byte` – **Output** `$byte` on the data bus (but do not burn it).
* `T` – **Test** mode: Select the ROM and read the data bus, and
  leave the ROM selected with output enabled (e.g., to probe with
  a multimeter). The command `A` can be used while staying in this mode.
* `M` – Output the amount free **memory** (RAM) on the microcontroller.

## Reading

To read a pin-compatible ROM, choose the nearest matching chip type
and use the `R $start $end` to read the desired address range. Don't read
beyond the end of the chip, since there may be other functionality
associated with the `A14` and/or `A15` address lines. (If the chip type
is selected correctly, reading won't go past the end.)

During reading, you may use software flow control (XON/XOFF) to pause/
resume the output. Sending a single exclamation mark `!` aborts the read.

## Burning

Before you can write ("burn") a PROM, it must first be erased. Erasing
compatible EEPROM can bo done with the `E` command, but the chip type
must first be correctly selected and the appropriate erase voltage must
be present as the `Vpp` (some chips need 14V to erase while using 12V
for identifying and programming, check the datasheet). New PROMs may
already be blank, which can be tested by issuing the `V` (verify)
command. A blank EEPROM reads as all bits set (`$FF`).

The write mode is entered by issuing the `W` command. In write mode, any
valid Intel HEX line received is immediately written to the EEPROM and
verified by reading it back. The microcontroller does not have enough RAM
to buffer previous lines, so the final verification must be done by
reading back the entire EEPROM and comparing the data on the host
computer.

Since each IHEX line specifies an address, it is possible to write
selectively to any part of the PROM – simply send only those lines of
IHEX that you wish to write. Retrying the same line is also as simple as
sending it again. The burner prevents you from exceeding the size of the
selected chip type, i.e., addresses will _not_ wrap around at the end.
It is even possible to write the same PROM in parts, as long as you
avoid writing different data to already-written areas (writing the same
data is harmless). The `bin2ihex` tool of my `kk_ihex` library can
easily convert binary files to IHEX starting from an arbitrary address.

Write mode may be exited by sending either the IHEX end of file line or a
single exclamation mark `!`.

Note that writing the PROM may be slower than the speed of the serial
connection, and the microcontroller's UART receive buffer is very limited.
This means that the sender (you or your terminal) must throttle the speed
at which data is sent. Software flow control (XON/XOFF) may be used for
this, or you may simply send one line, wait for the response, and send
another. The latter approach also has the benefit that you can observe
errors as they happen, and is preferable.

### terminal.rb

A simple "terminal" program written in Ruby for Linux and other \*nix
systems is provided. It passes input and output directly to the burner,
but supports two additional commands to make reading and writing simpler:

* `write filename.hex` – Enters **write** mode and uploads the contents
  of the Intel HEX file given (`filename.hex`) one line at a time.
* `read filename.hex` or `read filename.hex $start $end` – Enters
  **read** mode and saves the received Intel HEX data into the file
  given (`filename.hex`), which will be overwritten. Optionally the
  start and end addresses may be given in hexadecimal (e.g., `read
  foo.hex $0 $1FFF`).

The commands remain otherwise the same as when directly connected to
the burner, i.e., use `C chip` to select the chip type, and the
command `E` to erase it before burning (if needed).

The usage is simply:

    ./terminal.rb /dev/ttyUSB0 19200

The two arguments specify the serial device connected to the burner and
its baud rate, respectively.
