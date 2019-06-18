# KKBurner

An EEPROM burner for AVR ATmega328p and similar microcontrollers.
Mainly intended for 27SF512 and 27SF256, but may work with many other
similar chips, such as: 27C512, 27C256, W27C512, 27C128, 27C64, and
possibly even 28C256. Probably reads any pin-compatible EEPROMs as
well, but burning them may or may not work.

This is a random DIY project for my very limited personal use, and
might not even be worth the time to build (given that you can buy
superior devices quite cheaply). However, it was an interesting
hobby project, and it works for me. Maybe someone else will also
find this (or parts of it) useful in some way.

~ [Kimmo Kulovesi](https://arkku.com/), 2019-06-16

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

I used a 14.318 MHz crystal for the microcontroller since I happened
to have it around, but 14.746 MHz would be better for the UART. You
can also use 16 MHz and 20 MHz. In any case, specify `F_CPU` as an
argument to `make`.

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
`make burn` and its fuses set with `make fuses`.

## Usage

Use of the programmer can be done via a terminal program, it just needs
to be set to the correct baud rate (`BAUD` defined at compile-time, e.g.,
57600) and 8N1. The following commands (each followed by CRLF) are
recognized:

* `I` – **Identify**: read the two-byte identifier from the chip. This
  applies higher voltage to A9, so beware using on random chips.
* `C` – List known **chip types**. Note that not every variant
  is listed separately, try using the closest match.
* `C name` – Choose **chip type** (e.g., `C 27SF512`).
* `R` – **Read** the entire EEPROM in Intel HEX (IHEX) format.
* `R $start $end` – **Read** from the address `start` to the address
  `end` (inclusive). The addresses may be given in hexadecimal with `$`
  or `0x` prefix (e.g., `R $0000 $1FFF`), or decimal (no prefix).
* `E` – **Erase** the chip and verify that it now reads blank. This
  must be done before writing to a chip that has previously been
  written (new chips tend to be already blank).
* `V` – **Verify** that the chip is empty (all bytes read `$FF`). This
  can be used on a fresh chip to check whether there is a need to
  erase it.
* `W` – **Write** enable. In write mode, further input is expected
  in the Intel Hex (IHEX) format. Each complete line of IHEX causes
  that line to be written to the EEPROM immediately. Write mode is
  exited when the IHEX end of file is sent. The burner may
  also exit write mode in case there are several errors.
* `!` – **Breaks** out of write and read modes, otherwise does nothing.
* `?` – **Help**  – print the list of commands.

In addition to the main commands listed above, several "debug" commands are
supported for testing purposes:

* `A $address` – Set the **address** bus to `$address` (e.g. `A $7FFF`).
* `D` – Read the **data** bus (a single byte).
* `O $byte` – **Output** `$byte` on the data bus (but do not burn it).
* `T` – **Test** mode: Select the EEPROM and read the data bus, and
  leave the EEPROM selected with output enabled (e.g., to probe with
  a multimeter). The command `A` can be used while staying in this mode.
* `M` – Output the amount free **memory** (RAM) on the microcontroller.

## Reading

To read a pin-compatible EEPROM, choose the nearest matching chip type
and use the `R $start $end` to read the desired address range. Don't read
beyond the end of the chip, since there may be other functionality
associated with the `A14` and/or `A15` address lines. (If the chip type
is selected correctly, reading won't go past the end.)

During reading, you may use software flow control (XON/XOFF) to pause/
resume the output. Sending a single exclamation mark `!` aborts the read.

## Burning

Before you can write ("burn") an EEPROM, it must first be erased. Erasing
can bo done with the `E` command, but the chip type must first be
correctly selected and the appropriate erase voltage must be present as
the `Vpp` (some chips need 14V to erase while using 12V for identifying
and programming, check the datasheet). New EEPROMs may already be blank,
which can be tested by issuing the `V` (verify) command. A blank EEPROM
reads as all bits set (`$FF`).

The write mode is entered by issuing the `W` command. In write mode, any
valid Intel HEX line received is immediately written to the EEPROM and
verified by reading it back. The microcontroller does not have enough RAM
to buffer previous lines, so the final verification must be done by
reading back the entire EEPROM and comparing the data on the host
computer.

Since each IHEX line specifies an address, it is possible to write
selectively to any part of the EEPROM – simply send only those lines of
IHEX that you wish to write. Retrying the same line is also as simple as
sending it again. The burner prevents you from exceeding the size of the
selected chip type, i.e., addresses will _not_ wrap around at the end.
It is even possible to write the same EEPROM in parts, as long as you
avoid writing different data to already-written areas (writing the same
data is harmless). The `bin2ihex` tool of my `kk_ihex` library can
easily convert binary files to IHEX starting from an arbitrary address.

Write mode may be exited by sending either the IHEX end of file line or a
single exclamation mark `!`.

Note that writing the EEPROM may be slower than the speed of the serial
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

    ./terminal.rb /dev/ttyUSB0 57600

The two arguments specify the serial device connected to the burner and
its baud rate, respectively.
