/**
 * burner.c: An EEPROM burner for AVR ATmega328p and similar microcontrollers.
 * Mainly intended for 27SF512 and 27SF256, but may work with many other
 * similar chips, such as: 27C512, 27C256, W27C512, 27C128, 27C64, and possibly
 * even 28C256. Probably reads any pin-compatible EEPROMs as well, but
 * burning may not work.
 *
 * In addition to the ATmega microcontroller and the EEPROM, this uses two
 * chained 74HC595 shift registers for the address bus, controlled via SPI.
 * The 16 outputs of the shift registers are all connected
 * to the address bus, but `A15` and `A9` also support high voltage (12-15 V).
 *
 * To support the high programming/erase/id voltage on pins 1 (`Vpp/A15`),
 * 22 (`!OE`), and 24 (`A9`) of the EEPROM, the following circuit is used
 * (three copies):
 *
 *                              +--------+--> Vpp supply
 *                              |        |
 *                            [22K]   +--+
 *                              |   |/e
 *                     +--[22K]-+---|  PN2907
 *                   |/             |\
 *  HV Ctl <--[22K]--|  PN2222        +
 *                   |\e              |
 *                    |               |
 *                    +------+->Gnd   |
 *                           |        |
 *                         [22K]      |
 *               Schottky    |        |
 *  LV Ctl <----->|----------+--------+------> EEPROM pin
 *
 *
 *  In the above schematic, `HV Ctl` is the output on the AVR that controls
 *  whether the programming voltage `Vpp` (e.g., 12V) is applied. Meanwhile
 *  `LV Ctl` is the regular logic level output (`!OE` on the AVR, `A9` and
 *  `A15` on the second 74HC595 as `QB` and `QH`, respectively). The
 *  Schottky diode protects the logic level output from `Vpp`, and the 22K
 *  resistor pulls the EEPROM pin low when neither `HV Ctl` or `LV Ctl` is
 *  on. The resistor values are largery arbitrary, I just used the same
 *  22K for simplicity.
 *
 *  The voltage `Vpp` is the same for every pin and is not controlled –
 *  personally I just used 12V input, which is then regulated to 5V. Many
 *  EEPROMs can be programmed with 12V but some require a higher voltage
 *  (such as 14V) for erasing. The way to do this is to just supply 14V
 *  as `Vpp`, erase the chip, and then change the `Vpp` supply to 12V
 *  for writing (e.g., for the W27C512 EEPROM).
 *
 *  The pin assignments can be seen (and edited) in this file (`burner.c`).
 *
 *  Use of the programmer can be done via a terminal program, it just needs
 *  to be set to the correct baud rate (`BAUD` defined at compile-time, e.g.,
 *  38400) and 8N1. The following commands (each followed by CRLF) are
 *  recognized:
 *
 *      * `I` – read the two-byte identifier from the chip. This applies
 *        the higher voltage to A9.
 *      * `C` – List known EEPROM chip types. Note that not every variant
 *        is listed separately, try using the closest match.
 *      * `C name` – Select the EEPROM chip type (e.g., `C 27SF512`).
 *      * `R` or `R $start $end` - Read the EEPROM. The optional `$start`
 *        and `$end` are addresses in hexadecimal (e.g., `$0 $7FFF`). The
 *        data is read from the chip and returned in Intel HEX (IHEX) format.
 *      * `E` – Erase the chip and verify that it is erased.
 *      * `V` – Verify that the chip is erased (all bytes `0xFF`). This
 *        can be used on a fresh chip to check whether there is a need to
 *        erase it.
 *      * `W` – Enter write mode. In write mode, further input is expected
 *        in the Intel Hex (IHEX) format. Each complete line of IHEX causes
 *        that line to be written to the EEPROM and read back to verify.
 *        The write mode is exited at the end of IHEX input. The burner may
 *        also exit write mode in case there are several errors. Each line
 *        of IHEX contains an address, so it is possible to write any part of
 *        the EEPROM by sending only the lines you wish to write. Retrying
 *        failed writes can thus also be done by just re-sending the same line.
 *      * `!` – Breaks out of write and read modes, otherwise does nothing.
 *      * `?` – Print the list of commands.
 *
 * During read and write mode, XON/XOFF software flow control is used.
 * In particular, you should not send more than one line of IHEX to write mode,
 * since the serial receive buffer may run out during the burn operation.
 * If using a normal terminal program, enabling software flow control should
 * work. Alternatively, a simple program on the computer side may be used to
 * explicitly wait for a response between each line of `W` – interactivity is
 * not really required, just send `C` to select the chip type and `W` followed
 * by the IHEX data one line at a time.
 *
 * In addition to the main commands listed above, several "debug" commands are
 * supported for testing purposes:
 *
 *      * `A $address` – Set the address bus to `$address` (e.g. `A $7FFF`).
 *      * `D` – Read the data bus (a single byte).
 *      * `O $byte` – Output `$byte` on the data bus (but do not burn it).
 *      * `T` – Select the EEPROM and read the data bus, leave the EEPROM
 *        selected (e.g., to probe with a multimeter).
 *      * `M` – Output the amount free RAM on the microcontroller.
 *
 * Copyright (c) 2019 Kimmo Kulovesi, https://arkku.com/
 * Provided with absolutely no warranty, use at your own risk only.
 * Use and distribute freely, mark modified copies as such.
 */

#ifndef F_CPU
/// Crystal frequency.
#define F_CPU   16000000UL
#endif

#include <stdio.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "kk_uart.h"
#include "kk_ihex_read.h"
#include "kk_ihex_write.h"

typedef uint16_t address_t;

#define HV_DDR          DDRD
#define HV_PORT         PORTD
#define HV_A9           ((uint8_t) (1U << 2))   // PD2
#define HV_OE           ((uint8_t) (1U << 3))   // PD3
#define HV_PIN1         ((uint8_t) (1U << 4))   // PD4

#define enable_A9_HV()          do { HV_PORT |= HV_A9; } while (0)
#define disable_A9_HV()         do { HV_PORT &= ~HV_A9; } while (0)

#define CE_DDR          DDRB
#define CE_PORT         PORTB
#define CE_PIN          ((uint8_t) (1U << 1))   // PB1 = EEPROM !CE

#define eeprom_select()         do { CE_PORT &= ~CE_PIN; _delay_us(1); } while (0)
#define eeprom_deselect()       do { CE_PORT |= CE_PIN; } while (0)

#define OE_DDR          DDRB
#define OE_PORT         PORTB
#define OE_PIN          ((uint8_t) (1U))        // PB0 = EEPROM !OE

#define eeprom_output_enable()  do { OE_PORT &= ~OE_PIN; } while (0)
#define eeprom_output_disable() do { OE_PORT |= OE_PIN; _delay_us(1); } while (0)

#define REG_CLEAR_DDR   DDRB
#define REG_CLEAR_PORT  PORTB
#define REG_CLEAR_PIN   ((uint8_t) (1U << 2))   // PB2 = !SCLR

#define REG_LATCH_DDR   DDRD
#define REG_LATCH_PORT  PORTD
#define REG_LATCH_PIN   ((uint8_t) (1U << 5))   // PD5 = RCK

#define DATA_LOW_DDR    DDRC
#define DATA_LOW_OUT    PORTC
#define DATA_LOW_IN     PINC
#define DATA_LOW_MASK   ((uint8_t) (0x3FU))     // 0b00111111
#define DATA_HIGH_DDR   DDRD
#define DATA_HIGH_OUT   PORTD
#define DATA_HIGH_IN    PIND
#define DATA_HIGH_MASK  ((uint8_t) (0xC0U))     // 0b11000000

#define SPI_DDR         DDRB
#define SPI_PORT        PORTB
#define MOSI_PIN        ((uint8_t) (1U << 3))   // PB3
#define MISO_PIN        ((uint8_t) (1U << 4))   // PB4
#define SCK_PIN         ((uint8_t) (1U << 5))   // PB5
#define spi_wait()      while (!(SPSR & (1U << SPIF)))
#define spi_putc(byte)  do { SPDR = (byte); spi_wait(); } while (0)

#define A14_MASK        ((address_t) (0x4000U))
#define A15_MASK        ((address_t) (0x8000U))

#define XON_CHAR        ((char) 0x11)
#define XOFF_CHAR       ((char) 0x13)
#define uart_xon()      uart_putc(XON_CHAR)
#define uart_xoff()     uart_putc(XOFF_CHAR)

static struct ihex_state ihex;

#ifdef IHEX_EXTERNAL_WRITE_BUFFER
static char buffer[IHEX_WRITE_BUFFER_LENGTH];
char *ihex_write_buffer = buffer;
#else
static char buffer[128];
#endif

/// Compare strings `a` and `b` case-insensitively.
static bool
streqi (const char *a, const char *b) {
    bool result;
    while ((result = (toupper(*a) == toupper(*b))) && *a) {
        ++a;
        ++b;
    }
    return result;
}

/// Delay approximately `delay` microseconds.
/// This is needed since `_delay_us` needs a compile-time constant.
static void
delay_us_approx (uint8_t delay) {
    if (delay >= 128) {
        _delay_us(128);
        delay -= 128;
    }
    if (delay >= 64) {
        _delay_us(64);
        delay -= 64;
    }
    if (delay >= 32) {
        _delay_us(32);
        delay -= 32;
    }
    if (delay >= 16) {
        _delay_us(16);
        delay -= 16;
    }
    if (delay >= 8) {
        _delay_us(8);
        delay -= 8;
    }
    if (delay >= 2) {
        _delay_us(2);
    } else if (delay) {
        _delay_us(1);
    }
}

/// Delay approximately `ms` milliseconds.
static void
delay_ms (uint_fast8_t ms) {
    while (ms >= 8) {
        _delay_ms(8);
        ms -= 8;
    }
    if (ms >= 4) {
        _delay_ms(4);
        ms -= 4;
    }
    if (ms >= 2) {
        _delay_ms(2);
        ms -= 2;
    }
    if (ms) {
        _delay_ms(1);
    }
}

/// Send a word to the SPI shift registers.
static void
spi_send (const address_t word) {
    // MSB first
    spi_putc(word >> 8);
    spi_putc(word & 0xFFU);
}

/// The configuration of a chip type.
typedef struct chip_config {
    const char *name;
    address_t max_address;
    uint8_t  write_pulse_us;
    unsigned pin_27_is_write_enable :1;
    unsigned oe_is_Vpp              :1;
    unsigned pin_1_is_Vpp           :1;
    unsigned is_28c_device          :1;
} chip_config_t;

/// Known chip types.
static const chip_config_t chips[] = {
    {
        .name = "27C512",
        .max_address = 0xFFFFU,
        .oe_is_Vpp = 1,
        .write_pulse_us = 100,
    },
    {
        .name = "27SF512",
        .max_address = 0xFFFFU,
        .oe_is_Vpp = 1,
        .write_pulse_us = 20,
    },
    {
        .name = "27C256",
        .max_address = 0x7FFFU,
        .pin_1_is_Vpp = 1,
        .write_pulse_us = 100
    },
    {
        .name = "27SF256",
        .max_address = 0x7FFFU,
        .pin_1_is_Vpp = 1,
        .write_pulse_us = 20
    },
    {
        .name = "28C256",
        .max_address = 0x7FFFU,
        .pin_27_is_write_enable = 1,
        .is_28c_device = 1,
        .write_pulse_us = 0
    },
    {
        .name = "27C128",
        .max_address = 0x3FFFU,
        .pin_27_is_write_enable = 1,
        .write_pulse_us = 100
    },
    {
        .name = "27C64",
        .max_address = 0x1FFFU,
        .pin_27_is_write_enable = 1,
        .write_pulse_us = 100
    }
};

/// The selected chip type.
static const chip_config_t *chip = chips;

/// Enable the programming voltage (Vpp) for `chip`.
static void
enable_Vpp (void) {
    if (chip->pin_1_is_Vpp) {
        HV_PORT &= ~HV_OE;
        HV_PORT |= HV_PIN1;
    } else if (chip->oe_is_Vpp) {
        HV_PORT &= ~HV_PIN1;
        HV_PORT |= HV_OE;
    } else {
        HV_PORT &= ~(HV_PIN1 | HV_OE);
    }
}

/// Disable the programming voltage.
static void
disable_Vpp (void) {
    HV_PORT &= ~(HV_PIN1 | HV_OE);
}

/// Select a chip by `name`. Returns `true` if a match was found.
static bool
select_chip_by_name (const char *name) {
    eeprom_deselect();
    disable_Vpp();
    const int num_chips = (sizeof chips) / (sizeof chips[0]);
    for (int i = 0; i < num_chips; ++i) {
        if (streqi(name, chips[i].name)) {
            chip = &chips[i];
            (void) fprintf_P(uart, PSTR("! Chip: %s\n"), chip->name);
            return true;
        }
    }
    (void) fprintf_P(uart, PSTR("Error: Unknown chip: %s\n"), name);
    return false;
}

/// Select the chip type by identifier.
/// Manufacturer is in the high byte, chip type in low byte.
/// Returns `true` if a match was found.
///
/// Note: Don't trust this list.
static bool
select_chip_by_id (const uint16_t identifier) {
    switch (identifier) {
    case 0xBFA4:
        return select_chip_by_name("27SF512");
    case 0xBFA3:
        return select_chip_by_name("27SF256");
    case 0x290D:
    case 0xDA08:
    case 0x0191:
    case 0x1F0D:
    case 0x1C83:
    case 0x8F85:
    case 0x9785:
    case 0xD591:
    case 0x89FD:
        return select_chip_by_name("27C512");
    case 0x298C:
    case 0x8F04:
    case 0x0110:
    case 0x1E8C:
    case 0x208D:
    case 0xD510:
        return select_chip_by_name("27C256");
    case 0x2983:
        return select_chip_by_name("27C128");
    case 0x2902:
        return select_chip_by_name("27C64");
    default:
        return false;
    }
}

static void
list_known_chips (void) {
    const int num_chips = (sizeof chips) / (sizeof chips[0]);
    for (int i = 0; i < num_chips; ++i) {
        (void) fprintf_P(uart, PSTR("! %-10s $%04X%s\n"), chips[i].name, chips[i].max_address, (chip == &chips[i]) ? " *" : "");
    }
}

/// Are we currently writing?
static bool is_writing = false;
/// Enable writing to the EEPROM.
static void
begin_write (void) {
    eeprom_output_disable();
    is_writing = true;
    DATA_LOW_DDR |= DATA_LOW_MASK;
    DATA_HIGH_DDR |= DATA_HIGH_MASK;
}

/// End writing to the EEPROM.
static void
end_write (void) {
    is_writing = false;
    DATA_LOW_DDR &= ~DATA_LOW_MASK;
    DATA_LOW_OUT &= ~DATA_LOW_MASK;
    DATA_HIGH_DDR &= ~DATA_HIGH_MASK;
    DATA_HIGH_OUT &= ~DATA_HIGH_MASK;
}

/// Begin reading from the EEPROM.
static void
begin_read (void) {
    end_write();
    eeprom_output_enable();
}

/// End reading from the EEPROM.
static void
end_read (void) {
    eeprom_output_disable();
}

/// Input a byte from the EEPROM.
static uint8_t
eeprom_input (void) {
    const uint8_t low = DATA_LOW_IN;
    const uint8_t high = DATA_HIGH_IN;
    return (high & DATA_HIGH_MASK) | (low & DATA_LOW_MASK);
}

/// Output a byte to the EEPROM.
static void
eeprom_output (const uint8_t byte) {
    DATA_LOW_OUT &= ~DATA_LOW_MASK;
    DATA_HIGH_OUT &= ~DATA_HIGH_MASK;
    DATA_LOW_OUT |= byte & DATA_LOW_MASK;
    DATA_HIGH_OUT |= byte & DATA_HIGH_MASK;
}

/// Set `address` on the bus, reformatting as needed for the
/// selected `chip` type. On some chips this also sets the
/// write enable state (if it is on pin 27).
static void
set_address (address_t address) {
    REG_CLEAR_PORT |= REG_CLEAR_PIN;
    REG_LATCH_PORT &= ~REG_LATCH_PIN;

    address &= chip->max_address;

    if (chip->pin_27_is_write_enable) {
        // Pin 1 is A14 (instead of A15)
        if (address & A14_MASK) {
            address |= A15_MASK;
            if (is_writing) {
                address &= ~A14_MASK;
            }
        } else if (!is_writing) {
            // Pin 27 is !WE (instead of A14)
            address |= A14_MASK;
        }
    }

    spi_send(address);

    REG_LATCH_PORT |= REG_LATCH_PIN;
    _delay_us(1);
    REG_LATCH_PORT &= ~REG_LATCH_PIN;
}

static bool
eeprom_verify_erased (void) {
    begin_read();
    address_t address = 0;
    do {
        set_address(address);
        eeprom_select();
        ++address;
        const uint8_t verification = eeprom_input();
        eeprom_deselect();
        if (verification != 0xFFU) {
            --address;
            (void) fprintf_P(uart, PSTR("Error: Verify $%04lX not empty\n"), (unsigned long) address);
            return false;
        }
    } while (address <= chip->max_address && address > 0);
    return true;
}

/// Erase the EEPROM chip.
static bool
eeprom_erase (void) {
    eeprom_deselect();
    begin_read();
    if (chip->is_28c_device) {
        set_address(0);
        HV_PORT |= HV_OE;
        is_writing = true;
        _delay_us(1);
        eeprom_select();
        set_address(0);
        delay_ms(20);
    } else {
        enable_A9_HV();
        enable_Vpp();
        _delay_us(1);
        eeprom_select();
        delay_ms(100);
    }
    eeprom_deselect();
    disable_Vpp();
    disable_A9_HV();

    delay_ms(100);

    if (!eeprom_verify_erased()) {
        uart_puts_P(PSTR("Error: Erase failed\n"));
        return false;
    }
    return true;
}

/// Read two id bytes from the EEPROM by setting A9 to high voltage.
static unsigned
eeprom_read_id (void) {
    unsigned id = 0;
    address_t address = 0;
    const address_t max_address = 1;

    eeprom_deselect();
    begin_read();
    enable_A9_HV();
    delay_ms(10);
    do {
        set_address(address);
        eeprom_select();
        ++address;
        id <<= 8;
        id |= eeprom_input();
        eeprom_deselect();
    } while (address <= max_address);
    disable_A9_HV();
    end_read();

    return id;
}

/// Read a single byte from the EEPROM (at whatever address is set).
static uint8_t
eeprom_read_byte (void) {
    begin_read();
    eeprom_select();
    uint8_t byte = eeprom_input();
    eeprom_deselect();
    end_read();
    return byte;
}

static bool is_paused = false;

typedef enum {
    FLOW_NONE = 0,
    FLOW_BREAK,
    FLOW_READ,
    FLOW_PAUSE = -1,
} flow_control_t;

/// Check for flow control (XON, XOFF, break). Also permits `!` to break.
/// When this returns, the next read is either `EOF` or a non-space
/// non-flow-control character.
static flow_control_t
uart_check_flow_control (void) {
    int c;
check_flow_control:
    c = uart_peekc();
    if (c == EOF) {
        return is_paused ? FLOW_PAUSE : FLOW_NONE;
    }
    switch ((char) c) {
    case '!':   // fallthrough
    case '\033': // fallthrough
    case '\0':
        (void) uart_getc();
        return FLOW_BREAK;
    case ' ':   // fallthrough
    case '\n':  // fallthrough
    case '\t':
        (void) uart_getc(); // Consume space and newlines
        goto check_flow_control;
    case XOFF_CHAR:
        (void) uart_getc();
        is_paused = true;
        goto check_flow_control;
    case XON_CHAR:
        (void) uart_getc();
        is_paused = false;
        goto check_flow_control;
    default:
        // For sake of simplicity, don't allow pause while also sending.
        // (Otherwise we would need to check for XON/XOFF everywhere,
        // in which case it should go in the UART receive interrupt.)
        is_paused = false;
        return FLOW_READ;
    }
}

/// Read the EEPROM from `address` to `end_address` (inclusive).
/// The read data is sent over the UART as IHEX.
static void
eeprom_read_ihex (address_t address, const address_t end_address) {
    ihex_init(&ihex);
    ihex_write_at_address(&ihex, address);

    eeprom_deselect();
    begin_read();
    do {
        flow_control_t status;
        do {
            status = uart_check_flow_control();
            if (status == FLOW_BREAK) {
                uart_puts_P(PSTR("! Read aborted\n"));
                goto abort_reading;
            }
        } while (status == FLOW_PAUSE);

        set_address(address);
        eeprom_select();
        ++address;
        uint8_t byte = eeprom_input();
        eeprom_deselect();
        ihex_write_byte(&ihex, byte);
    } while (address <= end_address && address > 0);
abort_reading:
    end_read();
    ihex_end_write(&ihex);
    uart_puts_P(PSTR("! Read end\n"));
}

static bool
eeprom_write (address_t address, const uint_fast8_t length, const uint8_t data[static length]) {
    bool success = true;
    uint_fast8_t written = 0;
    const uint_fast8_t chip_retry_count = chip->is_28c_device ? 128 : 16;
    uint_fast8_t retries_remaining = chip_retry_count;
    eeprom_deselect();
    while (written < length) {
        const uint8_t byte = data[written];

        begin_write();
        enable_Vpp();
        eeprom_output(byte);

        if (chip->pin_27_is_write_enable) {
            is_writing = false;
            set_address((address_t) address);
            eeprom_select();
            is_writing = true;
            set_address((address_t) address);
        } else {
            set_address((address_t) address);
            eeprom_select();
        }

        if (chip->write_pulse_us) {
            delay_us_approx(chip->write_pulse_us);
        }
        eeprom_deselect();
        if (chip->oe_is_Vpp) {
            disable_Vpp();
        }
        end_write();

        begin_read();
        set_address((address_t) address);

        eeprom_select();
        const uint8_t verification = eeprom_input();
        eeprom_deselect();
        if (verification == byte || !retries_remaining) {
            if (verification != byte) {
                if (success) {
                    (void) fprintf_P(uart, PSTR("Error: Write $%04lX failed\n"), (unsigned long) address);
                    success = false;
                }
            }
            ++address;
            ++written;
            retries_remaining = chip_retry_count;
        } else {
            --retries_remaining;
        }
        end_read();
    }
    return success;
}

static inline void
setup (void) {
    // Disable interrupts during setup
    cli();

    // Serial port
    uart_init(BAUD, UART_MODE_8N1);

    // High voltage control
    HV_DDR |= HV_PIN1 | HV_OE | HV_A9;
    disable_Vpp();
    disable_A9_HV();

    // EEPROM control
    CE_DDR |= CE_PIN;
    eeprom_deselect();
    OE_DDR |= OE_PIN;
    begin_read();
    end_read();

    // Shift register control
    REG_LATCH_DDR |= REG_LATCH_PIN;
    REG_LATCH_PORT &= ~REG_LATCH_PIN;
    REG_CLEAR_DDR |= REG_CLEAR_PIN;
    REG_CLEAR_PORT |= REG_CLEAR_PIN;

    // SPI (shift register data)
    SPI_DDR |= SCK_PIN | MOSI_PIN;
    SPI_DDR &= ~MISO_PIN;
    SPI_PORT &= ~(SCK_PIN | MOSI_PIN);
    SPCR = _BV(SPE) | _BV(MSTR);

    ihex_init(&ihex);

    // Enable interrupts
    sei();
}

static bool in_ihex_input = false;
static unsigned write_error_count = 0;
static address_t lowest_failed_address = 0xFFFFU;

static void
start_ihex_input (void) {
    write_error_count = 0;
    lowest_failed_address = 0xFFFFU;
    in_ihex_input = true;
    ihex_init(&ihex);
    begin_write();
    uart_puts_P(PSTR("! Write IHEX\n"));
    uart_xon();
}

static void
end_ihex_input (void) {
    disable_Vpp();
    end_write();
    uart_xon();

    if (write_error_count) {
        (void) fprintf_P(uart, PSTR("! Write had %u errors (first at $%04lX)\n"),
                         (unsigned) write_error_count, (unsigned long) lowest_failed_address);
    }
    in_ihex_input = false;
    uart_puts_P(PSTR("! Write end\n"));

    // Consume any further IHEX lines still in buffer
    do {
        int c = uart_peekc();
        if (c == ':' || c == '\n') {
            (void) uart_consume_line();
        }
    } while (uart_peekc() == ':');
}

int
main (void) {
    setup();

    unsigned counter = 0;
    for (;;) {
        ++counter;

        flow_control_t status;
        do {
            status = uart_check_flow_control();
            if (status == FLOW_BREAK) {
                if (in_ihex_input) {
                    uart_puts_P(PSTR("! Write aborted\n"));
                    end_ihex_input();
                } else {
                    // Break outside of IHEX is a NOP
                }
            }
        } while (status == FLOW_PAUSE);

        int c;

        if (status == FLOW_NONE || (c = uart_peekc()) == EOF) { // Idle
            if (!in_ihex_input && (counter & 0xFFU) == 0) {
                // Send periodic XON to ensure the other end is not blocked
                uart_xon();
            }
            _delay_ms(100);
            continue;
        }
        
        if (in_ihex_input) {
            // Write mode: input is IHEX
            int count = uart_getline(sizeof buffer, buffer);
            if (count > 0) {
                ihex_read_bytes(&ihex, buffer, count);
            }
            continue;
        }

        c = uart_getword(sizeof buffer, buffer);
        if (c != 1) {
            // All the commands are a single character.
            if (c > 0) {
            unknown_command:
                (void) fprintf_P(uart, PSTR("? Unknown command: \"%s\"\n"), buffer);
            } else {
                uart_puts("?\n");
            }
            (void) uart_consume_line();
            continue;
        }

        c = toupper(buffer[0]);
        switch ((char) c) {
        case 'C': { // Chip type
            if (uart_getword(sizeof buffer, buffer) > 0) {
                if (select_chip_by_name(buffer)) {
                    break;
                }
            }
            list_known_chips();
            break;
        }
        case 'R': { // Read
            unsigned long address = 0;
            unsigned long end_address = chip->max_address;
            int_fast8_t success = 1;

            address = uart_getlong(0, &success);
            if (!success) {
                if (!uart_consume_line()) {
                    uart_puts_P(PSTR("? R $start $end\n"));
                    break;
                }
                address = 0;
            } else {
                if (address > chip->max_address) {
                    (void) fprintf_P(uart, PSTR("Error: Address $%04lX > $%04lX\n"), address, (unsigned long) chip->max_address);
                    break;
                }

                success = 0;
                end_address = uart_getlong(0, &success);
                if (!success) {
                    if (!uart_consume_line()) {
                        uart_puts_P(PSTR("? R $start $end\n"));
                        break;
                    }
                }
            }
            if (!success || end_address > chip->max_address) {
                end_address = chip->max_address;
            }
            if (address > end_address) {
                    (void) fprintf_P(uart, PSTR("Error: Address $%04lX > $%04lX\n"), address, (unsigned long) end_address);
                break;
            }
            (void) fprintf(uart, PSTR("! Read IHEX $%04lX - $%04lX\n"), (unsigned long) address, (unsigned long) end_address);
            eeprom_read_ihex(address, end_address);
            break;
        }
        case 'E': { // Erase
            if (eeprom_erase()) {
                uart_puts_P(PSTR("! Erase ok\n"));
            }
            break;
        }
        case 'W': { // Write
            start_ihex_input();
            break;
        }
        case 'I': { // Identify
            uint16_t identifier = eeprom_read_id();
            (void) fprintf_P(uart, PSTR("! Id: 0x%04X\n"), (unsigned) identifier);
            (void) select_chip_by_id(identifier);
            break;
        }
        case 'V': { // Verify that the EEPROM is erased
            if (eeprom_verify_erased()) {
                uart_puts_P(PSTR("! Verify ok\n"));
            }
            break;
        }
        case 'A': { // Set address
            int_fast8_t success = 0;
            long address = uart_getlong(0, &success);
            if (!success) {
                goto unknown_command;
            }
            (void) fprintf_P(uart, PSTR("! Address $%04lX (%ld)\n"), address & 0xFFFF, address);
            set_address(address);
            break;
        }
        case 'T': // Test mode (leave EEPROM selected in read mode)
            eeprom_deselect();
            disable_Vpp();
            begin_read();
            eeprom_select();
            // fallthrough
        case 'D': { // Read one data byte at the current address
            uint8_t byte = (c == 'D') ? eeprom_read_byte() : eeprom_input();
            (void) fprintf_P(uart, PSTR("! Read $%02X (%u)\n"), byte, byte);
            break;
        }
        case 'O': { // Output data (for testing, EEPROM is not written)
            int_fast8_t success = 0;
            uint8_t data = uart_getlong(0, &success);
            if (!success) {
                goto unknown_command;
            }
            eeprom_deselect();
            disable_Vpp();
            begin_write();
            eeprom_output(data);
            (void) fprintf_P(uart, PSTR("! Out $%02X (%u)\n"), (unsigned) data, (unsigned) data);
            break;
        }
        case 'M': { // Memory status (of the microcontroller RAM)
            extern int __heap_start, *__brkval;
            int free_bytes;
            free_bytes = ((int) &free_bytes) - (__brkval ? (int) __brkval : (int) &__heap_start);
            (void) fprintf_P(uart, PSTR("! %d bytes RAM free\n"), free_bytes);
            break;
        }
        case '!': // NOP
            break;
        case 'H': // fallthrough
        case '?': // Help
            (void) fprintf_P(uart, PSTR("? %c: %s\n"), 'C', "Chip");
            (void) fprintf_P(uart, PSTR("? %c: %s\n"), 'I', "Id");
            (void) fprintf_P(uart, PSTR("? %c: %s\n"), 'E', "Erase");
            (void) fprintf_P(uart, PSTR("? %c: %s\n"), 'V', "Verify");
            (void) fprintf_P(uart, PSTR("? %c: %s\n"), 'R', "Read");
            uart_puts_P(PSTR("? R $start $end\n"));
            (void) fprintf_P(uart, PSTR("? %c: %s\n"), 'W', "Write");
            uart_puts_P(PSTR("?    Send IHEX after 'W' to write\n"));
            uart_puts_P(PSTR("? !: Abort read or write\n"));
            break;
        default:
            goto unknown_command;
        }
    }
}

void
ihex_flush_buffer (struct ihex_state *ihex, char *ascii, char *eptr) {
    *eptr = '\0';
    uart_puts(ascii);
}

ihex_bool_t
ihex_data_read (struct ihex_state *ihex,
                ihex_record_type_t type,
                ihex_bool_t checksum_error) {
    if (type == IHEX_END_OF_FILE_RECORD) {
        end_ihex_input();
        return !checksum_error;
    }
    if (type != IHEX_DATA_RECORD) {
        // Ignore non-data records
        return !checksum_error;
    }

    unsigned long address = (unsigned long) IHEX_LINEAR_ADDRESS(ihex);
    if (checksum_error) {
        (void) fprintf_P(uart, PSTR("Error: Invalid checksum at $%04lX\n"), address);
        goto write_error;
    } else if (address > chip->max_address) {
        (void) fprintf_P(uart, PSTR("Error: Address $%04lX > $%04lX\n"),
                         address, (unsigned long) chip->max_address);
        end_ihex_input();
    } else {
        uart_xoff();

        uint_fast8_t length = ihex->length;
        if ((address + (length - 1)) > chip->max_address) {
            (void) fprintf_P(uart, PSTR("Error: Address $%04lX > $%04lX\n"),
                             address + (length - 1), (unsigned long) chip->max_address);
            length = (chip->max_address - address) + 1;
        }

        if (eeprom_write((address_t) address, length, ihex->data)) {
            (void) fprintf_P(uart, PSTR("! $%04lX - $%04lX\n"),
                             address, address + (length - 1));
        } else {
        write_error:
            if (!write_error_count || address < lowest_failed_address) {
                lowest_failed_address = address;
            }
            ++write_error_count;
            if (write_error_count >= 64) {
                uart_puts_P(PSTR("! Excess errors\n"));
                uart_puts_P(PSTR("! Write aborted\n"));
                end_ihex_input();
            }
        }

        uart_xon();
    }

    return !checksum_error;
}
