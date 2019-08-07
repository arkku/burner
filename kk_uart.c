/**
 * kk_uart.c: A UART (serial port) library for AVR microcontrollers.
 * Provides an stdio-compatible `FILE *` handle to the UART, with
 * interrupt-driven and buffered receiving (but unbuffered transmitting).
 *
 * Copyright (c) 2019 Kimmo Kulovesi, https://arkku.com/
 * Provided with absolutely no warranty, use at your own risk only.
 * Use and distribute freely, mark modified copies as such.
 */
#include "kk_uart.h"

#ifndef F_CPU
#error F_CPU not defined
#endif

#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

static char rx_buffer[KK_UART_RECEIVE_BUFFER_SIZE];

typedef uint8_t buffer_index_t;

static volatile buffer_index_t rx_buffer_write = 0;
static volatile buffer_index_t rx_buffer_read = 0;

#define buffer_size ((unsigned int) (sizeof rx_buffer))

#define uart_has_data           ((UCSR0A & _BV(RXC0)) != 0)
#define uart_byte               UDR0
#define uart_wait_to_read()     loop_until_bit_is_set(UCSR0A, RXC0)
#define uart_wait_to_write()    loop_until_bit_is_set(UCSR0A, UDRE0);
#define uart_send(chr)          do { uart_wait_to_write(); uart_byte = (chr); } while (0)

#define UART_RX_PIN             ((uint8_t) 1U)
#define UART_TX_PIN             ((uint8_t) 2U)
#define UART_DDR                DDRD
#define UART_PORT               PORTD

#if KK_UART_RECEIVE_BUFFER_SIZE == 256
#define modulo_buffer_size(x)   ((uint8_t) (x))
#elif KK_UART_RECEIVE_BUFFER_SIZE > 256
#error KK_UART_RECEIVE_BUFFER_SIZE too large (max. 256)!
#else
#define modulo_buffer_size(x)   ((buffer_index_t) ((x) % buffer_size))
#endif

#if KK_UART_CONVERT_CRLF_IN_TO_LF != 0
static bool previous_was_converted_cr = false;
#define IS_END_OF_LINE(b)   ((b) == '\n')
#define consume_newline_after_cr(b) do { } while (0)
#else
#define IS_END_OF_LINE(b)   ((b) == '\r' || (b) == '\n')
#define consume_newline_after_cr(b) do { if ((b) == '\r') { consume_newline(); } } while (0)
/// Consume a single LF. Used to get rid of the LF of a CRLF pair after
/// the CR has already been consumed. This only tries for a limited time
/// to avoid blocking indefinitely in case the LF is lost or not sent.
static void
consume_newline (void) {
    uint_fast8_t attempts = 64;
    int c;
    do {
        c = uart_peekc();
        if (c == '\n') {
            (void) uart_getc();
            return;
        }
    } while (c == EOF && --attempts);
}
#endif /* KK_UART_CONVERT_CRLF_IN_TO_LF == 0 */

uint8_t
uart_bytes_available (void) {
    uint8_t unread_count = modulo_buffer_size(buffer_size + rx_buffer_write - rx_buffer_read);
#if KK_UART_CONVERT_CRLF_IN_TO_LF != 0
    if (unread_count && previous_was_converted_cr) {
        unread_count -= (rx_buffer[rx_buffer_read] == '\n') ? 1 : 0;
    }
#endif
    return unread_count;
}

void
uart_flush_unread (void) {
    rx_buffer_write = 0;
    rx_buffer_read = 0;
#if KK_UART_CONVERT_CRLF_IN_TO_LF != 0
    previous_was_converted_cr = false;
#endif
}

int
uart_getc (void) {
    char c;
    buffer_index_t pos;
#if KK_UART_CONVERT_CRLF_IN_TO_LF != 0
get_next_from_buffer:
#endif
    pos = rx_buffer_read;
    if (pos == rx_buffer_write) {
        return EOF;
    }
    c = rx_buffer[pos];
    rx_buffer_read = modulo_buffer_size(pos + 1);

#if KK_UART_CONVERT_CRLF_IN_TO_LF != 0
    if (previous_was_converted_cr) {
        previous_was_converted_cr = false;
        if (c == '\n') {
            goto get_next_from_buffer;
        }
    }
    if (c == '\r') {
        previous_was_converted_cr = true;
        c = '\n';
    }
#endif

    return c;
}

int
uart_getc_wait (void) {
    int c;
    do {
        c = uart_getc();
    } while (c == EOF); // TODO: Return EOF on break
    return c;
}

int
uart_peekc (void) {
#if KK_UART_CONVERT_CRLF_IN_TO_LF != 0
    buffer_index_t pos = rx_buffer_read;
    if (pos == rx_buffer_write) {
        return EOF;
    }
    char c = rx_buffer[pos];
    if (previous_was_converted_cr) {
        if (c == '\n') {
            pos = modulo_buffer_size(pos + 1);
            if (pos == rx_buffer_write) {
                return EOF;
            }
            c = rx_buffer[pos];
        }
    }
    return (c != '\r') ? c : '\n';
#else
    return (rx_buffer_read == rx_buffer_write) ? EOF : rx_buffer[rx_buffer_read];
#endif
}

int
uart_peekc_wait (void) {
    int c;
    do {
        c = uart_peekc();
    } while (c == EOF); // TODO: Return EOF on break (change callers here to handle it)
    return c;
}

void 
uart_putc (const char c) {
#if KK_UART_CONVERT_LF_OUT_TO_CRLF != 0
    if (c == '\n') {
        uart_send('\r');
    }
#endif
    uart_send(c);
}

void
uart_puts (const char * restrict str) {
    char c;
    while ((c = *str++)) {
        uart_putc(c);
    }
}

/// Write the string `pstr` from program space (`PSTR`) to the UART.
void
uart_puts_P (const char * restrict pstr) {
    char c;
    while ((c = pgm_read_byte(pstr++))) {
        uart_putc(c);
    }
}

int
uart_consume_line (void) {
    int result = 1;
    for (;;) {
        int c = uart_getc_wait();
        if (c == EOF) {
            return EOF;
        }
        switch ((char) c) {
        case ' ':
            break;
#if KK_UART_CONVERT_CRLF_IN_TO_LF == 0
        case '\r':
            consume_newline();
            // fallthrough
#endif
        case '\n':
            return result;
        case '\t':
            break;
        default:
            if (((uint8_t) c) >= 0x20) {
                // There were non-space, non-control characters
                result = 0;
            }
            break;
        }
    }
}

int
uart_consume_space_and_newlines (void) {
    int result = 0;

    do {
        char c = uart_peekc_wait();

        if (!(c == ' ' || IS_END_OF_LINE(c) || c == '\t')) {
            break;
        }
    } while ((result = uart_getc()) != EOF);

    return result;
}

int
uart_consume_space (void) {
    int result = 0;

    do {
        int c;
        do {
            c = uart_peekc();
        } while (c == EOF);
        if (!(c == ' ' || c == '\t')) {
            break;
        }
    } while ((result = uart_getc()) != EOF);

    return result;
}

int
uart_getline (int bufsize, char buf[static bufsize]) {
    int c;
    int count = 0;

    if (bufsize <= 0) {
        return -1;
    }
    --bufsize;

    while (count < bufsize) {
        c = uart_getc_wait();
        if (IS_END_OF_LINE(c) || c == EOF) {
            consume_newline_after_cr(c);
            break;
        }
        if (c == '\b' && count) {
            // Handle backspace inside the line
            --count;
        } else if (c >= 0x20 || c == '\t') {
            // Only take non-control characters
            buf[count++] = c;
        }
    }

    buf[count] = '\0';

    return count;
}

int
uart_getword (int bufsize, char buf[static bufsize]) {
    int count = 0;

    if (bufsize <= 0) {
        return -1;
    }
    --bufsize;

    while (count < bufsize) {
        char c = uart_peekc_wait(); // TODO: Return on break?

        if (c == ' ' || c == '\t') {
            if (count) {
                break;
            }
            // Skip leading space
        } else if (IS_END_OF_LINE(c)) {
            break;
        } else if (c == '\b' && count) {
            // Handle backspace inside the word
            --count;
        } else if (c >= 0x20) {
            // Don't record control characters
            buf[count++] = c;
        }
        (void) uart_getc();
    }

    buf[count] = '\0';

    return count;
}

long
uart_getlong (int_fast8_t base, int_fast8_t *success) {
    bool allow_base_prefix = false;
    bool negative = false;

    if (base <= 0) {
        allow_base_prefix = true;
        base = 10;
    }

    // Skip leading space
    (void) uart_consume_space();

    unsigned long result = 0;
    bool digits_read = false;
    for (;;) {
        char c = uart_peekc_wait();

        if (c >= '0' && c <= '9') {
            c -= '0';
            if (c < base) {
                result *= base;
                result += c;
                digits_read = true;
            } else {
                break; // Digit not valid for this base
            }
        } else if (c == ' ' || IS_END_OF_LINE(c) || c == '\t') {
            break;
        } else if (c < 0x20) { // Ignore control characters
            (void) uart_getc();
            continue;
        } else if (base > 10) {
            c &= ~32; // Convert lowercase to uppercase
            c -= 'A';
            c += 10;
            if (c >= 10 && c < base) {
                result *= base;
                result += c;
                digits_read = true;
            } else {
                break; // Digit not valid for this base
            }
        } else if (result == 0) {
            if (!digits_read) {
                if (c == '-') {
                    negative = !negative;
                } else if (c == '+') {
                    // No effect, but allow the sign
                } else if (allow_base_prefix) {
                    if (c == '$') {
                        base = 16;
                    } else if (c == '%') {
                        base = 2;
                    } else {
                        break;
                    }
                    allow_base_prefix = false;
                } else {
                    break;
                }
            } else if (allow_base_prefix) {
                if (c == 'x' || c == 'X') {
                    // 0x for hexadecimal
                    base = 16;
                } else if (c == 'b') {
                    // 0b for binary
                    base = 2;
                } else {
                    break;
                }
                allow_base_prefix = false;
            }
        }

        (void) uart_getc();
    }

    if (success) {
        *success = digits_read ? base : 0;
    }

    // Note: This allows overflow on purpose, it is implementation-defined here
    // and ok for AVR-GCC (which is the only target of this code). If porting,
    // you may wish to check overflow (compare to `LONG_MAX`) before casting.
    return negative ? -((long) result) : (long) result;
}

volatile uint8_t uart_last_rx_error = 0;

#ifdef USART_RX_vect
ISR(USART_RX_vect)
#else
ISR(USART0_RX_vect)
#endif
{
    uint8_t status = UCSR0A;
    char c = uart_byte;
    if (status & (_BV(FE0) | _BV(UPE0))) {
        uart_last_rx_error = status;
    } else {
        const buffer_index_t next_pos = modulo_buffer_size(rx_buffer_write + 1);
        if (next_pos != rx_buffer_read) {
            rx_buffer[rx_buffer_write] = c;
            rx_buffer_write = next_pos;
        } else {
            status |= _BV(DOR0);
        }
        if (status & _BV(DOR0)) {
            uart_last_rx_error = status;
        }
    }
}

void
uart_init (uint32_t baud, uart_mode_t mode) {
    UART_DDR |= UART_TX_PIN;
    UART_DDR &= ~UART_RX_PIN;
    UART_PORT |= UART_RX_PIN; // Enable pull-up for RX

    uint16_t speed = (((uint32_t) F_CPU >> 2) / baud - 1) >> 1;
    if (speed < 4096) {
        // Ok for 2X mode
        UCSR0A |= _BV(U2X0);
    } else {
        // Must use 1X mode
        UCSR0A &= ~(_BV(U2X0));
        speed = (((uint32_t) F_CPU >> 3) / baud - 1) >> 1;
    }
    UBRR0H = speed >> 8;
    UBRR0L = speed & 0xFFU;

    UCSR0C = (uint8_t) mode; // Mode

    UCSR0B = _BV(RXEN0) | _BV(TXEN0); // Enable
    UCSR0B |= _BV(RXCIE0);  // Receive interrupt enable
}

void
uart_disable (void) {
    UCSR0B = 0;
}

static int
getc_uart(FILE *file) {
    return uart_getc_wait();
}

static int
putc_uart(char c, FILE *file) {
    uart_putc(c);
    return 0;
}

/// The serial port UART device.
static FILE uart_fdev = FDEV_SETUP_STREAM((putc_uart), (getc_uart), _FDEV_SETUP_RW);

FILE *uart = &uart_fdev;
