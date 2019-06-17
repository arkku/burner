/**
 * kk_uart.h: A UART (serial port) library for AVR microcontrollers.
 * Provides an stdio-compatible `FILE *` handle to the UART, with
 * interrupt-driven and buffered receiving (but unbuffered transmitting).
 *
 * Copyright (c) 2019 Kimmo Kulovesi, https://arkku.com/
 * Provided with absolutely no warranty, use at your own risk only.
 * Use and distribute freely, mark modified copies as such.
 */
#ifndef KK_UART_H
#define KK_UART_H

#ifdef __cplusplus
#ifndef restrict
#define restrict
#endif
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef KK_UART_RECEIVE_BUFFER_SIZE
#define KK_UART_RECEIVE_BUFFER_SIZE 128
#endif

#ifndef KK_UART_CONVERT_LF_OUT_TO_CRLF
/// If this is non-zero, sending LF (`\n`) sends CRLF (`\r\n`) instead.
#define KK_UART_CONVERT_LF_OUT_TO_CRLF 0
#endif

#ifndef KK_UART_CONVERT_CRLF_IN_TO_LF
/// If this is non-zero, each received CR (`\r`) is converted to a LF (`\n`),
/// and the immediately following LF is skipped. The number of bytes available
/// does not account for this conversion beyond ensuring that a non-zero value
/// is not falsely returned when there are no bytes available.
#define KK_UART_CONVERT_CRLF_IN_TO_LF 0
#endif

typedef enum {
    UART_MODE_8N1 = 0x06,
    UART_MODE_7N1 = 0x04,
    UART_MODE_8N2 = 0x0E,
    UART_MODE_7N2 = 0x0C,

    UART_MODE_8E1 = 0x26,
    UART_MODE_7E1 = 0x24,
    UART_MODE_8E2 = 0x2E,
    UART_MODE_7E2 = 0x2C,

    UART_MODE_8O1 = 0x36,
    UART_MODE_7O1 = 0x34,
    UART_MODE_8O2 = 0x3E,
    UART_MODE_7O2 = 0x3C
} uart_mode_t;

/// An stdio-compatible `FILE` handle for the UART.
extern FILE *uart;

/// Set up the UART at `baud` in `mode`. Note that this does _not_ enable
/// interrupts globally (e.g., with `sei()`), and the interrupt-driven receiving
/// doesn't work without enabling them.
void uart_init(uint32_t baud, uart_mode_t mode);

/// Get a character from the UART. Returns `EOF` if none available. Does not
/// wait for characters to arrive (see `uart_getc_wait()` and `uart_peekc()`).
int uart_getc(void);

/// Get a character from the UART, waiting for one to become available.
/// Note that this may block indefinitely - you may use `uart_bytes_available`
/// to check whether the next call may block.
///
/// Currently this never returns `EOF` (i.e., blocks forever), but this may
/// change in the future to detect error conditions on the UART.
int uart_getc_wait(void);

/// The number of bytes available for reading from the UART.
///
/// Note: If the conversion of incoming CRLF to LF is enabled, this count does
/// not take that conversion into account, other than ensuring that a non-zero
/// value is not falsely returned, i.e., if this returns 2 or greater, there
/// may be fewer bytes actually readable.
uint8_t uart_bytes_available(void);

/// Peek at the next character from the UART. Returs `EOF` is none available.
/// For a non-`EOF` result, calling `uart_getc()` immediately returns the same.
int uart_peekc(void);

/// Write the character `c` to the UART.
void uart_putc(const char c);

/// Write the string `str` to the UART.
void uart_puts(const char *str);

/// Get at most one less than `bufsize` non-control characters up to the next
/// end of line from the UART into `buf`. The end of line is not stored in
/// `buf` but is consumed from the UART. Returns the number of characters
/// stored in `buf`, not counting the terminating NUL.
///
/// If incoming CRLFs are not being converted to LFs, it is non-deterministic
/// whether the LF following a CR is also consumed or not. (Usually it is.)
/// See `KK_UART_CONVERT_CRLF_IN_TO_LF`.
int uart_getline(int bufsize, char buf[static bufsize]);

/// Get at most one less than `bufsize` non-control characters up to the next
/// whitespace from the UART into `buf`, skipping any leading spaces before
/// the first non-space character. Trailing whitespace is not consumed.
/// Returns the number of characters characters stored (not counting the NUL).
///
/// This can be used together with `uart_getlong` and `uart_consume_line` to
/// implement a field-by-field parser without an intermediate line buffer.
int uart_getword(int bufsize, char buf[static bufsize]);

/// Parse an integer in `base` from the UART. If `base` is zero, it
/// defaults to decimal, but permits `$` or `0x` prefix for hexadecimal,
/// and `%` or `0b` prefix for binary. If the `success` pointer is not
/// `NULL`, it will be set to zero on failure and to the (actual) base
/// used on success. Note that a plain `0x` or `0b` prefix is also
/// considered a successful reading of zero (with the `x` or `b` being
/// consumed). A plain sign (`-` or `+`) or prefix (`$` or `%`) is also
/// consumed by this, even on failure. Leading spaces and tabs are
/// consumed and ignored.
///
/// This can be used together with `uart_getword` and `uart_consume_line` to
/// implement a field-by-field parser without an intermediate line buffer.
long uart_getlong(int_fast8_t base, int_fast8_t *success);

/// Consume characters up to and including the next end of line.
/// Returns positive if only whitespace and control characters were consumed,
/// 0 if other characters were also present (to check for extra arguments),
/// and `EOF` on error.
///
/// If incoming CRLFs are not being converted to LFs, it is non-deterministic
/// whether the LF following a CR is also consumed or not. (Usually it is.)
/// See `KK_UART_CONVERT_CRLF_IN_TO_LF`.
int uart_consume_line(void);

/// Consume spaces and tabs until the next other character (including end
/// of line) is encountered. Returns positive if any characters were
/// consumed, `EOF` on error, and 0 otherwise.
int uart_consume_space(void);

/// Consume spaces, tabs and newlines until the first other character is
/// encountered. Returns positive if any characters were consumed, `EOF`
/// on error, and 0 otherwise.
int uart_consume_space_and_newlines(void);

/// Discard any unread bytes currently pending on the UART. The effect of
/// this largely unpredictable due to buffering, and should generally only
/// be used while the UART is turned off (e.g., when switching modes).
void uart_flush_unread(void);

/// Disable the UART.
void uart_disable(void);

#ifdef __cplusplus
}
#endif
#endif
