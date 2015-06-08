MicroModemGP
==========

MicroModemGP is a general purpose firmware for [MicroModem](http://unsigned.io/micromodem). 

It supports both KISS mode serial connections, and direct serial connection without framing for easy communication with anything with a serial port.

You can buy a complete modem from [my shop](http://unsigned.io/shop), or you can build one yourself pretty easily. Take a look at the documentation in the [MicroModem](https://github.com/markqvist/MicroModem) repository for information and getting started guides!

## Some features

- Easily send and receive packets over mostly any radio
- Full modulation and demodulation in software
- Flexibility in how received packets are output over serial connection
- Can run with open squelch
- Supports KISS mode for use with programs on a host computer
- 12,8 Hamming-code forward error correction and 12-byte interleaving
- CRC checksum on packets ensure data integrity
- Supports packets with up to 792 bytes of data

## Serial connection settings

By default, the modem uses __9600 baud, 8N1__ serial. The baudrate can be modified in the "device.h" file.

## KISS mode or direct serial framing

You can configure whether to use KISS serial framing or direct serial framing in the "config.h" file.

When the modem is running in KISS mode, there's really not much more to it than connecting the modem to a computer, opening whatever program you want to use with it, and off you go.

You can also configure the modem for direct serial framing. If using direct serial framing, the firmware uses time-sensitive input, which means that it will buffer serial data as it comes in, and when it has received no data for a few milliseconds, it will start sending whatever it has received.

If you're manually typing things to the modem from a terminal, you should therefore set your serial terminal program to not send data for every keystroke, but only on new-line, or pressing send or whatever. You can also compile the firmware for KISS mode serial connection, if you have a host program using KISS.

## Other notes

The project has been implemented in your normal C with makefile style, and uses AVR Libc. The firmware is compatible with Arduino-based products, although it was not written in the Arduino IDE.

Visit [my site](http://unsigned.io) for questions, comments and other details.
