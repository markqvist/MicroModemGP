#include <stdlib.h>
#include <string.h>

#include "device.h"
#include "KISS.h"

static uint8_t serialBuffer[LLP_MAX_FRAME_LENGTH]; // Buffer for holding incoming serial data
LLPCtx *llpCtx;
Afsk *channel;
Serial *serial;
size_t frame_len;
bool IN_FRAME;
bool ESCAPE;

uint8_t command = CMD_UNKNOWN;
unsigned long custom_preamble = CONFIG_AFSK_PREAMBLE_LEN;
unsigned long custom_tail = CONFIG_AFSK_TRAILER_LEN;

unsigned long slotTime = 200;
uint8_t p = 255;
ticks_t timeout_ticks;

void kiss_init(LLPCtx *ctx, Afsk *afsk, Serial *ser) {
    llpCtx = ctx;
    serial = ser;
    channel = afsk;
}

void kiss_messageCallback(LLPCtx *ctx) {
    if (SERIAL_FRAMING == SERIAL_FRAMING_DIRECT) {
        for (unsigned i = 0; i < ctx->frame_len; i++) {
            uint8_t b = ctx->buf[i];
            fputc(b, &serial->uart0);
        }
    } else {
        fputc(FEND, &serial->uart0);
        fputc(0x00, &serial->uart0);
        for (unsigned i = 0; i < ctx->frame_len; i++) {
            uint8_t b = ctx->buf[i];
            if (b == FEND) {
                fputc(FESC, &serial->uart0);
                fputc(TFEND, &serial->uart0);
            } else if (b == FESC) {
                fputc(FESC, &serial->uart0);
                fputc(TFESC, &serial->uart0);
            } else {
                fputc(b, &serial->uart0);
            }
        }
        fputc(FEND, &serial->uart0);
    }
}

void kiss_csma(LLPCtx *ctx, uint8_t *buf, size_t len) {
    bool sent = false;
    while (!sent) {
        //puts("Waiting in CSMA");
        if(!channel->hdlc.receiving) {
            uint8_t tp = rand() & 0xFF;
            if (tp < p) {
                //llp_sendRaw(ctx, buf, len);
                llp_broadcast(ctx, buf, len);
                sent = true;
            } else {
                ticks_t start = timer_clock();
                long slot_ticks = ms_to_ticks(slotTime);
                while (timer_clock() - start < slot_ticks) {
                    cpu_relax();
                }
            }
        } else {
            while (!sent && channel->hdlc.receiving) {
                // Continously poll the modem for data
                // while waiting, so we don't overrun
                // receive buffers
                llp_poll(llpCtx);

                if (channel->status != 0) {
                    // If an overflow or other error
                    // occurs, we'll back off and drop
                    // this packet silently.
                    channel->status = 0;
                    sent = true;
                }
            }
        }

    }
    
}

void kiss_checkTimeout(bool force) {
    if (force || (IN_FRAME && timer_clock() - timeout_ticks > ms_to_ticks(TX_MAXWAIT))) {
        kiss_csma(llpCtx, serialBuffer, frame_len);
        IN_FRAME = false;
        frame_len = 0;
    }
    
}

void kiss_serialCallback(uint8_t sbyte) {
    #if SERIAL_FRAMING == SERIAL_FRAMING_DIRECT
        timeout_ticks = timer_clock();
        IN_FRAME = true;
        serialBuffer[frame_len++] = sbyte;
        if (frame_len >= LLP_MAX_FRAME_LENGTH) kiss_checkTimeout(true);
    #else
        if (IN_FRAME && sbyte == FEND && command == CMD_DATA) {
            IN_FRAME = false;
            kiss_csma(llpCtx, serialBuffer, frame_len);
        } else if (sbyte == FEND) {
            IN_FRAME = true;
            command = CMD_UNKNOWN;
            frame_len = 0;
        } else if (IN_FRAME && frame_len < LLP_MAX_FRAME_LENGTH) {
            // Have a look at the command byte first
            if (frame_len == 0 && command == CMD_UNKNOWN) {
                // MicroModem supports only one HDLC port, so we
                // strip off the port nibble of the command byte
                sbyte = sbyte & 0x0F;
                command = sbyte;
            } else if (command == CMD_DATA) {
                if (sbyte == FESC) {
                    ESCAPE = true;
                } else {
                    if (ESCAPE) {
                        if (sbyte == TFEND) sbyte = FEND;
                        if (sbyte == TFESC) sbyte = FESC;
                        ESCAPE = false;
                    }
                    serialBuffer[frame_len++] = sbyte;
                }
            } else if (command == CMD_TXDELAY) {
                custom_preamble = sbyte * 10UL;
            } else if (command == CMD_TXTAIL) {
                custom_tail = sbyte * 10;
            } else if (command == CMD_SLOTTIME) {
                slotTime = sbyte * 10;
            } else if (command == CMD_P) {
                p = sbyte;
            } 
            
        }
    #endif
}