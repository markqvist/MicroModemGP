#include <string.h>
#include <ctype.h>
#include "LLP.h"
#include "protocol/HDLC.h"
#include "util/CRC-CCIT.h"
#include "../hardware/AFSK.h"

#define DISABLE_INTERLEAVE false
#define PASSALL false
#define STRIP_HEADERS true

// The GET_BIT macro is used in the interleaver
// and deinterleaver to access single bits of a
// byte.
inline bool GET_BIT(uint8_t byte, int n) { return (byte & (1 << (8-n))) == (1 << (8-n)); }

// We need an indicator to tell us whether we
// should send a parity byte. This happens
// whenever two normal bytes of data has been
// sent. We also keep the last sent byte in
// memory because we need it to calculate the
// parity byte.
static bool sendParityBlock = false;
static uint8_t lastByte = 0x00;

LLPAddress broadcast_address;

void llp_decode(LLPCtx *ctx) {
    if (ctx->hook) {
        size_t length = ctx->frame_len;
        uint8_t *buffer = (uint8_t*)&ctx->buf;
        size_t padding = buffer[LLP_HEADER_SIZE-1];
        size_t address_size = 2*sizeof(LLPAddress);
        #if STRIP_HEADERS
            uint8_t strip_headers = 1;
        #else
            uint8_t strip_headers = 0;
        #endif
        size_t subtraction = (address_size + (LLP_HEADER_SIZE - address_size))*strip_headers + padding;
        ctx->frame_len = length - subtraction - LLP_CHECKSUM_SIZE;

        for (int i = 0; i < ctx->frame_len; i++) {
            #if STRIP_HEADERS
                buffer[i] = buffer[i+subtraction];
            #else
                if ( i >= LLP_HEADER_SIZE ) {
                    buffer[i] = buffer[i+padding];
                } else {
                    buffer[i] = buffer[i];
                }
            #endif
        }

        ctx->hook(ctx);
    }
}

void llp_poll(LLPCtx *ctx) {
    int c;
    
    #if DISABLE_INTERLEAVE
        while ((c = fgetc(ctx->ch)) != EOF) {
            if (!ctx->escape && c == HDLC_FLAG) {
                if (ctx->frame_len >= LLP_MIN_FRAME_LENGTH) {
                    if (PASSALL || ctx->crc_in == LLP_CRC_CORRECT) {
                        #if OPEN_SQUELCH == true
                            LED_RX_ON();
                        #endif
                        llp_decode(ctx);
                    }
                }
                ctx->sync = true;
                ctx->crc_in = CRC_CCIT_INIT_VAL;
                ctx->frame_len = 0;
                continue;
            }

            if (!ctx->escape && c == HDLC_RESET) {
                ctx->sync = false;
                continue;
            }

            if (!ctx->escape && c == LLP_ESC) {
                ctx->escape = true;
                continue;
            }

            if (ctx->sync) {
                if (ctx->frame_len < LLP_MAX_FRAME_LENGTH) {
                    ctx->buf[ctx->frame_len++] = c;
                    ctx->crc_in = update_crc_ccit(c, ctx->crc_in);
                } else {
                    ctx->sync = false;
                }
            }
            ctx->escape = false;
        }
    #else
        while ((c = fgetc(ctx->ch)) != EOF) {
            
            /////////////////////////////////////////////
            // Start of forward error correction block //
            /////////////////////////////////////////////
            if ((ctx->sync && (c != LLP_ESC )) || (ctx->sync && (ctx->escape && (c == LLP_ESC || c == HDLC_FLAG || c == HDLC_RESET)))) {
                // We have a byte, increment our read counter
                ctx->readLength++;

                // Check if we have read 12 bytes. If we
                // have, we should now have a block of two
                // data bytes and a parity byte. This block
                if (ctx->readLength % LLP_INTERLEAVE_SIZE == 0) {
                    // If the last character in the block
                    // looks like a control character, we
                    // need to set the escape indicator to
                    // false, since the next byte will be
                    // read immediately after the FEC
                    // routine, and thus, the normal reading
                    // code will not reset the indicator.
                    if (c == LLP_ESC || c == HDLC_FLAG || c == HDLC_RESET) ctx->escape = false;
                    
                    // The block is interleaved, so we will
                    // first put the received bytes in the
                    // deinterleaving buffer
                    for (int i = 1; i < LLP_INTERLEAVE_SIZE; i++) {
                        ctx->interleaveIn[i-1] = ctx->buf[ctx->frame_len-(LLP_INTERLEAVE_SIZE-i)];
                    }
                    ctx->interleaveIn[LLP_INTERLEAVE_SIZE-1] = c;

                    // We then deinterleave the block
                    llpDeinterleave(ctx);

                    // Adjust the packet length, since we will get
                    // parity bytes in the data buffer with block
                    // sizes larger than 3
                    ctx->frame_len -= LLP_INTERLEAVE_SIZE/3 - 1;

                    // For each 3-byte block in the deinterleaved
                    // bytes, we apply forward error correction
                    for (int i = 0; i < LLP_INTERLEAVE_SIZE; i+=3) {
                        // We now calculate a parity byte on the
                        // received data.

                        // Deinterleaved data bytes
                        uint8_t a = ctx->interleaveIn[i];
                        uint8_t b = ctx->interleaveIn[i+1];

                        // Deinterleaved parity byte
                        uint8_t p = ctx->interleaveIn[i+2];

                        ctx->calculatedParity = llpParityBlock(a, b);

                        // By XORing the calculated parity byte
                        // with the received parity byte, we get
                        // what is called the "syndrome". This
                        // number will tell us if we had any
                        // errors during transmission, and if so
                        // where they are. Using Hamming code, we
                        // can only detect single bit errors in a
                        // byte though, which is why we interleave
                        // the data, since most errors will usually
                        // occur in bursts of more than one bit.
                        // With 2 data byte interleaving we can
                        // correct 2 consecutive bit errors.
                        uint8_t syndrome = ctx->calculatedParity ^ p;
                        if (syndrome == 0x00) {
                            // If the syndrome equals 0, we either
                            // don't have any errors, or the error
                            // is unrecoverable, so we don't do
                            // anything
                        } else {
                            // If the syndrome is not equal to 0,
                            // there is a problem, and we will try
                            // to correct it. We first need to split
                            // the syndrome byte up into the two
                            // actual syndrome numbers, one for
                            // each data byte.
                            uint8_t syndromes[2];
                            syndromes[0] = syndrome & 0x0f;
                            syndromes[1] = (syndrome & 0xf0) >> 4;

                            // Then we look at each syndrome number
                            // to determine what bit in the data
                            // bytes to correct.
                            for (int i = 0; i < 2; i++) {
                                uint8_t s = syndromes[i];
                                uint8_t correction = 0x00;
                                if (s == 1 || s == 2 || s == 4 || s == 8) {
                                    // This signifies an error in the
                                    // parity block, so we actually
                                    // don't need any correction
                                    continue;
                                }

                                // The following determines what
                                // bit to correct according to
                                // the syndrome value.
                                if (s == 3)  correction = 0x01;
                                if (s == 5)  correction = 0x02;
                                if (s == 6)  correction = 0x04;
                                if (s == 7)  correction = 0x08;
                                if (s == 9)  correction = 0x10;
                                if (s == 10) correction = 0x20;
                                if (s == 11) correction = 0x40;
                                if (s == 12) correction = 0x80;

                                // And finally we apply the correction
                                if (i == 1) a ^= correction;
                                if (i == 0) b ^= correction;

                                // This is just for testing purposes.
                                // Nice to know when corrections were
                                // actually made.
                                if (s != 0) ctx->correctionsMade += 1;
                            }
                        }

                        // We now update the checksum of the packet
                        // with the deinterleaved and possibly
                        // corrected bytes.
                        
                        ctx->crc_in = update_crc_ccit(a, ctx->crc_in);
                        ctx->crc_in = update_crc_ccit(b, ctx->crc_in);

                        ctx->buf[ctx->frame_len-(LLP_DATA_BLOCK_SIZE)+((i/3)*2)] = a;
                        ctx->buf[ctx->frame_len-(LLP_DATA_BLOCK_SIZE-1)+((i/3)*2)] = b;
                    }

                    continue;
                }
            }
            /////////////////////////////////////////////
            // End of forward error correction block   //
            /////////////////////////////////////////////

            if (!ctx->escape && c == HDLC_FLAG) {
                if (ctx->frame_len >= LLP_MIN_FRAME_LENGTH) {
                    if (PASSALL || ctx->crc_in == LLP_CRC_CORRECT) {
                        #if OPEN_SQUELCH == true
                            LED_RX_ON();
                        #endif
                        llp_decode(ctx);
                    }
                }
                ctx->sync = true;
                ctx->crc_in = CRC_CCIT_INIT_VAL;
                ctx->frame_len = 0;
                ctx->readLength = 0;
                ctx->correctionsMade = 0;
                continue;
            }

            if (!ctx->escape && c == HDLC_RESET) {
                ctx->sync = false;
                continue;
            }

            if (!ctx->escape && c == LLP_ESC) {
                ctx->escape = true;
                continue;
            }

            if (ctx->sync) {
                if (ctx->frame_len < LLP_MAX_FRAME_LENGTH) {
                    ctx->buf[ctx->frame_len++] = c;
                } else {
                    ctx->sync = false;
                }
            }
            ctx->escape = false;
        }
    #endif
}

static void llp_putchar(LLPCtx *ctx, uint8_t c) {
    if (c == HDLC_FLAG || c == HDLC_RESET || c == LLP_ESC) fputc(LLP_ESC, ctx->ch);
    fputc(c, ctx->ch);
}

static void llp_sendchar(LLPCtx *ctx, uint8_t c) {
    llpInterleave(ctx, c);
    ctx->crc_out = update_crc_ccit(c, ctx->crc_out);

    if (sendParityBlock) {
        uint8_t p = llpParityBlock(lastByte, c);
        llpInterleave(ctx, p);
    }

    lastByte = c;
    sendParityBlock ^= true;
}

void llp_sendaddress(LLPCtx *ctx, LLPAddress *address) {
    llp_sendchar(ctx, address->network >> 8);
    llp_sendchar(ctx, address->network & 0xff);
    llp_sendchar(ctx, address->host >> 8);
    llp_sendchar(ctx, address->host & 0xff);
}

void llp_broadcast(LLPCtx *ctx, const void *_buf, size_t len) {
    llp_send(ctx, &broadcast_address, _buf, len);
}

void llp_send(LLPCtx *ctx, LLPAddress *dst, const void *_buf, size_t len) {
    ctx->interleaveCounter = 0;
    ctx->crc_out = CRC_CCIT_INIT_VAL;
    uint8_t *buffer = (uint8_t*)_buf;

    LLPHeader header;
    memset(&header, 0, sizeof(header));

    LLPAddress *localAddress = ctx->address;
    header.src.network = localAddress->network;
    header.src.host    = localAddress->host;
    header.dst.network = dst->network;
    header.dst.host    = dst->host;
    header.flags       = 0x00;
    header.padding     = (len + LLP_HEADER_SIZE + LLP_CRC_SIZE) % LLP_DATA_BLOCK_SIZE;
    if (header.padding != 0) {
        header.padding = LLP_DATA_BLOCK_SIZE - header.padding;
    }

    // Transmit the HDLC_FLAG to signify start of TX
    fputc(HDLC_FLAG, ctx->ch);

    // Transmit source & destination addresses
    llp_sendaddress(ctx, &header.src);
    llp_sendaddress(ctx, &header.dst);

    // Transmit header flags & padding count
    llp_sendchar(ctx, header.flags);
    llp_sendchar(ctx, header.padding);

    // Transmit padding
    while (header.padding--) {
        llp_sendchar(ctx, 0x00);
    }

    // Transmit payload
    while (len--) {
        llp_sendchar(ctx, *buffer++);
    }

    // Send CRC checksum
    uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
    uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
    llp_sendchar(ctx, crcl);
    llp_sendchar(ctx, crch);

    // And transmit a HDLC_FLAG to signify
    // end of the transmission.
    fputc(HDLC_FLAG, ctx->ch);
}

void llp_sendRaw(LLPCtx *ctx, const void *_buf, size_t len) {
    ctx->crc_out = CRC_CCIT_INIT_VAL;
    fputc(HDLC_FLAG, ctx->ch);
    const uint8_t *buf = (const uint8_t *)_buf;
    while (len--) llp_putchar(ctx, *buf++);

    uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
    uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
    llp_putchar(ctx, crcl);
    llp_putchar(ctx, crch);

    fputc(HDLC_FLAG, ctx->ch);
}

void llp_init(LLPCtx *ctx, LLPAddress *address, FILE *channel, llp_callback_t hook) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->ch = channel;
    ctx->hook = hook;
    ctx->address = address;
    ctx->crc_in = ctx->crc_out = CRC_CCIT_INIT_VAL;

    memset(&broadcast_address, 0, sizeof(broadcast_address));
    broadcast_address.network = LLP_ADDR_BROADCAST;
    broadcast_address.host    = LLP_ADDR_BROADCAST;
}

// This function calculates and returns a parity
// byte for two input bytes. The parity byte is
// used for correcting errors in the transmission.
// The error correction algorithm is a standard
// (12,8) Hamming code.
inline bool BIT(uint8_t byte, int n) { return ((byte & _BV(n-1))>>(n-1)); }
uint8_t llpParityBlock(uint8_t first, uint8_t other) {
    uint8_t parity = 0x00;

    parity =     ((BIT(first, 1) ^ BIT(first, 2) ^ BIT(first, 4) ^ BIT(first, 5) ^ BIT(first, 7))) +
                ((BIT(first, 1) ^ BIT(first, 3) ^ BIT(first, 4) ^ BIT(first, 6) ^ BIT(first, 7))<<1) +
                ((BIT(first, 2) ^ BIT(first, 3) ^ BIT(first, 4) ^ BIT(first, 8))<<2) +
                ((BIT(first, 5) ^ BIT(first, 6) ^ BIT(first, 7) ^ BIT(first, 8))<<3) +

                ((BIT(other, 1) ^ BIT(other, 2) ^ BIT(other, 4) ^ BIT(other, 5) ^ BIT(other, 7))<<4) +
                ((BIT(other, 1) ^ BIT(other, 3) ^ BIT(other, 4) ^ BIT(other, 6) ^ BIT(other, 7))<<5) +
                ((BIT(other, 2) ^ BIT(other, 3) ^ BIT(other, 4) ^ BIT(other, 8))<<6) +
                ((BIT(other, 5) ^ BIT(other, 6) ^ BIT(other, 7) ^ BIT(other, 8))<<7);

    return parity;
}

// Following is the functions responsible
// for interleaving and deinterleaving
// blocks of data. The interleaving table
// for 3-byte interleaving is also included.
// The table for 12-byte is much simpler,
// and should be inferable from looking
// at the function.

///////////////////////////////
// Interleave-table (3-byte) //
///////////////////////////////
//
// Non-interleaved:
// aaaaaaaa bbbbbbbb cccccccc
// 12345678 12345678 12345678
// M      L
// S      S
// B      B
//
// Interleaved:
// abcabcab cabcabca bcabcabc
// 11144477 22255578 63336688
//
///////////////////////////////

void llpInterleave(LLPCtx *ctx, uint8_t byte) {
    ctx->interleaveOut[ctx->interleaveCounter] = byte;
    ctx->interleaveCounter++;
    if (!DISABLE_INTERLEAVE) {
        if (ctx->interleaveCounter == LLP_INTERLEAVE_SIZE) {
            // We have the bytes we need for interleaving
            // in the buffer and are ready to interleave them.

            uint8_t a = (GET_BIT(ctx->interleaveOut[0], 1) << 7) +
                        (GET_BIT(ctx->interleaveOut[1], 1) << 6) +
                        (GET_BIT(ctx->interleaveOut[3], 1) << 5) +
                        (GET_BIT(ctx->interleaveOut[4], 1) << 4) +
                        (GET_BIT(ctx->interleaveOut[6], 1) << 3) +
                        (GET_BIT(ctx->interleaveOut[7], 1) << 2) +
                        (GET_BIT(ctx->interleaveOut[9], 1) << 1) +
                        (GET_BIT(ctx->interleaveOut[10],1));
            llp_putchar(ctx, a);

            uint8_t b = (GET_BIT(ctx->interleaveOut[0], 2) << 7) +
                        (GET_BIT(ctx->interleaveOut[1], 2) << 6) +
                        (GET_BIT(ctx->interleaveOut[3], 2) << 5) +
                        (GET_BIT(ctx->interleaveOut[4], 2) << 4) +
                        (GET_BIT(ctx->interleaveOut[6], 2) << 3) +
                        (GET_BIT(ctx->interleaveOut[7], 2) << 2) +
                        (GET_BIT(ctx->interleaveOut[9], 2) << 1) +
                        (GET_BIT(ctx->interleaveOut[10],2));
            llp_putchar(ctx, b);

            uint8_t c = (GET_BIT(ctx->interleaveOut[0], 3) << 7) +
                        (GET_BIT(ctx->interleaveOut[1], 3) << 6) +
                        (GET_BIT(ctx->interleaveOut[3], 3) << 5) +
                        (GET_BIT(ctx->interleaveOut[4], 3) << 4) +
                        (GET_BIT(ctx->interleaveOut[6], 3) << 3) +
                        (GET_BIT(ctx->interleaveOut[7], 3) << 2) +
                        (GET_BIT(ctx->interleaveOut[9], 3) << 1) +
                        (GET_BIT(ctx->interleaveOut[10],3));
            llp_putchar(ctx, c);

            uint8_t d = (GET_BIT(ctx->interleaveOut[0], 4) << 7) +
                        (GET_BIT(ctx->interleaveOut[1], 4) << 6) +
                        (GET_BIT(ctx->interleaveOut[3], 4) << 5) +
                        (GET_BIT(ctx->interleaveOut[4], 4) << 4) +
                        (GET_BIT(ctx->interleaveOut[6], 4) << 3) +
                        (GET_BIT(ctx->interleaveOut[7], 4) << 2) +
                        (GET_BIT(ctx->interleaveOut[9], 4) << 1) +
                        (GET_BIT(ctx->interleaveOut[10],4));
            llp_putchar(ctx, d);

            uint8_t e = (GET_BIT(ctx->interleaveOut[0], 5) << 7) +
                        (GET_BIT(ctx->interleaveOut[1], 5) << 6) +
                        (GET_BIT(ctx->interleaveOut[3], 5) << 5) +
                        (GET_BIT(ctx->interleaveOut[4], 5) << 4) +
                        (GET_BIT(ctx->interleaveOut[6], 5) << 3) +
                        (GET_BIT(ctx->interleaveOut[7], 5) << 2) +
                        (GET_BIT(ctx->interleaveOut[9], 5) << 1) +
                        (GET_BIT(ctx->interleaveOut[10],5));
            llp_putchar(ctx, e);

            uint8_t f = (GET_BIT(ctx->interleaveOut[0], 6) << 7) +
                        (GET_BIT(ctx->interleaveOut[1], 6) << 6) +
                        (GET_BIT(ctx->interleaveOut[3], 6) << 5) +
                        (GET_BIT(ctx->interleaveOut[4], 6) << 4) +
                        (GET_BIT(ctx->interleaveOut[6], 6) << 3) +
                        (GET_BIT(ctx->interleaveOut[7], 6) << 2) +
                        (GET_BIT(ctx->interleaveOut[9], 6) << 1) +
                        (GET_BIT(ctx->interleaveOut[10],6));
            llp_putchar(ctx, f);

            uint8_t g = (GET_BIT(ctx->interleaveOut[0], 7) << 7) +
                        (GET_BIT(ctx->interleaveOut[1], 7) << 6) +
                        (GET_BIT(ctx->interleaveOut[3], 7) << 5) +
                        (GET_BIT(ctx->interleaveOut[4], 7) << 4) +
                        (GET_BIT(ctx->interleaveOut[6], 7) << 3) +
                        (GET_BIT(ctx->interleaveOut[7], 7) << 2) +
                        (GET_BIT(ctx->interleaveOut[9], 7) << 1) +
                        (GET_BIT(ctx->interleaveOut[10],7));
            llp_putchar(ctx, g);

            uint8_t h = (GET_BIT(ctx->interleaveOut[0], 8) << 7) +
                        (GET_BIT(ctx->interleaveOut[1], 8) << 6) +
                        (GET_BIT(ctx->interleaveOut[3], 8) << 5) +
                        (GET_BIT(ctx->interleaveOut[4], 8) << 4) +
                        (GET_BIT(ctx->interleaveOut[6], 8) << 3) +
                        (GET_BIT(ctx->interleaveOut[7], 8) << 2) +
                        (GET_BIT(ctx->interleaveOut[9], 8) << 1) +
                        (GET_BIT(ctx->interleaveOut[10],8));
            llp_putchar(ctx, h);

            uint8_t p = (GET_BIT(ctx->interleaveOut[2], 1) << 7) +
                        (GET_BIT(ctx->interleaveOut[2], 5) << 6) +
                        (GET_BIT(ctx->interleaveOut[5], 1) << 5) +
                        (GET_BIT(ctx->interleaveOut[5], 5) << 4) +
                        (GET_BIT(ctx->interleaveOut[8], 1) << 3) +
                        (GET_BIT(ctx->interleaveOut[8], 5) << 2) +
                        (GET_BIT(ctx->interleaveOut[11],1) << 1) +
                        (GET_BIT(ctx->interleaveOut[11],5));
            llp_putchar(ctx, p);

            uint8_t q = (GET_BIT(ctx->interleaveOut[2], 2) << 7) +
                        (GET_BIT(ctx->interleaveOut[2], 6) << 6) +
                        (GET_BIT(ctx->interleaveOut[5], 2) << 5) +
                        (GET_BIT(ctx->interleaveOut[5], 6) << 4) +
                        (GET_BIT(ctx->interleaveOut[8], 2) << 3) +
                        (GET_BIT(ctx->interleaveOut[8], 6) << 2) +
                        (GET_BIT(ctx->interleaveOut[11],2) << 1) +
                        (GET_BIT(ctx->interleaveOut[11],6));
            llp_putchar(ctx, q);

            uint8_t s = (GET_BIT(ctx->interleaveOut[2], 3) << 7) +
                        (GET_BIT(ctx->interleaveOut[2], 7) << 6) +
                        (GET_BIT(ctx->interleaveOut[5], 3) << 5) +
                        (GET_BIT(ctx->interleaveOut[5], 7) << 4) +
                        (GET_BIT(ctx->interleaveOut[8], 3) << 3) +
                        (GET_BIT(ctx->interleaveOut[8], 7) << 2) +
                        (GET_BIT(ctx->interleaveOut[11],3) << 1) +
                        (GET_BIT(ctx->interleaveOut[11],7));
            llp_putchar(ctx, s);

            uint8_t t = (GET_BIT(ctx->interleaveOut[2], 4) << 7) +
                        (GET_BIT(ctx->interleaveOut[2], 8) << 6) +
                        (GET_BIT(ctx->interleaveOut[5], 4) << 5) +
                        (GET_BIT(ctx->interleaveOut[5], 8) << 4) +
                        (GET_BIT(ctx->interleaveOut[8], 4) << 3) +
                        (GET_BIT(ctx->interleaveOut[8], 8) << 2) +
                        (GET_BIT(ctx->interleaveOut[11],4) << 1) +
                        (GET_BIT(ctx->interleaveOut[11],8));
            llp_putchar(ctx, t);

            ctx->interleaveCounter = 0;
        }
    } else {
        if (ctx->interleaveCounter == LLP_INTERLEAVE_SIZE) {
            for (int i = 0; i < LLP_INTERLEAVE_SIZE; i++) {
                llp_putchar(ctx, ctx->interleaveOut[i]);
            }
            ctx->interleaveCounter = 0;
        }

    }
}


void llpDeinterleave(LLPCtx *ctx) {
    uint8_t a = (GET_BIT(ctx->interleaveIn[0], 1) << 7) +
                (GET_BIT(ctx->interleaveIn[1], 1) << 6) +
                (GET_BIT(ctx->interleaveIn[2], 1) << 5) +
                (GET_BIT(ctx->interleaveIn[3], 1) << 4) +
                (GET_BIT(ctx->interleaveIn[4], 1) << 3) +
                (GET_BIT(ctx->interleaveIn[5], 1) << 2) +
                (GET_BIT(ctx->interleaveIn[6], 1) << 1) +
                (GET_BIT(ctx->interleaveIn[7], 1));

    uint8_t b = (GET_BIT(ctx->interleaveIn[0], 2) << 7) +
                (GET_BIT(ctx->interleaveIn[1], 2) << 6) +
                (GET_BIT(ctx->interleaveIn[2], 2) << 5) +
                (GET_BIT(ctx->interleaveIn[3], 2) << 4) +
                (GET_BIT(ctx->interleaveIn[4], 2) << 3) +
                (GET_BIT(ctx->interleaveIn[5], 2) << 2) +
                (GET_BIT(ctx->interleaveIn[6], 2) << 1) +
                (GET_BIT(ctx->interleaveIn[7], 2));

    uint8_t p = (GET_BIT(ctx->interleaveIn[8], 1) << 7) +
                (GET_BIT(ctx->interleaveIn[9], 1) << 6) +
                (GET_BIT(ctx->interleaveIn[10],1) << 5) +
                (GET_BIT(ctx->interleaveIn[11],1) << 4) +
                (GET_BIT(ctx->interleaveIn[8], 2) << 3) +
                (GET_BIT(ctx->interleaveIn[9], 2) << 2) +
                (GET_BIT(ctx->interleaveIn[10],2) << 1) +
                (GET_BIT(ctx->interleaveIn[11],2));

    uint8_t c = (GET_BIT(ctx->interleaveIn[0], 3) << 7) +
                (GET_BIT(ctx->interleaveIn[1], 3) << 6) +
                (GET_BIT(ctx->interleaveIn[2], 3) << 5) +
                (GET_BIT(ctx->interleaveIn[3], 3) << 4) +
                (GET_BIT(ctx->interleaveIn[4], 3) << 3) +
                (GET_BIT(ctx->interleaveIn[5], 3) << 2) +
                (GET_BIT(ctx->interleaveIn[6], 3) << 1) +
                (GET_BIT(ctx->interleaveIn[7], 3));

    uint8_t d = (GET_BIT(ctx->interleaveIn[0], 4) << 7) +
                (GET_BIT(ctx->interleaveIn[1], 4) << 6) +
                (GET_BIT(ctx->interleaveIn[2], 4) << 5) +
                (GET_BIT(ctx->interleaveIn[3], 4) << 4) +
                (GET_BIT(ctx->interleaveIn[4], 4) << 3) +
                (GET_BIT(ctx->interleaveIn[5], 4) << 2) +
                (GET_BIT(ctx->interleaveIn[6], 4) << 1) +
                (GET_BIT(ctx->interleaveIn[7], 4));

    uint8_t q = (GET_BIT(ctx->interleaveIn[8], 3) << 7) +
                (GET_BIT(ctx->interleaveIn[9], 3) << 6) +
                (GET_BIT(ctx->interleaveIn[10],3) << 5) +
                (GET_BIT(ctx->interleaveIn[11],3) << 4) +
                (GET_BIT(ctx->interleaveIn[8], 4) << 3) +
                (GET_BIT(ctx->interleaveIn[9], 4) << 2) +
                (GET_BIT(ctx->interleaveIn[10],4) << 1) +
                (GET_BIT(ctx->interleaveIn[11],4));

    uint8_t e = (GET_BIT(ctx->interleaveIn[0], 5) << 7) +
                (GET_BIT(ctx->interleaveIn[1], 5) << 6) +
                (GET_BIT(ctx->interleaveIn[2], 5) << 5) +
                (GET_BIT(ctx->interleaveIn[3], 5) << 4) +
                (GET_BIT(ctx->interleaveIn[4], 5) << 3) +
                (GET_BIT(ctx->interleaveIn[5], 5) << 2) +
                (GET_BIT(ctx->interleaveIn[6], 5) << 1) +
                (GET_BIT(ctx->interleaveIn[7], 5));

    uint8_t f = (GET_BIT(ctx->interleaveIn[0], 6) << 7) +
                (GET_BIT(ctx->interleaveIn[1], 6) << 6) +
                (GET_BIT(ctx->interleaveIn[2], 6) << 5) +
                (GET_BIT(ctx->interleaveIn[3], 6) << 4) +
                (GET_BIT(ctx->interleaveIn[4], 6) << 3) +
                (GET_BIT(ctx->interleaveIn[5], 6) << 2) +
                (GET_BIT(ctx->interleaveIn[6], 6) << 1) +
                (GET_BIT(ctx->interleaveIn[7], 6));

    uint8_t s = (GET_BIT(ctx->interleaveIn[8], 5) << 7) +
                (GET_BIT(ctx->interleaveIn[9], 5) << 6) +
                (GET_BIT(ctx->interleaveIn[10],5) << 5) +
                (GET_BIT(ctx->interleaveIn[11],5) << 4) +
                (GET_BIT(ctx->interleaveIn[8], 6) << 3) +
                (GET_BIT(ctx->interleaveIn[9], 6) << 2) +
                (GET_BIT(ctx->interleaveIn[10],6) << 1) +
                (GET_BIT(ctx->interleaveIn[11],6));

    uint8_t g = (GET_BIT(ctx->interleaveIn[0], 7) << 7) +
                (GET_BIT(ctx->interleaveIn[1], 7) << 6) +
                (GET_BIT(ctx->interleaveIn[2], 7) << 5) +
                (GET_BIT(ctx->interleaveIn[3], 7) << 4) +
                (GET_BIT(ctx->interleaveIn[4], 7) << 3) +
                (GET_BIT(ctx->interleaveIn[5], 7) << 2) +
                (GET_BIT(ctx->interleaveIn[6], 7) << 1) +
                (GET_BIT(ctx->interleaveIn[7], 7));

    uint8_t h = (GET_BIT(ctx->interleaveIn[0], 8) << 7) +
                (GET_BIT(ctx->interleaveIn[1], 8) << 6) +
                (GET_BIT(ctx->interleaveIn[2], 8) << 5) +
                (GET_BIT(ctx->interleaveIn[3], 8) << 4) +
                (GET_BIT(ctx->interleaveIn[4], 8) << 3) +
                (GET_BIT(ctx->interleaveIn[5], 8) << 2) +
                (GET_BIT(ctx->interleaveIn[6], 8) << 1) +
                (GET_BIT(ctx->interleaveIn[7], 8));

    uint8_t t = (GET_BIT(ctx->interleaveIn[8], 7) << 7) +
                (GET_BIT(ctx->interleaveIn[9], 7) << 6) +
                (GET_BIT(ctx->interleaveIn[10],7) << 5) +
                (GET_BIT(ctx->interleaveIn[11],7) << 4) +
                (GET_BIT(ctx->interleaveIn[8], 8) << 3) +
                (GET_BIT(ctx->interleaveIn[9], 8) << 2) +
                (GET_BIT(ctx->interleaveIn[10],8) << 1) +
                (GET_BIT(ctx->interleaveIn[11],8));

    ctx->interleaveIn[0] =  a;
    ctx->interleaveIn[1] =  b;
    ctx->interleaveIn[2] =  p;
    ctx->interleaveIn[3] =  c;
    ctx->interleaveIn[4] =  d;
    ctx->interleaveIn[5] =  q;
    ctx->interleaveIn[6] =  e;
    ctx->interleaveIn[7] =  f;
    ctx->interleaveIn[8] =  s;
    ctx->interleaveIn[9] =  g;
    ctx->interleaveIn[10] = h;
    ctx->interleaveIn[11] = t;
}