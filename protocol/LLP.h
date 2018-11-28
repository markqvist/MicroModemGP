#ifndef PROTOCOL_LLP_H
#define PROTOCOL_LLP_H

#include <stdio.h>
#include <stdbool.h>
#include "device.h"

#define LLP_ADDR_BROADCAST 0xFFFF

#define LLP_INTERLEAVE_SIZE 12
#define LLP_MIN_FRAME_LENGTH LLP_INTERLEAVE_SIZE
#define LLP_MAX_FRAME_LENGTH 48 * LLP_INTERLEAVE_SIZE
#define LLP_HEADER_SIZE 10
#define LLP_CHECKSUM_SIZE 2
#define LLP_MAX_DATA_SIZE LLP_MAX_FRAME_LENGTH - LLP_HEADER_SIZE - LLP_CHECKSUM_SIZE
#define LLP_DATA_BLOCK_SIZE ((LLP_INTERLEAVE_SIZE/3)*2)

#define LLP_CRC_SIZE 2
#define LLP_CRC_CORRECT  0xF0B8

struct LLPCtx;     // Forward declarations

typedef void (*llp_callback_t)(struct LLPCtx *ctx);

typedef struct LLPAddress {
    uint16_t network;
    uint16_t host;
} LLPAddress;

typedef struct LLPHeader {
    LLPAddress src;
    LLPAddress dst;
    uint8_t flags;
    uint8_t padding;
} LLPHeader;

typedef struct LLPMsg {
    LLPHeader header;
    const uint8_t *data;
    size_t len;
} LLPMsg;

typedef struct LLPCtx {
    uint8_t buf[LLP_MAX_FRAME_LENGTH];
    FILE *ch;
    LLPAddress *address;
    size_t frame_len;
    size_t readLength;
    uint16_t crc_in;
    uint16_t crc_out;
    uint8_t calculatedParity;
    long correctionsMade;
    llp_callback_t hook;
    bool sync;
    bool escape;
    bool ready_for_data;
    uint8_t interleaveCounter;                      // Keeps track of when we have received an entire interleaved block
    uint8_t interleaveOut[LLP_INTERLEAVE_SIZE];     // A buffer for interleaving bytes before they are sent
    uint8_t interleaveIn[LLP_INTERLEAVE_SIZE];      // A buffer for storing interleaved bytes before they are deinterleaved
} LLPCtx;

void llp_broadcast(LLPCtx *ctx, const void *_buf, size_t len);
void llp_send(LLPCtx *ctx, LLPAddress *dst, const void *_buf, size_t len);
void llp_sendRaw(LLPCtx *ctx, const void *_buf, size_t len);
void llp_poll(LLPCtx *ctx);
void llp_init(LLPCtx *ctx, LLPAddress *address, FILE *channel, llp_callback_t hook);

void llpInterleave(LLPCtx *ctx, uint8_t byte);
void llpDeinterleave(LLPCtx *ctx);
uint8_t llpParityBlock(uint8_t first, uint8_t other);

#endif