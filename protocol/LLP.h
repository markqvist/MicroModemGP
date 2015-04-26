#ifndef PROTOCOL_LLP_H
#define PROTOCOL_LLP_H

#include <stdio.h>
#include <stdbool.h>
#include "device.h"

#define LLP_MIN_FRAME_LEN 18
#ifndef CUSTOM_FRAME_SIZE
    #define LLP_MAX_FRAME_LEN 620
#else
    #define LLP_MAX_FRAME_LEN CUSTOM_FRAME_SIZE
#endif

#define LLP_CRC_CORRECT  0xF0B8

struct LLPCtx;     // Forward declarations

typedef void (*llp_callback_t)(struct LLPCtx *ctx);

typedef struct LLPAddress {
    uint16_t network;
    uint16_t host;
} LLPAddress;

typedef struct LLPMsg {
    LLPAddress src;
    LLPAddress dst;
    const uint8_t *data;
    size_t len;
} LLPMsg;

typedef struct LLPCtx {
    uint8_t buf[LLP_MAX_FRAME_LEN];
    FILE *ch;
    LLPAddress *address;
    size_t frame_len;
    uint16_t crc_in;
    uint16_t crc_out;
    llp_callback_t hook;
    bool sync;
    bool escape;
} LLPCtx;

void llp_send(LLPCtx *ctx, LLPAddress *dst, const void *_buf, size_t len);
void llp_sendRaw(LLPCtx *ctx, const void *_buf, size_t len);
void llp_poll(LLPCtx *ctx);
void llp_init(LLPCtx *ctx, LLPAddress *address, FILE *channel, llp_callback_t hook);

#endif