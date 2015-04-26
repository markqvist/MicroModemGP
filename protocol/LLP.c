#include <string.h>
#include <ctype.h>
#include "LLP.h"
#include "protocol/HDLC.h"
#include "util/CRC-CCIT.h"
#include "../hardware/AFSK.h"

void llp_send(LLPCtx *ctx, LLPAddress *dst, const void *_buf, size_t len) {

}

void llp_poll(LLPCtx *ctx) {

}

void llp_init(LLPCtx *ctx, LLPAddress *address, FILE *channel, llp_callback_t hook) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->ch = channel;
    ctx->hook = hook;
    ctx->address = address;
    ctx->crc_in = ctx->crc_out = CRC_CCIT_INIT_VAL;
}