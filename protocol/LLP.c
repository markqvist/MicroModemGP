#include <string.h>
#include <ctype.h>
#include "LLP.h"
#include "protocol/HDLC.h"
#include "util/CRC-CCIT.h"
#include "../hardware/AFSK.h"

void llp_decode(LLPCtx *ctx) {
    if (ctx->hook) ctx->hook(ctx);
}

void llp_poll(LLPCtx *ctx) {
    int c;
    
    while ((c = fgetc(ctx->ch)) != EOF) {
        if (!ctx->escape && c == HDLC_FLAG) {
            if (ctx->frame_len >= LLP_MIN_FRAME_LEN) {
                if (ctx->crc_in == LLP_CRC_CORRECT) {
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
            if (ctx->frame_len < LLP_MAX_FRAME_LEN) {
                ctx->buf[ctx->frame_len++] = c;
                ctx->crc_in = update_crc_ccit(c, ctx->crc_in);
            } else {
                ctx->sync = false;
            }
        }
        ctx->escape = false;
    }
}

static void llp_putchar(LLPCtx *ctx, uint8_t c) {
    if (c == HDLC_FLAG || c == HDLC_RESET || c == LLP_ESC) fputc(LLP_ESC, ctx->ch);
    ctx->crc_out = update_crc_ccit(c, ctx->crc_out);
    fputc(c, ctx->ch);
}

void llp_send(LLPCtx *ctx, LLPAddress *dst, const void *_buf, size_t len) {
    // TODO: implement
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
}