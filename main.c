#include <stdbool.h>
#include <string.h>
#include <avr/io.h>

#include "device.h"
#include "util/FIFO.h"
#include "util/time.h"
#include "hardware/AFSK.h"
#include "hardware/Serial.h"
#include "protocol/AX25.h"
#include "protocol/LLP.h"
#include "protocol/KISS.h"


Serial serial;
Afsk modem;
LLPAddress localAdress;
LLPCtx llp;

static void llp_callback(struct LLPCtx *ctx) {
    kiss_messageCallback(ctx);
}

void init(void) {
    sei();

    AFSK_init(&modem);

    memset(&localAdress, 0, sizeof(localAdress));
    localAdress.network = 0xF000;
    localAdress.host    = 0x0001;
    llp_init(&llp, &localAdress, &modem.fd, llp_callback);

    serial_init(&serial);    
    stdout = &serial.uart0;
    stdin  = &serial.uart0;

    kiss_init(&llp, &modem, &serial);
}

int main (void) {
    init();

    while (true) {
        llp_poll(&llp);
        
        if (serial_available(0)) {
            char sbyte = uart0_getchar_nowait();
            kiss_serialCallback(sbyte);
        }
    }

    return(0);
}