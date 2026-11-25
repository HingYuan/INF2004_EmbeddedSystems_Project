#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#ifndef PROHIBIT_DEF_SIZE_T
#define PROHIBIT_DEF_SIZE_T
#endif
/* SDK/newlib macros can collide with T-Kernel names; preserve them for callers. */
#pragma push_macro("B")
#pragma push_macro("_B")
#undef B
#undef _B
#include <tk/typedef.h>
#pragma pop_macro("_B")
#pragma pop_macro("B")

/* Network task owns this state; no cross-core writers. */
typedef struct __attribute__((packed)) {
    UW request_length : 9; // 0..511.
    UW response_sent : 1;
    UW tx_active : 1;
    UW tx_output_pending : 1;
    UW tx_header_length : 7; // Current response headers are <=127 bytes.
    UW tx_header_offset : 7;
    UH tx_body_length;     // Static HTML and API bodies are <=65535 bytes.
    UH tx_offset;
} WEB_RUNTIME_STATE;

struct netif;

void web_server_init(struct netif *netif);
void web_server_poll(struct netif *netif);

#endif
