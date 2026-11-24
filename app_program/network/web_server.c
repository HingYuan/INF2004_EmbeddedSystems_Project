/**
 * @file web_server.c
 * @brief Embedded lwIP raw-API HTTP server for dashboard and JSON endpoints.
 */

#include "web_server.h"

#if (TM_WIFI_NETIF && TM_WEB_SERVER)

/* Prevent micro T-Kernel syslib.h from redefining ISO C size_t. */
#define PROHIBIT_DEF_SIZE_T

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <tm/tmonitor.h>

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "../robot_tasks.h"
#include "index_html.h"
#include "mqtt_client.h"

#define WEB_SERVER_PORT                 (8080u)
#define WEB_REQUEST_MAX                 (512u)
#define WEB_RESPONSE_MAX                (2048u)
#define WEB_TX_CHUNK_SIZE               (1024u)
#define WEB_POLL_INTERVAL_TICKS         (2u)
#define WEB_MAX_HEADER_BYTES            (128u)
#define WEB_MAX_BODY_BYTES              (65535u)
#define WEB_FAVICON_PREFIX_LEN          (16u)
#define WEB_MQTT_API_PREFIX_LEN         (13u)
#define WEB_TELEM_API_PREFIX_LEN        (18u)

/* Network task owns this state exclusively; no cross-core writers. */
typedef struct ROBOT_TASKS_PACKED
{
    uint32_t request_length    : 9;
    uint32_t b_response_sent   : 1;
    uint32_t b_tx_active       : 1;
    uint32_t b_tx_out_pending  : 1;
    uint32_t tx_header_length  : 7;
    uint32_t tx_header_offset  : 7;
    uint16_t tx_body_length;
    uint16_t tx_offset;
} web_server_state_t;

ROBOT_TASKS_STATIC_ASSERT(web_state_size, (8u == sizeof(web_server_state_t)));
ROBOT_TASKS_STATIC_ASSERT(web_html_len_fit,
                          (INDEX_HTML_BODY_LEN <= WEB_MAX_BODY_BYTES));
ROBOT_TASKS_STATIC_ASSERT(web_buf_limits,
                          ((WEB_REQUEST_MAX <= 512u) &&
                           (WEB_RESPONSE_MAX <= WEB_MAX_BODY_BYTES)));

static const char g_html_response_hdr[] =
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Connection: close\r\n"
    "\r\n";

static const char g_api_response_hdr[] =
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: application/json\r\n"
    "Cache-Control: no-store\r\n"
    "Connection: close\r\n"
    "\r\n";

static const char g_not_found_hdr[] =
    "HTTP/1.0 404 Not Found\r\n"
    "Content-Length: 0\r\n"
    "Connection: close\r\n"
    "\r\n";

ROBOT_TASKS_STATIC_ASSERT(web_hdr_lengths,
                          ((sizeof(g_html_response_hdr) <=
                            WEB_MAX_HEADER_BYTES) &&
                           (sizeof(g_api_response_hdr) <=
                            WEB_MAX_HEADER_BYTES) &&
                           (sizeof(g_not_found_hdr) <=
                            WEB_MAX_HEADER_BYTES)));

static struct tcp_pcb *    gp_web_listener    = NULL;
static struct tcp_pcb *    gp_web_client      = NULL;
static web_server_state_t  g_web_state        = { 0u };
static char                g_web_request[WEB_REQUEST_MAX]      = { '\0' };
static char                g_web_api_resp[WEB_RESPONSE_MAX]    = { '\0' };
static const char *        gp_web_tx_header   = NULL;
static const char *        gp_web_tx_body     = NULL;

static void web_bind_and_listen (struct tcp_pcb * p_listener,
                                 struct netif * p_netif);
static void web_reset_request (void);
static void web_close_client (struct tcp_pcb * p_pcb);
static void web_on_error (void * p_arg, err_t err_code);
static err_t web_write_segment (struct tcp_pcb * p_pcb,
                                const char * p_src,
                                uint32_t remaining,
                                uint32_t available,
                                uint32_t * p_sent_out);
static err_t web_send_more (struct tcp_pcb * p_pcb);
static void web_continue_send (struct tcp_pcb * p_pcb);
static err_t web_on_sent (void * p_arg, struct tcp_pcb * p_pcb, u16_t len);
static err_t web_on_poll (void * p_arg, struct tcp_pcb * p_pcb);
static err_t web_send_response (struct tcp_pcb * p_pcb);
static void web_process_payload (struct tcp_pcb * p_pcb,
                                 struct pbuf * p_packet);
static err_t web_on_receive (void * p_arg,
                             struct tcp_pcb * p_pcb,
                             struct pbuf * p_packet,
                             err_t err_code);
static err_t web_on_accept (void * p_arg,
                            struct tcp_pcb * p_new_client,
                            err_t err_code);

/**
 * @brief Bind a freshly allocated listener PCB to port 8080 and start accept.
 * @param[in,out] p_listener Allocated lwIP TCP PCB.
 * @param[in]     p_netif    Active lwIP network interface.
 */
static void web_bind_and_listen (struct tcp_pcb * p_listener,
                                 struct netif * p_netif)
{
    err_t bind_err = ERR_OK;

    tcp_bind_netif(p_listener, p_netif);
    bind_err = tcp_bind(p_listener, IP_ADDR_ANY, WEB_SERVER_PORT);

    if (ERR_OK != bind_err)
    {
        tm_printf((UB *)"[web] tcp_bind failed: %d\n", (INT)bind_err);
        tcp_abort(p_listener);
    }
    else
    {
        gp_web_listener = tcp_listen(p_listener);

        if (NULL == gp_web_listener)
        {
            tm_printf((UB *)"[web] tcp_listen failed\n");
            tcp_abort(p_listener);
        }
        else
        {
            tcp_accept(gp_web_listener, web_on_accept);
            tm_printf((UB *)"[web] listening on port %u\n",
                      (UINT)WEB_SERVER_PORT);
        }
    }
}

/**
 * @brief Initialize the TCP listener on port 8080 when network is ready.
 * @param[in] p_netif Active lwIP network interface.
 */
void web_server_init (struct netif * p_netif)
{
    if ((NULL != p_netif) && (NULL == gp_web_listener))
    {
        struct tcp_pcb * p_listener = tcp_new();

        if (NULL == p_listener)
        {
            tm_printf((UB *)"[web] tcp_new failed\n");
        }
        else
        {
            web_bind_and_listen(p_listener, p_netif);
        }
    }
}

/**
 * @brief Poll network readiness and lazily initialize the HTTP listener.
 * @param[in] p_netif Active lwIP network interface.
 */
void web_server_poll (struct netif * p_netif)
{
    if ((NULL == gp_web_listener) &&
        (NULL != p_netif) &&
        (0u != netif_is_up(p_netif)) &&
        (0u != netif_is_link_up(p_netif)))
    {
        web_server_init(p_netif);
    }
}

/**
 * @brief Reset HTTP request and active TX tracking state for a new client.
 */
static void web_reset_request (void)
{
    g_web_state.request_length   = 0u;
    g_web_request[0]             = '\0';
    g_web_state.b_response_sent  = 0u;
    gp_web_tx_header             = NULL;
    g_web_state.tx_header_length = 0u;
    g_web_state.tx_header_offset = 0u;
    gp_web_tx_body               = NULL;
    g_web_state.tx_body_length   = 0u;
    g_web_state.tx_offset        = 0u;
    g_web_state.b_tx_active      = 0u;
    g_web_state.b_tx_out_pending = 0u;
}

/**
 * @brief Detach lwIP callbacks and cleanly close (or abort) the client PCB.
 * @param[in,out] p_pcb Client TCP protocol control block.
 */
static void web_close_client (struct tcp_pcb * p_pcb)
{
    if (NULL != p_pcb)
    {
        err_t close_err = ERR_OK;

        if (p_pcb == gp_web_client)
        {
            gp_web_client = NULL;
        }

        g_web_state.b_tx_active      = 0u;
        g_web_state.b_tx_out_pending = 0u;

        tcp_arg(p_pcb, NULL);
        tcp_recv(p_pcb, NULL);
        tcp_sent(p_pcb, NULL);
        tcp_err(p_pcb, NULL);
        tcp_poll(p_pcb, NULL, 0u);

        close_err = tcp_close(p_pcb);

        if (ERR_OK != close_err)
        {
            tcp_abort(p_pcb);
        }
    }
}

/**
 * @brief lwIP fatal TCP connection error callback.
 * @param[in] p_arg    Client PCB pointer passed via tcp_arg().
 * @param[in] err_code lwIP error code (unused).
 */
static void web_on_error (void * p_arg, err_t err_code)
{
    /* Cast restores PCB pointer registered by tcp_arg(). */
    struct tcp_pcb * p_pcb = (struct tcp_pcb *)p_arg;

    (void)err_code;

    if (p_pcb == gp_web_client)
    {
        gp_web_client = NULL;
    }

    g_web_state.b_tx_active      = 0u;
    g_web_state.b_tx_out_pending = 0u;
}

/**
 * @brief Queue one bounded TCP write chunk up to WEB_TX_CHUNK_SIZE and sndbuf.
 * @param[in,out] p_pcb      Client TCP PCB.
 * @param[in]     p_src      Pointer to next unsent byte.
 * @param[in]     remaining  Bytes remaining in current segment.
 * @param[in]     available  Available lwIP send-buffer bytes.
 * @param[out]    p_sent_out Number of bytes successfully queued.
 * @return ERR_OK, ERR_MEM, or fatal lwIP write error.
 */
static err_t web_write_segment (struct tcp_pcb * p_pcb,
                                const char * p_src,
                                uint32_t remaining,
                                uint32_t available,
                                uint32_t * p_sent_out)
{
    uint32_t chunk_len = remaining;
    err_t    write_err = ERR_OK;

    *p_sent_out = 0u;

    if (chunk_len > WEB_TX_CHUNK_SIZE)
    {
        chunk_len = WEB_TX_CHUNK_SIZE;
    }

    if (chunk_len > available)
    {
        chunk_len = available;
    }

    /* Range-safe: chunk_len <= WEB_TX_CHUNK_SIZE (1024) fits in u16_t. */
    write_err = tcp_write(p_pcb, p_src, (u16_t)chunk_len, TCP_WRITE_FLAG_COPY);

    if (ERR_OK == write_err)
    {
        *p_sent_out = chunk_len;
    }

    return write_err;
}

/**
 * @brief Stream queued response header and body bytes while sndbuf has space.
 * @param[in,out] p_pcb Client TCP PCB.
 * @return ERR_OK on success or fatal lwIP error code.
 */
static err_t web_send_more (struct tcp_pcb * p_pcb)
{
    err_t ret_err = ERR_OK;
    bool  b_loop  = true;

    while (b_loop &&
           ((g_web_state.tx_header_offset < g_web_state.tx_header_length) ||
            (g_web_state.tx_offset < g_web_state.tx_body_length)))
    {
        uint32_t available = (uint32_t)tcp_sndbuf(p_pcb);

        if (0u == available)
        {
            b_loop = false;
        }
        else if (g_web_state.tx_header_offset < g_web_state.tx_header_length)
        {
            uint32_t remain   = g_web_state.tx_header_length -
                                g_web_state.tx_header_offset;
            uint32_t queued   = 0u;
            err_t    wr_err   = web_write_segment(
                p_pcb,
                &gp_web_tx_header[g_web_state.tx_header_offset],
                remain,
                available,
                &queued);

            if (ERR_MEM == wr_err)
            {
                b_loop = false;
            }
            else if (ERR_OK != wr_err)
            {
                ret_err = wr_err;
                b_loop  = false;
            }
            else
            {
                g_web_state.tx_header_offset += queued;
            }
        }
        else
        {
            uint32_t remain = ((uint32_t)g_web_state.tx_body_length) -
                              ((uint32_t)g_web_state.tx_offset);
            uint32_t queued = 0u;
            err_t    wr_err = web_write_segment(
                p_pcb,
                &gp_web_tx_body[g_web_state.tx_offset],
                remain,
                available,
                &queued);

            if (ERR_MEM == wr_err)
            {
                b_loop = false;
            }
            else if (ERR_OK != wr_err)
            {
                ret_err = wr_err;
                b_loop  = false;
            }
            else
            {
                /* Range-safe: tx_offset <= tx_body_length <= 65535. */
                g_web_state.tx_offset =
                    (uint16_t)(g_web_state.tx_offset + queued);
            }
        }
    }

    if (ERR_OK == ret_err)
    {
        ret_err = tcp_output(p_pcb);

        if (ERR_OK == ret_err)
        {
            bool b_more_hdr =
                (g_web_state.tx_header_offset < g_web_state.tx_header_length);
            bool b_more_body =
                (g_web_state.tx_offset < g_web_state.tx_body_length);

            g_web_state.b_tx_out_pending = (b_more_hdr || b_more_body)
                                           ? 1u
                                           : 0u;
        }
    }

    return ret_err;
}

/**
 * @brief Stream the next response chunk and close the client on failure.
 * @param[in,out] p_pcb Client TCP PCB.
 */
static void web_continue_send (struct tcp_pcb * p_pcb)
{
    err_t send_err = web_send_more(p_pcb);

    if (ERR_OK != send_err)
    {
        tm_printf((UB *)"[web] send continuation failed: %d\n", (INT)send_err);
        web_close_client(p_pcb);
    }
}

/**
 * @brief lwIP callback invoked when remote client ACKs transmitted data.
 * @param[in]     p_arg User argument (unused).
 * @param[in,out] p_pcb Client TCP PCB.
 * @param[in]     len   Acknowledged byte count (unused).
 * @return ERR_OK.
 */
static err_t web_on_sent (void * p_arg, struct tcp_pcb * p_pcb, u16_t len)
{
    (void)p_arg;
    (void)len;

    if (0u != g_web_state.b_tx_active)
    {
        bool b_done =
            ((g_web_state.tx_header_offset == g_web_state.tx_header_length) &&
             (g_web_state.tx_offset == g_web_state.tx_body_length));

        if (b_done)
        {
            g_web_state.b_tx_active = 0u;
            web_close_client(p_pcb);
        }
        else
        {
            web_continue_send(p_pcb);
        }
    }

    return ERR_OK;
}

/**
 * @brief lwIP periodic poll callback retrying stalled response transmissions.
 * @param[in]     p_arg User argument (unused).
 * @param[in,out] p_pcb Client TCP PCB.
 * @return ERR_OK.
 */
static err_t web_on_poll (void * p_arg, struct tcp_pcb * p_pcb)
{
    (void)p_arg;

    if ((0u != g_web_state.b_tx_active) &&
        (0u != g_web_state.b_tx_out_pending))
    {
        err_t send_err = web_send_more(p_pcb);

        if (ERR_OK != send_err)
        {
            tm_printf((UB *)"[web] poll send failed: %d\n", (INT)send_err);
            web_close_client(p_pcb);
        }
    }

    return ERR_OK;
}

/**
 * @brief Route HTTP request path and initiate chunked TCP transmission.
 * @param[in,out] p_pcb Client TCP PCB.
 * @return ERR_OK on success or lwIP error code on initial send failure.
 */
static err_t web_send_response (struct tcp_pcb * p_pcb)
{
    const char * p_hdr     = g_html_response_hdr;
    const char * p_body    = index_html_get_body();
    uint32_t     body_len  = index_html_get_length();
    err_t        send_err  = ERR_OK;

    if (0 == strncmp(g_web_request,
                     "GET /favicon.ico",
                     WEB_FAVICON_PREFIX_LEN))
    {
        p_hdr    = g_not_found_hdr;
        p_body   = "";
        body_len = 0u;
    }
    else if (0 == strncmp(g_web_request,
                          "GET /api/mqtt",
                          WEB_MQTT_API_PREFIX_LEN))
    {
        body_len = mqtt_client_dashboard_json(g_web_api_resp,
                                              (uint32_t)sizeof(g_web_api_resp));

        if (body_len >= (uint32_t)sizeof(g_web_api_resp))
        {
            body_len = ((uint32_t)sizeof(g_web_api_resp)) - 1u;
        }

        p_hdr  = g_api_response_hdr;
        p_body = g_web_api_resp;
    }
    else if (0 == strncmp(g_web_request,
                          "GET /api/telemetry",
                          WEB_TELEM_API_PREFIX_LEN))
    {
        body_len = mqtt_client_telemetry_json(g_web_api_resp,
                                              (uint32_t)sizeof(g_web_api_resp));

        if (body_len >= (uint32_t)sizeof(g_web_api_resp))
        {
            body_len = ((uint32_t)sizeof(g_web_api_resp)) - 1u;
        }

        p_hdr  = g_api_response_hdr;
        p_body = g_web_api_resp;
    }
    else
    {
        p_hdr    = g_html_response_hdr;
        p_body   = index_html_get_body();
        body_len = index_html_get_length();
    }

    tm_printf((UB *)"[web] response body length=%u sndbuf=%u\n",
              (UINT)body_len,
              (UINT)tcp_sndbuf(p_pcb));

    gp_web_tx_header             = p_hdr;
    /* Range-safe: static headers are asserted <= 128 bytes (7-bit field). */
    g_web_state.tx_header_length = (uint32_t)strlen(p_hdr);
    g_web_state.tx_header_offset = 0u;
    gp_web_tx_body               = p_body;
    /* Range-safe: body_len <= 65535 asserted at compile time. */
    g_web_state.tx_body_length   = (uint16_t)body_len;
    g_web_state.tx_offset        = 0u;
    g_web_state.b_tx_active      = 1u;
    g_web_state.b_tx_out_pending = 0u;
    g_web_state.b_response_sent  = 1u;

    tcp_sent(p_pcb, web_on_sent);
    tcp_poll(p_pcb, web_on_poll, WEB_POLL_INTERVAL_TICKS);

    send_err = web_send_more(p_pcb);

    if (ERR_OK != send_err)
    {
        tm_printf((UB *)"[web] initial send failed: %d\n", (INT)send_err);
        g_web_state.b_tx_active = 0u;
    }
    else if ((g_web_state.tx_header_offset == g_web_state.tx_header_length) &&
             (g_web_state.tx_offset == g_web_state.tx_body_length) &&
             (0u == g_web_state.tx_header_length))
    {
        g_web_state.b_tx_active = 0u;
        web_close_client(p_pcb);
    }
    else
    {
        /* Remaining bytes or FIN close will complete via web_on_sent. */
    }

    return send_err;
}

/**
 * @brief Accumulate received HTTP bytes and dispatch once headers complete.
 * @param[in,out] p_pcb    Client TCP PCB.
 * @param[in,out] p_packet Non-NULL incoming lwIP pbuf chain.
 */
static void web_process_payload (struct tcp_pcb * p_pcb,
                                 struct pbuf * p_packet)
{
    u16_t    pkt_len   = p_packet->tot_len;
    uint32_t avail_len =
        (WEB_REQUEST_MAX - 1u) - g_web_state.request_length;

    if (0u != g_web_state.b_response_sent)
    {
        tcp_recved(p_pcb, pkt_len);
        (void)pbuf_free(p_packet);
    }
    else if ((((uint32_t)pkt_len) > avail_len) ||
             (pkt_len != pbuf_copy_partial(
                 p_packet,
                 &g_web_request[g_web_state.request_length],
                 pkt_len,
                 0u)))
    {
        tcp_recved(p_pcb, pkt_len);
        (void)pbuf_free(p_packet);
        web_close_client(p_pcb);
    }
    else
    {
        g_web_state.request_length += (uint32_t)pkt_len;
        g_web_request[g_web_state.request_length] = '\0';
        tcp_recved(p_pcb, pkt_len);
        (void)pbuf_free(p_packet);

        if ((NULL != strstr(g_web_request, "\r\n\r\n")) &&
            (ERR_OK != web_send_response(p_pcb)))
        {
            tm_printf((UB *)"[web] response setup failed\n");
            web_close_client(p_pcb);
        }
    }
}

/**
 * @brief lwIP receive callback assembling incoming HTTP request headers.
 * @param[in]     p_arg    User argument (unused).
 * @param[in,out] p_pcb    Client TCP PCB.
 * @param[in]     p_packet Received lwIP pbuf chain, or NULL on remote close.
 * @param[in]     err_code Receive status code.
 * @return ERR_OK or received lwIP error code.
 */
static err_t web_on_receive (void * p_arg,
                             struct tcp_pcb * p_pcb,
                             struct pbuf * p_packet,
                             err_t err_code)
{
    err_t ret_err = ERR_OK;

    (void)p_arg;

    if (ERR_OK != err_code)
    {
        if (NULL != p_packet)
        {
            (void)pbuf_free(p_packet);
        }

        web_close_client(p_pcb);
        ret_err = err_code;
    }
    else if (NULL == p_packet)
    {
        web_close_client(p_pcb);
    }
    else
    {
        web_process_payload(p_pcb, p_packet);
    }

    return ret_err;
}

/**
 * @brief lwIP accept callback admitting one active HTTP client at a time.
 * @param[in]     p_arg        User argument (unused).
 * @param[in,out] p_new_client Accepted client TCP PCB.
 * @param[in]     err_code     Accept status code.
 * @return ERR_OK, ERR_VAL, or ERR_ABRT.
 */
static err_t web_on_accept (void * p_arg,
                            struct tcp_pcb * p_new_client,
                            err_t err_code)
{
    err_t ret_err = ERR_OK;

    (void)p_arg;

    if ((ERR_OK != err_code) || (NULL == p_new_client))
    {
        ret_err = ERR_VAL;
    }
    else if (NULL != gp_web_client)
    {
        tcp_abort(p_new_client);
        ret_err = ERR_ABRT;
    }
    else
    {
        gp_web_client = p_new_client;
        web_reset_request();
        tcp_arg(p_new_client, p_new_client);
        tcp_recv(p_new_client, web_on_receive);
        tcp_err(p_new_client, web_on_error);
    }

    return ret_err;
}

#else

/**
 * @brief No-op web server init stub when TM_WEB_SERVER is disabled.
 * @param[in] p_netif Network interface pointer (unused).
 */
void web_server_init (struct netif * p_netif)
{
    (void)p_netif;
}

/**
 * @brief No-op web server poll stub when TM_WEB_SERVER is disabled.
 * @param[in] p_netif Network interface pointer (unused).
 */
void web_server_poll (struct netif * p_netif)
{
    (void)p_netif;
}

#endif /* TM_WIFI_NETIF && TM_WEB_SERVER */

/*** end of file ***/
