
#include "web_server.h"

#if TM_WIFI_NETIF && TM_WEB_SERVER

#include <stddef.h>
#include <string.h>

#include <tm/tmonitor.h>

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "templates/index_html.h"
#include "mqtt_client.h"

#define WEB_SERVER_PORT       8080U
#define WEB_REQUEST_MAX       512U
#define WEB_RESPONSE_MAX      2048U /* Includes all 66 binary mapping entries in dashboard JSON. */
#define WEB_TX_CHUNK_SIZE     1024U

static struct tcp_pcb *web_listener;
static struct tcp_pcb *web_client;

static WEB_RUNTIME_STATE web_state;
_Static_assert(WEB_INDEX_HTML_LEN <= 65535U, "HTTP body length field");
_Static_assert(WEB_REQUEST_MAX <= 512U && WEB_RESPONSE_MAX <= 65535U, "HTTP buffer fields");
static char web_request[WEB_REQUEST_MAX];
static char web_api_response[WEB_RESPONSE_MAX];


/* Active response state. */
static const char *web_tx_header;

static const char *web_tx_body;



/* Reset state for a new HTTP request. */
static void web_reset_request(void)
{
    web_state.request_length = 0;
    web_request[0] = '\0';

    web_state.response_sent = 0;

    web_tx_header = NULL;
    web_state.tx_header_length = 0;
    web_state.tx_header_offset = 0;

    web_tx_body = NULL;
    web_state.tx_body_length = 0;
    web_state.tx_offset = 0;

    web_state.tx_active = 0;
    web_state.tx_output_pending = 0;
}


/* Close the client connection. */
static void web_close_client(struct tcp_pcb *pcb)
{
    err_t result;

    if(pcb == NULL)
        return;

    if(pcb == web_client)
        web_client = NULL;

    web_state.tx_active = 0;
    web_state.tx_output_pending = 0;

    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_err(pcb, NULL);
    tcp_poll(pcb, NULL, 0);

    result = tcp_close(pcb);

    if(result != ERR_OK) {
        /*
         * tcp_abort() invalidates the PCB. The caller must not
         * use this PCB afterward.
         */
        tcp_abort(pcb);
    }
}


/* Called when lwIP reports a fatal connection error. */
static void web_error(void *arg, err_t err)
{
    struct tcp_pcb *pcb = (struct tcp_pcb *)arg;

    (void)err;

    if(pcb == web_client)
        web_client = NULL;

    web_state.tx_active = 0;
    web_state.tx_output_pending = 0;
}


/*
 * Queue response data while send-buffer space is available.
 *
 * Returns ERR_OK when queued data is waiting to be transmitted
 * or acknowledged. tcp_sent() calls this function again later.
 */
static err_t web_send_more(struct tcp_pcb *pcb)
{
    err_t result;
    size_t remaining;
    size_t chunk;
    u16_t available;

    while(web_state.tx_header_offset < web_state.tx_header_length
          || web_state.tx_offset < web_state.tx_body_length) {

        available = tcp_sndbuf(pcb);

        if(available == 0U)
            break;

        if(web_state.tx_header_offset < web_state.tx_header_length) {
            remaining =
                web_state.tx_header_length - web_state.tx_header_offset;

            chunk = remaining;

            if(chunk > WEB_TX_CHUNK_SIZE)
                chunk = WEB_TX_CHUNK_SIZE;

            if(chunk > available)
                chunk = available;

            result = tcp_write(
                pcb,
                web_tx_header + web_state.tx_header_offset,
                (u16_t)chunk,
                TCP_WRITE_FLAG_COPY);

            if(result == ERR_MEM)
                break;

            if(result != ERR_OK)
                return result;

            web_state.tx_header_offset += chunk;
        } else {
            remaining = web_state.tx_body_length - web_state.tx_offset;

            chunk = remaining;

            if(chunk > WEB_TX_CHUNK_SIZE)
                chunk = WEB_TX_CHUNK_SIZE;

            if(chunk > available)
                chunk = available;

            result = tcp_write(
                pcb,
                web_tx_body + web_state.tx_offset,
                (u16_t)chunk,
                TCP_WRITE_FLAG_COPY);

            if(result == ERR_MEM)
                break;

            if(result != ERR_OK)
                return result;

            web_state.tx_offset += chunk;
        }
    }

    /*
     * Flush queued data. Even if tcp_write() previously returned
     * ERR_MEM, there may be earlier queued data to transmit.
     */
    result = tcp_output(pcb);

    if(result != ERR_OK)
        return result;

    web_state.tx_output_pending =
        (web_state.tx_header_offset < web_state.tx_header_length
         || web_state.tx_offset < web_state.tx_body_length);

    return ERR_OK;
}


/* Called when the remote client acknowledges transmitted data. */
static err_t web_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
    err_t result;

    (void)arg;
    (void)len;

    if(!web_state.tx_active)
        return ERR_OK;

    /*
     * Close only after all response bytes have been queued.
     * tcp_close() queues the FIN after already queued TCP data.
     */
    if(web_state.tx_header_offset == web_state.tx_header_length
       && web_state.tx_offset == web_state.tx_body_length) {

        web_state.tx_active = 0;
        web_close_client(pcb);

        return ERR_OK;
    }

    result = web_send_more(pcb);

    if(result != ERR_OK) {
        tm_printf((UB *)"[web] send continuation failed: %d\n",
                  result);

        web_close_client(pcb);
    }

    return ERR_OK;
}


/* Called periodically by lwIP if a poll callback is registered. */
static err_t web_poll(void *arg, struct tcp_pcb *pcb)
{
    err_t result;

    (void)arg;

    if(!web_state.tx_active || !web_state.tx_output_pending)
        return ERR_OK;

    result = web_send_more(pcb);

    if(result != ERR_OK) {
        tm_printf((UB *)"[web] poll send failed: %d\n", result);
        web_close_client(pcb);
        return ERR_OK;
    }

    return ERR_OK;
}


/* Select and start an HTTP response. */
static err_t web_send_response(struct tcp_pcb *pcb)
{
    static const char html_response_start[] =
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Connection: close\r\n"
        "\r\n";

    static const char api_response_start[] =
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n"
        "\r\n";

    static const char not_found_response[] =
        "HTTP/1.0 404 Not Found\r\n"
        "Content-Length: 0\r\n"
        "Connection: close\r\n"
        "\r\n";

    _Static_assert(sizeof(html_response_start) <= 128 && sizeof(api_response_start) <= 128 &&
                   sizeof(not_found_response) <= 128, "HTTP header length field");
    const char *response_start;
    const char *body;
    size_t body_length;
    err_t result;

    if(strncmp(web_request, "GET /favicon.ico", 16U) == 0) {
        response_start = not_found_response;
        body = "";
        body_length = 0;
    } else if(strncmp(web_request, "GET /api/mqtt", 13U) == 0) {
        body_length = mqtt_dashboard_json(
            web_api_response,
            sizeof(web_api_response));

        if(body_length >= sizeof(web_api_response))
            body_length = sizeof(web_api_response) - 1U;

        response_start = api_response_start;
        body = web_api_response;
    } else if(strncmp(web_request, "GET /api/telemetry", 18U) == 0) {
        body_length = telemetry_json(
            web_api_response,
            sizeof(web_api_response));

        if(body_length >= sizeof(web_api_response))
            body_length = sizeof(web_api_response) - 1U;

        response_start = api_response_start;
        body = web_api_response;
    } else {
        response_start = html_response_start;
        body = web_index_html;
        body_length = WEB_INDEX_HTML_LEN;
    }

    tm_printf((UB *)"[web] response body length=%u sndbuf=%u\n",
              (unsigned int)body_length,
              (unsigned int)tcp_sndbuf(pcb));

    web_tx_header = response_start;
    web_state.tx_header_length = strlen(response_start);
    web_state.tx_header_offset = 0;

    web_tx_body = body;
    web_state.tx_body_length = body_length;
    web_state.tx_offset = 0;

    web_state.tx_active = 1;
    web_state.tx_output_pending = 0;
    web_state.response_sent = 1;

    tcp_sent(pcb, web_sent);
    tcp_poll(pcb, web_poll, 2U);

    result = web_send_more(pcb);

    if(result != ERR_OK) {
        tm_printf((UB *)"[web] initial send failed: %d\n", result);
        web_state.tx_active = 0;
        return result;
    }

    /*
     * Empty responses (for example, favicon 404) may have no
     * body bytes to acknowledge. The sent callback closes normal
     * responses after the header has been queued and acknowledged.
     */
    if(web_state.tx_header_offset == web_state.tx_header_length
       && web_state.tx_offset == web_state.tx_body_length
       && web_state.tx_header_length == 0U) {

        web_state.tx_active = 0;
        web_close_client(pcb);
    }

    return ERR_OK;
}


/* Receive and assemble the HTTP request. */
static err_t web_receive(void *arg, struct tcp_pcb *pcb,
                         struct pbuf *packet, err_t err)
{
    size_t available;
    u16_t packet_length;

    (void)arg;

    if(err != ERR_OK) {
        if(packet != NULL)
            pbuf_free(packet);

        web_close_client(pcb);
        return err;
    }

    if(packet == NULL) {
        web_close_client(pcb);
        return ERR_OK;
    }

    packet_length = packet->tot_len;

    if(web_state.response_sent) {
        tcp_recved(pcb, packet_length);
        pbuf_free(packet);
        return ERR_OK;
    }

    available = WEB_REQUEST_MAX - 1U - web_state.request_length;

    if((size_t)packet_length > available
       || pbuf_copy_partial(
              packet,
              &web_request[web_state.request_length],
              packet_length,
              0) != packet_length) {

        tcp_recved(pcb, packet_length);
        pbuf_free(packet);
        web_close_client(pcb);

        return ERR_OK;
    }

    web_state.request_length += packet_length;
    web_request[web_state.request_length] = '\0';

    tcp_recved(pcb, packet_length);
    pbuf_free(packet);

    if(strstr(web_request, "\r\n\r\n") == NULL)
        return ERR_OK;

    if(web_send_response(pcb) != ERR_OK) {
        tm_printf((UB *)"[web] response setup failed\n");
        web_close_client(pcb);
    }

    return ERR_OK;
}


/* Accept a new HTTP connection. */
static err_t web_accept(void *arg, struct tcp_pcb *new_client, err_t err)
{
    (void)arg;

    if(err != ERR_OK || new_client == NULL)
        return ERR_VAL;

    if(web_client != NULL) {
        tcp_abort(new_client);
        return ERR_ABRT;
    }

    web_client = new_client;

    web_reset_request();

    tcp_arg(new_client, new_client);
    tcp_recv(new_client, web_receive);
    tcp_err(new_client, web_error);

    return ERR_OK;
}


/* Initialize the HTTP listener. */
void web_server_init(struct netif *netif)
{
    err_t result;
    struct tcp_pcb *listener;

    if(netif == NULL || web_listener != NULL)
        return;

    listener = tcp_new();

    if(listener == NULL) {
        tm_printf((UB *)"[web] tcp_new failed\n");
        return;
    }

    tcp_bind_netif(listener, netif);

    result = tcp_bind(listener, IP_ADDR_ANY, WEB_SERVER_PORT);

    if(result != ERR_OK) {
        tm_printf((UB *)"[web] tcp_bind failed: %d\n", result);
        tcp_abort(listener);
        return;
    }

    web_listener = tcp_listen(listener);

    if(web_listener == NULL) {
        tm_printf((UB *)"[web] tcp_listen failed\n");
        tcp_abort(listener);
        return;
    }

    tcp_accept(web_listener, web_accept);

    tm_printf((UB *)"[web] listening on port %u\n",
              WEB_SERVER_PORT);
}


/* Restart initialization if the network becomes available later. */
void web_server_poll(struct netif *netif)
{
    if(web_listener == NULL
       && netif != NULL
       && netif_is_up(netif)
       && netif_is_link_up(netif)) {

        web_server_init(netif);
    }
}

#endif
