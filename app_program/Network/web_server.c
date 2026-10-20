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

#define WEB_SERVER_PORT 8080U
#define WEB_REQUEST_MAX 512U
#define WEB_RESPONSE_MAX 1024U

static struct tcp_pcb *web_listener;
static struct tcp_pcb *web_client;
static size_t web_request_length;
static char web_request[WEB_REQUEST_MAX];
static char web_api_response[WEB_RESPONSE_MAX];
static int web_response_sent;

static void web_reset_request(void)
{
    web_request_length = 0;
    web_request[0] = '\0';
    web_response_sent = 0;
}

static void web_close_client(struct tcp_pcb *pcb)
{
    if(pcb == NULL) return;
    if(pcb == web_client) web_client = NULL;
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_err(pcb, NULL);
    tcp_poll(pcb, NULL, 0);
    if(tcp_close(pcb) != ERR_OK) tcp_abort(pcb);
}

static void web_error(void *arg, err_t err)
{
    struct tcp_pcb *pcb = (struct tcp_pcb *)arg;

    (void)err;
    if(pcb == web_client) web_client = NULL;
}

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
    const char *response_start;
    const char *body;
    size_t body_length;
    err_t result;

    if(strncmp(web_request, "GET /favicon.ico", 16U) == 0) {
        response_start = not_found_response;
        body = "";
        body_length = 0;
    } else if(strncmp(web_request, "GET /api/mqtt", 13U) == 0) {
        body_length = mqtt_dashboard_json(web_api_response,
                                          sizeof(web_api_response));
        response_start = api_response_start;
        body = web_api_response;
    } else {
        response_start = html_response_start;
        body = web_index_html;
        body_length = WEB_INDEX_HTML_LEN;
    }

    result = tcp_write(pcb, response_start, strlen(response_start),
                       TCP_WRITE_FLAG_COPY);
    if(result != ERR_OK) return result;
    result = tcp_write(pcb, body, (u16_t)body_length,
                       TCP_WRITE_FLAG_COPY);
    if(result != ERR_OK) return result;

    web_response_sent = 1;
    (void)tcp_output(pcb);
    web_close_client(pcb);
    return ERR_OK;
}

static err_t web_receive(void *arg, struct tcp_pcb *pcb, struct pbuf *packet,
                         err_t err)
{
    size_t available;

    (void)arg;
    if(err != ERR_OK) {
        if(packet != NULL) pbuf_free(packet);
        web_close_client(pcb);
        return err;
    }
    if(packet == NULL) {
        web_close_client(pcb);
        return ERR_OK;
    }

    if(web_response_sent) {
        tcp_recved(pcb, packet->tot_len);
        pbuf_free(packet);
        return ERR_OK;
    }

    available = WEB_REQUEST_MAX - 1U - web_request_length;
    if((size_t)packet->tot_len > available
       || pbuf_copy_partial(packet, &web_request[web_request_length],
                            packet->tot_len, 0) != packet->tot_len) {
        tcp_recved(pcb, packet->tot_len);
        pbuf_free(packet);
        web_close_client(pcb);
        return ERR_OK;
    }

    web_request_length += packet->tot_len;
    web_request[web_request_length] = '\0';
    tcp_recved(pcb, packet->tot_len);
    pbuf_free(packet);

    if(strstr(web_request, "\r\n\r\n") == NULL) return ERR_OK;

    if(web_send_response(pcb) != ERR_OK) {
        web_close_client(pcb);
    }
    return ERR_OK;
}

static err_t web_accept(void *arg, struct tcp_pcb *new_client, err_t err)
{
    (void)arg;
    if(err != ERR_OK || new_client == NULL) return ERR_VAL;

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

void web_server_init(struct netif *netif)
{
    err_t result;
    struct tcp_pcb *listener;

    if(netif == NULL || web_listener != NULL) return;

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
    tm_printf((UB *)"[web] listening on port %u\n", WEB_SERVER_PORT);
}

void web_server_poll(struct netif *netif)
{
    if(web_listener == NULL && netif != NULL
       && netif_is_up(netif) && netif_is_link_up(netif)) {
        web_server_init(netif);
    }
}

#endif
