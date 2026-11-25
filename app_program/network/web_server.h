/**
 * @file web_server.h
 * @brief Embedded HTTP dashboard and telemetry REST API server interface.
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

typedef struct netif web_server_netif_t;

/**
 * @brief Initialize the TCP listener on port 8080 when network is ready.
 * @param[in] p_netif Active lwIP network interface.
 */
void web_server_init(web_server_netif_t * p_netif);

/**
 * @brief Poll network readiness and lazily initialize the HTTP listener.
 * @param[in] p_netif Active lwIP network interface.
 */
void web_server_poll(web_server_netif_t * p_netif);

#endif /* WEB_SERVER_H */

/*** end of file ***/
