#ifndef WEB_SERVER_H
#define WEB_SERVER_H

struct netif;

void web_server_init(struct netif *netif);
void web_server_poll(struct netif *netif);

#endif
