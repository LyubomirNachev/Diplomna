/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_ot_dns64.h"

#include "esp_openthread.h"
#include "esp_openthread_dns64.h"
#include "lwip/dns.h"
#include "openthread/cli.h"
#include "openthread/netdata.h"

static esp_err_t set_dns64(const ip4_addr_t *dns_server)
{
    ip_addr_t dns_server_addr = {};

    dns_server_addr.type = IPADDR_TYPE_V6;
    if (esp_openthread_get_nat64_prefix(&dns_server_addr.u_addr.ip6) != ESP_OK) {
        otCliOutputFormat("Cannot find NAT64 prefix\n");
        return ESP_ERR_NOT_FOUND;
    }

    dns_server_addr.u_addr.ip6.addr[3] = dns_server->addr;
    dns_setserver(0, &dns_server_addr);
    return ESP_OK;
}

otError esp_openthread_process_dns64_server(void *aContext, uint8_t aArgsLength, char *aArgs[])
{
    if (aArgsLength == 0) {
        otCliOutputFormat("%s", "dns64server DNS_SERVER_URL\n");
        return OT_ERROR_INVALID_ARGS;
    }
    ip4_addr_t server_addr;

    if (ip4addr_aton(aArgs[0], &server_addr) != 1) {
        otCliOutputFormat("Invalid DNS server\n");
        return OT_ERROR_INVALID_ARGS;
    }

    if (set_dns64(&server_addr) != ESP_OK) {
        otCliOutputFormat("Failed to set DNS server\n");
        return OT_ERROR_INVALID_ARGS;
    }
    return OT_ERROR_NONE;
}
