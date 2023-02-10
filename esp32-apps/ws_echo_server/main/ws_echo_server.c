#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"

#include <esp_http_server.h>

/* A simple example that demonstrates using websocket echo server
 */
static const char *TAG = "ws_echo_server";

/*
 * Structure holding server handle
 * and internal socket fd in order
 * to use out of request send
 */
struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
};

/*
 * async send function, which we put into the httpd work queue
 */
static void ws_async_resp(void *arg)
{
    char http_str[250];
    char *data_str = "Hello from ESP32 websocket server ...";
    sprintf(http_str, "HTTP/1.1 200 OK\r\nContent-Length: %d\r\n\r\n", strlen(data_str));
    struct async_resp_arg *resp_arg = (struct async_resp_arg *)arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    ESP_LOGI(TAG, "Executing queued work fd: %d", fd);
    httpd_socket_send(hd, fd, http_str, strlen(http_str), 0);
    httpd_socket_send(hd, fd, data_str, strlen(data_str), 0);

    // httpd_ws_frame_t ws_pkt;
    // memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    // ws_pkt.payload = (uint8_t*)data;
    // ws_pkt.len = strlen(data);
    // ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    // httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(arg);
}

static esp_err_t async_get_handler(httpd_req_t *req)
{
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    httpd_queue_work(req->handle, ws_async_resp, resp_arg);
    return ESP_OK;
}

static const httpd_uri_t ws = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = async_get_handler,
        .user_ctx   = NULL,
        .is_websocket = true
};

static void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    httpd_start(&server, &config);
    httpd_register_uri_handler(server, &ws);
}

void app_main(void)
{
    static httpd_handle_t server = NULL;
    start_webserver();
}
