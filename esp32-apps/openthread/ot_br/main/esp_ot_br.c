#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_netif_net_stack.h"
#include "esp_openthread.h"
#include "esp_openthread_border_router.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "esp_ot_cli_extension.h"
#include "esp_ot_config.h"
#include "esp_ot_wifi_cmd.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "openthread/backbone_router_ftd.h"
#include "openthread/border_router.h"
#include "openthread/cli.h"
#include "openthread/dataset.h"
#include "openthread/dataset_ftd.h"
#include "openthread/dataset_updater.h"
#include "openthread/error.h"
#include "openthread/instance.h"
#include "openthread/ip6.h"
#include "openthread/logging.h"
#include "openthread/tasklet.h"
#include "openthread/thread_ftd.h"

#include "esp_ot_udp_socket.h"
#include "lwip/err.h"
#include "lwip/mld6.h"
#include "lwip/sockets.h"

#include "esp_system.h"
#include "spi_flash_mmap.h"
#include <esp_http_server.h>
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>


#define TAG "esp_ot_br"



//
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
//


//display
#include "ssd1306.h"

//





#if CONFIG_OPENTHREAD_BR_AUTO_START
static int hex_digit_to_int(char hex)
{
    if ('A' <= hex && hex <= 'F') {
        return 10 + hex - 'A';
    }
    if ('a' <= hex && hex <= 'f') {
        return 10 + hex - 'a';
    }
    if ('0' <= hex && hex <= '9') {
        return hex - '0';
    }
    return -1;
}

static size_t hex_string_to_binary(const char *hex_string, uint8_t *buf, size_t buf_size)
{
    int num_char = strlen(hex_string);

    if (num_char != buf_size * 2) {
        return 0;
    }
    for (size_t i = 0; i < num_char; i += 2) {
        int digit0 = hex_digit_to_int(hex_string[i]);
        int digit1 = hex_digit_to_int(hex_string[i + 1]);

        if (digit0 < 0 || digit1 < 0) {
            return 0;
        }
        buf[i / 2] = (digit0 << 4) + digit1;
    }

    return buf_size;
}

static void create_config_network(otInstance *instance)
{
    otOperationalDataset dataset;

    if (otDatasetGetActive(instance, &dataset) == OT_ERROR_NONE) {
        ESP_LOGI(TAG, "Already has network, skip configuring OpenThread network.");
        return;
    }

    uint16_t network_name_len = strlen(CONFIG_OPENTHREAD_NETWORK_NAME);

    assert(network_name_len <= OT_NETWORK_NAME_MAX_SIZE);

    if (otDatasetCreateNewNetwork(instance, &dataset) != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to create OpenThread network dataset.");
        abort();
    }
    dataset.mChannel = CONFIG_OPENTHREAD_NETWORK_CHANNEL;
    dataset.mComponents.mIsChannelPresent = true;
    dataset.mPanId = CONFIG_OPENTHREAD_NETWORK_PANID;
    dataset.mComponents.mIsPanIdPresent = true;
    memcpy(dataset.mNetworkName.m8, CONFIG_OPENTHREAD_NETWORK_NAME, network_name_len);
    dataset.mComponents.mIsNetworkNamePresent = true;
    if (hex_string_to_binary(CONFIG_OPENTHREAD_NETWORK_EXTPANID, dataset.mExtendedPanId.m8,
                             sizeof(dataset.mExtendedPanId.m8)) != sizeof(dataset.mExtendedPanId.m8)) {
        ESP_LOGE(TAG, "Cannot convert OpenThread extended pan id. Please double-check your config.");
        abort();
    }
    dataset.mComponents.mIsExtendedPanIdPresent = true;
    if (hex_string_to_binary(CONFIG_OPENTHREAD_NETWORK_MASTERKEY, dataset.mNetworkKey.m8,
                             sizeof(dataset.mNetworkKey.m8)) != sizeof(dataset.mNetworkKey.m8)) {
        ESP_LOGE(TAG, "Cannot convert OpenThread master key. Please double-check your config.");
        abort();
    }
    dataset.mComponents.mIsNetworkKeyPresent = true;
    if (hex_string_to_binary(CONFIG_OPENTHREAD_NETWORK_PSKC, dataset.mPskc.m8, sizeof(dataset.mPskc.m8)) !=
            sizeof(dataset.mPskc.m8)) {
        ESP_LOGE(TAG, "Cannot convert OpenThread pre-shared commissioner key. Please double-check your config.");
        abort();
    }
    dataset.mComponents.mIsPskcPresent = true;
    if (otDatasetSetActive(instance, &dataset) != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set OpenThread active dataset.");
        abort();
    }
    return;
}

static void launch_openthread_network(otInstance *instance)
{
    if (otIp6SetEnabled(instance, true) != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to enable OpenThread IP6 link");
        abort();
    }
    if (otThreadSetEnabled(instance, true) != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to enable OpenThread");
        abort();
    }
    if (otBorderRouterRegister(instance) != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to register border router.");
        abort();
    }
    otBackboneRouterSetEnabled(instance, true);
}
#endif // CONFIG_OPENTHREAD_BR_AUTO_START

static void ot_task_worker(void *aContext)
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t       *openthread_netif = esp_netif_new(&cfg);
    assert(openthread_netif != NULL);
    // Initialize the OpenThread stack

    ESP_ERROR_CHECK(esp_openthread_init(&config));

    // Initialize border routing features
    esp_openthread_lock_acquire(portMAX_DELAY);
    ESP_ERROR_CHECK(esp_netif_attach(openthread_netif, esp_openthread_netif_glue_init(&config)));

    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
    esp_openthread_cli_init();
#if CONFIG_OPENTHREAD_BR_AUTO_START
    ESP_ERROR_CHECK(esp_openthread_border_router_init());
    create_config_network(esp_openthread_get_instance());
    launch_openthread_network(esp_openthread_get_instance());
#endif // CONFIG_OPENTHREAD_BR_AUTO_START
    esp_cli_custom_command_init();
    esp_openthread_lock_release();

    // Run the main loop
    esp_openthread_cli_create_task();
    esp_openthread_launch_mainloop();

    // Clean up
    esp_netif_destroy(openthread_netif);
    esp_openthread_netif_glue_deinit();
    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}

//UDP Socket function

char rx_buffer[128];
int len;

static void udp_socket_server_task(void *pvParameters)
{
    int err = 0;
    esp_err_t ret = ESP_OK;
    char addr_str[128];
    int listen_sock;
    int port = CONFIG_OPENTHREAD_CLI_UDP_SERVER_PORT;
    
    struct timeval timeout = {0};
    struct sockaddr_storage source_addr;
    struct sockaddr_in6 listen_addr;

    inet6_aton("::", &listen_addr.sin6_addr);
    listen_addr.sin6_family = AF_INET6;
    listen_addr.sin6_port = htons(port);

    listen_sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP); // socket(domain,type,protocol)   
    ESP_GOTO_ON_FALSE((listen_sock >= 0), ESP_OK, exit, TAG, "Unable to create socket: errno %d", errno);
    ESP_LOGI(TAG, "Socket created");

    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));

    err = bind(listen_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr));
    ESP_GOTO_ON_FALSE((err == 0), ESP_FAIL, exit, TAG, "Socket unable to bind: errno %d", errno);
    ESP_LOGI(TAG, "Socket bound, port %d", port);
    ESP_LOGI(TAG, "Waiting for data, no timeout");
    while(1){
        timeout.tv_sec = 0;
        setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        socklen_t socklen = sizeof(source_addr);
        len = recvfrom(listen_sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        // Error occurred during receiving
        ESP_GOTO_ON_FALSE((len >= 0), ESP_FAIL, exit, TAG, "recvfrom failed: errno %d", errno);
        // Data received
        // Get the sender's ip address as string
        inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);

        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
        ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
        ESP_LOGI(TAG, "%s", rx_buffer);
    }
exit:
    if (ret != ESP_OK) {
        shutdown(listen_sock, 0);
        close(listen_sock);
    }
    ESP_LOGI(TAG, "Socket server is closed.");
    vTaskDelete(NULL);
}

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
    char *data_str = &rx_buffer;
    sprintf(http_str, "HTTP/1.1 200 OK\r\nContent-Length: %d\r\n\r\n", strlen(data_str));
    struct async_resp_arg *resp_arg = (struct async_resp_arg *)arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    ESP_LOGI(TAG, "Executing queued work fd: %d", fd);
    httpd_socket_send(hd, fd, http_str, strlen(http_str), 0);
    httpd_socket_send(hd, fd, data_str, strlen(data_str), 0);
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
    esp_netif_ip_info_t ip_info;
    // Used eventfds:
    // * netif
    // * task queue
    // * border router
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3,
    };
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    #if CONFIG_OPENTHREAD_BR_AUTO_START
        ESP_ERROR_CHECK(example_connect());
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
        esp_openthread_set_backbone_netif(get_example_netif());
    #else
        esp_ot_wifi_netif_init();
        esp_openthread_set_backbone_netif(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"));
    #endif // CONFIG_OPENTHREAD_BR_AUTO_START
        ESP_ERROR_CHECK(mdns_init());
        ESP_ERROR_CHECK(mdns_hostname_set("esp-ot-br"));
        //OT_BR_INIT
        xTaskCreate(ot_task_worker, "ot_br_main", 20480, xTaskGetCurrentTaskHandle(), 5, NULL);
        //Udp init
        xTaskCreate(udp_socket_server_task, "ot_udp_scoket_server", 4096, xTaskGetCurrentTaskHandle(), 4, NULL);
        
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        //Web Page init
        ESP_LOGI(TAG, "Socket is running ... ...\n");
        start_webserver();
        SSD1306_t dev;
        i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);	
        ssd1306_init(&dev, 128, 64);
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);
        while(1){
            char data[16];
            ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info));
            sprintf(data, "%d.%d.%d.%d", IP2STR(&ip_info.ip));
            ssd1306_display_text(&dev, 1, "IP:          ", 16, false);
            ssd1306_display_text(&dev, 2, data, 16, false);
            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }
}





















// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>

// #include "esp_check.h"
// #include "esp_err.h"
// #include "esp_event.h"
// #include "esp_log.h"
// #include "esp_netif.h"
// #include "esp_netif_ip_addr.h"
// #include "esp_netif_net_stack.h"
// #include "esp_openthread.h"
// #include "esp_openthread_border_router.h"
// #include "esp_openthread_cli.h"
// #include "esp_openthread_lock.h"
// #include "esp_openthread_netif_glue.h"
// #include "esp_openthread_types.h"
// #include "esp_ot_cli_extension.h"
// #include "esp_ot_config.h"
// #include "esp_ot_wifi_cmd.h"
// #include "esp_vfs_dev.h"
// #include "esp_vfs_eventfd.h"
// #include "esp_wifi.h"
// #include "mdns.h"
// #include "nvs_flash.h"
// #include "protocol_examples_common.h"
// #include "sdkconfig.h"
// #include "driver/uart.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "hal/uart_types.h"
// #include "openthread/backbone_router_ftd.h"
// #include "openthread/border_router.h"
// #include "openthread/cli.h"
// #include "openthread/dataset.h"
// #include "openthread/dataset_ftd.h"
// #include "openthread/dataset_updater.h"
// #include "openthread/error.h"
// #include "openthread/instance.h"
// #include "openthread/ip6.h"
// #include "openthread/logging.h"
// #include "openthread/tasklet.h"
// #include "openthread/thread_ftd.h"

// #define TAG "esp_ot_br"


// #if CONFIG_OPENTHREAD_BR_AUTO_START
// static int hex_digit_to_int(char hex)
// {
//     if ('A' <= hex && hex <= 'F') {
//         return 10 + hex - 'A';
//     }
//     if ('a' <= hex && hex <= 'f') {
//         return 10 + hex - 'a';
//     }
//     if ('0' <= hex && hex <= '9') {
//         return hex - '0';
//     }
//     return -1;
// }

// static size_t hex_string_to_binary(const char *hex_string, uint8_t *buf, size_t buf_size)
// {
//     int num_char = strlen(hex_string);

//     if (num_char != buf_size * 2) {
//         return 0;
//     }
//     for (size_t i = 0; i < num_char; i += 2) {
//         int digit0 = hex_digit_to_int(hex_string[i]);
//         int digit1 = hex_digit_to_int(hex_string[i + 1]);

//         if (digit0 < 0 || digit1 < 0) {
//             return 0;
//         }
//         buf[i / 2] = (digit0 << 4) + digit1;
//     }

//     return buf_size;
// }

// static void create_config_network(otInstance *instance)
// {
//     otOperationalDataset dataset;

//     if (otDatasetGetActive(instance, &dataset) == OT_ERROR_NONE) {
//         ESP_LOGI(TAG, "Already has network, skip configuring OpenThread network.");
//         return;
//     }

//     uint16_t network_name_len = strlen(CONFIG_OPENTHREAD_NETWORK_NAME);

//     assert(network_name_len <= OT_NETWORK_NAME_MAX_SIZE);

//     if (otDatasetCreateNewNetwork(instance, &dataset) != OT_ERROR_NONE) {
//         ESP_LOGE(TAG, "Failed to create OpenThread network dataset.");
//         abort();
//     }
//     dataset.mChannel = CONFIG_OPENTHREAD_NETWORK_CHANNEL;
//     dataset.mComponents.mIsChannelPresent = true;
//     dataset.mPanId = CONFIG_OPENTHREAD_NETWORK_PANID;
//     dataset.mComponents.mIsPanIdPresent = true;
//     memcpy(dataset.mNetworkName.m8, CONFIG_OPENTHREAD_NETWORK_NAME, network_name_len);
//     dataset.mComponents.mIsNetworkNamePresent = true;
//     if (hex_string_to_binary(CONFIG_OPENTHREAD_NETWORK_EXTPANID, dataset.mExtendedPanId.m8,
//                              sizeof(dataset.mExtendedPanId.m8)) != sizeof(dataset.mExtendedPanId.m8)) {
//         ESP_LOGE(TAG, "Cannot convert OpenThread extended pan id. Please double-check your config.");
//         abort();
//     }
//     dataset.mComponents.mIsExtendedPanIdPresent = true;
//     if (hex_string_to_binary(CONFIG_OPENTHREAD_NETWORK_MASTERKEY, dataset.mNetworkKey.m8,
//                              sizeof(dataset.mNetworkKey.m8)) != sizeof(dataset.mNetworkKey.m8)) {
//         ESP_LOGE(TAG, "Cannot convert OpenThread master key. Please double-check your config.");
//         abort();
//     }
//     dataset.mComponents.mIsNetworkKeyPresent = true;
//     if (hex_string_to_binary(CONFIG_OPENTHREAD_NETWORK_PSKC, dataset.mPskc.m8, sizeof(dataset.mPskc.m8)) !=
//             sizeof(dataset.mPskc.m8)) {
//         ESP_LOGE(TAG, "Cannot convert OpenThread pre-shared commissioner key. Please double-check your config.");
//         abort();
//     }
//     dataset.mComponents.mIsPskcPresent = true;
//     if (otDatasetSetActive(instance, &dataset) != OT_ERROR_NONE) {
//         ESP_LOGE(TAG, "Failed to set OpenThread active dataset.");
//         abort();
//     }
//     return;
// }

// static void launch_openthread_network(otInstance *instance)
// {
//     if (otIp6SetEnabled(instance, true) != OT_ERROR_NONE) {
//         ESP_LOGE(TAG, "Failed to enable OpenThread IP6 link");
//         abort();
//     }
//     if (otThreadSetEnabled(instance, true) != OT_ERROR_NONE) {
//         ESP_LOGE(TAG, "Failed to enable OpenThread");
//         abort();
//     }
//     if (otBorderRouterRegister(instance) != OT_ERROR_NONE) {
//         ESP_LOGE(TAG, "Failed to register border router.");
//         abort();
//     }
//     otBackboneRouterSetEnabled(instance, true);
// }
// #endif // CONFIG_OPENTHREAD_BR_AUTO_START

// static void ot_task_worker(void *aContext)
// {
//     esp_openthread_platform_config_t config = {
//         .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
//         .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
//         .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
//     };

//     esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
//     esp_netif_t       *openthread_netif = esp_netif_new(&cfg);
//     assert(openthread_netif != NULL);
//     // Initialize the OpenThread stack

//     ESP_ERROR_CHECK(esp_openthread_init(&config));

//     // Initialize border routing features
//     esp_openthread_lock_acquire(portMAX_DELAY);
//     ESP_ERROR_CHECK(esp_netif_attach(openthread_netif, esp_openthread_netif_glue_init(&config)));

//     (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
//     esp_openthread_cli_init();
// #if CONFIG_OPENTHREAD_BR_AUTO_START
//     ESP_ERROR_CHECK(esp_openthread_border_router_init());
//     create_config_network(esp_openthread_get_instance());
//     launch_openthread_network(esp_openthread_get_instance());
// #endif // CONFIG_OPENTHREAD_BR_AUTO_START
//     esp_cli_custom_command_init();
//     esp_openthread_lock_release();

//     // Run the main loop
//     esp_openthread_cli_create_task();
//     esp_openthread_launch_mainloop();

//     // Clean up
//     esp_netif_destroy(openthread_netif);
//     esp_openthread_netif_glue_deinit();
//     esp_vfs_eventfd_unregister();
//     vTaskDelete(NULL);
// }

// void app_main(void)
// {
//     // Used eventfds:
//     // * netif
//     // * task queue
//     // * border router
//     esp_vfs_eventfd_config_t eventfd_config = {
//         .max_fds = 3,
//     };
//     ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());

//     #if CONFIG_OPENTHREAD_BR_AUTO_START
//         ESP_ERROR_CHECK(example_connect());
//         ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
//         esp_openthread_set_backbone_netif(get_example_netif());
//     #else
//         esp_ot_wifi_netif_init();
//         esp_openthread_set_backbone_netif(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"));
//     #endif // CONFIG_OPENTHREAD_BR_AUTO_START
//         ESP_ERROR_CHECK(mdns_init());
//         ESP_ERROR_CHECK(mdns_hostname_set("esp-ot-br"));
//         //OT_BR_INIT
//         xTaskCreate(ot_task_worker, "ot_br_main", 20480, xTaskGetCurrentTaskHandle(), 5, NULL);
// }
