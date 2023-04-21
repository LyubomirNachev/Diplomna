// Standard C libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Standard ESP, FreeRTOS, lwIP(Lightweight Ip) and OpenThread libraries
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
#include "esp_eth.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include <esp_http_server.h>
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>
#include <sys/param.h>
#include "ssd1306.h"

#define TAG "esp_ot_br"

// Enable the use of these functions if the the border router is configured with auto start
#if CONFIG_OPENTHREAD_BR_AUTO_START 

// Convert hexadecimal digit to an integer
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

// Convert hexadecimal string to binary data 
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

// Configure the OpenThread network
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

// Start the OpenThread network
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

// Initialize and run the OpenThread stack
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

char rx_buffer[128];
int len;
char substring[20];
//Initialize the UDP socket server
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

    inet6_aton("::", &listen_addr.sin6_addr); // Convert the unspecified address from string to binary number
    listen_addr.sin6_family = AF_INET6;
    listen_addr.sin6_port = htons(port);

    listen_sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP); // socket(domain,type,protocol)   
    ESP_GOTO_ON_FALSE((listen_sock >= 0), ESP_OK, exit, TAG, "Unable to create socket: errno %d", errno);
    ESP_LOGI(TAG, "Socket created");

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)); // Allow the socket to bind to an address that is already in use
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt)); // Disable the dual-stack setting and use only IPv6

    err = bind(listen_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr));
    ESP_GOTO_ON_FALSE((err == 0), ESP_FAIL, exit, TAG, "Socket unable to bind: errno %d", errno);
    ESP_LOGI(TAG, "Socket bound, port %d", port);
    ESP_LOGI(TAG, "Waiting for data, no timeout");
    
    while(1){
        // Configure the timeout for receiving data to 0 seconds
        timeout.tv_sec = 0; 
        setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
        socklen_t socklen = sizeof(source_addr);

        // Get the length of the received data and store it in rx_buffer 
        len = recvfrom(listen_sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);  

        // Error occurred during receiving
        ESP_GOTO_ON_FALSE((len >= 0), ESP_FAIL, exit, TAG, "recvfrom failed: errno %d", errno);

        // Get the sender's ip address as string
        inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);

        rx_buffer[len] = 0; // Null-terminate whatever we received and treat as a string
        ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
        ESP_LOGI(TAG, "%s", rx_buffer);
        int pos = 4;
        int len_get = 16;
        strncpy(substring,rx_buffer+(pos-1),len_get);
        //ESP_LOGI(TAG, "%s", substring);
    }
exit: // On error 
    if (ret != ESP_OK) {
        shutdown(listen_sock, 0);
        close(listen_sock);
    }
    ESP_LOGI(TAG, "Socket server is closed.");
    vTaskDelete(NULL);
}

/* Websocket functions */
httpd_handle_t server = NULL;
// Asynchronous response data structure
struct async_resp_arg {
    httpd_handle_t hd;  // Server instance 
    int fd;             // Session socket file descriptor
};

    char adr_temp[17];
    char addresses1[10][17];
    int i;
// Send an HTTP response asynchronously
char* page_html = "<!DOCTYPE html> <html> <head> <link rel=\"shortcut icon\" href=\"#\"> <meta charset=\"utf-8\"> <title>Data</title> <style> html{ font-family: Segoe, sans-serif; } body{ background: #454545; } .t{ padding: 12px 12px; border: 8px solid whitesmoke; background-image: linear-gradient(to top, #108db5 0%%, #6f86d6 100%%); } p{ padding: 20px 15px; color:whitesmoke; text-transform: uppercase; border-bottom: solid 2px rgba(210,255,255,0.1); } </style> </head> <body> <div class=\"t\"> <p class = \"data1\"> Data1: <span id=\"data1\"> \"%d\" </span> </p> <p class = \"data2\"> Data2: <span id=\"data2\"> \"%d\" </span> </p> <p class = \"data3\"> Data3: <span id=\"data3\"> \"%d\" </span> </p></div> <script> let gateway = new WebSocket(`ws://${window.location.hostname}/ws`); gateway.onopen = function (event) { console.log('Connection opened'); }; gateway.onmessage = function (event) { console.log(event.data); let data = event.data; if((data.substring(3,19)) == \"335E5C3FC13D147D\") {document.getElementById('data1').innerHTML = data;}else if((data.substring(3,19)) == \"565C82D311ACA88E\"){ document.getElementById('data2').innerHTML = data;} else if((data.substring(3,19))== \"EEDDAE09C3BE1366\"){ document.getElementById('data3').innerHTML = data;} }; gateway.onclose = function(event) { console.log(\"oh oh\") }; setInterval(function(){gateway.send(\"update data\");},3000);</script> </body></html>";
char page[2000];

static void ws_async_send(void *arg) 
{
    char data1[250];
    char data2[250];
    char data3[250];
    char* addresses[3] = { "335E5C3FC13D147D", "565C82D311ACA88E", "EEDDAE09C3BE1366" };
    char buff[800];
    memset(buff, 0, sizeof(buff));
    if (!strcmp(addresses[0],substring)){
        memcpy(data1, rx_buffer, strlen(rx_buffer)+1);
        sprintf(buff, "%s", data1);
    }else if (!strcmp(addresses[1],substring)){
        memcpy(data2, rx_buffer, strlen(rx_buffer)+1);
        sprintf(buff, "%s", data2);
    }else if (!strcmp(addresses[2],substring)){
        memcpy(data3, rx_buffer, strlen(rx_buffer)+1);
        sprintf(buff, "%s", data3);
    }
    httpd_ws_frame_t ws_pkt;
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    
    
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t *)buff;
    ws_pkt.len = strlen(buff);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    static size_t max_clients = CONFIG_LWIP_MAX_LISTENING_TCP;
    size_t fds = max_clients;
    int client_fds[max_clients];

    esp_err_t ret = httpd_get_client_list(server, &fds, client_fds);

    if (ret != ESP_OK) {
        return;
    }
    for (int i = 0; i < fds; i++) {
        int client_info = httpd_ws_get_fd_info(server, client_fds[i]);
        if (client_info == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(hd, client_fds[i], &ws_pkt);
        }
    }
    free(resp_arg);
}



esp_err_t get_req_handler(httpd_req_t *req)
{
    int response;
    sprintf(page, page_html, 1,2,3);
    response = httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
    return response;
}


//int already_established = 0;
// Handle HTTP GET requests asynchronously (enable full duplex comunication)
static esp_err_t async_get_handler(httpd_handle_t handle, httpd_req_t *req) 
{
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    return httpd_queue_work(handle, ws_async_send, resp_arg);
}

// Define the '/ws' URI for the server that is added at the end of the IP
static esp_err_t handle_ws_req(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }

    if (ws_pkt.len)
    {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
    }

    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT)
    {
        free(buf);
        return async_get_handler(req->handle, req);
    }
    return ESP_OK;
}
// Start the web server
httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t uri_get = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_req_handler,
        .user_ctx = NULL};

    httpd_uri_t ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = handle_ws_req,
        .user_ctx = NULL,
        .is_websocket = true};

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &ws);
    }

    return server;
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
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config)); // Initialize the eventfd virtual filesystem
    ESP_ERROR_CHECK(nvs_flash_init()); // Initialize the NVS flash memory
    ESP_ERROR_CHECK(esp_netif_init()); // Create the network interface for Thread
    
    // Create the system Event task and initialize an app event’s callback function
    ESP_ERROR_CHECK(esp_event_loop_create_default()); 

    #if CONFIG_OPENTHREAD_BR_AUTO_START
        ESP_ERROR_CHECK(example_connect()); // Establish a connection with the Wi-Fi network
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Set the power saving mode for Wi-Fi to none
        esp_openthread_set_backbone_netif(get_example_netif()); // Connect OpenThread to the external network
    #else
        esp_ot_wifi_netif_init(); //Set up the communication between OpenThread and the Wi-Fi interface
        esp_openthread_set_backbone_netif(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")); 
    #endif // CONFIG_OPENTHREAD_BR_AUTO_START

        ESP_ERROR_CHECK(mdns_init()); //mDNS initialization 
        ESP_ERROR_CHECK(mdns_hostname_set("esp-ot-br"));

        // Create a task for the OpenThread stack
        xTaskCreate(ot_task_worker, "ot_br_main", 20480, xTaskGetCurrentTaskHandle(), 5, NULL); 

        // Create a task for the UDP socket
        xTaskCreate(udp_socket_server_task, "ot_udp_scoket_server", 4096, xTaskGetCurrentTaskHandle(), 4, NULL);
    
        start_webserver(); // Initialize the websocket server
        ESP_LOGI(TAG, "Socket is running ... ...\n");

        SSD1306_t dev;
        i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);	// I2C master initialization
        ssd1306_init(&dev, 128, 64); // Initialize the display
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);

        // Refresh every 10 seconds and update the display information
        while(1){ 
            char data[16];

            // Get the IPv4 address that is assigned to the ESP32
            ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info)); 

            sprintf(data, "%d.%d.%d.%d", IP2STR(&ip_info.ip));
            ssd1306_display_text(&dev, 1, "IP:          ", 16, false); 
            ssd1306_display_text(&dev, 2, data, 16, false);
            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }
}
