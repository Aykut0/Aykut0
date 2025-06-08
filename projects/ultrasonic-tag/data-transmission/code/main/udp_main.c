#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/spi_slave.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_timer.h"

// WiFi Configuration
#define WIFI_SSID "Aykut"
#define WIFI_PASS "12345678"
#define MAX_RETRY 5

// UDP server configuration
#define UDP_SERVER_IP   "192.168.1.100"  // Change to your PC's IP
#define UDP_SERVER_PORT 12345

// SPI Configuration
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS   5

#define ADC_BUF_SIZE 700
#define SPI_BUFFER_SIZE (ADC_BUF_SIZE * 4) // 700 samples * 2 ADCs * 2 bytes each

static const char *TAG = "ESP32_SPI_UDP";
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
const int WIFI_FAIL_BIT = BIT1;

static uint8_t spi_rx_buffer[SPI_BUFFER_SIZE];
static SemaphoreHandle_t spi_data_ready_semaphore;

static int udp_socket = -1;
static struct sockaddr_in dest_addr;

// WiFi event handler
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP (%d/%d)", s_retry_num, MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void spi_slave_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL
    };

    ESP_ERROR_CHECK(spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI slave initialized");
}

static void udp_client_init(void)
{
    dest_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_SERVER_PORT);

    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "UDP socket created");
    }
}

static void udp_send_data(uint8_t *data, size_t len)
{
    if (udp_socket < 0) return;
    int err = sendto(udp_socket, data, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "Sent %d bytes", err);
    }
}

static void spi_receiver_task(void *pvParameters)
{
    spi_slave_transaction_t transaction = {
        .length = SPI_BUFFER_SIZE * 8,
        .rx_buffer = spi_rx_buffer,
        .tx_buffer = NULL,
        .user = NULL,
    };

    while (1) {
        esp_err_t ret = spi_slave_queue_trans(HSPI_HOST, &transaction, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to queue SPI transaction: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        spi_slave_transaction_t *trans_result;
        ret = spi_slave_get_trans_result(HSPI_HOST, &trans_result, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get SPI transaction result: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        udp_send_data(spi_rx_buffer, SPI_BUFFER_SIZE);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    spi_data_ready_semaphore = xSemaphoreCreateBinary();

    wifi_init_sta();
    spi_slave_init();

    EventBits_t wifi_status_bits = xEventGroupGetBits(s_wifi_event_group);
    if (wifi_status_bits & WIFI_CONNECTED_BIT) {
        udp_client_init();
        xTaskCreate(spi_receiver_task, "spi_receiver", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "WiFi connection failed. SPI receiver task not started");
    }
}
