/* ESP32 Firebase Data Sender with SPI Receiver
 * This program connects to WiFi, receives data from STM32 via SPI,
 * and sends it to Firebase in real-time.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "esp_timer.h" // For esp_timer_get_time()
#include "esp_crt_bundle.h" // Added for esp_crt_bundle_attach
#include "driver/spi_slave.h" // For SPI slave communication

// WiFi Configuration
#define WIFI_SSID "Aykut"
#define WIFI_PASS "12345678"
#define MAX_RETRY 5

// Firebase project settings
#define FIREBASE_HOST "us-tag-project-default-rtdb.europe-west1.firebasedatabase.app"
#define FIREBASE_DATABASE_SECRET "d8JQQjymzk1Thda3A2Z7dXL2P3WycFSBs4P9G5cR" // Your Firebase Database Secret
#define FIREBASE_DATA_PATH "/real_time_data.json" // Path for real-time data

// SPI Configuration
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS   5

// SPI DMA buffer size (matches STM32's ADC_BUF_SIZE * 2 values * 2 bytes per value)
#define ADC_BUF_SIZE 700
#define SPI_BUFFER_SIZE (ADC_BUF_SIZE * 4) // 700 samples * 2 values (ADC1+ADC2) * 2 bytes each

static const char *TAG = "ESP32_SPI_FIREBASE";
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
const int WIFI_FAIL_BIT = BIT1; // New bit for WiFi failure

// SPI data buffer
static uint8_t spi_rx_buffer[SPI_BUFFER_SIZE];
static uint16_t adc1_values[ADC_BUF_SIZE];
static uint16_t adc2_values[ADC_BUF_SIZE];
static SemaphoreHandle_t spi_data_ready_semaphore;

// WiFi event handler
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Attempting to connect to WiFi...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP (%d/%d)", s_retry_num, MAX_RETRY);
        } else {
            ESP_LOGE(TAG, "Failed to connect to the AP after %d attempts.", MAX_RETRY);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); // Set FAIL bit
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Set CONNECTED bit
    }
}

// Initialize WiFi
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif); // Ensure netif was created

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
    
    ESP_LOGI(TAG, "wifi_init_sta finished. Waiting for connection...");
    
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, // Wait for EITHER connected or fail
                                           pdFALSE, // Don't clear bits on exit
                                           pdFALSE, // Wait for any bit (OR logic)
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s after %d retries. Data sending tasks might not run.", WIFI_SSID, MAX_RETRY);
    } else {
        ESP_LOGE(TAG, "Unexpected event from xEventGroupWaitBits while waiting for WiFi.");
    }
}

// Initialize SPI in slave mode
void spi_slave_init(void)
{
    ESP_LOGI(TAG, "Initializing SPI slave mode...");
    
    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    
    // Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,                      // SPI mode 0
        .spics_io_num = GPIO_CS,        // CS pin
        .queue_size = 3,                // Queue size
        .flags = 0,                     // No flags specified
        .post_setup_cb = NULL,          // No callback after setup
        .post_trans_cb = NULL           // No callback after transaction
    };
    
    // Initialize SPI slave
    ESP_ERROR_CHECK(spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI slave initialized successfully");
}

// SPI transaction ready callback
void spi_trans_ready(void *arg)
{
    // Notify that new data is available
    xSemaphoreGiveFromISR(spi_data_ready_semaphore, NULL);
}

// HTTP event handler for Firebase requests
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
            if (evt->data && evt->data_len > 0) { // Check if error data is available
                 ESP_LOGE(TAG, "Error data: %.*s", evt->data_len, (char*)evt->data);
            }
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGI(TAG, "HTTP_EVENT_REDIRECT");
            break;
    }
    return ESP_OK;
}

// Send ADC data to Firebase
// Modified function to send data to Firebase correctly
void send_to_firebase(uint16_t* adc1_data, uint16_t* adc2_data, size_t data_len)
{
    uint64_t timestamp = esp_timer_get_time() / 1000;
    
    // Create the JSON payload
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to create cJSON root.");
        return;
    }
    
    // Add timestamp and metadata
    cJSON_AddNumberToObject(root, "timestamp", timestamp);
    cJSON_AddNumberToObject(root, "sample_count", data_len);
    
    // Create arrays for ADC1 and ADC2 data
    cJSON *adc1_array = cJSON_CreateArray();
    cJSON *adc2_array = cJSON_CreateArray();
    if (adc1_array == NULL || adc2_array == NULL) {
        ESP_LOGE(TAG, "Failed to create cJSON arrays.");
        cJSON_Delete(root);
        return;
    }
    
    cJSON_AddItemToObject(root, "adc1", adc1_array);
    cJSON_AddItemToObject(root, "adc2", adc2_array);
    
    // Add all data points
    for (int i = 0; i < data_len; i++) {
        cJSON_AddItemToArray(adc1_array, cJSON_CreateNumber(adc1_data[i]));
        cJSON_AddItemToArray(adc2_array, cJSON_CreateNumber(adc2_data[i]));
    }

    char *post_data = cJSON_PrintUnformatted(root);
    if (!post_data) {
        ESP_LOGE(TAG, "Failed to print JSON data.");
        cJSON_Delete(root);
        return;
    }

    ESP_LOGI(TAG, "Sending %d data points to Firebase, timestamp: %llu", data_len, timestamp);

    // CHANGE 1: Use POST instead of PUT for this path
    // CHANGE 2: Format the URL correctly for Firebase REST API
    char path_with_auth[256];
    sprintf(path_with_auth, "/sensor_data.json?auth=%s", FIREBASE_DATABASE_SECRET);

    esp_http_client_config_t config = {
        .host = FIREBASE_HOST,
        .path = path_with_auth,
        .event_handler = _http_event_handler,
        .method = HTTP_METHOD_POST,  // Changed from PUT to POST
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client.");
        cJSON_Delete(root);
        free(post_data);
        return;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "HTTP POST Status = %d, data points: %d", status_code, data_len);
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s (0x%x)", esp_err_to_name(err), err);
    }

    esp_http_client_cleanup(client);
    cJSON_Delete(root);
    free(post_data);
    
    // Also update a "latest" node for quick access
    // Using PUT is correct for this specific named node
    sprintf(path_with_auth, "/sensor_data_latest.json?auth=%s", FIREBASE_DATABASE_SECRET);
    
    cJSON *latest = cJSON_CreateObject();
    if (latest != NULL) {
        cJSON_AddNumberToObject(latest, "timestamp", timestamp);
        
        // Add the latest sample from each ADC
        if (data_len > 0) {
            cJSON_AddNumberToObject(latest, "latest_adc1", adc1_data[data_len-1]);
            cJSON_AddNumberToObject(latest, "latest_adc2", adc2_data[data_len-1]);
        }
        
        char *latest_data = cJSON_PrintUnformatted(latest);
        if (latest_data) {
            config.path = path_with_auth;
            config.method = HTTP_METHOD_PUT;  // PUT is correct for replacing a node
            client = esp_http_client_init(&config);
            if (client != NULL) {
                esp_http_client_set_header(client, "Content-Type", "application/json");
                esp_http_client_set_post_field(client, latest_data, strlen(latest_data));
                esp_http_client_perform(client);
                esp_http_client_cleanup(client);
            }
            free(latest_data);
        }
        cJSON_Delete(latest);
    }
}

// SPI receiver task
void spi_receiver_task(void *pvParameters)
{
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    if (!(bits & WIFI_CONNECTED_BIT) || (bits & WIFI_FAIL_BIT)) {
        ESP_LOGE(TAG, "WiFi not connected or failed. SPI receiver task will not run.");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "WiFi connected. Starting SPI receiver task.");
    
    // Transaction structure for SPI transfer
    spi_slave_transaction_t transaction = {
        .length = SPI_BUFFER_SIZE * 8,     // Length in bits
        .rx_buffer = spi_rx_buffer,        // Buffer for received data
        .tx_buffer = NULL,                 // We're not sending data
        .user = NULL,                      // No user data
    };
    
    while (1) {
        // Queue transaction and wait for execution
        ESP_LOGI(TAG, "Waiting for SPI data...");
        
        // Start SPI transaction
        esp_err_t ret = spi_slave_queue_trans(HSPI_HOST, &transaction, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to queue SPI transaction: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Wait for transaction to complete
        spi_slave_transaction_t *trans_result;
        ret = spi_slave_get_trans_result(HSPI_HOST, &trans_result, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get SPI transaction result: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        ESP_LOGI(TAG, "SPI data received! Processing %d bytes", SPI_BUFFER_SIZE);
        
        ESP_LOGI(TAG, "Raw SPI bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                spi_rx_buffer[0], spi_rx_buffer[1], spi_rx_buffer[2], spi_rx_buffer[3],
                spi_rx_buffer[4], spi_rx_buffer[5], spi_rx_buffer[6], spi_rx_buffer[7]);

        // The STM32 is sending little-endian data with byte order: low byte, high byte
        for (int i = 0; i < ADC_BUF_SIZE; i++) {
            // Match the exact byte order from STM32: E4 0C E4 0C pattern
            adc1_values[i] = (spi_rx_buffer[i*4+1] << 8) | spi_rx_buffer[i*4];
            adc2_values[i] = (spi_rx_buffer[i*4+3] << 8) | spi_rx_buffer[i*4+2];
        }
        
        // Print a few samples for debugging
        ESP_LOGI(TAG, "Sample data - ADC1[0]: %d, ADC2[0]: %d", adc1_values[0], adc2_values[0]);
        
        // Send to Firebase 
        send_to_firebase(adc1_values, adc2_values, ADC_BUF_SIZE);
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

    ESP_LOGI(TAG, "ESP32 SPI-Firebase Example: Initializing...");
    
    // Initialize semaphore for SPI data
    spi_data_ready_semaphore = xSemaphoreCreateBinary();
    
    // Initialize WiFi
    wifi_init_sta();
    
    // Initialize SPI slave
    spi_slave_init();

    EventBits_t wifi_status_bits = xEventGroupGetBits(s_wifi_event_group);
    if (wifi_status_bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi Connected. Starting SPI receiver task.");
        xTaskCreate(spi_receiver_task, "spi_receiver", 8192, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "WiFi connection failed. SPI receiver task will not be started.");
    }
    
    ESP_LOGI(TAG, "app_main finished setup.");
}