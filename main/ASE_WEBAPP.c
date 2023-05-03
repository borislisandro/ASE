#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include <sys/param.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include <esp_http_server.h>

#include "driver/gpio.h"

#include <stdio.h>
#include <string.h>

static char upperhalf[] = "<!DOCTYPE html>\
<html>\
<head>\
<style>\
.button {\
  background-color: #919191;\
  border: none;\
  color: white;\
  padding: 15px 32px;\
  text-align: center;\
  text-decoration: none;\
  display: inline-block;\
  font-size: 16px;\
  margin: 4px 2px;\
  cursor: pointer;\
}\
\
</style>\
</head>\
<body>\
\
<h2>ESP-32 HC-SR04</h2>\
\
<h3>Distance measured in the sensor is ";

static char lowerhalf[]= " cm</h3>\
\
\
<button class=\"button button1\" onclick =\"window.location.href='/distance'\" >Update sensor value</button>\
\
</body>\
</html>";

static char temp[1000];

static char distance_str[4] = "NANN";
/********************************* WEB SERVER CODE BEGINS ***********************************/
static const char *TAG = "web server";

/* An HTTP GET handler */
static esp_err_t distance_get_handler(httpd_req_t *req)
{
    esp_err_t error;
    ESP_LOGI(TAG,"distance read");
    strcpy(temp, upperhalf);
    strcat(temp,distance_str);
    strcat(temp,lowerhalf);
    const char *responce = (const char *) req->user_ctx;
    error = httpd_resp_send(req, responce,strlen(responce));
    if( error != ESP_OK ) {
        ESP_LOGI(TAG, "Error %d while sending response",error);
    }
    else ESP_LOGI(TAG, "Response sent Sucessfully");

    return error;
}

static const httpd_uri_t distance = {
    .uri       = "/distance",
    .method    = HTTP_GET,
    .handler   = distance_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = temp
};

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &distance);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK) {
            *server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      "esp_wifi"
#define EXAMPLE_ESP_WIFI_PASS      "password"
#define EXAMPLE_ESP_WIFI_CHANNEL   7
#define EXAMPLE_MAX_STA_CONN       10

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                    .required = false,
            },
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}


#define output_pin 5
#define input_pin 17
#define calibration_constant 10.584615


void GPIO_SETUP(void) {
    esp_rom_gpio_pad_select_gpio(output_pin);
    gpio_set_direction(output_pin,GPIO_MODE_OUTPUT);
    gpio_pulldown_en(output_pin);
    gpio_pullup_dis(output_pin);
    esp_rom_gpio_pad_select_gpio(input_pin);
    gpio_set_direction(input_pin,GPIO_MODE_INPUT);
    gpio_pulldown_en(input_pin);
    gpio_pullup_dis(input_pin);
}


void app_main(void)
{

    static httpd_handle_t server = NULL;

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &connect_handler, &server));
    //ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));

    GPIO_SETUP();

    int duration;
    int distance;

    while (1) {
        duration = 0;

        gpio_set_level(output_pin,0);
        vTaskDelay(1);
        gpio_set_level(output_pin,1);
        vTaskDelay(10);
        gpio_set_level(output_pin,0);

        while(gpio_get_level(input_pin) == 0);
        duration += 1;
        while(gpio_get_level(input_pin) == 1){
            duration += 1;
        }        

        distance = (int)((duration*0.001)*calibration_constant/2);
        sprintf(distance_str, "%d", distance);
        printf("Distance =%d\n",distance);

        vTaskDelay(10);
    }
}
