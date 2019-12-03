/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "mcp23017.h"
// For WiFi
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "lwip/sockets.h"

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY
#define PORT CONFIG_EXAMPLE_PORT


#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

// OE = 19, CE = 18, WE = 17
#define GPIO_OE 19
#define GPIO_CE 18
#define GPIO_WE 17
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OE) | (1ULL<<GPIO_CE) | (1ULL << GPIO_WE))

uint16_t ReadFlash(int address);
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event 
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

static const char *TAG = "flashtality";

static int s_retry_num = 0;

static gpio_num_t i2c_gpio_sda = 21;
static gpio_num_t i2c_gpio_scl = 22;
static uint32_t i2c_frequency = 400000;
static i2c_port_t i2c_port = I2C_NUM_0;
esp_err_t i2c_err = ESP_OK;

//I2C Addresses of our MCP chips
int MCP_ADDR_1 = 0x20;
int MCP_ADDR_2 = 0x21;
int MCP_DATA = 0x22;

static void process_cmd(const int sock)
{
    int len;
    char rx_buffer[128];
    char * cmd_tok;
    char * start_addr;
    char * size_str;
    int addr,size;
    

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);
            cmd_tok = strtok(rx_buffer,":");
            if(cmd_tok == NULL){
                ESP_LOGI(TAG,"Invalid command recieved!");
                return;
            }
            if(!strcmp(cmd_tok,"r")){
                start_addr = strtok(NULL,":");  
                if(start_addr == NULL){
                    ESP_LOGI(TAG,"Invalid start address! Exiting command loop now");
                    return;
                }
                ESP_LOGI(TAG,"start_addr: %s",start_addr);
                addr = strtol(start_addr,NULL,16);
                ESP_LOGI(TAG,"addr: 0x%x",addr);
                size_str = strtok(NULL,":");  
                if (size_str == NULL){
                    ESP_LOGI(TAG,"Invalid size! Exiting command loop now");
                    return;
                }
                ESP_LOGI(TAG,"size: %s",size_str);
                size = strtol(size_str,NULL,16);
                ESP_LOGI(TAG,"size: 0x%x",size);
            }else{
                ESP_LOGI(TAG,"Command not recognized");
                return;
            }
            int to_write = size;
            while(to_write>0){
                uint16_t dword = ReadFlash((size-to_write)/2);
                int written = send(sock, &dword, sizeof(dword), 0);
                if(written<0){
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
            /*
            }
            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation. 
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
            */
        }
    } while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;


#ifdef CONFIG_EXAMPLE_IPV4
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
    struct sockaddr_in6 dest_addr;
    bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(PORT);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
    inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Convert ip address to string
        if (source_addr.sin6_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        } else if (source_addr.sin6_family == PF_INET6) {
            inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        process_cmd(sock);

        //shutdown(sock, 0);
        //close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

static void event_handler(void* arg, esp_event_base_t event_base, 
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));


    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

// Initialize and install the I2C driver
static void i2c_init(){
    int i2c_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = i2c_gpio_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = i2c_gpio_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = i2c_frequency;
    i2c_param_config(i2c_port,&conf);
    i2c_driver_install(i2c_port,conf.mode,0,0,0);
}

//Write a value to a command register
esp_err_t write_mcp_register(int mcp_addr,int reg_addr, int reg_value){
    esp_err_t i2c_ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,mcp_addr<<1|WRITE_BIT,1);
    i2c_master_write_byte(cmd,reg_addr,1);
    i2c_master_write_byte(cmd,reg_value,1);
    i2c_master_stop(cmd);
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_0,cmd,(1000/portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    return i2c_ret;
}

void mcp_configure(){
    // Set the bits of the first MCP chip to output mode (addr 0:15)
    write_mcp_register(MCP_ADDR_1,GPIOA,0);
    write_mcp_register(MCP_ADDR_1,GPIOB,0);
    write_mcp_register(MCP_ADDR_1,IODIRA,0);
    write_mcp_register(MCP_ADDR_1,IODIRB,0);

    // Set the bits of the second MCP chip to output mode (addr 16:20)
    write_mcp_register(MCP_ADDR_2,GPIOA,0);
    write_mcp_register(MCP_ADDR_2,GPIOB,0);
    write_mcp_register(MCP_ADDR_2,IODIRA,0);
    write_mcp_register(MCP_ADDR_2,IODIRB,0);


    // Configure the pins to be connected to the data lines to be input pins
    write_mcp_register(MCP_DATA,IODIRA,0xFF);;
    write_mcp_register(MCP_DATA,IODIRB,0xFF);
}

void gpio_configure(){
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}
void MXICWriteAddr(int address){
    write_mcp_register(MCP_ADDR_1,GPIOA,(address&0xFF));
    write_mcp_register(MCP_ADDR_1,GPIOB,((address>>8)&0xFF));
    write_mcp_register(MCP_ADDR_2,GPIOA,((address>>16)&0xFF));
}

uint16_t MXICReadData(int mcp_addr,int flash_addr){
    uint8_t dath = 0;
    uint8_t datl = 0;
    uint16_t dword = 0;
    esp_err_t i2c_ret = ESP_OK;
    // Perform a write operation to tell the IC what register we want to read from
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,mcp_addr<<1|WRITE_BIT,1);
    i2c_master_write_byte(cmd,GPIOA,1);
    i2c_master_stop(cmd);
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_0,cmd,(.1/portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    // Now read from the IC at the given register
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,mcp_addr<<1|READ_BIT,1);
    i2c_master_read_byte(cmd,&dath,0);
    i2c_master_read_byte(cmd,&datl,1);
    i2c_master_stop(cmd);
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_0,cmd,(.1/portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    dword = dath;
    dword = dword << 8;
    dword |= datl;
    return dword;
    /*
    printf("%X:%X:",dath,datl);
    if(flash_addr % 16 == 0){
        printf("\r\n");
    }
    */
}

uint16_t ReadFlash(int addr){
    // Enable the chip as well as output!
    uint16_t dword = 0;
    gpio_set_level(GPIO_CE,0);
    gpio_set_level(GPIO_OE,0);
    MXICWriteAddr(addr);
    dword = MXICReadData(MCP_DATA,addr);
    gpio_set_level(GPIO_CE,1);
    gpio_set_level(GPIO_OE,1);
    return dword;
}

void app_main(void)
{
    // Set up and configure WiFi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    esp_netif_init();
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    // Now dump flash
    uint16_t data = 0; 
    i2c_init();
    gpio_configure();
    mcp_configure();
    int addr =0;
    gpio_set_level(GPIO_CE,1);
    gpio_set_level(GPIO_OE,1);
    /*
    unsigned int uptime = esp_log_timestamp();
    printf("\r\nStarting Flash Dump\r\n");
    for(addr=0;addr<0x200000;addr++){
        ReadFlash(addr);
    }
    unsigned int endtime = esp_log_timestamp();
    */
}
