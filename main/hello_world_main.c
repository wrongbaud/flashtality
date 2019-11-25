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
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "mcp23017.h"

static gpio_num_t i2c_gpio_sda = 21;
static gpio_num_t i2c_gpio_scl = 22;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;
esp_err_t i2c_err = ESP_OK;

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

int MCP_ADDR_1 = 0x20;
int MCP_ADDR_2 = 0x21;
int MCP_DATA = 0x22;
// Initialize and install the driver
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

    // Set the bits of the second MCP chip to output mode (addr 0:15)
    write_mcp_register(MCP_ADDR_2,GPIOA,0);
    write_mcp_register(MCP_ADDR_2,GPIOB,0);
    write_mcp_register(MCP_ADDR_2,IODIRA,0);
    write_mcp_register(MCP_ADDR_2,IODIRB,0);


    // Configure the pins to be connected to the data lines to be input pins
    write_mcp_register(MCP_DATA,IODIRA,0xFF);;
    write_mcp_register(MCP_DATA,IODIRB,0xFF);
    write_mcp_register(MCP_DATA,GPPUA,0xFF);
    write_mcp_register(MCP_DATA,GPPUB,0xFF);
}
//0x46c10?
// OE = 19, CE = 18, WE = 17
#define GPIO_OE 19
#define GPIO_CE 18
#define GPIO_WE 17
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OE) | (1ULL<<GPIO_CE) | (1ULL << GPIO_WE))

void gpio_configure(){
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    // May need to change this?
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}
void MXICWriteAddr(int address){
    write_mcp_register(MCP_ADDR_1,GPIOA,(address&0xFF));
    write_mcp_register(MCP_ADDR_1,GPIOB,((address>>8)&0xFF));
    write_mcp_register(MCP_ADDR_2,GPIOA,((address>>16)&0xFF));
    // Here we're going to write the high two bits high for CE/OE! highest bit being WE#, second being OE#
    //write_mcp_register(MCP_ADDR_2,GPIOB,(((address>>16)&0xFF)|| 0x80));
}

void MXICReadData(int mcp_addr,uint16_t data){
    data = 0;
    uint8_t dath = 0;
    uint8_t datl = 0;
    esp_err_t i2c_ret = ESP_OK;
    // Perform a write operation to tell the IC what register we want to read from
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,mcp_addr<<1|WRITE_BIT,1);
    i2c_master_write_byte(cmd,GPIOA,1);
    i2c_master_stop(cmd);
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_0,cmd,(1000/portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    // Now read from the IC at the given register
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,mcp_addr<<1|READ_BIT,1);
    i2c_master_read_byte(cmd,&dath,0);
    i2c_master_read_byte(cmd,&datl,1);
    i2c_master_stop(cmd);
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_0,cmd,(1000/portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    printf("0x%02X 0x%02X\r\n",dath,datl);
    data = dath;
    data = data << 8;
    data |= datl;
    printf("Data: %02X\r\n",data);
}

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
            CHIP_NAME,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    uint16_t data = 0; 
    i2c_init();
    gpio_configure();
    mcp_configure();
    // Pull WE High D0
    /*
    gpio_set_level(GPIO_CE,1);
    gpio_set_level(GPIO_WE,1);
    gpio_set_level(GPIO_OE,1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // Pull CE Low D1 
    gpio_set_level(GPIO_CE,0);
    MXICWriteAddr(0x123456);
    gpio_set_level(GPIO_OE,0);
    // Pull OE Low D2
    printf("Data: %02X\r\n",data);
    gpio_set_level(GPIO_CE,1);
    gpio_set_level(GPIO_OE,1);
    */
    MXICReadData(MCP_DATA,data);
    printf("0x%02X\r\n",data);
    printf("Restarting now.\n");
    fflush(stdout);
    //esp_restart();
}
