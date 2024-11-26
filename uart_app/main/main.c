#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_ota_ops.h"

#define BAUD_RATE       115200
#define UART_PORT_NUM   UART_NUM_1
#define UART_TX         GPIO_NUM_10
#define UART_RX         GPIO_NUM_11
#define BUF_SIZE        1024
#define TASK_STACK_SIZE 1024

static const char *TAG = "UART";
static QueueHandle_t uart_queue;
static volatile bool task_running = false;

const char *command_request = "\nLista de comandos disponibles:\n--------------------------------\n1.-Cargar\n2.-Crear programa\n--------------------------------\n";
const char *load_request = "\nPrograma a cargar:\n--------------------------------\n1.-LED_BLINK\n2.-RGB\n--------------------------------\n";
const char *program_request = "\nConfiguracion de programa\nOpcciones de configuracion:\n--------------------------------\n1.-Salir\n";

esp_err_t init_uart(void);
void task_uart(void *param);
void second_read(void);
void read_command(void);
void switch_to_partition(esp_partition_subtype_t target_subtype);

void app_main(void)
{
    init_uart();
}

esp_err_t init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate  = BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 5, &uart_queue, 0)); 
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreate(task_uart, "task_uart", TASK_STACK_SIZE * 3, NULL, 3, NULL);
    return ESP_OK;
}

void task_uart(void *param)
{
    while (true)
    {
        uart_event_t event;
        uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
        uart_write_bytes(UART_PORT_NUM, command_request, strlen(command_request));
        bzero(data,BUF_SIZE);
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
        {
            if (event.type == UART_DATA && !task_running)
            {
                task_running = true;
                int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
                data[len-1] = '\0'; 
                    if (strcmp((char *)data, "1") == 0)
                    {
                        second_read();
                    }
                    else if (strcmp((char *)data, "2") == 0)
                    {
                        read_command();
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Unrecognized command");
                    }
                task_running = false;
            }
        }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void second_read(void)
{
    uart_event_t event;
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uart_write_bytes(UART_PORT_NUM, load_request, strlen(load_request));
    bzero(data,BUF_SIZE);
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
    {
        if (event.type == UART_DATA)
        {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        data[len-1] = '\0';
            if (strcmp((char *)data, "1") == 0)
            {
                switch_to_partition(ESP_PARTITION_SUBTYPE_APP_OTA_1);
            }
            else if (strcmp((char *)data, "2") == 0)
            {
                switch_to_partition(ESP_PARTITION_SUBTYPE_APP_OTA_2);
            }
            else
            {
                ESP_LOGI(TAG, "Unrecognized command");
            }
        }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void switch_to_partition(esp_partition_subtype_t target_subtype) {
    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, target_subtype, NULL);
    if (partition != NULL) {
        esp_err_t err = esp_ota_set_boot_partition(partition);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Reboot in progress...");
            esp_restart();
        } else {
            ESP_LOGE(TAG, "Error changing partition: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "The requested partition was not found");
    }
}

void read_command(void)
{
    uart_event_t event;
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    while (true)
    {
    uart_write_bytes(UART_PORT_NUM, program_request, strlen(program_request));
    bzero(data,BUF_SIZE);
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
    {
        if (event.type == UART_DATA)
        {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        data[len-1] = '\0';
            if (strcmp((char *)data, "1") == 0)
            {
                break;
            }
            else if (strcmp((char *)data, "2") == 0)
            {
                ESP_LOGI(TAG, "command_try");
            }
            else
            {
                ESP_LOGI(TAG, "Unrecognized command");
            }
        }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
