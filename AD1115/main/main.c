#include <stdio.h>
#include <string.h>
#include <stdlib.h>  
#include <math.h>
#include "ADS1115.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/adc.h"

#define I2C_MASTER_FREQ_HZ     100000
#define I2C_MASTER_SCL         GPIO_NUM_12
#define I2C_MASTER_SDA         GPIO_NUM_11 
#define I2C_PORT               I2C_NUM_0
#define ADS1115_ADDR           0x48

#define BAUD_RATE              115200
#define UART_PORT_NUM          UART_NUM_1
#define UART_TX_PIN            GPIO_NUM_17
#define UART_RX_PIN            GPIO_NUM_16
#define UART_BUF_SIZE          1024
#define TASK_STACK_SIZE        1024
#define QUEUE_SIZE             5

//#define R_REF                  10000000
//#define R_REF                  1500000
#define R_REF                  99000
//#define R_REF                  1000

#define ADC_CHANNEL            ADC_CHANNEL_0
#define ADC_CHANNEL_VREF       ADC_CHANNEL_1 
#define ADC_ATTEN              ADC_ATTEN_DB_12
#define ADC_WIDTH              ADC_WIDTH_BIT_12

#define VOLTAGE_HIGH           4000
#define VOLTAGE_MEDIUM_HIGH    2000
#define VOLTAGE_MEDIUM_LOW     1000
#define VOLTAGE_LOW            500
#define VOLTAGE_VERY_LOW       200

static const char *TAG = "App_ADS";

static QueueHandle_t uart_queue;
static QueueHandle_t dataQueue;
static adc_cali_handle_t cali_handle = NULL;

static volatile bool task_running = false;

static const char *initial_request = "\nLista de comandos disponibles:\n--------------------------------\n1.-Medir voltaje\n2.-Medir resitencia\n--------------------------------\n";
static const char *clear = "\033c";
static const char *return_prompt  = "\nVolver al menu principal (y/n)\n--------------------------------\n\n";
/*
double kalman_estimate = 0.0;   
double kalman_error_estimate = 1.0; 
const double kalman_Q = 0.01;         
const double kalman_R = .1;          
*/
#define AVERAGE_WINDOW_SIZE 3  
#define MEDIAN_FILTER_WINDOW_SIZE 5

static double voltage_buffer[MEDIAN_FILTER_WINDOW_SIZE];
static double average_buffer[AVERAGE_WINDOW_SIZE];
static int buffer_index = 0;
static int avg_index = 0;


esp_err_t i2c_init(void);
esp_err_t init_uart(void);
void ads1115_task(void *param);
void task_uart(void *param);
void read_voltage(void);
void read_resistor(void);
static bool init_adc_calibration(void);
ads1115_fsr_t calibration_adc(void);
double calibration_adc_ref(void);
//double kalman_filter(double measurement);
int compare(const void *a, const void *b);
double median_weighted_average(double *buffer, int size);
double apply_median_average_filter(double new_voltage);

void app_main() {
    dataQueue = xQueueCreate(QUEUE_SIZE, sizeof(double));
    if (i2c_init() == ESP_OK && init_uart() == ESP_OK) {
        xTaskCreate(ads1115_task, "ads1115_task", TASK_STACK_SIZE * 3, NULL, 5, NULL);
        xTaskCreate(task_uart, "task_uart", TASK_STACK_SIZE * 3, NULL, 4, NULL);
        ESP_LOGI(TAG, "Application initialized");
    } else {
        ESP_LOGE(TAG, "Error initializing peripherals");
    }
}

esp_err_t i2c_init(void)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    esp_err_t ret = i2c_param_config(I2C_PORT, &i2c_config);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_param_config(UART_PORT_NUM, &uart_config);
    if (ret != ESP_OK) return ret;

    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ret = uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 5, &uart_queue, 0);
    return ret;
}

void ads1115_task(void *param)
{
    
    ads1115_t ads = ads1115_config(I2C_PORT, ADS1115_ADDR);
    
    ads1115_set_rdy_pin(&ads, GPIO_NUM_5);
    ads1115_set_mux(&ads, ADS1115_MUX_0_1);
    ads1115_set_mode(&ads, ADS1115_MODE_CONTINUOUS);
    ads1115_set_sps(&ads, ADS1115_SPS_8);
    ads1115_set_max_ticks(&ads, pdMS_TO_TICKS(100));
    while (true)
    {
    ESP_LOGI("Valor del mux", "Valor return: %d", calibration_adc());
    ads1115_set_pga(&ads, calibration_adc());
    uint16_t count = 5;
    int16_t raw_value;
    double voltage;
    double filtered_voltage;
    for (size_t i = 0; i < count; i++)
    {
    raw_value = ads1115_get_raw(&ads);
    voltage = ads1115_get_voltage(&ads);
    //double filtered_voltage = kalman_filter(voltage);
    filtered_voltage = apply_median_average_filter(voltage);
    }
    ESP_LOGI("ADS","Valor crudo: %d\n", raw_value);
    ESP_LOGI("ADS","Voltaje: %f V\n", voltage);
    ESP_LOGI("Kalman", "Voltaje filtrado: %f V\n", filtered_voltage);

    if (xQueueSend(dataQueue, &filtered_voltage, portMAX_DELAY) != pdPASS) {
        printf("Error sending data to queue\n");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task_uart(void *param)
{
    uart_event_t event;
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Error de memoria");
        return;
    }
    while (true){
    uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
    uart_write_bytes(UART_PORT_NUM, initial_request, strlen(initial_request));
    bzero(data,UART_BUF_SIZE);
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)){
        if (event.type == UART_DATA && !task_running){
            task_running = true;
            int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
            data[len - 1] = '\0'; 
            if (strcmp((char *)data, "1") == 0){
                read_voltage();
            }
            else if (strcmp((char *)data, "2") == 0){
                read_resistor();
            }
            else{
                ESP_LOGI(TAG, "Unrecognized command");
            }
            task_running = false;
            }
        }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(data); 
}

void read_voltage(void)
{
    double voltage = 0;
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
    if (data == NULL) {
        ESP_LOGE(TAG, "Error de memoria");
        return;
    }

    uart_write_bytes(UART_PORT_NUM, return_prompt, strlen(return_prompt));

    while (true) {
    if (xQueueReceive(dataQueue, &voltage, portMAX_DELAY) == pdPASS) {
        char buffer[50];
        snprintf(buffer, sizeof(buffer), "\rVoltaje: %f V           ", voltage);  
        uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
        ESP_LOGI(TAG, "Voltaje actualizado: %f", voltage);
    }

    int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
    if (len > 0) {
        data[len - 1] = '\0';  
        if (strcmp((char *)data, "y") == 0) { 
            break;  
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    }
    free(data); 
}

void read_resistor(void)
{
    double receivedData;
    double resistor_value = 0;
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
    if (data == NULL) {
        ESP_LOGE(TAG, "Error de memoria");
        return;
    }

    uart_write_bytes(UART_PORT_NUM, return_prompt, strlen(return_prompt));

    while (true){
        
    if (xQueueReceive(dataQueue, &receivedData, portMAX_DELAY) == pdPASS) {
        char buffer[50];
        resistor_value = (receivedData * R_REF) / (calibration_adc_ref() - receivedData);
        snprintf(buffer, sizeof(buffer), "\rResistencia: %f R         ", resistor_value);
        uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer)); 
        ESP_LOGI(TAG, "Resistencia: %f\n", resistor_value);
    }

    int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
    if (len > 0) {
        data[len - 1] = '\0'; 
        if (strcmp((char *)data, "y") == 0) { 
            break;  
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static bool init_adc_calibration() {
    esp_err_t ret;
    
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH
    };

    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Esquema de calibración de curva no soportado, probando esquema de línea.");
    }
    ESP_LOGI(TAG, "Calibración de ADC inicializada correctamente.");
    return true;

}

ads1115_fsr_t calibration_adc(void)
{
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    if (!init_adc_calibration()) {
        ESP_LOGE(TAG, "Fallo al inicializar la calibración del ADC");
        return ADS1115_FSR_6_144;
    }
        int raw_value = adc1_get_raw(ADC_CHANNEL);
        int calibrated_voltage = 0;

        if (adc_cali_raw_to_voltage(cali_handle, raw_value, &calibrated_voltage) == ESP_OK) {
            ESP_LOGI("La escala es:", "RAW: %d, Voltaje Calibrado: %d mV", raw_value, calibrated_voltage);
            if (calibrated_voltage <= VOLTAGE_HIGH && calibrated_voltage > VOLTAGE_MEDIUM_HIGH) return ADS1115_FSR_4_096;
            if (calibrated_voltage <= VOLTAGE_MEDIUM_HIGH && calibrated_voltage > VOLTAGE_MEDIUM_LOW) return ADS1115_FSR_2_048;
            if (calibrated_voltage <= VOLTAGE_MEDIUM_LOW && calibrated_voltage > VOLTAGE_LOW) return ADS1115_FSR_1_024;
            if (calibrated_voltage <= VOLTAGE_LOW && calibrated_voltage > VOLTAGE_VERY_LOW) return ADS1115_FSR_0_512;
            if (calibrated_voltage < VOLTAGE_VERY_LOW)return ADS1115_FSR_0_256;
        } else {
            ESP_LOGE(TAG, "Error al convertir el valor RAW a voltaje calibrado");
        }
        return ADS1115_FSR_6_144;
        vTaskDelay(pdMS_TO_TICKS(1000));  
}

double calibration_adc_ref(void)
{
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL_VREF, ADC_ATTEN);

    if (!init_adc_calibration()) {
        ESP_LOGE(TAG, "Fallo al inicializar la calibración del ADC");
    }
        int raw_value = adc1_get_raw(ADC_CHANNEL_VREF);
        int calibrated_voltage = 0;

        if (adc_cali_raw_to_voltage(cali_handle, raw_value, &calibrated_voltage) == ESP_OK) {
            ESP_LOGI(TAG, "RAW: %d, Voltaje Calibrado: %d mV", raw_value, calibrated_voltage);
            double voltage_in_volts = (calibrated_voltage) / 1000.0;
            return voltage_in_volts;
            ESP_LOGI("Process", "Voltaje procesado: %f V", voltage_in_volts);
        } else {
            ESP_LOGE(TAG, "Error al convertir el valor RAW a voltaje calibrado");
        }
        return 0;
        vTaskDelay(pdMS_TO_TICKS(1000));  
}

/*double kalman_filter(double measurement) {
    double kalman_gain = kalman_error_estimate / (kalman_error_estimate + kalman_R);

    kalman_estimate = kalman_estimate + kalman_gain * (measurement - kalman_estimate);

    kalman_error_estimate = (1 - kalman_gain) * kalman_error_estimate + fabs(kalman_estimate) * kalman_Q;

    return kalman_estimate;
}
*/

int compare(const void *a, const void *b) {
    return (*(double *)a - *(double *)b);
}

double median_weighted_average(double *buffer, int size) {
    qsort(buffer, size, sizeof(double), compare);
    double sum = 0;
    int valid_elements = size - 2; 
    for (int i = 1; i < size - 1; i++) {
        sum += buffer[i];
    }
    return sum / valid_elements;
}

double apply_median_average_filter(double new_voltage) {
    voltage_buffer[buffer_index] = new_voltage;
    buffer_index = (buffer_index + 1) % MEDIAN_FILTER_WINDOW_SIZE;

    double median = median_weighted_average(voltage_buffer, MEDIAN_FILTER_WINDOW_SIZE);

    average_buffer[avg_index] = median;
    avg_index = (avg_index + 1) % AVERAGE_WINDOW_SIZE;

    double sum = 0.0;
    for (int i = 0; i < AVERAGE_WINDOW_SIZE; i++) {
        sum += average_buffer[i];
    }
    return sum / AVERAGE_WINDOW_SIZE;
}
