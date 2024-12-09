/**
 * @file main.c
 * @brief Control del multiplexor y cálculo de resistencias atraves de comandos UART.
 * @author Juan Mercado
 * @date 28/11/2024
 */
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

// Definitions of I2C device
#define I2C_MASTER_FREQ_HZ     100000
#define I2C_MASTER_SCL         GPIO_NUM_12
#define I2C_MASTER_SDA         GPIO_NUM_11 
#define I2C_PORT               I2C_NUM_0
#define ADS1115_ADDR           0x48

// Definitions of UART device
#define BAUD_RATE              115200
#define UART_PORT_NUM          UART_NUM_1
#define UART_TX_PIN            GPIO_NUM_17
#define UART_RX_PIN            GPIO_NUM_18
#define UART_BUF_SIZE          1024
#define TASK_STACK_SIZE        1024
#define QUEUE_SIZE             2

// Setup R_reference
#define R_REF                  995000
//#define R_REF                  1500000
//#define R_REF                  99000
//#define R_REF                  1000

// Definitions for ADC_calibrations
#define ADC_CHANNEL            ADC_CHANNEL_0
#define ADC_CHANNEL_VREF       ADC_CHANNEL_1 
#define ADC_ATTEN              ADC_ATTEN_DB_12
#define ADC_WIDTH              ADC_WIDTH_BIT_12

#define VOLTAGE_HIGH           4000
#define VOLTAGE_MEDIUM_HIGH    2000
#define VOLTAGE_MEDIUM_LOW     1000
#define VOLTAGE_LOW            500
#define VOLTAGE_VERY_LOW       200

// Definitions for media filter
#define AVERAGE_WINDOW_SIZE 3  
#define MEDIAN_FILTER_WINDOW_SIZE 5

// Definitions for control muxes
#define MUX2_S3 35
#define MUX2_S2 36
#define MUX2_S1 37
#define MUX2_S0 38
#define MUX1_S3 19
#define MUX1_S2 20
#define MUX1_S1 21
#define MUX1_S0 47

static const char *TAG = "App_ADS";
static QueueHandle_t uart_queue;
static QueueHandle_t dataQueue;
static adc_cali_handle_t cali_handle = NULL;
TaskHandle_t xTaskSensorHandle = NULL;
static volatile bool task_running = false;

static const char *channels_request = "\nSelector de canales de multiplexor\n--------------------------------\nSalir (y)\nCanales a seleccionar (XX,XX):";
static const char *channel_ref_request = "\nCanal a comparar con todos\n--------------------------------\nSalir (y)\nCanal a comparar (XX):";
static const char *channel_arr_request = "\nArray de canales a comparar\n--------------------------------\nSalir (y)\nArray a comparar (XX,XX):";
static const char *clear = "\033c";
static const char *return_prompt  = "\nVolver al menu principal (y/n)\n--------------------------------\n";
static const char *funtion_request = "\nLista de comandos disponibles:\n--------------------------------\n1.-Medir voltaje\n2.-Medir resitencia\n3.-Salir\n--------------------------------\n";
static const char *mux_request = "\nComados multiplexor\n--------------------------------\n1.-Canales individuales\n2.-Canal de referencia\n3.-Canal matriz\n4.-Todos los canales";

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
int compare(const void *a, const void *b);
double median_weighted_average(double *buffer, int size);
double apply_median_average_filter(double new_voltage);
esp_err_t mux_init(void);
void set_mux(int16_t mux_1_ch, int16_t mux_2_ch);
void select_request(void);
void pin_1_1(void);
void pin_ref(void);
void pin_arr(void);
void pin_all(void);
void mux_ref(int8_t mux_1_ch);
void mux_arr(int8_t mux_1_ch, int8_t mux_2_ch);
void reset_filter(void);
void prueba(void);

void app_main() {
    dataQueue = xQueueCreate(QUEUE_SIZE, sizeof(double));           //Creue between ads1115_task and UART_task                       
    if (i2c_init() == ESP_OK && init_uart() == ESP_OK && mux_init() == ESP_OK) {    //Control of error initializineation of qug task
        xTaskCreate(ads1115_task, "ads1115_task", TASK_STACK_SIZE * 3, NULL, 5, &xTaskSensorHandle);
        xTaskCreate(task_uart, "task_uart", TASK_STACK_SIZE * 3, NULL, 4, NULL);
        ESP_LOGI(TAG, "Application initialized");
    } else {
        ESP_LOGE(TAG, "Error initializing peripherals");
    }
}
/**
* @brief initializing I2C channel (required for ads1115 component)
*/
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
/**
* @brief initializing UART channel
*/
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
/**
* @brief task to get and send values of ads1115 to queue 
*/
void ads1115_task(void *param)
{
    double voltage;
    double filtered_voltage;
    uint8_t count= 5;

    //ADS1115 component parameter initialization
    ads1115_t ads = ads1115_config(I2C_PORT, ADS1115_ADDR);
    //ads1115_set_rdy_pin(&ads, GPIO_NUM_5);
    ads1115_set_mux(&ads, ADS1115_MUX_0_1);
    ads1115_set_mode(&ads, ADS1115_MODE_CONTINUOUS);
    ads1115_set_sps(&ads, ADS1115_SPS_8);
    ads1115_set_max_ticks(&ads, pdMS_TO_TICKS(100));

    while (true)
    {
        //ESP_LOGI("Valor del mux", "Valor return: %d", calibration_adc()); 
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    //waiting to continue receiving and sending data

        ads1115_set_pga(&ads, calibration_adc());   //recalibrating scale of ads
    
        for (size_t i = 0; i < count; i++)
        {
            voltage = ads1115_get_voltage(&ads);
            filtered_voltage = apply_median_average_filter(voltage);
        }

        xQueueReset(dataQueue);                   //clear queue before sending data
        if (xQueueSend(dataQueue, &filtered_voltage, pdMS_TO_TICKS(100)) != pdPASS){
            printf("Error sending data to queue\n");
        }
        reset_filter();     //reset filter for new medition                        
    }
}
/**
* @brief task to UART transmition to serial monitor
*/
void task_uart(void *param)
{
    uart_event_t event;     //activate event in uart message
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);  
    if (data == NULL) {
        ESP_LOGE(TAG, "Memory error");
        return;
    }
    while (true){

    uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));          //clear serial monitor
    uart_write_bytes(UART_PORT_NUM, mux_request, strlen(mux_request));
    bzero(data,UART_BUF_SIZE);      //clear data arr

    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)){      //wait for UART message 
        if (event.type == UART_DATA ){
            int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);     //read bytes in UART buffer and save in data
            data[len - 1] = '\0';       //ensures null at end of string
            if (strcmp((char *)data, "1") == 0){    //compare data strig with message
                pin_1_1();
            }
            else if (strcmp((char *)data, "2") == 0){
                pin_ref();
            }
            else if (strcmp((char *)data, "3") == 0){
                pin_arr();
            }
            else if (strcmp((char *)data, "4") == 0){
                pin_all();
            }
            else if (strcmp((char *)data, "5") == 0){
                prueba();
            }
            else{
                ESP_LOGI(TAG, "Unrecognized command");
            }
            }
        }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(data);     
}
/**
* @brief Read voltage in funtion 1-1
*/
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
    xTaskNotifyGive(xTaskSensorHandle);     //Activate in task ads1115 to continue reading
    if (xQueueReceive(dataQueue, &voltage, portMAX_DELAY) == pdPASS) {
        char buffer[50];
        snprintf(buffer, sizeof(buffer), "\rVoltaje: %f V           ", voltage);        //convert data in string and put in arr
        uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));        //send buffer to serial terminal
        ESP_LOGI(TAG, "Voltaje actualizado: %f", voltage);
    }

    int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);     //read UART for message
    if (len > 0) {
        data[len - 1] = '\0';  
        if (strcmp((char *)data, "y") == 0) {       //compare message and exit
            break;  
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));        //wait for send message to serial monitor
    }
    free(data); 
}
/**
* @brief Read resistence in funtion 1-1
*/
void read_resistor(void)
{
    double receivedData;
    double resistor_value = 0;
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));      //clear serial monitor
    if (data == NULL) {
        ESP_LOGE(TAG, "Error de memoria");
        return;
    }

    uart_write_bytes(UART_PORT_NUM, return_prompt, strlen(return_prompt));      

    while (true){
    xTaskNotifyGive(xTaskSensorHandle);    //Activate in task ads1115 to continue reading
    if (xQueueReceive(dataQueue, &receivedData, portMAX_DELAY) == pdPASS) {     //wait to recive data from task ads1115
        char buffer[50];
        resistor_value = (receivedData * R_REF) / (calibration_adc_ref() - receivedData);   //calculate resitance with the actual voltage supply
        snprintf(buffer, sizeof(buffer), "\rResistencia: %f Ω         ", resistor_value);   //convert to int to string and save in buffer
        uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
        ESP_LOGI(TAG, "Resistencia: %f\n", resistor_value);
    }

    int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS); //wait for exit message 
    if (len > 0) {
        data[len - 1] = '\0'; 
        if (strcmp((char *)data, "y") == 0) { 
            break;  
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/**
* @brief initialization of esp_adc calibration
* @return true if initialize correctly
*/
static bool init_adc_calibration() {
    esp_err_t ret;
    
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH
    };

    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Esquema de calibración de curva no soportado");
    }
    ESP_LOGI(TAG, "Calibración de ADC inicializada correctamente.");
    return true;

}
/**
* @brief esp_adc calibration for escale in ads1115
* @return adequate escale for ads1115
*/
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
        //vTaskDelay(pdMS_TO_TICKS(1000));  
}
/**
* @brief esp_adc calibrate voltage to calculate resistance
* @return supply voltage 
*/
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
        //vTaskDelay(pdMS_TO_TICKS(1000));  
}
/**
* @brief initializing MUX channels
*/
esp_err_t mux_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MUX1_S0) | (1ULL << MUX1_S1) | (1ULL << MUX1_S2) | (1ULL << MUX1_S3) | (1ULL << MUX2_S0) | (1ULL << MUX2_S1) | (1ULL << MUX2_S2) | (1ULL << MUX2_S3);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    return ESP_OK;
}
/**
* @brief Menu for seleccion actions in channel vs channel
*/
void select_request(void)
{
    uart_event_t event;
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Error de memoria");
        return;
    }
    while (true){

    uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
    uart_write_bytes(UART_PORT_NUM, funtion_request, strlen(funtion_request));
    bzero(data,UART_BUF_SIZE);
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)){
        if (event.type == UART_DATA ){
            int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, 20 / portTICK_PERIOD_MS); // read message form UART and save in data buff
            data[len - 1] = '\0';       //incert end of word

            /*Compare and select options*/
            if (strcmp((char *)data, "1") == 0){        
                read_voltage();
            }
            else if (strcmp((char *)data, "2") == 0){
                read_resistor();
            }
            else if (strcmp((char *)data, "3") == 0){
                break;
            }
            else{
                ESP_LOGI(TAG, "Unrecognized command");
            }
            }
        }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(data); 
}
/**
* @brief set_mux compare between two channels
* @param mux_1_ch channel of principal mux
* @param mux_2_ch channel of second mux
*/
void set_mux(int16_t mux_1_ch, int16_t mux_2_ch) {
    const int mux_1[4] = {MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3};
    const int mux_2[4] = {MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3};
    /*set the correct convination in mux channels*/
    for (int i = 0; i < 4; i++) {
        gpio_set_level(mux_1[i], (mux_1_ch >> i) & 0x01);     
        gpio_set_level(mux_2[i], (mux_2_ch >> i) & 0x01);
    }
    ESP_LOGI(TAG, "Mux settings applied");
}
/**
* @brief Menu for selection of channel vs channel
*/
void pin_1_1(void) {
    uart_event_t event;
    uint8_t flag_break = 0;

    uart_flush(UART_PORT_NUM);  // Clear UART buffers
    xQueueReset(uart_queue);    // Clear the event queue

    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Error de memoria");
        return;
    }

    while (true) {
        uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
        uart_write_bytes(UART_PORT_NUM, channels_request, strlen(channels_request));
        bzero(data, UART_BUF_SIZE);

        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                task_running = true;

                int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
                if (len > 0) {
                    data[len - 1] = '\0';  // Safe chain termination
                    ESP_LOGI(TAG, "Valor de data: %s", (char *)data);

                    /*Compare exit condition*/
                    if (strcmp((char *)data, "y") == 0) {
                        flag_break = 1;     // Activate the exit flag
                        break;
                    }
                    /*extract the channel to use*/
                    int16_t mux_1_ch = ((data[0] - '0') * 10) + (data[1] - '0');
                    int16_t mux_2_ch = ((data[3] - '0') * 10) + (data[4] - '0');

                    if (mux_1_ch >= 0 && mux_1_ch < 16 && mux_2_ch >= 0 && mux_2_ch < 16) {
                        set_mux(mux_1_ch, mux_2_ch);  // Configure the MUX pins
                        select_request();            // Process the selection
                    } else {
                        ESP_LOGI(TAG, "Comando no reconocido");
                    }
                }

                task_running = false;
            }
        }

        if (flag_break == 1) {
            break;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    free(data);             // free memory
    uart_flush(UART_PORT_NUM);  // clear UART
}
/**
* @brief Menu for selection of channel vs reference
*/
void pin_ref(void) {
    uart_event_t event;
    uint8_t flag_break = 0;

    uart_flush(UART_PORT_NUM);  // Clear UART buffers
    xQueueReset(uart_queue);    // Clear the event queue

    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Error de memoria");
        return;
    }

    while (true) {
        uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
        uart_write_bytes(UART_PORT_NUM, channel_ref_request, strlen(channel_ref_request));
        bzero(data, UART_BUF_SIZE);

        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                task_running = true;

                int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
                if (len > 0) {
                    data[len - 1] = '\0';  // Safe chain termination
                    ESP_LOGI(TAG, "Valor de data: %s", (char *)data);

                    /*Compare exit condition*/
                    if (strcmp((char *)data, "y") == 0) {
                        flag_break = 1;
                        break;
                    }

                    /*process reference mux channel*/
                    int8_t mux_1_ch = ((data[0] - '0') * 10) + (data[1] - '0');
                    if (mux_1_ch >= 0 && mux_1_ch < 16) {
                        mux_ref(mux_1_ch);  //Call to process with reference channel
                    } else {
                        ESP_LOGI(TAG, "Comando no reconocido");
                    }
                }

                task_running = false;
            }
        }

        if (flag_break == 1) {
            break;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    free(data);            
    uart_flush(UART_PORT_NUM);  // clear UART
}
/**
* @brief Process mux reference
* @param mux_1_ch channel of principal mux for reference
*/
void mux_ref(int8_t mux_1_ch) {
    uart_event_t event;
    const int mux_1[4] = {MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3};
    const int mux_2[4] = {MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3};
    double receivedData, resistor_value;
    uint8_t flag_break = 0;
    double supply_voltage = calibration_adc_ref();
    uint8_t data[UART_BUF_SIZE]; // Usamos un búfer estático

    uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
    uart_write_bytes(UART_PORT_NUM, return_prompt, strlen(return_prompt));  // Enviar mensaje para volver al monitor

    // Configuración inicial de los pines GPIO para mux_1
    for (int i = 0; i < 4; i++) {
        gpio_set_level(mux_1[i], (mux_1_ch >> i) & 0x01); // Configura los pines del multiplexor
    }

    while (true) {
        for (int mux_2_ch = 0; mux_2_ch < 16; mux_2_ch++) { // Recorre todos los canales de mux_2
            // Configurar los pines de mux_2
            for (int i = 0; i < 4; i++) {
                gpio_set_level(mux_2[i], (mux_2_ch >> i) & 0x01);
            }
            xTaskNotifyGive(xTaskSensorHandle); // Notificar a la tarea ADS1115

            if (xQueueReceive(dataQueue, &receivedData, pdMS_TO_TICKS(100)) == pdPASS) { // Tiempo de espera ajustado
                char buffer[50];
                resistor_value = (receivedData * R_REF) / (supply_voltage - receivedData); // Calcular resistencia
                snprintf(buffer, sizeof(buffer), "\nResistencia[%d]: %f Ω         ", mux_2_ch, resistor_value);
                uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
                ESP_LOGI(TAG, "Resistencia: %f\n", resistor_value);
            } else {
                // Manejar la situación cuando no se reciben datos
                ESP_LOGW(TAG, "No se recibieron datos a tiempo");
            }

            int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS); // Esperar la condición de salida
            if (len > 0) {
                data[len - 1] = '\0';  // Asegurar terminación de cadena
                if (strcmp((char *)data, "y") == 0) {
                    flag_break = 1;
                    break;
                }
            }

            if (flag_break) {
                break;
            }
        }

        if (flag_break) {
            flag_break = 0;
            break;
        }

        // Revisa eventos UART en la cola
        if (xQueueReceive(uart_queue, (void *)&event, pdMS_TO_TICKS(100))) { // Tiempo de espera ajustado
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
                if (len > 0) {
                    data[len - 1] = '\0';  // Asegurar terminación de cadena
                    if (strcmp((char *)data, "y") == 0) {
                        flag_break = 1;
                        break;
                    }
                }
            }
        }

        uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
        uart_write_bytes(UART_PORT_NUM, return_prompt, strlen(return_prompt));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
* @brief Menu for selection of arr (primary mux, cap values)
*/
void pin_arr(void) {
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Error de memoria");
        return;
    }

    uint8_t flag_break = 0;

    uart_flush(UART_PORT_NUM);  // Clear UART buffers
    xQueueReset(uart_queue);    // Clear the event queue

    while (true) {
        uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
        uart_write_bytes(UART_PORT_NUM, channel_arr_request, strlen(channel_arr_request));
        bzero(data, UART_BUF_SIZE);     // Clear data values

        // Wait for UART event
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
                if (len > 0) {
                    data[len - 1] = '\0';  // Safe chain termination
                    ESP_LOGI(TAG, "Valor de data: %s", (char *)data);

                    // wait for exit condition
                    if (strcmp((char *)data, "y") == 0) {
                        flag_break = 1;
                        break;
                    }
                    // Porcess primary mux reference and cap value
                    int16_t mux_1_ch = ((data[0] - '0') * 10) + (data[1] - '0');
                    int16_t mux_2_ch = ((data[3] - '0') * 10) + (data[4] - '0');

                    if (mux_1_ch >= 0 && mux_1_ch < 16 && mux_2_ch >= 0 && mux_2_ch < 16) {
                        mux_arr(mux_1_ch, mux_2_ch);
                    } else {
                        ESP_LOGI(TAG, "Comando no reconocido");
                    }
                }
            }
        }

        if (flag_break == 1) {
            break;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    free(data);             
    uart_flush(UART_PORT_NUM);  // Clear UART 
}
/**
* @brief Process mux arr
* @param mux_1_ch channel of principal mux for reference
* @param mux_2_ch Cap of values to read
*/
void mux_arr(int8_t mux_1_ch, int8_t mux_2_ch) {
    /*asignation of mux channels*/
    const int mux_1[4] = {MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3};
    const int mux_2[4] = {MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3};
    uart_event_t event;
    double receivedData;
    double resistor_value = 0;
    uint8_t flag_break = 0;

    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Error de memoria");
        return;
    }

    uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
    uart_write_bytes(UART_PORT_NUM, return_prompt, strlen(return_prompt)); // Send exit menu

    while (true) {
        // Copy of parameters
        int8_t mux_1_ch_f = mux_1_ch;
        int8_t mux_2_ch_f = mux_1_ch;

        for (size_t i = mux_1_ch; i <= mux_2_ch; i++) {     // Compare the values ​​of the reference mux up to the limit of values
            for (size_t j = mux_1_ch_f; j <= mux_2_ch; j++) { // Increments the value of the secondary mux
                for (size_t k = 0; k < 4; k++) {        // Re-assing the muxes value
                    gpio_set_level(mux_1[k], (mux_1_ch_f >> k) & 0x01);
                    gpio_set_level(mux_2[k], (mux_2_ch_f >> k) & 0x01);
                }

                xTaskNotifyGive(xTaskSensorHandle);     // Send notification to task_ads1115 for sending data
                if (xQueueReceive(dataQueue, &receivedData, portMAX_DELAY) == pdPASS) {
                    char buffer[50];
                    resistor_value = (receivedData * R_REF) / (calibration_adc_ref() - receivedData);       // Calculate resitence
                    snprintf(buffer, sizeof(buffer), "\nResistencia[%d][%d]: %f Ω", mux_1_ch_f, mux_2_ch_f, resistor_value);        // Convert the resistence value to string and save in buff
                    uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
                    ESP_LOGI(TAG, "Resistencia: %f", resistor_value);
                }

                mux_2_ch_f++;      // Increment the secondary mux
                // Wait for exit condition
                int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
                if (len > 0) {
                    data[len - 1] = '\0';
                    if (strcmp((char *)data, "y") == 0) {
                        flag_break = 1;
                        break;
                    }
                }
            }
            if (flag_break == 1) break;
            mux_1_ch_f++;    // Increment the primary mux value
            mux_2_ch_f = mux_1_ch_f;    // Match primary mux and secondary mux 
        }

        if (flag_break == 1) break;
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                task_running = true;

                int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
                if (len > 0) {
                    data[len - 1] = '\0';  // Safe chain termination
                    if (strcmp((char *)data, "y") == 0) {
                    flag_break = 1;
                    break;  
                    }
                }
            }
    }
        uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
        uart_write_bytes(UART_PORT_NUM, return_prompt, strlen(return_prompt));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    free(data);  
}
/**
* @brief Process all conditions
*/
void pin_all(void)
{
    /*asignation of mux channels*/
    const int mux_1[4] = {MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3};
    const int mux_2[4] = {MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3};
    uart_event_t event;
    int8_t mux_1_ch = 0;
    int8_t mux_2_ch = 0;
    double receivedData;
    double resistor_value = 0;
    uint8_t flag_break = 0;

    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));

    if (data == NULL) {
        ESP_LOGE(TAG, "Error de memoria");
        return;
    }

    uart_write_bytes(UART_PORT_NUM, return_prompt, strlen(return_prompt));

    while (true){
    for (size_t i = 0; i < 16; i++)         // Process all primary mux value as reference
    {
        for (size_t j = 0; j < 16; j++)     // Process all secondary mux 
        {
            for (size_t i = 0; i < 4; i++)      // Re-assing muxes
            {
            gpio_set_level(mux_1[i], (mux_1_ch >> i) & 0x01);
            gpio_set_level(mux_2[i], (mux_2_ch >> i) & 0x01);
            }
            xTaskNotifyGive(xTaskSensorHandle);     // Send notification to task_ads1115 for sending data
            if (xQueueReceive(dataQueue, &receivedData, portMAX_DELAY) == pdPASS) {
                char buffer[50];
                resistor_value = (receivedData * R_REF) / (calibration_adc_ref() - receivedData);       // Calculate resistence
                snprintf(buffer, sizeof(buffer), "\nResistencia[%d][%d]: %f R         ",i, j , resistor_value);     // Convert the resistence value to string and save in buff
                uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));        // Send resitence value 
                ESP_LOGI(TAG, "Resistencia: %f\n", resistor_value);
            }
            mux_2_ch++;    // Increment secondary mux value
            // Wait for exit condition
            int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
            if (len > 0) {
                data[len - 1] = '\0'; 
                if (strcmp((char *)data, "y") == 0) {
                    flag_break = 1;
                    break;  
                }
            }
        }
        if (flag_break == 1)
        {
            break;
        }
        mux_1_ch++;      // Increment primary mux value
    }
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                task_running = true;

                int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
                if (len > 0) {
                    data[len - 1] = '\0';  // Safe chain termination
                    if (strcmp((char *)data, "y") == 0) {
                    flag_break = 1;
                    break;  
                    }
                }
            }
    }
    uart_write_bytes(UART_PORT_NUM, clear, strlen(clear));
    uart_write_bytes(UART_PORT_NUM, return_prompt, strlen(return_prompt));
    if (flag_break == 1)
    {
        flag_break = 0;
        break;
    }
    mux_1_ch = 0; // Reset primary mux value
    int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, portMAX_DELAY);
            if (len > 0) {
                data[len - 1] = '\0'; 
                if (strcmp((char *)data, "y") == 0) {
                    flag_break = 1;
                    break;  
                }
            }
    vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/**
* @brief Reset filter values
*/
void reset_filter(void) {
    buffer_index = 0;
    avg_index = 0;
    memset(voltage_buffer, 0, MEDIAN_FILTER_WINDOW_SIZE * sizeof(double));
    memset(average_buffer, 0, AVERAGE_WINDOW_SIZE * sizeof(double));
    ESP_LOGI(TAG, "Filtros de voltaje y promedio reiniciados.");
}
/**
* @brief comparate elements of arr
* @param a item to compare
* @param b item to compare
* @return difference of items
*/
int compare(const void *a, const void *b) {
    return (*(double *)a - *(double *)b);
}
/**
* @brief Combined median and average filter
* @param new_voltage voltage to apply filter
* @return average of values
*/
double apply_median_average_filter(double new_voltage) {
    voltage_buffer[buffer_index] = new_voltage;
    buffer_index = (buffer_index + 1) % MEDIAN_FILTER_WINDOW_SIZE;      //stores voltages to apply filter

    double median = median_weighted_average(voltage_buffer, MEDIAN_FILTER_WINDOW_SIZE); // apply media filter

    average_buffer[avg_index] = median;
    avg_index = (avg_index + 1) % AVERAGE_WINDOW_SIZE;

    double sum = 0.0;
    for (int i = 0; i < AVERAGE_WINDOW_SIZE; i++) {     //calculate the average
        sum += average_buffer[i];
    }
    return sum / AVERAGE_WINDOW_SIZE;
}
/**
* @brief calculates an average after sorting the elements and removing extreme values
* @param buffer arr to be ordered
* @param size size of the arr.
* @return average of internal elements
*/
double median_weighted_average(double *buffer, int size) {
    qsort(buffer, size, sizeof(double), compare);       //order arr in asendent order
    double sum = 0;
    int valid_elements = size - 2;      //excludes first and last element of the arr
    for (int i = 1; i < size - 1; i++) {
        sum += buffer[i];
    }
    return sum / valid_elements;
}

void prueba(void)
{
    double receivedData;
    double resistor_value = 0;
    uint8_t count = 65;
    while (true)
    {
    for (size_t i = 0; i < count; i++)
    {
    xTaskNotifyGive(xTaskSensorHandle);     // Send notification to task_ads1115 for sending data
            if (xQueueReceive(dataQueue, &receivedData, portMAX_DELAY) == pdPASS) {
                char buffer[50];
                resistor_value = (receivedData * R_REF) / (calibration_adc_ref() - receivedData);       // Calculate resistence
                snprintf(buffer, sizeof(buffer), "\nResistencia%f R         ", resistor_value);     // Convert the resistence value to string and save in buff
                uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));        // Send resitence value 
                ESP_LOGI(TAG, "Resistencia: %f\n", resistor_value);
            }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}