#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h" // For delay functions

#define TAG "ADG731"

/**
 * ADG731 Configuration
 */
#define ADG731_SPI_HOST SPI2_HOST
#define ADG731_PIN_MOSI 23
#define ADG731_PIN_SCLK 18
#define ADG731_PIN_CS   5

/**
 * ADG731 Command Masks
 */
#define ADG731_ENABLE  0x10 // Bit 4 set to enable a channel
#define ADG731_DISABLE 0x00 // Bit 4 clear to disable all channels

/**
 * ADG731 Timing Constants (in nanoseconds)
 */
#define ADG731_MIN_SYNC_LOW_TIME_NS   40  // t5: Minimum SYNC low time (40 ns)
#define ADG731_MIN_SYNC_HIGH_TIME_NS  33  // t8: Minimum SYNC high time (33 ns)

/**
 * Structure to hold ADG731 SPI device handle and configurations
 */
typedef struct {
    spi_device_handle_t spi_handle;
    int cs_pin;
} adg731_t;

/**
 * Delay for minimum required time in nanoseconds
 */
static inline void adg731_delay_ns(uint32_t ns) {
    esp_rom_delay_us((ns + 999) / 1000); // Convert ns to us with rounding
}

/**
 * Initialize the SPI device for ADG731
 */
esp_err_t adg731_init(adg731_t *dev) {
    if (!dev) {
        ESP_LOGE(TAG, "Invalid device handle");
        return ESP_ERR_INVALID_ARG;
    }

    // Configure CS pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ADG731_PIN_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(ADG731_PIN_CS, 1); // Set CS high initially

    // SPI bus configuration
    spi_bus_config_t buscfg = {
        .mosi_io_num = ADG731_PIN_MOSI,
        .sclk_io_num = ADG731_PIN_SCLK,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8
    };

    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(ADG731_SPI_HOST, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }

    // SPI device configuration
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 30 * 1000 * 1000, // 30 MHz maximum
        .mode = 0,                          // SPI mode 0
        .spics_io_num = -1,                 // Manual CS control
        .queue_size = 1
    };

    // Add SPI device
    ret = spi_bus_add_device(ADG731_SPI_HOST, &devcfg, &dev->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }

    dev->cs_pin = ADG731_PIN_CS;
    ESP_LOGI(TAG, "ADG731 initialized successfully");
    return ESP_OK;
}

/**
 * Set a channel on the ADG731
 */
esp_err_t adg731_set_channel(adg731_t *dev, uint8_t channel) {
    if (!dev || channel > 31) {
        ESP_LOGE(TAG, "Invalid channel or device");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t command = ADG731_ENABLE | (channel & 0x1F); // Create command byte
    gpio_set_level(dev->cs_pin, 0); // Enable CS
    adg731_delay_ns(ADG731_MIN_SYNC_LOW_TIME_NS); // Ensure minimum CS low time

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &command
    };

    esp_err_t ret = spi_device_transmit(dev->spi_handle, &t);
    adg731_delay_ns(ADG731_MIN_SYNC_HIGH_TIME_NS); // Ensure minimum SYNC high time
    gpio_set_level(dev->cs_pin, 1); // Disable CS

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set channel");
        return ret;
    }

    ESP_LOGI(TAG, "Channel %d set successfully", channel);
    return ESP_OK;
}

/**
 * Turn off all channels on the ADG731
 */
esp_err_t adg731_all_off(adg731_t *dev) {
    if (!dev) {
        ESP_LOGE(TAG, "Invalid device handle");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t command = ADG731_DISABLE; // Command to disable all channels
    gpio_set_level(dev->cs_pin, 0); // Enable CS
    adg731_delay_ns(ADG731_MIN_SYNC_LOW_TIME_NS); // Ensure minimum CS low time

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &command
    };

    esp_err_t ret = spi_device_transmit(dev->spi_handle, &t);
    adg731_delay_ns(ADG731_MIN_SYNC_HIGH_TIME_NS); // Ensure minimum SYNC high time
    gpio_set_level(dev->cs_pin, 1); // Disable CS

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn off all channels");
        return ret;
    }

    ESP_LOGI(TAG, "All channels turned off successfully");
    return ESP_OK;
}

void app_main(void)
{

}
