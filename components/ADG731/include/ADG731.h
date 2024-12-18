#include "driver/gpio.h"  
#include "esp_err.h"

// Comando para apagar todas las salidas
#define ADG731_ALLOFF 0x80

// Estructura de configuración para ADG731
typedef struct {
    gpio_num_t clockPin;   // Pin de reloj
    gpio_num_t dataPin;    // Pin de datos
    gpio_num_t syncPin;    // Pin de sincronización
    uint8_t channel;       // Canal actual
} ADG731_t;

/**
 * @brief Inicializa la estructura y los pines GPIO del ADG731
 * 
 * @param dev Puntero a la estructura ADG731_t
 * @param clockPin GPIO asignado al pin de reloj
 * @param dataPin GPIO asignado al pin de datos
 * @param syncPin GPIO asignado al pin de sincronización
 */
void ADG731_init(ADG731_t *dev, gpio_num_t clockPin, gpio_num_t dataPin, gpio_num_t syncPin);

/**
 * @brief Configura el canal activo del multiplexor
 * 
 * @param dev Puntero a la estructura ADG731_t
 * @param channel Canal a activar (0-31)
 */
void ADG731_setChannel(ADG731_t *dev, uint8_t channel);

/**
 * @brief Obtiene el canal actualmente activo
 * 
 * @param dev Puntero a la estructura ADG731_t
 * @return uint8_t Canal actual
 */
uint8_t ADG731_getChannel(ADG731_t *dev);

/**
 * @brief Devuelve el número total de canales
 * 
 * @return uint8_t Número de canales (32)
 */
uint8_t ADG731_channelCount(void);

/**
 * @brief Apaga todas las salidas del multiplexor
 * 
 * @param dev Puntero a la estructura ADG731_t
 */
void ADG731_allOff(ADG731_t *dev);
