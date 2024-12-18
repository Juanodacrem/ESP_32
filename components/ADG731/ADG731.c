#include "ADG731.h"

// Inicializa la configuración y los pines del ADG731
void ADG731_init(ADG731_t *dev, gpio_num_t clockPin, gpio_num_t dataPin, gpio_num_t syncPin)
{
    dev->clockPin = clockPin;
    dev->dataPin = dataPin;
    dev->syncPin = syncPin;
    dev->channel = ADG731_ALLOFF;

    // Configurar pines como salida
    gpio_reset_pin(dev->clockPin);
    gpio_set_direction(dev->clockPin, GPIO_MODE_OUTPUT);
    gpio_reset_pin(dev->dataPin);
    gpio_set_direction(dev->dataPin, GPIO_MODE_OUTPUT);
    gpio_reset_pin(dev->syncPin);
    gpio_set_direction(dev->syncPin, GPIO_MODE_OUTPUT);

    // Estado inicial HIGH
    gpio_set_level(dev->clockPin, 1);
    gpio_set_level(dev->dataPin, 1);
    gpio_set_level(dev->syncPin, 1);
}

// Escribe el canal al ADG731
static void ADG731_write(ADG731_t *dev, uint8_t data)
{
    gpio_set_level(dev->syncPin, 0);  // SYNC LOW

    for (int i = 0; i < 8; i++) {
        gpio_set_level(dev->clockPin, 1);  // Reloj HIGH

        // Enviar el bit más significativo
        gpio_set_level(dev->dataPin, (data & 0x80) ? 1 : 0);

        data <<= 1;  // Desplazar a la izquierda
        gpio_set_level(dev->clockPin, 0);  // Reloj LOW
    }

    gpio_set_level(dev->syncPin, 1);  // SYNC HIGH
}

// Configura un canal específico
void ADG731_setChannel(ADG731_t *dev, uint8_t channel)
{
    dev->channel = channel & 0x1F;  // Solo los primeros 5 bits son válidos
    ADG731_write(dev, dev->channel);
}

// Devuelve el canal actual
uint8_t ADG731_getChannel(ADG731_t *dev)
{
    return dev->channel;
}

// Devuelve el número de canales (32)
uint8_t ADG731_channelCount(void)
{
    return 32;
}

// Apaga todas las salidas
void ADG731_allOff(ADG731_t *dev)
{
    dev->channel = ADG731_ALLOFF;
    ADG731_write(dev, dev->channel);
}
