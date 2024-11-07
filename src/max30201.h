#ifndef MAX30201_H
#define MAX30201_H

#include <stdint.h>
#include "esp_err.h"

// Endereço I2C do MAX30201
#define MAX30201_ADDR 0x57

// Funções para inicializar e ler do sensor
esp_err_t max30201_init(void);
int16_t max30201_read_data(void);

#endif // MAX30201_H
