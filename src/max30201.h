#ifndef MAX30201_H
#define MAX30201_H

#include "esp_err.h"
#include "driver/i2c.h"

// Endereço I2C do MAX30201
#define MAX30201_ADDR 0x50

// Função para inicializar o sensor MAX30201
esp_err_t max30201_init(i2c_port_t i2c_num);

// Função para ler dados do sensor MAX30201
int max30201_read_data(i2c_port_t i2c_num);

#endif // MAX30201_H
