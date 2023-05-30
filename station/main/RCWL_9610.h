#pragma once
#include "driver/i2c.h"

esp_err_t RCWL_9610_init(i2c_port_t i2cPort, int sdaPin, int sclPin, uint32_t clkSpeedHz);

esp_err_t RCWL_9610_free(i2c_port_t i2cPort);

esp_err_t RCWL_9610_start(i2c_port_t i2cPort, uint8_t sensAddr, TickType_t timeOut);

esp_err_t RCWL_9610_start_and_read_distance(i2c_port_t i2cPort, uint8_t sensAddr,
                                    TickType_t timeOut, uint8_t* pData);


