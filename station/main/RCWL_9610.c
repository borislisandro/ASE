#include "driver/i2c.h"

esp_err_t RCWL_9610_init(i2c_port_t i2cPort, int sdaPin, int sclPin, uint32_t clkSpeedHz) {
    // Configure I2C communication parameters
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sdaPin,
        .scl_io_num = sclPin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clkSpeedHz
    };

    // Install I2C driver
    esp_err_t ret = i2c_param_config(i2cPort, &conf);
    if (ret != ESP_OK) {
        printf("Failed to configure I2C communication: %d\n", ret);
        return ret;
    }

    ret = i2c_driver_install(i2cPort, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        printf("Failed to install I2C driver: %d\n", ret);
        return ret;
    }

    return ESP_OK;
}

esp_err_t RCWL_9610_free(i2c_port_t i2cPort) {
    // Uninstall I2C driver
    esp_err_t ret = i2c_driver_delete(i2cPort);
    if (ret != ESP_OK) {
        printf("Failed to delete I2C driver: %d\n", ret);
    }
    return ret;
}

esp_err_t RCWL_9610_start(i2c_port_t i2cPort, uint8_t sensAddr, TickType_t timeOut) {
    esp_err_t ret;
    uint8_t data[1] = {0x01};

    ret = i2c_master_write_to_device(i2cPort, sensAddr, &data, sizeof(data), timeOut);

    // Wait for distance to be ready
    vTaskDelay(15);

    if (ret != ESP_OK) {
        printf("Failed to start RCWL_9610: %d\n", ret);
    }
    return ESP_OK;
}

esp_err_t RCWL_9610_start_and_read_distance(i2c_port_t i2cPort, uint8_t sensAddr, TickType_t timeOut, uint8_t pData[3]) {
    esp_err_t ret;

    // Wake up the sensor
    ret = RCWL_9610_start(i2cPort, sensAddr, timeOut);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t write_buffer[1] = {0X57};
    uint8_t read_buffer[3] = {0x00,0x00,0x00};

    ret = i2c_master_write_read_device(i2cPort, sensAddr, &write_buffer, sizeof(write_buffer),&read_buffer, sizeof(read_buffer), timeOut);

    if (ret != ESP_OK) {
        printf("Failed read distance register RCWL_9610: %d\n", ret);
    }
    pData[2] = read_buffer[2];
    pData[1] = read_buffer[1];
    pData[0] = read_buffer[0];

    return ESP_OK;
}
