#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include <Fusion.h>

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
#define SAMPLE_PERIOD (0.01f)

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00}; // Reset
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;

    // Lê os dados de aceleração
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Lê os dados do giroscópio
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Lê a temperatura
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = buffer[0] << 8 | buffer[1];
}

void send_data(int axis, int value) {
    uart_putc(UART0_BASE, axis);      // Enviar o eixo
    uart_putc(UART0_BASE, (value >> 8) & 0xFF);       // Enviar o byte mais significativo
    uart_putc(UART0_BASE, value & 0xFF);              // Enviar o byte menos significativo
    uart_putc(UART0_BASE, 0xFF);                      // Enviar o byte de fim de pacote
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    while(1) {
        // Lê os dados da IMU
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Converte os valores
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };
        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        // Atualiza a fusão de dados
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Envia os dados da aceleração e giroscópio pela UART
        //send_data(0, euler.angle.pitch);  // Eixo X do giroscópio
        send_data(0, -euler.angle.yaw);  // Eixo Y do giroscópio
        send_data(1, -euler.angle.roll);  // Eixo Z do giroscópio
        if (accelerometer.axis.y > 1.5) {
            send_data(2, 1);
        } 
        // Detecta movimento brusco na horizontal (para simular o clique)

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main() {
    stdio_init_all();
    xTaskCreate(mpu6050_task, "mpu6050_task", 8192, NULL, 1, NULL);
    vTaskStartScheduler();
    while (true);
}
