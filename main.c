#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "shell.h"
#include "shell_commands.h"
#include "ztimer.h"

#include "bmi160.h"
#include "periph/i2c.h"



/* Macros for frames to be read */

#define ACC_FRAMES	10 /* 10 Frames are available every 100ms @ 100Hz */
#define GYR_FRAMES	10
#define MAG_FRAMES	10
/* 10 frames containing a 1 byte header, 6 bytes of accelerometer,
 * 6 bytes of gyroscope and 8 bytes of magnetometer data. This results in
 * 21 bytes per frame. Additional 40 bytes in case sensor time readout is enabled */
#define FIFO_SIZE	250

/* Variable declarations */

struct bmi160_dev bmi;

uint8_t fifo_buff[FIFO_SIZE];
struct bmi160_fifo_frame fifo_frame;
struct bmi160_sensor_data gyro_data[GYR_FRAMES], accel_data[ACC_FRAMES];

int8_t rslt;

int dev = I2C_DEV(0);

int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // printf("i2c_read_regs(%d, %x, %x, ...)\n", dev, dev_addr, reg_addr);
    return i2c_read_regs(dev, dev_addr, reg_addr, data, len, 0);
}

int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // printf("i2c_write_regs(%d, %x, %x, ...)\n", dev, dev_addr, reg_addr);
    return i2c_write_regs(dev, dev_addr, reg_addr, data, len, 0);
}

void user_delay(uint32_t period)
{
    ztimer_sleep(ZTIMER_MSEC, period);
}

/* accel params and conversion constants */
#define AC 2048.0 // for 16G
// #define AC 16384.0 // for 2G
/* gyro params and conversion constants */
#define GC 16.4 // for 2000 DPS
// #define GC 131.2 // for 250 DPS

int main(void)
{
    (void) puts("Welcome to RIOT!");

    i2c_init(dev);
    i2c_acquire(dev);

    /* Initialize your host interface to the BMI160 */

    /* This example uses I2C as the host interface */
    bmi.id = BMI160_I2C_ADDR;
    bmi.read = user_i2c_read;
    bmi.write = user_i2c_write;
    bmi.delay_ms = user_delay;
    bmi.intf = BMI160_I2C_INTF;

    rslt = bmi160_init(&bmi);
    if (rslt == BMI160_OK) {
        printf("Success initializing BMI160 - Chip ID 0x%X\n", bmi.chip_id);
    } else if (rslt == BMI160_E_DEV_NOT_FOUND) {
        printf("Error initializing BMI160: device not found\n");
        return 1;
    } else {
        printf("Error initializing BMI160 - %d\n", rslt);
        return 1;
    }

    /* Select the Output data rate, range of accelerometer sensor */
    bmi.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    // bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    bmi.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    // bmi.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
    bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi);
    if (rslt != BMI160_OK) {
        printf("Error configuring BMI160 - %d\n", rslt);
        return 1;
    }

    /* Link the FIFO memory location */
    fifo_frame.data = fifo_buff;
    fifo_frame.length = FIFO_SIZE;
    bmi.fifo = &fifo_frame;
    /* Clear all existing FIFO configurations */
    rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK , BMI160_DISABLE, &bmi);
    if (rslt != BMI160_OK) {
        printf("Error clearing fifo - %d\n", rslt);
        return 1;
    }

    uint8_t fifo_config = BMI160_FIFO_HEADER |  BMI160_FIFO_ACCEL | BMI160_FIFO_GYRO;
    rslt = bmi160_set_fifo_config(fifo_config, BMI160_ENABLE, &bmi);
    if (rslt != BMI160_OK) {
        printf("Error enabling fifo - %d\n", rslt);
        return 1;
    }
    /* Check rslt for any error codes */
    i2c_release(dev);

    while(rslt == 0) {
        /* Wait for 100ms for the FIFO to fill */
        user_delay(100);

        /* It is VERY important to reload the length of the FIFO memory as after the
         * call to bmi160_get_fifo_data(), the bmi.fifo->length contains the
         * number of bytes read from the FIFO */
        bmi.fifo->length = FIFO_SIZE;
        i2c_acquire(dev);
        rslt = bmi160_get_fifo_data(&bmi);
        if (rslt != BMI160_OK) {
            printf("Error getting fifo data - %d\n", rslt);
            return 1;
        }
        i2c_release(dev);

        uint8_t gyr_inst = GYR_FRAMES, acc_inst = ACC_FRAMES;
        rslt = bmi160_extract_gyro(gyro_data, &gyr_inst, &bmi);
        if (rslt != BMI160_OK) {
            printf("Error extracting gyro data - %d\n", rslt);
            return 1;
        }

        rslt = bmi160_extract_accel(accel_data, &acc_inst, &bmi);
        if (rslt != BMI160_OK) {
            printf("Error extracting accel data - %d\n", rslt);
            return 1;
        }

        for (size_t i = 0; i < acc_inst && i < gyr_inst; i++) {
            printf("Accel & gyro txyz is:     ");
            printf("%"PRIu32" %6.2f %6.2f %6.2f    ",
                accel_data[i].sensortime,
                accel_data[i].x / AC,
                accel_data[i].y / AC,
                accel_data[i].z / AC);
            printf("%"PRIu32" %6.2f %6.2f %6.2f    ",
                gyro_data[i].sensortime,
                gyro_data[i].x / AC,
                gyro_data[i].y / AC,
                gyro_data[i].z / AC);
            printf("\n");
        }
    }

    // never reached, for now...
    // char line_buf[SHELL_DEFAULT_BUFSIZE];
    // shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
