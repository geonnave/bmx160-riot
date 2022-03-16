#include "bmi160.h"
#include "bmm150.h"

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
struct bmm150_dev bmm;

uint8_t fifo_buff[FIFO_SIZE];
struct bmi160_fifo_frame fifo_frame;
struct bmi160_aux_data aux_data[MAG_FRAMES];
struct bmm150_mag_data mag_data[MAG_FRAMES];
struct bmi160_sensor_data gyro_data[GYR_FRAMES], accel_data[ACC_FRAMES];

int8_t rslt;

/* Auxiliary function declarations */
int8_t bmm150_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
int8_t bmm150_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);

void main(void)
{
    /* Initialize your host interface to the BMI160 */

    /* This example uses I2C as the host interface */
    bmi.id = BMI160_I2C_ADDR;
    bmi.read = user_i2c_read;
    bmi.write = user_i2c_write;
    bmi.delay_ms = user_delay_ms;
    bmi.interface = BMI160_I2C_INTF;

    /* The BMM150 API tunnels through the auxiliary interface of the BMI160 */
    /* Check the pins of the BMM150 for the right I2C address */
    bmm.dev_id = BMM150_DEFAULT_I2C_ADDRESS;
    bmm.intf = BMM150_I2C_INTF;
    bmm.read = bmm150_aux_read;
    bmm.write = bmm150_aux_write;
    bmm.delay_ms = user_delay_ms;

    rslt = bmi160_init(&bmi);
    /* Check rslt for any error codes */

    /* Configure the BMI160's auxiliary interface for the BMM150 */
    bmi.aux_cfg.aux_sensor_enable = BMI160_ENABLE;
    bmi.aux_cfg.aux_i2c_addr = bmm.dev_id;
    bmi.aux_cfg.manual_enable = BMI160_ENABLE; /* Manual mode */
    bmi.aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_3; /* 8 bytes */

    rslt = bmi160_aux_init(&bmi);
    /* Check rslt for any error codes */

    rslt = bmm150_init(&bmm);
    /* Check rslt for any error codes */

    /* Configure the accelerometer */
    bmi.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Configure the gyroscope */
    bmi.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    bmi.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    bmi.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    rslt = bmi160_set_sens_conf(&bmi);
    /* Check rslt for any error codes */

    /* Configure the magnetometer. The regular preset supports up to 100Hz in Forced mode */
    bmm.settings.preset_mode = BMM150_PRESETMODE_REGULAR;
    rslt = bmm150_set_presetmode(&bmm);
    /* Check rslt for any error codes */

    /* It is important that the last write to the BMM150 sets the forced mode.
     * This is because the BMI160 writes the last value to the auxiliary sensor 
     * after every read */
    bmm.settings.pwr_mode = BMM150_FORCED_MODE;
    rslt = bmm150_set_op_mode(&bmm);
    /* Check rslt for any error codes */

    uint8_t bmm150_data_start = BMM150_DATA_X_LSB;
    bmi.aux_cfg.aux_odr = BMI160_AUX_ODR_100HZ;
    rslt = bmi160_set_aux_auto_mode(&bmm150_data_start, &bmi);
    /* Check rslt for any error codes */

    /* Link the FIFO memory location */
    fifo_frame.data = fifo_buff;
    fifo_frame.length = FIFO_SIZE;
    bmi.fifo = &fifo_frame;

    /* Clear all existing FIFO configurations */
    rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK , BMI160_DISABLE, &bmi);
    /* Check rslt for any error codes */

    uint8_t fifo_config = BMI160_FIFO_HEADER | BMI160_FIFO_AUX |  BMI160_FIFO_ACCEL | BMI160_FIFO_GYRO;
    rslt = bmi160_set_fifo_config(fifo_config, BMI160_ENABLE, &bmi);
    /* Check rslt for any error codes */

    while(rslt != 0) {
        /* Wait for 100ms for the FIFO to fill */
        user_delay_ms(100);

        /* It is VERY important to reload the length of the FIFO memory as after the 
         * call to bmi160_get_fifo_data(), the bmi.fifo->length contains the
         * number of bytes read from the FIFO */
        bmi.fifo->length = FIFO_SIZE;
        rslt = bmi160_get_fifo_data(&bmi);
        /* Check rslt for any error codes */

        uint8_t aux_inst = MAG_FRAMES, gyr_inst = GYR_FRAMES, acc_inst = ACC_FRAMES;
        rslt = bmi160_extract_aux(aux_data, &aux_inst, &bmi);
        /* Check rslt for any error codes */
        rslt = bmi160_extract_gyro(gyro_data, &gyr_inst, &bmi);
        /* Check rslt for any error codes */
        rslt = bmi160_extract_accel(accel_data, &acc_inst, &bmi);
        /* Check rslt for any error codes */

        for (uint8_t i = 0; i < aux_inst; i++) {
            rslt = bmm150_aux_mag_data(&aux_data[i].data[0], &bmm);
            /* Check rslt for any error codes */
            /* Copy the compensated magnetometer data */
            mag_data[i] = bmm.data;
        }
    }
}

/* Auxiliary function definitions */
int8_t bmm150_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

    (void) id; /* id is unused here */

    return bmi160_aux_read(reg_addr, reg_data, len, &bmi);
}

int8_t bmm150_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

    (void) id; /* id is unused here */

    return bmi160_aux_write(reg_addr, reg_data, len, &bmi);
}
