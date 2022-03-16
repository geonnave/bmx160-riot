/* defines the axis for acc */
#define ACC_NOOF_AXIS 3
#define GYR_NOOF_AXIS 2

/* bmi160 slave address */
#define BMI160_ADDR((0x69) << 1)

#define RAD_DEG 57.29577951

I2C i2c(p13, p15);

int16_t acc_sample_buffer[ACC_NOOF_AXIS] = {
    0x5555,
    0x5555,
    0x5555
};
int16_t gyr_sample_buffer[GYR_NOOF_AXIS] = {
    0x5555,
    0x5555
};

double acc_result_buffer[ACC_NOOF_AXIS] = {
    0x5555,
    0x5555,
    0x5555
};
double gyr_result_buffer[GYR_NOOF_AXIS] = {
    0x5555,
    0x5555
};

double acc_previous_result_buffer[ACC_NOOF_AXIS] = {
    0x5555,
    0x5555,
    0x5555
};
double delta_acc_result_buffer[ACC_NOOF_AXIS] = {
    0x5555,
    0x5555,
    0x5555
};
double coord_result_buffer[ACC_NOOF_AXIS] = {
    0x0000,
    0x0000
};

double accel_ang_x, accel_ang_y;
double tiltx, tilty;
double tiltx_prev, tilty_prev;

char i2c_reg_buffer[2] = {
    0
};

void BMX160_config(void) {

    i2c.frequency(20000);

    /*Reset BMI160*/
    i2c_reg_buffer[0] = 0x7E;
    i2c_reg_buffer[1] = 0xB6;
    i2c.write(BMI160_ADDR, i2c_reg_buffer, sizeof(i2c_reg_buffer), false);
    wait_ms(200);
    printf("BMI160 Resetado\n\r");

    /*Habilita o Acelerometro*/
    i2c_reg_buffer[0] = 0x7E;
    i2c_reg_buffer[1] = 0x11; //PMU Normal   
    i2c.write(BMI160_ADDR, i2c_reg_buffer, sizeof(i2c_reg_buffer), false);
    printf("Acc Habilitado\n\r");

    /*Habilita o Giroscopio*/
    i2c_reg_buffer[0] = 0x7E;
    i2c_reg_buffer[1] = 0x15; //PMU Normal 
    i2c.write(BMI160_ADDR, i2c_reg_buffer, sizeof(i2c_reg_buffer), false);
    printf("Gyr Habilitado\n\r");

    /*Config o Data Rate ACC em 1600Hz*/
    i2c_reg_buffer[0] = 0x40;
    i2c_reg_buffer[1] = 0x2C;
    i2c.write(BMI160_ADDR, i2c_reg_buffer, sizeof(i2c_reg_buffer), false);
    printf("Data Rate ACC Selecionado a 1600Hz\n\r");

    /*Config o Data Rate GYR em 1600Hz*/
    i2c_reg_buffer[0] = 0x42;
    i2c_reg_buffer[1] = 0x2C;
    i2c.write(BMI160_ADDR, i2c_reg_buffer, sizeof(i2c_reg_buffer), false);
    printf("Data Rate GYR Selecionado a 1600Hz\n\r");

    /*Config o Range GYR em 250º/s*/
    i2c_reg_buffer[0] = 0x43;
    i2c_reg_buffer[1] = 0x03;
    i2c.write(BMI160_ADDR, i2c_reg_buffer, sizeof(i2c_reg_buffer), false);
    printf("Range GYR Selecionado a 250deg/s\n\r");

    wait(0.1);

    printf("BMI160 Configurado\n\r");
}

void BMX160_read_acc(void) {

    i2c.frequency(20000);

    /*Le os Registradores do Acelerometro*/
    i2c_reg_buffer[0] = 0x12;
    i2c.write(BMI160_ADDR, i2c_reg_buffer, 1, true);
    i2c.read(BMI160_ADDR, (char * ) & acc_sample_buffer, sizeof(acc_sample_buffer), false);

    /*Ajusta dados brutos Acelerometro em unidades de g */
    acc_result_buffer[0] = (acc_sample_buffer[0] / 16384.0);
    acc_result_buffer[1] = (acc_sample_buffer[1] / 16384.0);
    acc_result_buffer[2] = (acc_sample_buffer[2] / 16384.0);
    //printf("Acc results: %f, %f, %f \n",acc_result_buffer[0], acc_result_buffer[1], acc_result_buffer[2]); 
    //wait(0.5);

    delta_acc_result_buffer[0] = acc_result_buffer[0] - acc_previous_result_buffer[0];
    delta_acc_result_buffer[1] = acc_result_buffer[1] - acc_previous_result_buffer[1];
    delta_acc_result_buffer[2] = acc_result_buffer[2] - acc_previous_result_buffer[2];
    //printf("Delta acc results: %f, %f, %f \n",delta_acc_result_buffer[0], delta_acc_result_buffer[1], delta_acc_result_buffer[2]); 
    //wait(0.5);

    acc_previous_result_buffer[0] = acc_result_buffer[0];
    acc_previous_result_buffer[1] = acc_result_buffer[1];
    acc_previous_result_buffer[2] = acc_result_buffer[2];

    //f_latitude = f_latitude + delta_acc_result_buffer[1]*10;
    //f_longitude = f_longitude + delta_acc_result_buffer[0]*10;

}

void BMX160_read_gyr(void) {

    i2c.frequency(20000);

    /*Le os Registradores do Giroscopio*/
    i2c_reg_buffer[0] = 0x0C;
    i2c.write(BMI160_ADDR, i2c_reg_buffer, 1, true);
    i2c.read(BMI160_ADDR, (char * ) & gyr_sample_buffer, sizeof(gyr_sample_buffer), false);

    /*Ajusta dados Brutos do Giroscopio em unidades de deg/s */
    gyr_result_buffer[0] = (gyr_sample_buffer[0] / 131.2);
    gyr_result_buffer[1] = (gyr_sample_buffer[1] / 131.2);
    printf("Gyr results: %f, %f \n", gyr_result_buffer[0], gyr_result_buffer[1]);
    wait(0.5);

}

void BMX160_read_mag(void) {

    i2c.frequency(20000);

    /*Le os Registradores do Acelerometro*/
    i2c_reg_buffer[0] = 0x04;
    i2c.write(BMI160_ADDR, i2c_reg_buffer, 1, true);
    i2c.read(BMI160_ADDR, (char * ) & acc_sample_buffer, sizeof(acc_sample_buffer), false);

    /*Ajusta dados brutos Acelerometro em unidades de g */
    acc_result_buffer[0] = (acc_sample_buffer[0]);
    acc_result_buffer[1] = (acc_sample_buffer[1]);
    acc_result_buffer[2] = (acc_sample_buffer[2]);
    printf("Mag results: %f, %f, %f \n", acc_result_buffer[0], acc_result_buffer[1], acc_result_buffer[2]);
    wait(0.5);

}

void BMX160_print(void) {

    int32_t float_to_32;

    BMX160_read_acc();
    BMX160_read_gyr();
    BMX160_read_mag();

    /*Calcula os Angulos de Inclinacao com valor do Acelerometro*/
    accel_ang_x = atan(acc_result_buffer[0] / sqrt(pow(acc_result_buffer[1], 2) + pow(acc_result_buffer[2], 2))) * RAD_DEG;
    accel_ang_y = atan(acc_result_buffer[1] / sqrt(pow(acc_result_buffer[0], 2) + pow(acc_result_buffer[2], 2))) * RAD_DEG;

    /*Calcula os Angulos de Rotacao com valor do Giroscopio e aplica filtro complementar realizando a fusao*/
    tiltx = (0.98 * (tiltx_prev + (gyr_result_buffer[0] * 0.001))) + (0.02 * (accel_ang_x));
    tilty = (0.98 * (tilty_prev + (gyr_result_buffer[1] * 0.001))) + (0.02 * (accel_ang_y));

    /*Imprime os dados ACC pre-formatados*/
    printf("Acc: %.3f,%.3f\n\r", tiltx, tilty);

    float_to_32 = tiltx * 100 + 127;
    float_to_32 = (float_to_32 < 0 ? 0 : float_to_32);

    float_to_32 = 0;

    float_to_32 = tilty * 100 + 127;
    float_to_32 = (float_to_32 < 0 ? 0 : float_to_32);

    //imuz = acc_result_buffer[2];

}
