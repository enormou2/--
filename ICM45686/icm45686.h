/*
 * icm45686.h
 * ICM45686 IMU chip driver — initialization and raw data reading
 */

#ifndef ICM45686_H_
#define ICM45686_H_

#include <stdint.h>

/**
 * @brief Initialize and configure the ICM45686 IMU.
 * @param use_ln   1 = Low-Noise mode, 0 = Low-Power mode
 * @param accel_en 1 = enable accelerometer
 * @param gyro_en  1 = enable gyroscope
 * @return 0 on success, non-zero on error
 */
int setup_imu(int use_ln, int accel_en, int gyro_en);

/**
 * @brief Read raw sensor data and convert to physical units.
 * @param accel_mg  Output: accelerometer data in mg (3 elements)
 * @param gyro_dps  Output: gyroscope data in degrees/sec (3 elements)
 * @param temp_degc Output: temperature in degrees Celsius
 * @return 0 on success, non-zero on error
 */
int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc);

#endif /* ICM45686_H_ */
