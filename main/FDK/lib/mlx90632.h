/**
 * @file mlx90632.h
 * @brief MLX90632 driver with virtual i2c communication
 * @internal
 *
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @endinternal
 *
 * @addtogroup mlx90632_API MLX90632 Driver Library API
 * Implementation of MLX90632 driver with virtual i2c read/write functions
 *
 * @details
 * Copy of Kernel driver, except that it is stripped of Linux kernel specifics
 * which are replaced by simple i2c read/write functions. There are some Linux
 * kernel macros left behind as they make code more readable and easier to
 * understand, but if you already have your own implementation then preprocessor
 * should handle it just fine.
 *
 * Repository contains README.md for compilation and unit-test instructions.
 *
 * @{
 *
 */
#ifndef _MLX90632_LIB_
#define _MLX90632_LIB_

/* Including CRC calculation functions */
#include <errno.h>

/* Solve errno not defined values */
#ifndef ETIMEDOUT
#define ETIMEDOUT 110 /**< From linux errno.h */
#endif
#ifndef EINVAL
#define EINVAL 22 /**< From linux errno.h */
#endif
#ifndef EPROTONOSUPPORT
#define EPROTONOSUPPORT 93 /**< From linux errno.h */
#endif

/* BIT, GENMASK and ARRAY_SIZE macros are imported from kernel */
#ifndef BIT
#define BIT(x)(1UL << (x))
#endif
#ifndef GENMASK
#ifndef BITS_PER_LONG
//#warning "Using default BITS_PER_LONG value"
#define BITS_PER_LONG 32 /**< Define how many bits per long your CPU has */
#endif
#define GENMASK(h, l) \
    (((~0UL) << (l)) & (~0UL >> (BITS_PER_LONG - 1 - (h))))
#endif
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0])) /**< Return number of elements in array */
#endif

/* Memory sections addresses */
#define MLX90632_ADDR_RAM   0x4000 /**< Start address of ram */
#define MLX90632_ADDR_EEPROM    0x2480 /**< Start address of user eeprom */

/* EEPROM addresses - used at startup */
#define MLX90632_EE_CTRL    0x24d4 /**< Control register initial value */
#define MLX90632_EE_CONTROL MLX90632_EE_CTRL /**< More human readable for Control register */

#define MLX90632_EE_I2C_ADDRESS 0x24d5 /**< I2C address register initial value */
#define MLX90632_EE_VERSION 0x240b /**< EEPROM version reg - assumed 0x101 */

#define MLX90632_EE_P_R     0x240c /**< Calibration constant ambient reference register 32bit */
#define MLX90632_EE_P_G     0x240e /**< Calibration constant ambient gain register 32bit */
#define MLX90632_EE_P_T     0x2410 /**< Calibration constant ambient tc2 register 32bit	*/
#define MLX90632_EE_P_O     0x2412 /**< Calibration constant ambient offset register 32bit */
#define MLX90632_EE_Aa      0x2414 /**< Aa calibration const register 32bit */
#define MLX90632_EE_Ab      0x2416 /**< Ab calibration const register 32bit */
#define MLX90632_EE_Ba      0x2418 /**< Ba calibration const register 32bit */
#define MLX90632_EE_Bb      0x241a /**< Bb calibration const register 32bit */
#define MLX90632_EE_Ca      0x241c /**< Ca calibration const register 32bit */
#define MLX90632_EE_Cb      0x241e /**< Cb calibration const register 32bit */
#define MLX90632_EE_Da      0x2420 /**< Da calibration const register 32bit */
#define MLX90632_EE_Db      0x2422 /**< Db calibration const register 32bit */
#define MLX90632_EE_Ea      0x2424 /**< Ea calibration constant register 32bit */
#define MLX90632_EE_Eb      0x2426 /**< Eb calibration constant register 32bit */
#define MLX90632_EE_Fa      0x2428 /**< Fa calibration constant register 32bit */
#define MLX90632_EE_Fb      0x242a /**< Fb calibration constant register 32bit */
#define MLX90632_EE_Ga      0x242c /**< Ga calibration constant register 32bit */

#define MLX90632_EE_Gb      0x242e /**< Ambient Beta calibration constant 16bit */
#define MLX90632_EE_Ka      0x242f /**< IR Beta calibration constant 16bit */

#define MLX90632_EE_Ha      0x2481 /**< Ha customer calibration value register 16bit */
#define MLX90632_EE_Hb      0x2482 /**< Hb customer calibration value register 16bit */

/* Register addresses - volatile */
#define MLX90632_REG_I2C_ADDR   0x3000 /**< Chip I2C address register */

/* Control register address - volatile */
#define     MLX90632_REG_CTRL   0x3001 /**< Control Register address */
#define   MLX90632_CFG_SOC_SHIFT 3 /**< Start measurement in step mode */
#define   MLX90632_CFG_SOC_MASK BIT(MLX90632_CFG_SOC_SHIFT)
#define   MLX90632_CFG_PWR_MASK GENMASK(2, 1) /**< PowerMode Mask */
/* PowerModes statuses */
#define MLX90632_PWR_STATUS(ctrl_val) (ctrl_val << 1)
#define MLX90632_PWR_STATUS_HALT MLX90632_PWR_STATUS(0) /**< Pwrmode hold */
#define MLX90632_PWR_STATUS_SLEEP_STEP MLX90632_PWR_STATUS(1) /**< Pwrmode sleep step*/
#define MLX90632_PWR_STATUS_STEP MLX90632_PWR_STATUS(2) /**< Pwrmode step */
#define MLX90632_PWR_STATUS_CONTINUOUS MLX90632_PWR_STATUS(3) /**< Pwrmode continuous*/

/* Device status register - volatile */
#define MLX90632_REG_STATUS 0x3fff /**< Device status register */
#define   MLX90632_STAT_BUSY    BIT(10) /**< Device busy indicator */
#define   MLX90632_STAT_EE_BUSY BIT(9) /**< Device EEPROM busy indicator */
#define   MLX90632_STAT_BRST    BIT(8) /**< Device brown out reset indicator */
#define   MLX90632_STAT_CYCLE_POS GENMASK(6, 2) /**< Data position in measurement table */
#define   MLX90632_STAT_DATA_RDY    BIT(0) /**< Data ready indicator */

/* RAM_MEAS address-es for each channel */
#define MLX90632_RAM_1(meas_num)    (MLX90632_ADDR_RAM + 3 * meas_num)
#define MLX90632_RAM_2(meas_num)    (MLX90632_ADDR_RAM + 3 * meas_num + 1)
#define MLX90632_RAM_3(meas_num)    (MLX90632_ADDR_RAM + 3 * meas_num + 2)

/* Timings (ms) */
#define MLX90632_TIMING_EEPROM 100 /**< Time between EEPROM writes */

/* Magic constants */
#define MLX90632_DSPv5 0x05 /* EEPROM DSP version */
#define MLX90632_EEPROM_VERSION 0x105 /**< EEPROM DSP version for constants */
#define MLX90632_EEPROM_WRITE_KEY 0x554C /**< EEPROM write key 0x55 and 0x4c */
#define MLX90632_RESET_CMD  0x0006 /**< Reset sensor (address or global) */
#define MLX90632_MAX_MEAS_NUM   31 /**< Maximum number of measurements in list */
#define MLX90632_EE_SEED    0x3f6d /**< Seed for the CRC calculations */
#define MLX90632_REF_12 12.0 /**< ResCtrlRef value of Channel 1 or Channel 2 */
#define MLX90632_REF_3  12.0 /**< ResCtrlRef value of Channel 3 */

/** Read raw ambient and object temperature
 *
 * Trigger and read raw ambient and object temperatures. This values still need
 * to be pre-processed via @link mlx90632_preprocess_temp_ambient @endlink and @link
 * mlx90632_preprocess_temp_object @endlink functions and then processed via @link
 * mlx90632_calc_temp_ambient @endlink and @link mlx90632_calc_temp_object @endlink
 * to retrieve values in milliCelsius
 *
 * @param[out] ambient_new_raw Pointer to where new raw ambient temperature is written
 * @param[out] object_new_raw Pointer to where new raw object temperature is written
 * @param[out] ambient_old_raw Pointer to where old raw ambient temperature is written
 * @param[out] object_old_raw Pointer to where old raw object temperature is written
 *
 * @retval 0 Successfully read both temperatures
 * @retval <0 Something went wrong. Check errno.h for more details
 */
int32_t mlx90632_read_temp_raw(int16_t *ambient_new_raw, int16_t *ambient_old_raw,
                               int16_t *object_new_raw, int16_t *object_old_raw);

/** Calculation of raw ambient output
 *
 * Preprocessing of the raw ambient value
 *
 * @param[in] ambient_new_raw ambient temperature from @link MLX90632_RAM_3 @endlink. The meas_num 1 or 2 is
 *								determined by value in @link MLX90632_STAT_CYCLE_POS @endlink
 * @param[in] ambient_old_raw ambient temperature from @link MLX90632_RAM_3 @endlink. The meas_num 1 or 2 is
 *								determined by value not in @link MLX90632_STAT_CYCLE_POS @endlink
 * @param[in] Gb Register value on @link MLX90632_EE_Gb @endlink
 *
 * @return Calculated ambient raw output
 */
double mlx90632_preprocess_temp_ambient(int16_t ambient_new_raw, int16_t ambient_old_raw, int16_t Gb);

/** Calculation of raw object output
 *
 * Preprocessing of the raw object value
 *
 * @param[in] object_new_raw object temperature from @link MLX90632_RAM_1 @endlink or @link MLX90632_RAM_2 @endlink.
 *								The meas_number 1 or 2 is determined by value in @link MLX90632_STAT_CYCLE_POS @endlink
 * @param[in] object_old_raw object temperature from @link MLX90632_RAM_1 @endlink or @link MLX90632_RAM_2 @endlink.
 *								The meas_number 1 or 2 is determined by value not in @link MLX90632_STAT_CYCLE_POS @endlink
 * @param[in] ambient_new_raw ambient temperature from @link MLX90632_RAM_3 @endlink. The meas_number 1 or 2 is
 *								determined by value in @link MLX90632_STAT_CYCLE_POS @endlink
 * @param[in] ambient_old_raw ambient temperature from @link MLX90632_RAM_3 @endlink. The meas_number 1 or 2 is
 *								determined by value not in @link MLX90632_STAT_CYCLE_POS @endlink
 * @param[in] Ka Register value on @link MLX90632_EE_Ka @endlink
 *
 * @return Calculated object raw output
 */
double mlx90632_preprocess_temp_object(int16_t object_new_raw, int16_t object_old_raw,
                                       int16_t ambient_new_raw, int16_t ambient_old_raw,
                                       int16_t Ka);

/** Calculation of ambient temperature
 *
 * DSPv5 implementation of ambient temperature calculation
 *
 * @param[in] ambient_new_raw ambient temperature from @link MLX90632_RAM_3 @endlink. The channel 1 or 2 is
 *								determined by value in @link MLX90632_STAT_CYCLE_POS @endlink
 * @param[in] ambient_old_raw ambient temperature from @link MLX90632_RAM_3 @endlink. The channel 1 or 2 is
 *								determined by value not in @link MLX90632_STAT_CYCLE_POS @endlink
 * @param[in] P_T Register value on @link MLX90632_EE_P_T @endlink
 * @param[in] P_R Register value on @link MLX90632_EE_P_R @endlink
 * @param[in] P_G Register value on @link MLX90632_EE_P_G @endlink
 * @param[in] P_O Register value on @link MLX90632_EE_P_O @endlink
 * @param[in] Gb Register value on @link MLX90632_EE_Gb @endlink
 *
 * @return Calculated ambient temperature degrees Celsius
 */
double mlx90632_calc_temp_ambient(int16_t ambient_new_raw, int16_t ambient_old_raw, int32_t P_T,
                                  int32_t P_R, int32_t P_G, int32_t P_O, int16_t Gb);

/** Calculation of object temperature
 *
 * DSPv5 implementation of object temperature calculation with customer
 * calibration data
 *
 * @param[in] object object temperature from @link mlx90632_preprocess_temp_object @endlink
 * @param[in] ambient ambient temperature from @link mlx90632_preprocess_temp_ambient @endlink
 * @param[in] Ea Register value on @link MLX90632_EE_Ea @endlink
 * @param[in] Eb Register value on @link MLX90632_EE_Eb @endlink
 * @param[in] Ga Register value on @link MLX90632_EE_Ga @endlink
 * @param[in] Fb Register value on @link MLX90632_EE_Fb @endlink
 * @param[in] Fa Register value on @link MLX90632_EE_Fa @endlink
 * @param[in] Ha Register value on @link MLX90632_EE_Ha @endlink
 * @param[in] Hb Register value on @link MLX90632_EE_Hb @endlink
 *
 * @note emissivity Value provided by user of the object emissivity
 * using @link mlx90632_set_emissivity @endlink function.
 *
 * @return Calculated object temperature in milliCelsius
 */
double mlx90632_calc_temp_object(int32_t object, int32_t ambient,
                                 int32_t Ea, int32_t Eb, int32_t Ga, int32_t Fa, int32_t Fb,
                                 int16_t Ha, int16_t Hb);

/** Initialize MLX90632 driver and confirm EEPROM version
 *
 * EEPROM version is important to match sensor EEPROM content and calculations.
 * This is why this function checks for correct EEPROM version before it does
 * checksum validation of the EEPROM content.
 *
 * @note EEPROM version can have swapped high and low bytes due to CPU or I2C.
 * Please confirm that i2c read (16bit) is functioning as expected.
 *
 * @retval 0 Successfully initialized MLX90632 driver
 * @retval <0 Something went wrong. Consult errno.h for more details.
 */
int32_t mlx90632_init(void);

/** Set emissivity which is retained in single variable.
 *
 * @param[in] value Value provided by user of object emissivity. Defaults to 1.0 and cannot be 0.0.
 *
 * @warning This is not suitable for multi-process calculations as we do not use instances
 */
void mlx90632_set_emissivity(double value);

/** Read value of emissivity
 */
double mlx90632_get_emissivity(void);

///@}

#ifdef TEST
int mlx90632_start_measurement(void);
int32_t mlx90632_read_temp_ambient_raw(int16_t *ambient_new_raw, int16_t *ambient_old_raw);
int32_t mlx90632_read_temp_object_raw(int32_t start_measurement_ret,
                                      int16_t *object_new_raw, int16_t *object_old_raw);
#endif

#endif