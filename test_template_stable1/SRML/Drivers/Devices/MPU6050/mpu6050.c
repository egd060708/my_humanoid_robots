/**
******************************************************************************
* Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
* @file    mpu6050.c
* @author  Lutoo e19135693@163.com
* @brief   Code for mpu6050.
* @date    2021-06-18
* @version 2.0
* @par Change Log：
* <table>
* <tr><th>Date        <th>Version  <th>Author    		<th>Description
* <tr><td>2018-10-18  <td> 1.0     <td>mannychen  	<td>Creator
* <tr><td>2019-11-21  <td> 1.1     <td>YDX        	<td>remove config macro.
* <tr><td>2020-04-02  <td> 1.1     <td>YDX        	<td>Reduce parameters in initialization function and add notes. 
* <tr><td>2021-06-18  <td> 2.0     <td>Lutoo      	<td>reconstitution
* </table>
*
==============================================================================
					##### How to use this driver #####
==============================================================================
  @note

				https://git.scutbot.cn/WFshuli/INS_Driver/src/branch/main/mpu6050_driver_lib.md
  @warning

******************************************************************************
* @attention
*
* if you had modified this file, please make sure your code does not have many
* bugs, update the version Number, write dowm your name and the date, the most
* important is make sure the users will have clear and definite understanding
* through your new brief.
*
* <h2><center>&copy; Copyright (c) 2021 - ~, SCUT-RobotLab Development Team.
* All rights reserved.</center></h2>
******************************************************************************
*/
#include "srml_config.h"

#if USE_SRML_MPU6050

/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include <math.h>
#include <string.h>
#include "inv_mpu_dmp_motion_driver.h"
/* Private define ------------------------------------------------------------*/
#define Rad2Deg  	57.2957795f

//mpu FIFO default  rate
const unsigned short MPU_FIFO_DEFAULT_RATE = 1000;
//dmp FIFO default  rate
const unsigned short DMP_FIFO_DEFAULT_RATE = 200;

/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* mpu config struct */
struct mpu_config_s mpu_config = {
		.sensors 		= INV_XYZ_GYRO|INV_XYZ_ACCEL,
		.fifo_config 	= INV_XYZ_GYRO|INV_XYZ_ACCEL,
		.sample_rate 	= MPU_FIFO_DEFAULT_RATE,
		.accel_fsr		= ACCEL_FSR_2G,
		.gyro_fsr		= GYRO_FSR_2000DPS,
		.lpf 			= LPF_98HZ,
		.gyro_orientation = { 1, 0 , 0 , 0 , 1 , 0 , 0 , 0 , 1 }
};

/* dmp config struct */
struct dmp_config_s dmp_config = {
		.dmp_on = 1,
		.dmp_feature = DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_GYRO_CAL|
						DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO ,
		.interrupt_mode = DMP_INT_CONTINUOUS,
		.fifo_rate = DMP_FIFO_DEFAULT_RATE,
};


/* Private function declarations ---------------------------------------------*/
static inline unsigned short inv_row_2_scale(const signed char *row);
static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
uint16_t get_gpio_pin_num(uint16_t GPIO_Pin);
/* function prototypes -------------------------------------------------------*/

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

uint16_t get_gpio_pin_num(uint16_t GPIO_Pin)
{
  uint16_t x = 0;
  while (GPIO_Pin >>= 1)x++;
  return x;
}

uint8_t MPU6050_Config_Pin(GPIO_TypeDef *gpiox, uint16_t scl_pinx, uint16_t sda_pinx)
{
	if(!IS_GPIO_PIN(scl_pinx) || (scl_pinx & (scl_pinx - 1)))return 1;
	if(!IS_GPIO_PIN(sda_pinx) || (sda_pinx & (sda_pinx - 1)))return 1;

	/* MPU6050_IIC initialization */
	MPU_IIC_PIN.IIC_GPIO_PORT = gpiox;
	MPU_IIC_PIN.IIC_SCL_PIN = scl_pinx;
	MPU_IIC_PIN.IIC_SDA_PIN = sda_pinx;

	MPU_IIC_PIN.IIC_SCL_PIN_NUM = get_gpio_pin_num(scl_pinx);
	MPU_IIC_PIN.IIC_SDA_PIN_NUM = get_gpio_pin_num(sda_pinx);

	return 0;
}

/*
 * @brief mpu6050 init
 * @param 	_mpu_s(struct mpu_s*) mpu config struct
 * 			_dmp_s(struct dmp_s*) dmp config struct
 * 			_rec_s(struct rec_s*) receive data struct
 */
unsigned char MPU6050_Init(struct mpu_config_s *_mpu_s,struct dmp_config_s *_dmp_s)
{
	/* initial iic bus */
	IIC_Init(&MPU_IIC_PIN);

	/* mpu init */
	if(mpu_init()){
		delay_ms_nos(100);
		__set_FAULTMASK(1); //reset
		NVIC_SystemReset();}

	/* Get/set hardware configuration. Start gyro. */
	/* Wake sensors. */
	mpu_set_sensors(_mpu_s->sensors);
	/* config mpu FIFO. */
	mpu_configure_fifo(_mpu_s->fifo_config);
	/* config mou sample rate */
	mpu_set_sample_rate(_mpu_s->sample_rate);
	/* set accel fsr and gyro fsr */
	mpu_set_accel_fsr(_mpu_s->accel_fsr);
	mpu_set_gyro_fsr(_mpu_s->gyro_fsr);
	/* set low pass filter */
	if(_mpu_s->lpf){
		mpu_set_lpf(_mpu_s->lpf);}

	if(!_dmp_s->dmp_on){return 0;}
	/* initialize the DMP */
	/* Load the DMP firware */
	dmp_load_motion_driver_firmware();
	/* Push gyro and accel orientation to the DMP */
	dmp_set_orientation(
	        inv_orientation_matrix_to_scalar(_mpu_s->gyro_orientation));
	/* Enable DMP features */
	dmp_enable_feature(_dmp_s->dmp_feature);
	/* config dmp fifo rate */
	if(_dmp_s->fifo_rate && _dmp_s->fifo_rate <= 200){
		dmp_set_fifo_rate(_dmp_s->fifo_rate);}
	/* config dmp interrupt mode */
	if(_dmp_s->interrupt_mode){
		mpu_set_int_level(1);
		dmp_set_interrupt_mode(_dmp_s->interrupt_mode);}
	/* turn on dmp */
	mpu_set_dmp_state(1);

	return 0;
}
/**
 * @brief mpu6050自检，初始化后再使用
 * 
 * @param is_set_accel_bias 是否使用自检的加速度值作为偏差
 * @param max_try_time 自检失败时，最大尝试次数
 */
void MPU6050_run_self_test(uint8_t is_set_accel_bias, uint32_t max_try_time)
{
    uint32_t try_time = 0;
    long gyro[3], accel[3];
    unsigned char i = 0;
	while(mpu_run_self_test(gyro, accel) != 0x7 && try_time++ < max_try_time) {};
	/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
	for(i = 0; i<3; i++) {
		gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
		accel[i] *= 2048.f; //convert to +-16G
		accel[i] = accel[i] >> 16;
		gyro[i] = (long)(gyro[i] >> 16);
	}
	mpu_set_gyro_bias_reg(gyro);
	if(is_set_accel_bias)
		mpu_set_accel_bias_6050_reg(accel);
}

/*
 * @brief 读取mpu fifo的数据
 * @param  struct rec_s* _rec_s
 */
uint8_t  dmp_read_data(struct mpu_rec_s *_rec_s)
{
	uint8_t i = 0;
	float q0,q1,q2,q3;
	short gyro_16bit[3] = {0};
	short accel_16bit[3] = {0};
	if(dmp_read_fifo(gyro_16bit, accel_16bit,_rec_s->quat,&(_rec_s->sensor_timestamp),
			&(_rec_s->sensors),	&(_rec_s->more)) != 0)
		return 1;
	for(i = 0;i <3;i++){
		_rec_s->gyro[i] = (float)gyro_16bit[i]/32768.0f*mpu_config.gyro_fsr;
	}
	for(i = 0;i <3;i++){
		_rec_s->accel[i] = (float)accel_16bit[i]/32768.0f*mpu_config.accel_fsr;
	}
	q0 = (float)_rec_s->quat[0] / (float)(1<<30);				//q30格式转换为浮点数
	q1 = (float)_rec_s->quat[1] / (float)(1<<30);
	q2 = (float)_rec_s->quat[2] / (float)(1<<30);
	q3 = (float)_rec_s->quat[3] / (float)(1<<30);
	_rec_s->roll	= atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)) *Rad2Deg;
	_rec_s->pitch 	= asin(2*(q0*q2 - q1*q3))*Rad2Deg;
	_rec_s->yaw		= atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))*Rad2Deg;
	return 0;
}

/*
 * @brief read mpu fifo data
 * @param	struct rec_s* _rec_s
 */
uint8_t mpu_read_data(struct mpu_rec_s* _rec_s){
	uint8_t i = 0;
	short gyro_16bit[3] = {0};
	short accel_16bit[3] = {0};
	if(mpu_read_fifo(gyro_16bit, accel_16bit,&(_rec_s->sensor_timestamp),
			(unsigned char*)&(_rec_s->sensors),	&(_rec_s->more)) != 0)
		return 1;
	for(i = 0;i <3;i++){
		_rec_s->gyro[i] = (float)gyro_16bit[i]/32768.0f*mpu_config.gyro_fsr;
	}
	for(i = 0;i <3;i++){
		_rec_s->accel[i] = (float)accel_16bit[i]/32768.0f*mpu_config.accel_fsr;
	}
	return 0;
}

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

