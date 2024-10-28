/*
 * ltc2959.h
 *
 *  Created on: May 28, 2024
 *      Author: rahul
 */

#ifndef LTC2959_H_
#define LTC2959_H_
#endif															//I2C port is connected to LTC2944

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include <stdio.h>

//#define _DEBUG

#define LTC2959_I2C_ADDR					(0b1100011 << 1)
#ifndef LTC2959_I2C_PORT
#define LTC2959_I2C_PORT       				hi2c2				// I2C handle. Change it to whatever

// Input parameters
#define DEFAULT_RSENSE						50		// 50 milli-Ohms (Do not Change)
#define USER_RSENSE							20		// Input the sense resistor value in milli-ohms
#define RSENSE_CALIBRATION_FACTOR			1
#define ACR_CALIBARITION_FACTOR				1.23

//
#define ACR_LSB_nAh 						533				// ACR LSB Size 533nAh
#define ACR_LSB_mAh    						(ACR_LSB_nAh / 1000000.0) // LSB in milliamp-hours
#define ACR_MID_SCALE 						(float)(1UL << 31)		// 2^31
#define QLSB								(ACR_LSB_mAh * (DEFAULT_RSENSE / USER_RSENSE) * ACR_CALIBARITION_FACTOR)

// Register addresses
#define REG_STATUS                         	0x00  // Status Register
#define REG_ADC_CONTROL                    	0x01  // ADC Control Register
#define REG_COULOMB_COUNTER_CONTROL        	0x02  // Coulomb Counter Control Register
#define REG_ACCUMULATED_CHARGE_MSB         	0x03  // Accumulated Charge <31:24>
#define REG_ACCUMULATED_CHARGE_23_16       	0x04  // Accumulated Charge <23:16>
#define REG_ACCUMULATED_CHARGE_15_8        	0x05  // Accumulated Charge <15:8>
#define REG_ACCUMULATED_CHARGE_LSB         	0x06  // Accumulated Charge <7:0>
#define REG_CHARGE_THRESHOLD_LOW_MSB       	0x07  // Charge Threshold Low <31:24>
#define REG_CHARGE_THRESHOLD_LOW_23_16     	0x08  // Charge Threshold Low <23:16>
#define REG_CHARGE_THRESHOLD_LOW_15_8      	0x09  // Charge Threshold Low <15:8>
#define REG_CHARGE_THRESHOLD_LOW_LSB       	0x0A  // Charge Threshold Low <7:0>
#define REG_CHARGE_THRESHOLD_HIGH_MSB      	0x0B  // Charge Threshold High <31:24>
#define REG_CHARGE_THRESHOLD_HIGH_23_16    	0x0C  // Charge Threshold High <23:16>
#define REG_CHARGE_THRESHOLD_HIGH_15_8     	0x0D  // Charge Threshold High <15:8>
#define REG_CHARGE_THRESHOLD_HIGH_LSB      	0x0E  // Charge Threshold High <7:0>
#define REG_VOLTAGE_MSB                    	0x0F  // Voltage MSB
#define REG_VOLTAGE_LSB                    	0x10  // Voltage LSB
#define REG_VOLTAGE_THRESHOLD_HIGH_MSB     	0x11  // Voltage Threshold High MSB
#define REG_VOLTAGE_THRESHOLD_HIGH_LSB     	0x12  // Voltage Threshold High LSB
#define REG_VOLTAGE_THRESHOLD_LOW_MSB      	0x13  // Voltage Threshold Low MSB
#define REG_VOLTAGE_THRESHOLD_LOW_LSB      	0x14  // Voltage Threshold Low LSB
#define REG_MAX_VOLTAGE_MSB                	0x15  // Max Voltage MSB
#define REG_MAX_VOLTAGE_LSB                	0x16  // Max Voltage LSB
#define REG_MIN_VOLTAGE_MSB                	0x17  // Min Voltage MSB
#define REG_MIN_VOLTAGE_LSB                	0x18  // Min Voltage LSB
#define REG_CURRENT_MSB                    	0x19  // Current MSB
#define REG_CURRENT_LSB                    	0x1A  // Current LSB
#define REG_CURRENT_THRESHOLD_HIGH_MSB     	0x1B  // Current Threshold High MSB
#define REG_CURRENT_THRESHOLD_HIGH_LSB     	0x1C  // Current Threshold High LSB
#define REG_CURRENT_THRESHOLD_LOW_MSB      	0x1D  // Current Threshold Low MSB
#define REG_CURRENT_THRESHOLD_LOW_LSB      	0x1E  // Current Threshold Low LSB
#define REG_MAX_CURRENT_MSB                	0x1F  // Max Current MSB
#define REG_MAX_CURRENT_LSB                	0x20  // Max Current LSB
#define REG_MIN_CURRENT_MSB                	0x21  // Min Current MSB
#define REG_MIN_CURRENT_LSB                	0x22  // Min Current LSB
#define REG_TEMPERATURE_MSB                	0x23  // Temperature MSB
#define REG_TEMPERATURE_LSB                	0x24  // Temperature LSB
#define REG_TEMPERATURE_THRESHOLD_HIGH_MSB 	0x25  // Temperature Threshold High MSB
#define REG_TEMPERATURE_THRESHOLD_HIGH_LSB 	0x26  // Temperature Threshold High LSB
#define REG_TEMPERATURE_THRESHOLD_LOW_MSB  	0x27  // Temperature Threshold Low MSB
#define REG_TEMPERATURE_THRESHOLD_LOW_LSB  	0x28  // Temperature Threshold Low LSB
#define REG_GPIO_VOLTAGE_MSB               	0x29  // GPIO Voltage MSB
#define REG_GPIO_VOLTAGE_LSB               	0x2A  // GPIO Voltage LSB
#define REG_GPIO_THRESHOLD_HIGH_MSB        	0x2B  // GPIO Threshold High MSB
#define REG_GPIO_THRESHOLD_HIGH_LSB        	0x2C  // GPIO Threshold High LSB
#define REG_GPIO_THRESHOLD_LOW_MSB         	0x2D  // GPIO Threshold Low MSB
#define REG_GPIO_THRESHOLD_LOW_LSB         	0x2E  // GPIO Threshold Low LSB


// Define bit masks for the Status register
#define STAT_A0_UVLO_ALERT               	(0b1 << 0) 	// 00000001 (Default 1)
#define STAT_A1_VOLTAGE_ALERT            	(0b1 << 1) 	// 00000010 (Default 0)
#define STAT_A2_CHARGE_ALERT_LOW         	(0b1 << 2) 	// 00000100 (Default 0)
#define STAT_A3_CHARGE_ALERT_HIGH        	(0b1 << 3) 	// 00001000 (Default 0)
#define STAT_A4_TEMPERATURE_ALERT        	(0b1 << 4) 	// 00010000 (Default 0)
#define STAT_A5_CHARGE_OVERFLOW_UNDERFLOW 	(0b1 << 5) 	// 00100000 (Default 0)
#define STAT_A6_CURRENT_ALERT            	(0b1 << 6) 	// 01000000 (Default 0)
#define STAT_A7_GPIO_ALERT               	(0b1 << 7) 	// 10000000 (Default 0)


// Bit positions and masks for ADC Mode (B[7:5] (Default 000))
#define CTRL_ADC_MODE_MASK         			(0b111 << 5)	// 11100000
/** @defgroup ADC_MODE
  * @{
  */
#define CTRL_ADC_MODE_SLEEP        			(0b000 << 5)	// 00000000
#define CTRL_ADC_MODE_SMART_SLEEP  			(0b001 << 5)	// 00100000
#define CTRL_ADC_MODE_CONT_V       			(0b010 << 5)	// 01000000
#define CTRL_ADC_MODE_CONT_I       			(0b011 << 5)	// 01100000
#define CTRL_ADC_MODE_CONT_ALT_V_I      	0b10000000 //(0b100 << 5)	// 10000000
#define CTRL_ADC_MODE_SINGLE_SHOT_V_I_T		(0b101 << 5)	// 10100000
#define CTRL_ADC_MODE_CONT_V_I_T   			(0b110 << 5)	// 11000000
/**
  * @}
  */


// Bit positions and masks for GPIO Configure (B[4:3])
#define CTRL_GPIO_CONFIG_MASK               (0b11 << 3)		// 00011000
/** @defgroup GPIO_CONFIG
  * @{
  */
#define CTRL_GPIO_CONFIG_ALERT_MODE         	(0b00 << 3)		// 00000000
#define CTRL_GPIO_CONFIG_CHARGE_COMPLETE    	(0b01 << 3)		// 00001000
#define CTRL_GPIO_CONFIG_ANALOG_INPUT_97mV   	(0b10 << 3)		// 00010000
#define CTRL_GPIO_CONFIG_ANALOG_INPUT_1560mV  	(0b11 << 3)		// 00011000
/**
  * @}
  */


// Bit position and mask for Configure Voltage Input (B[2])
#define CTRL_CONFIG_VOLTAGE_INPUT_MASK		(0b1 << 2)		// 00000100
/** @defgroup VOLTAGE_INPUT
  * @{
  */
#define CTRL_CONFIG_VOLTAGE_INPUT_VDD       (0b0 << 2)		// 00000000
#define CTRL_CONFIG_VOLTAGE_INPUT_SENSEN    (0b1 << 2)		// 00000100
/**
  * @}
  */


// Bit Positions and mask for Coulomb Counter Configuration (C) 8-bit (Default 01)
#define CC_CONFIG_DEADBAND_MASK      (0b11 << 6)
/** @defgroup COULOMB_COUNTER_DEADBAND
  * @{
  */
#define CC_CONFIG_DEADBAND_0				(0b00 << 6)		// 00000000 0uV, no deadband
#define CC_CONFIG_DEADBAND_20				(0b01 << 6)		// 01000000 20uV
#define CC_CONFIG_DEADBAND_40				(0b10 << 6)		// 10000000 40uV
#define CC_CONFIG_DEADBAND_80				(0b11 << 6)		// 11000000 80uV
/**
  * @}
  */


// Bit position and mask for Do Not Count (C[3])
#define CC_CONFIG_DO_NOT_COUNT_MASK  		(0b1 << 3)		// 00000000 (Do Not Count) Stops coulomb counting
/** @defgroup COULOMB_COUNTER_ON_OFF
  * @{
  */
#define CC_CONFIG_DO_NOT_COUNT       		CC_CONFIG_DO_NOT_COUNT_MASK
#define CC_CONFIG_COUNT              		(0b0 << 3)
/**
  * @}
  */

// Bit positions and masks for Reserved bits (C[5:4])
#define CC_CONFIG_RESERVED_54_MASK   		(0b11 << 4)
#define CC_CONFIG_RESERVED_54_DEFAULT 		(0b01 << 4) 	// Default value for reserved bits

// Bit positions and masks for Reserved bits (C[2:0])
#define CC_CONFIG_RESERVED_20_MASK   		(0b111 << 0)
#define CC_CONFIG_RESERVED_20_DEFAULT 		(0b000 << 0) 	// Default value for reserved bits




typedef struct{
	uint8_t ADC_mode;			/*!< Specifies the ADC Mode.
                                  This parameter can be a value of @ref ADC_MODE */
	uint8_t GPIO_config;		/*!< Configures the GPIO pin.
                                  This parameter can be a value of @ref GPIO_CONFIG */
	uint8_t voltage_input;		/*!< Specifies which pin to use as main input to the ADC.
                                  This parameter can be a value of @ref VOLTAGE_INPUT */
	uint8_t CC_deadband;		/*!< Sets the VSENSE threshold below which no charge is added to the ACR.
                                  This parameter can be a value of @ref COULOMB_COUNTER_DEADBAND */
}LTC2959_Config_t;


void LTC2959_Init(LTC2959_Config_t *config_t);

float LTC2959_Get_Acc_Charge();
float LTC2959_Get_Voltage();
float LTC2959_Get_Current();
bool LTC2959_Chg_Over_Under(void);
bool LTC2959_Chg_Alert_High(void);
bool LTC2959_Chg_Alert_High(void);
void Set_Do_Not_Count(bool dnc);

extern LTC2959_Config_t ltc2959;


#endif /* LTC2959_H_ */
