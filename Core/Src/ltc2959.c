/*
 * ltc2959.c
 *
 *  Created on: May 28, 2024
 *      Author: rahul
 */

#include "i2c.h"
#include "ltc2959.h"

static void Write_Reg(uint8_t reg, uint8_t value) {
	uint8_t buf[1] = {value};

	HAL_I2C_Mem_Write(&LTC2959_I2C_PORT, LTC2959_I2C_ADDR, reg, 1, &buf[0], 1, 10);
}

static uint8_t Read_Reg(uint8_t reg){
	uint8_t value;
	HAL_I2C_Mem_Read(&LTC2959_I2C_PORT, LTC2959_I2C_ADDR, reg, 1, &value, 1, 10);
	return value;
}


/*
Param: mode - This parameter can be a value of @ref ADC_MODE
Specifies the ADC MODE
*/
static void Set_ADC_Mode(uint8_t mode){
    uint8_t value = Read_Reg(REG_ADC_CONTROL);
    MODIFY_REG(value, CTRL_ADC_MODE_MASK, mode);
//    value = (value & ~CTRL_ADC_MODE_MASK) | mode;
    Write_Reg(REG_ADC_CONTROL, value);
}


/*
Param: mode - This parameter can be a value of @ref GPIO_CONFIG
Specifies the GPIO PIN in different modes
*/
static void Set_GPIO_Configure(uint8_t config) {
    uint8_t value = Read_Reg(REG_ADC_CONTROL);
    MODIFY_REG(value, CTRL_GPIO_CONFIG_MASK,config);
//    value = (value & ~CTRL_GPIO_CONFIG_MASK) | config;
    Write_Reg(REG_ADC_CONTROL, value);
}


/*
Param: mode - This parameter can be a value of @ref VOLTAGE_INPUT
Specifies the VOLTAGE INPUT PIN BETWEEN VDD AND SENSEN
*/
static void Set_Voltage_Input(uint8_t config) {
    uint8_t value = Read_Reg(REG_ADC_CONTROL);
    MODIFY_REG(value, CTRL_CONFIG_VOLTAGE_INPUT_MASK, config);
//    value = (value & ~CTRL_CONFIG_VOLTAGE_INPUT_MASK) | config;
    Write_Reg(REG_ADC_CONTROL, value);
}


/*
Param: mode - This parameter can be a value of @ref COULOMB_COUNTER_DEADBAND
Specifies the DEADBAND OF COULOMB COUNTER
*/
static void Set_Coulomb_Counter_Deadband(uint8_t deadband){
	uint8_t value = Read_Reg(REG_COULOMB_COUNTER_CONTROL);
    // Set the deadband
	MODIFY_REG(value, CC_CONFIG_DEADBAND_MASK, deadband);
    // Ensure reserved bits are set to their default values
	MODIFY_REG(value, CC_CONFIG_RESERVED_54_MASK, CC_CONFIG_RESERVED_54_DEFAULT);
	MODIFY_REG(value, CC_CONFIG_RESERVED_20_MASK, CC_CONFIG_RESERVED_20_DEFAULT);
	Write_Reg(REG_COULOMB_COUNTER_CONTROL, value);
}


/*
Param: mode - This parameter can be a value of @ref COULOMB_COUNTER_ON_OFF
Controls the COULOMB COUNTER
*/
static void Set_Do_Not_Count(uint8_t dnc){
	uint8_t value = Read_Reg(REG_COULOMB_COUNTER_CONTROL);
	MODIFY_REG(value, CC_CONFIG_DO_NOT_COUNT_MASK, dnc);
	// Ensure reserved bits are set to their default values
	MODIFY_REG(value, CC_CONFIG_RESERVED_54_MASK, CC_CONFIG_RESERVED_54_DEFAULT);
	MODIFY_REG(value, CC_CONFIG_RESERVED_20_MASK, CC_CONFIG_RESERVED_20_DEFAULT);
	Write_Reg(REG_COULOMB_COUNTER_CONTROL, value);
}


void LTC2959_Init(LTC2959_Config_t *config_t){
	Set_ADC_Mode(config_t->ADC_mode);
	Set_GPIO_Configure(config_t->GPIO_config);
	Set_Voltage_Input(config_t->GPIO_config);
	Set_Coulomb_Counter_Deadband(config_t->CC_deadband);
}










