/*
 * ltc2959.c
 *
 *  Created on: May 28, 2024
 *      Author: rahul
 */

#include "../../ltc2959/ltc2959.h"

#include "i2c.h"

static HAL_StatusTypeDef Write_Reg(uint8_t reg, uint8_t value) {
	uint8_t buf[1] = {value};

	if(!HAL_I2C_Mem_Write(&LTC2959_I2C_PORT, LTC2959_I2C_ADDR, reg, 1, &buf[0], 1, 10));
	return HAL_ERROR;
}

static uint8_t Read_Reg(uint8_t reg){
	uint8_t value;
	HAL_I2C_Mem_Read(&LTC2959_I2C_PORT, LTC2959_I2C_ADDR, reg, 1, &value, 1, 10);
	return value;
}


/* ********************* /// CONTROL REGISTER \\\ **************************** */

/*
 * Param: mode - This parameter can be a value of @ref ADC_MODE
 * Specifies the ADC MODE
*/
static void Set_ADC_Mode(uint8_t mode){
    uint8_t value = Read_Reg(REG_ADC_CONTROL);
    MODIFY_REG(value, CTRL_ADC_MODE_MASK, mode);
//    value = (value & ~CTRL_ADC_MODE_MASK) | mode;
    Write_Reg(REG_ADC_CONTROL, value);
}


/*
 * Param: config - This parameter can be a value of @ref GPIO_CONFIG
 * Specifies the GPIO PIN in different modes
*/
static void Set_GPIO_Configure(uint8_t config) {
    uint8_t value = Read_Reg(REG_ADC_CONTROL);
    MODIFY_REG(value, CTRL_GPIO_CONFIG_MASK, config);
//    value = (value & ~CTRL_GPIO_CONFIG_MASK) | config;
    Write_Reg(REG_ADC_CONTROL, value);
}


/*
 * Param: input - This parameter can be a value of @ref VOLTAGE_INPUT
 * Specifies the VOLTAGE INPUT PIN, BETWEEN VDD OR SENSEN
*/
static void Set_Voltage_Input(uint8_t input) {
    uint8_t value = Read_Reg(REG_ADC_CONTROL);
    MODIFY_REG(value, CTRL_CONFIG_VOLTAGE_INPUT_MASK, input);
//    value = (value & ~CTRL_CONFIG_VOLTAGE_INPUT_MASK) | config;
    Write_Reg(REG_ADC_CONTROL, value);
}


/*
 * Param: deadband - This parameter can be a value of @ref COULOMB_COUNTER_DEADBAND
 * Specifies the DEADBAND OF COULOMB COUNTER
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
 * Param: dnc - This parameter can be a value of @ref COULOMB_COUNTER_ON_OFF
 * Controls the COULOMB COUNTER
*/
static void Set_Do_Not_Count(uint8_t dnc){
	uint8_t value = Read_Reg(REG_COULOMB_COUNTER_CONTROL);
	MODIFY_REG(value, CC_CONFIG_DO_NOT_COUNT_MASK, dnc);
	// Ensure reserved bits are set to their default values
	MODIFY_REG(value, CC_CONFIG_RESERVED_54_MASK, CC_CONFIG_RESERVED_54_DEFAULT);
	MODIFY_REG(value, CC_CONFIG_RESERVED_20_MASK, CC_CONFIG_RESERVED_20_DEFAULT);
	Write_Reg(REG_COULOMB_COUNTER_CONTROL, value);
}

/*
 * Param: config_t - pointer to configuration data for LTC2959
 * Init function for LTC2959
 */
void LTC2959_Init(LTC2959_Config_t *config_t){
	Set_ADC_Mode(config_t->ADC_mode);
	Set_GPIO_Configure(config_t->GPIO_config);
	Set_Voltage_Input(config_t->GPIO_config);
	Set_Coulomb_Counter_Deadband(config_t->CC_deadband);
}


/* ********************* /// STATUS REGISTER \\\ *************************** */

bool LTC2959_Chg_Over_Under(void){
	uint8_t value = Read_Reg(REG_STATUS);
	if(READ_BIT(value, STAT_A5_CHARGE_OVERFLOW_UNDERFLOW)){
	return 1;
	}else return 0;
}

bool LTC2959_Chg_Alert_High(void){
	uint8_t value = Read_Reg(REG_STATUS);
	if(READ_BIT(value, STAT_A3_CHARGE_ALERT_HIGH)){
		return 1;
	}else return 0;
}

bool LTC2959_Chg_Alert_Low(void){
	uint8_t value = Read_Reg(REG_STATUS);
	if(READ_BIT(value, STAT_A2_CHARGE_ALERT_LOW)){
		return 1;
	}else return 0;
}



/* ******************** // GET OUTPUT \\ ********************* */
float LTC2959_Get_Acc_Charge(){
	uint8_t buf[4];
	uint32_t charge;
	float total_charge;

	// Read the MSB and LSB of the voltage register
	buf[0] = Read_Reg(REG_ACCUMULATED_CHARGE_MSB);
	buf[1] = Read_Reg(REG_ACCUMULATED_CHARGE_15_8);
	buf[2] = Read_Reg(REG_ACCUMULATED_CHARGE_23_16);
	buf[3] = Read_Reg(REG_ACCUMULATED_CHARGE_LSB);

	charge = 	((uint32_t)buf[0] << 24) |
				((uint32_t)buf[1] << 16) |
				((uint32_t)buf[2] << 8)	 |
				(uint32_t)buf[3];
	total_charge= charge * ACR_LSB;

	return total_charge;
}


float LTC2959_Get_Voltage(){
	uint8_t buf[2];
	uint16_t value;
	float voltage;

	// Read the MSB and LSB of the voltage register
	buf[0] = Read_Reg(REG_VOLTAGE_MSB);
	buf[1] = Read_Reg(REG_VOLTAGE_LSB);
	value = (buf[0] << 8) | buf[1];
	voltage = 62.6 * ((float)value / 65536.0);

	return voltage;
}


float LTC2959_Get_Current(LTC2959_Config_t *config_t){
	uint8_t buf[2];
	int16_t value;
	float current;

	// Read the MSB and LSB of the current register
	buf[0] = Read_Reg(REG_CURRENT_MSB);
	buf[1] = Read_Reg(REG_CURRENT_LSB);

	// Combine MSB and LSB into 16-bit signed value
	value = (int16_t)((buf[0] << 8) | buf[1]);

	// Calculate the current
	current = ((97.5f / config_t->sense_resistor) * (value / 32768.0f));

	return current;
}


float LTC2959_Get_Temperature(){
	uint8_t buf[2];
	uint16_t value;
	float temp;

	// Read the MSB and LSB of the temperature register
	buf[0] = Read_Reg(REG_TEMPERATURE_MSB);
	buf[1] = Read_Reg(REG_TEMPERATURE_LSB);

	value = (int16_t)((buf[0] << 8) | buf[1]);
	temp = (825.0 * ((float)value / 65536.0)) - 273.15;

	return temp;
}


float LTC2959_Get_GPIO_ADC_Voltage(LTC2959_Config_t *config_t){
	uint8_t buf[2];
	int16_t value;
	float adc_val;

	// Read the MSB and LSB of the current register
	buf[0] = Read_Reg(REG_GPIO_VOLTAGE_MSB);
	buf[1] = Read_Reg(REG_GPIO_VOLTAGE_LSB);

	// Combine MSB and LSB into 16-bit signed value
	value = (int16_t)((buf[0] << 8) | buf[1]);

	// Calculate the GPIO_Pin Voltage
	switch(config_t->GPIO_config){
	case CTRL_GPIO_CONFIG_ANALOG_INPUT_97mV:
		// ±97.5mV full-scale input, convert mV to V
		adc_val = (97.5f * ((float)value / 32768.0f)) / 1000.0f;  // result in Volts
		break;
	case CTRL_GPIO_CONFIG_ANALOG_INPUT_1560mV:
		// ±97.5mV full-scale input, convert mV to V
		adc_val = (1560.0f * ((float)value / 32768.0f)) / 1000.0f;  // result in Volts
		break;
	default:
		 adc_val = -0.0f;	//`
		break;
	}

	return adc_val;
}



/* ******************** // SET \\********************* */

/*
 * Param: value - This can be any value between 2000mV to 60000mV
 * Specifies the Low voltage threshold.
 */
HAL_StatusTypeDef LTC2959_Set_Volt_Thrs_Low(uint16_t value){
	if(value <= 2000)value = 2000;					// Value should not be less than 2V or 2000mV
	else if (value >=60000) value = 60000;			// Value shpuld not be greater than 60V or 60000mV

	uint16_t val = value * 65536 / 62600;
	uint8_t msb = (val >> 8) & 0xFF;  				// High byte (MSB)
	uint8_t lsb = val & 0xFF;         				// Low byte (LSB)

	if(!Write_Reg(REG_VOLTAGE_THRESHOLD_LOW_MSB, msb))
		return HAL_ERROR;
	if(!Write_Reg(REG_VOLTAGE_THRESHOLD_LOW_LSB, lsb))
		return HAL_ERROR;
	return HAL_OK;
}

/*
 * Param: value - This can be any value between 2000mV to 60000mV
 * Specifies the High voltage threshold.
 */
HAL_StatusTypeDef LTC2959_Set_Volt_Thrs_High(uint16_t value){
	if(value <= 2000)value = 2000;					// Value should not be less than 2V or 2000mV
	else if (value >=60000) value = 60000;			// Value shpuld not be greater than 60V or 60000mV

	uint16_t val = value * 65536 / 62600;			// Result = Vbat * 65536 / 62.2V Pg-13
	uint8_t msb = (val >> 8) & 0xFF;  				// High byte (MSB)
	uint8_t lsb = val & 0xFF;         				// Low byte (LSB)

	if(!Write_Reg(REG_VOLTAGE_THRESHOLD_HIGH_MSB, msb))
		return HAL_ERROR;
	if(!Write_Reg(REG_VOLTAGE_THRESHOLD_HIGH_LSB, lsb))
		return HAL_ERROR;
	return HAL_OK;
}

