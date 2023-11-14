#include <stdio.h>
#include "stdlib.h"
#include "string.h"
// AW210XX HEADER FILES
#include "aw210xx.h"
#include "aw210xx_reg.h"
// NRF_CONNECT_SDK
#include <nrfx_twim.h>
#include <nrfx_twis.h>

#define AW210XX_DRIVER_VERSION          "V0.4.0"
#define TWI_INSTANCE_ID					0


// TRUE -> Start/Standby
// FALSE -> End/Finished
static volatile bool m_xfer_end = false;
static const nrfx_twis_t m_twis_inst = NRFX_TWIS_INSTANCE(TWI_INSTANCE_ID);



struct aw210xx *g_aw210xx;
AW210XX_CFG aw210xx_cfg_array[] = {
	{aw210xx_group_cfg_led_off, sizeof(aw210xx_group_cfg_led_off)},
	{aw21018_group_all_leds_on, sizeof(aw21018_group_all_leds_on)},
	{aw21018_group_red_leds_on, sizeof(aw21018_group_red_leds_on)},
	{aw21018_group_green_leds_on, sizeof(aw21018_group_green_leds_on)},
	{aw21018_group_blue_leds_on, sizeof(aw21018_group_blue_leds_on)},
	{aw21018_group_breath_leds_on, sizeof(aw21018_group_breath_leds_on)},
	{aw21012_group_all_leds_on, sizeof(aw21012_group_all_leds_on)},
	{aw21012_group_red_leds_on, sizeof(aw21012_group_red_leds_on)},
	{aw21012_group_green_leds_on, sizeof(aw21012_group_green_leds_on)},
	{aw21012_group_blue_leds_on, sizeof(aw21012_group_blue_leds_on)},
	{aw21012_group_breath_leds_on, sizeof(aw21012_group_breath_leds_on)},
	{aw21009_group_all_leds_on, sizeof(aw21009_group_all_leds_on)},
	{aw21009_group_red_leds_on, sizeof(aw21009_group_red_leds_on)},
	{aw21009_group_green_leds_on, sizeof(aw21009_group_green_leds_on)},
	{aw21009_group_blue_leds_on, sizeof(aw21009_group_blue_leds_on)},
	{aw21009_group_breath_leds_on, sizeof(aw21009_group_breath_leds_on)}
};

void twi_m_init(void){
	ret_code_t err_code;
	const nrfx_twim_default_config twi_config = {
		._pin_scl				= TWI_SCL_M,
		._pin_sda				= TWI_SDA_M,
		.frequency				= NRF_DRV_TWI_FREQ_250K,
		.interrupt_priority		= APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init			= false,
	};

	err_code = nrf_drv_twi_init(
		&m_twi,
		&twi_config,
		twi_handler,
		NULL,
	);

	APP_ERROR_CHECK(err_code);
	nrf_drv_twi_enable(&m_twi);
}

// NCS BASED TWIM(I2C)
void twi_handler(nrfx_twis_flag_no const *pEvent, void *pContect){
	switch (pEvent->type){
		case NRF_DRV_TWI_EVT_END:
			m_xfer_end = true;
			break;

		default:
			break;
	}
}

static HAL_StatusTypeDef aw210xx_i2c_read(struct aw210xx *aw210xx,
					  uint8_t reg_addr,
					  uint8_t *reg_data)
{
	uint8_t cnt = 0;
	uint8_t slave_data[1] = { 0 };
	HAL_StatusTypeDef status;

	while (cnt < AW210XX_I2C_RW_RETRIES) {
		status = HAL_I2C_Mem_Read(aw210xx->hi2cx,
					  AW210XX_I2C_ADDR << 1,
					  reg_addr,
					  AW210XX_REGADD_SIZE_8BIT,
					  slave_data,
					  AW210XX_REGDATA_SIZE_8BIT,
					  100);
		if (status != HAL_OK) {
			cnt++;
		} else {
			*reg_data = slave_data[0];
			break;
		}
	}

	if (status != HAL_OK)
		AWLOGD("%s: fail! status is %d\n", __func__, status);

	return status;
}

static HAL_StatusTypeDef aw210xx_i2c_write(struct aw210xx *aw210xx,
					   uint8_t reg_addr,
					   uint8_t reg_data)
{
	uint8_t cnt = 0;
	uint8_t slave_data[1] = { 0 };
	HAL_StatusTypeDef status;

	slave_data[0] = reg_data;

	while (cnt < AW210XX_I2C_RW_RETRIES) {
		status = HAL_I2C_Mem_Write(aw210xx->hi2cx,
					   AW210XX_I2C_ADDR << 1,
					   reg_addr,
					   AW210XX_REGADD_SIZE_8BIT,
					   slave_data,
					   AW210XX_REGDATA_SIZE_8BIT,
					   100);
		if (status != HAL_OK)
			cnt++;
		else
			break;
	}

	if (status != HAL_OK)
		AWLOGD("%s: fail! status is %d\n", __func__, status);

	return status;
}

static int32_t aw210xx_i2c_write_bits(struct aw210xx *aw210xx,
				      uint8_t reg_addr,
				      uint32_t reg_mask,
				      uint8_t reg_data)
{
	uint8_t val;

	aw210xx_i2c_read(aw210xx, reg_addr, &val);
	val &= reg_mask;
	val |= (reg_data & (~reg_mask));
	aw210xx_i2c_write(aw210xx, reg_addr, val);

	return 0;
}

/*****************************************************
* open short detect
*****************************************************/
void aw210xx_open_detect_cfg(struct aw210xx *aw210xx)
{
	HAL_StatusTypeDef status;
	/*enable open detect*/
	status = aw210xx_i2c_write(aw210xx, AW210XX_REG_OSDCR, AW210XX_OPEN_DETECT_EN);
	if (status != HAL_OK)
			AWLOGD("%s: open_detect_cfg failed\n", __func__);
	/*set DCPWM = 1*/
	aw210xx_i2c_write_bits(aw210xx, AW210XX_REG_SSCR, AW210XX_DCPWM_SET_MASK, AW210XX_DCPWM_SET);
	/*set Open threshold = 0.2v*/
	aw210xx_i2c_write_bits(aw210xx,
							AW210XX_REG_OSDCR,
							AW210XX_OPEN_THRESHOLD_SET_MASK,
							AW210XX_OPEN_THRESHOLD_SET);
}

void aw210xx_short_detect_cfg(struct aw210xx *aw210xx)
{
	HAL_StatusTypeDef status;
	/*enable short detect*/
	status = aw210xx_i2c_write(aw210xx, AW210XX_REG_OSDCR, AW210XX_SHORT_DETECT_EN);
	if (status != HAL_OK)
			AWLOGD("%s: short_detect_cfg failed\n", __func__);
	/*set DCPWM = 1*/
	aw210xx_i2c_write_bits(aw210xx, AW210XX_REG_SSCR, AW210XX_DCPWM_SET_MASK, AW210XX_DCPWM_SET);
	/*set Short threshold = 1v*/
	aw210xx_i2c_write_bits(aw210xx,
							AW210XX_REG_OSDCR,
							AW210XX_SHORT_THRESHOLD_SET_MASK,
							AW210XX_SHORT_THRESHOLD_SET);
}

void aw210xx_open_short_dis(struct aw210xx *aw210xx)
{
	aw210xx_i2c_write(aw210xx, AW210XX_REG_OSDCR, AW210XX_OPEN_SHORT_DIS);
	/*SET DCPWM = 0*/
	aw210xx_i2c_write_bits(aw210xx, AW210XX_REG_SSCR, AW210XX_DCPWM_SET_MASK, AW210XX_DCPWM_CLEAN);
}

void aw210xx_open_short_read(struct aw210xx *aw210xx)
{
	uint8_t reg_val[3] = {0};
	int i = 0;

	aw210xx_i2c_read(aw210xx, AW210XX_REG_OSST0, &reg_val[0]);
	aw210xx_i2c_read(aw210xx, AW210XX_REG_OSST1, &reg_val[1]);
	aw210xx_i2c_read(aw210xx, AW210XX_REG_OSST2, &reg_val[2]);
	for (i = 0; i < 3; i++)
		AWLOGD("%s: OSST%d:%#x\n", __func__, i, reg_val[i]);
}

/*****************************************************
* pin en pull high/low
*****************************************************/
int32_t aw210xx_hw_reset(void)
{
	if (g_aw210xx->gpio_port != NULL && g_aw210xx->en_gpio_pin != 0) {
		HAL_GPIO_WritePin(g_aw210xx->gpio_port, g_aw210xx->en_gpio_pin,
				  GPIO_PIN_RESET);
		HAL_Delay(2);
		HAL_GPIO_WritePin(g_aw210xx->gpio_port, g_aw210xx->en_gpio_pin,
				  GPIO_PIN_SET);
		HAL_Delay(3);
	} else {
		AWLOGD("%s:pin en reset error! no config gpio info!\n",
		       __func__);
		return -AW210XX_NO_GPIO_INFO;
	}

	return 0;
}

int32_t aw210xx_en_pull_high(void)
{
	if (g_aw210xx->gpio_port != NULL && g_aw210xx->en_gpio_pin != 0) {
		HAL_GPIO_WritePin(g_aw210xx->gpio_port, g_aw210xx->en_gpio_pin,
				  GPIO_PIN_SET);
		HAL_Delay(3);
		g_aw210xx->en_flag = 1;
		AWLOGD("%s:pin en pull high !\n", __func__);
	} else {
		AWLOGD("%s:pin en pull high error! no config gpio info!\n",
		       __func__);
		return -AW210XX_NO_GPIO_INFO;
	}

	return 0;
}

int32_t aw210xx_en_pull_low(void)
{
	if (g_aw210xx->gpio_port != NULL && g_aw210xx->en_gpio_pin != 0) {
		HAL_GPIO_WritePin(g_aw210xx->gpio_port, g_aw210xx->en_gpio_pin,
				  GPIO_PIN_RESET);
		HAL_Delay(2);
		g_aw210xx->en_flag = 0;
		AWLOGD("%s:pin en pull low !\n", __func__);
	} else {
		AWLOGD("%s:pin en pull low error! no config gpio info!\n",
		       __func__);
		return -AW210XX_NO_GPIO_INFO;
	}

	return 0;
}

/*****************************************************
* aw210xx sw reset
*****************************************************/
void aw210xx_sw_reset(void)
{
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_RESET, AW210XX_RESET_CHIP);
	HAL_Delay(3);
}

/*****************************************************
* led Interface: set effect
*****************************************************/
static void aw210xx_update_cfg_array(struct aw210xx *aw210xx,
				     uint8_t *p_cfg_data,
				     uint32_t cfg_size)
{
	unsigned int i = 0;

	for (i = 0; i < cfg_size; i += 2)
		aw210xx_i2c_write(aw210xx, p_cfg_data[i], p_cfg_data[i + 1]);
}

void aw210xx_effect_select(effect_select_t effect)
{
	g_aw210xx->effect = effect;
	aw210xx_update_cfg_array(g_aw210xx,
				 aw210xx_cfg_array[g_aw210xx->effect].p,
				 aw210xx_cfg_array[g_aw210xx->effect].count);
}

int32_t aw210xx_get_current_effect(void)
{
	switch (g_aw210xx->effect) {
	case GROUP_ALL_LED_OFF:
		AWLOGD("%s:effect is all led off!\n", __func__);
		break;
	case AW21018_GROUP_ALL_LEDS_ON:
		AWLOGD("%s:effect is aw21018 all led on!\n", __func__);
		break;
	case AW21018_GROUP_RED_LEDS_ON:
		AWLOGD("%s:effect is aw21018 red led on!\n", __func__);
		break;
	case AW21018_GROUP_GREEN_LEDS_ON:
		AWLOGD("%s:effect is aw21018 green led on!\n", __func__);
		break;
	case AW21018_GROUP_BLUE_LEDS_ON:
		AWLOGD("%s:effect is aw21018 blue led on!\n", __func__);
		break;
	case AW21018_GROUP_BREATH_LEDS_ON:
		AWLOGD("%s:effect is aw21018 breath led on!\n", __func__);
		break;
	case AW21012_GROUP_ALL_LEDS_ON:
		AWLOGD("%s:effect is aw21012 all led on!\n", __func__);
		break;
	case AW21012_GROUP_RED_LEDS_ON:
		AWLOGD("%s:effect is aw21012 red led on!\n", __func__);
		break;
	case AW21012_GROUP_GREEN_LEDS_ON:
		AWLOGD("%s:effect is aw21012 green led on!\n", __func__);
		break;
	case AW21012_GROUP_BLUE_LEDS_ON:
		AWLOGD("%s:effect is aw21012 blue led on!\n", __func__);
		break;
	case AW21012_GROUP_BREATH_LEDS_ON:
		AWLOGD("%s:effect is aw21012 breath led on!\n", __func__);
		break;
	case AW21009_GROUP_ALL_LEDS_ON:
		AWLOGD("%s:effect is aw21009 all led on!\n", __func__);
		break;
	case AW21009_GROUP_RED_LEDS_ON:
		AWLOGD("%s:effect is aw21009 red led on!\n", __func__);
		break;
	case AW21009_GROUP_GREEN_LEDS_ON:
		AWLOGD("%s:effect is aw21009 green led on!\n", __func__);
		break;
	case AW21009_GROUP_BLUE_LEDS_ON:
		AWLOGD("%s:effect is aw21009 blue led on!\n", __func__);
		break;
	case AW21009_GROUP_BREATH_LEDS_ON:
		AWLOGD("%s:effect is aw21009 breath led on!\n", __func__);
		break;
	default:
		AWLOGD("%s:this effect is unsupported in pwm mode!\n",
			__func__);
		return -AW210XX_EFFECT_MODE_UNSUPPORT;
	}
	return 0;
}

/*****************************************************
* led Interface: set reg & get reg
*****************************************************/
int32_t aw210xx_set_reg(uint8_t reg_addr, uint8_t reg_data)
{
	HAL_StatusTypeDef status;

	if (aw210xx_reg_access[reg_addr] & REG_WR_ACCESS) {
		status = aw210xx_i2c_write(g_aw210xx, reg_addr, reg_data);
		if (status == HAL_OK) {
			AWLOGD("%s:set reg 0x%02x = 0x%02x success!\n",
			       __func__, reg_addr, reg_data);
		} else {
			AWLOGD("%s:set reg failed!\n", __func__);
			return -AW210XX_WRITE_FAIL;
		}
	} else {
		AWLOGD("%s:reg [0x%02x] no write access!\n",
		       __func__, reg_addr);
		return -AW210XX_NO_ACCESS;
	}

	return 0;
}

int32_t aw210xx_get_reg(void)
{
	uint8_t i = 0;
	uint8_t reg_val = 0;
	uint8_t br_max = 0;
	uint8_t sl_val = 0;

	AWLOGD("\n*****get chip reg*****\n");
	aw210xx_i2c_read(g_aw210xx, AW210XX_REG_GCR, &reg_val);
	AWLOGD("reg:0x%02x = 0x%02x\n", AW210XX_REG_GCR, reg_val);
	switch (g_aw210xx->chipid) {
	case AW21018_CHIPID:
		br_max = AW210XX_REG_BR17H;
		sl_val = AW210XX_REG_SL17;
		break;
	case AW21012_CHIPID:
		br_max = AW210XX_REG_BR11H;
		sl_val = AW210XX_REG_SL11;
		break;
	case AW21009_CHIPID:
		br_max = AW210XX_REG_BR08H;
		sl_val = AW210XX_REG_SL08;
		break;
	default:
		AWLOGD("%s: chip is unsupported device!\n",
		       __func__);
		return -AW210XX_CHIPID_FAILD;
	}
	for (i = AW210XX_REG_BR00L; i <= br_max; i++) {
		if (!(aw210xx_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw210xx_i2c_read(g_aw210xx, i, &reg_val);
		AWLOGD("reg:0x%02x = 0x%02x\n", i, reg_val);
	}
	for (i = AW210XX_REG_SL00; i <= sl_val; i++) {
		if (!(aw210xx_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw210xx_i2c_read(g_aw210xx, i, &reg_val);
		AWLOGD("reg:0x%02x = 0x%02x\n", i, reg_val);
	}
	for (i = AW210XX_REG_GCCR; i <= AW210XX_REG_GCFG; i++) {
		if (!(aw210xx_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw210xx_i2c_read(g_aw210xx, i, &reg_val);
		AWLOGD("reg:0x%02x = 0x%02x\n", i, reg_val);
	}
	return 0;
}

/*****************************************************
* aw210xx led function set
*****************************************************/
int32_t aw210xx_clk_pwm_set(clk_pwm_t clk_pwm)
{
	switch (clk_pwm) {
	case CLK_FRQ_16M:
		AWLOGD("%s:osc is 16MHz!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_CLKFRQ_MASK,
				       AW210XX_BIT_CLKFRQ_16MH);
		break;
	case CLK_FRQ_8M:
		AWLOGD("%s:osc is 8MHz!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_CLKFRQ_MASK,
				       AW210XX_BIT_CLKFRQ_8MH);
		break;
	case CLK_FRQ_4M:
		AWLOGD("%s:osc is 4MHz!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_CLKFRQ_MASK,
				       AW210XX_BIT_CLKFRQ_4MH);
		break;
	case CLK_FRQ_2M:
		AWLOGD("%s:osc is 2MHz!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_CLKFRQ_MASK,
				       AW210XX_BIT_CLKFRQ_2MH);
		break;
	case CLK_FRQ_1M:
		AWLOGD("%s:osc is 1MHz!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_CLKFRQ_MASK,
				       AW210XX_BIT_CLKFRQ_1MH);
		break;
	case CLK_FRQ_512K:
		AWLOGD("%s:osc is 512KHz!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_CLKFRQ_MASK,
				       AW210XX_BIT_CLKFRQ_512KH);
		break;
	case CLK_FRQ_256K:
		AWLOGD("%s:osc is 256KHz!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_CLKFRQ_MASK,
				       AW210XX_BIT_CLKFRQ_256KH);
		break;
	case CLK_FRQ_125K:
		AWLOGD("%s:osc is 125KHz!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_CLKFRQ_MASK,
				       AW210XX_BIT_CLKFRQ_125KH);
		break;
	default:
		AWLOGD("%s:this clk_pwm is unsupported!\n",
		       __func__);
		return -AW210XX_CLK_MODE_UNSUPPORT;
	}
	return 0;
}

int32_t aw210xx_br_pwm_set(br_pwm_t br_pwm)
{
	g_aw210xx->br_pwm = br_pwm;
	switch (br_pwm) {
	case BR_RESOLUTION_8BIT:
		AWLOGD("%s:br resolution select 8bit!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_PWMRES_MASK,
				       AW210XX_BIT_PWMRES_8BIT);
		break;
	case BR_RESOLUTION_9BIT:
		AWLOGD("%s:br resolution select 9bit!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_PWMRES_MASK,
				       AW210XX_BIT_PWMRES_9BIT);
		break;
	case BR_RESOLUTION_12BIT:
		AWLOGD("%s:br resolution select 12bit!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_PWMRES_MASK,
				       AW210XX_BIT_PWMRES_12BIT);
		break;
	case BR_RESOLUTION_9_AND_3_BIT:
		AWLOGD("%s:br resolution select 9+3bit!\n", __func__);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_PWMRES_MASK,
				       AW210XX_BIT_PWMRES_9_AND_3_BIT);
		break;
	default:
		AWLOGD("%s:this br_pwm is unsupported!\n", __func__);
		return -AW210XX_CLK_MODE_UNSUPPORT;
	}
	return 0;
}

void aw210xx_apse_set(bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_APSE_MASK,
				       AW210XX_BIT_APSE_ENABLE);
	} else {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_APSE_MASK,
				       AW210XX_BIT_APSE_DISENA);
	}
}
/*****************************************************
* aw210xx basic function set
*****************************************************/
void aw210xx_chipen_set(bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_CHIPEN_MASK,
				       AW210XX_BIT_CHIPEN_ENABLE);
		HAL_Delay(1);
	} else {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR,
				       AW210XX_BIT_CHIPEN_MASK,
				       AW210XX_BIT_CHIPEN_DISENA);
	}
}

void aw210xx_uvlo_set(bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_UVCR,
				       AW210XX_BIT_UVPD_MASK,
				       AW210XX_BIT_UVPD_DISENA);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_UVCR,
				       AW210XX_BIT_UVDIS_MASK,
				       AW210XX_BIT_UVDIS_DISENA);
	} else {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_UVCR,
				       AW210XX_BIT_UVPD_MASK,
				       AW210XX_BIT_UVPD_ENABLE);
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_UVCR,
				       AW210XX_BIT_UVDIS_MASK,
				       AW210XX_BIT_UVDIS_ENABLE);
	}
}

void aw210xx_gsldis_set(bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCFG,
				       AW210XX_BIT_GSLDIS_MASK,
				       AW210XX_BIT_GSLDIS_DISENA);
	} else {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCFG,
				       AW210XX_BIT_GSLDIS_MASK,
				       AW210XX_BIT_GSLDIS_ENABLE);
	}
}

void aw210xx_rgbmd_set(bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR2,
				       AW210XX_BIT_RGBMD_MASK,
				       AW210XX_BIT_RGBMD_ENABLE);
		g_aw210xx->rgbmd_flag = 1;
	} else {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR2,
				       AW210XX_BIT_RGBMD_MASK,
				       AW210XX_BIT_RGBMD_DISENA);
		g_aw210xx->rgbmd_flag = 0;
	}
}

void aw210xx_sbmd_set(bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR2,
				       AW210XX_BIT_SBMD_MASK,
				       AW210XX_BIT_SBMD_ENABLE);
		g_aw210xx->sdmd_flag = 1;
	} else {
		aw210xx_i2c_write_bits(g_aw210xx,
				       AW210XX_REG_GCR2,
				       AW210XX_BIT_SBMD_MASK,
				       AW210XX_BIT_SBMD_DISENA);
		g_aw210xx->sdmd_flag = 0;
	}
}

/*****************************************************
* aw210xx compatible functions set
*****************************************************/
static int32_t aw210xx_group_gcfg_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		switch (aw210xx->chipid) {
		case AW21018_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21018_GROUP_ENABLE);
			return 0;
		case AW21012_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21012_GROUP_ENABLE);
			return 0;
		case AW21009_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21009_GROUP_ENABLE);
			return 0;
		default:
			AWLOGD("%s: chip is unsupported device!\n",
			       __func__);
			return -AW210XX_CHIPID_FAILD;
		}
	} else {
		switch (aw210xx->chipid) {
		case AW21018_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21018_GROUP_DISABLE);
			return 0;
		case AW21012_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21012_GROUP_DISABLE);
			return 0;
		case AW21009_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21009_GROUP_DISABLE);
			return 0;
		default:
			AWLOGD("%s: chip is unsupported device!\n",
			       __func__);
			return -AW210XX_CHIPID_FAILD;
		}
	}
}

static int32_t aw210xx_brnl_and_brnh_set(struct aw210xx *aw210xx,
					 uint16_t rgb_data)
{
	uint32_t i = 0;

	switch (aw210xx->br_pwm) {
	case BR_RESOLUTION_8BIT:
		aw210xx->brn_l = rgb_data;
		aw210xx->brn_h = 0x00;
		break;
	case BR_RESOLUTION_9BIT:
		aw210xx->brn_l = rgb_data;
		aw210xx->brn_h = (rgb_data >> 8) & 0x01;
		break;
	case BR_RESOLUTION_12BIT:
		aw210xx->brn_l = rgb_data;
		aw210xx->brn_h = (rgb_data >> 8) & 0x0f;
		break;
	default:
		AWLOGD("%s: chip is unsupported device!\n",
			__func__);
		return -AW210XX_CHIPID_FAILD;
	}
	for (i = AW210XX_REG_BR00L; i <= (aw210xx->br_max - 1); i += 2)
		aw210xx_i2c_write(aw210xx, i, aw210xx->brn_l);
	for (i = AW210XX_REG_BR00H; i <= aw210xx->br_max; i += 2)
		aw210xx_i2c_write(aw210xx, i, aw210xx->brn_h);

	return 0;
}

static int32_t aw210xx_br_set(struct aw210xx *aw210xx, uint16_t rgb_data)
{
	uint8_t br_data = 0;
	uint32_t i = 0;

	switch (aw210xx->chipid) {
	case AW21018_CHIPID:
		if (aw210xx->sdmd_flag == 1) {
			if (aw210xx->rgbmd_flag == 1)
				aw210xx->br_max = AW210XX_REG_BR02H;
			else
				aw210xx->br_max = AW210XX_REG_BR08H;
			br_data = rgb_data;
			for (i = AW210XX_REG_BR00L; i <= aw210xx->br_max; i++)
				aw210xx_i2c_write(aw210xx, i, br_data);
		} else {
			if (aw210xx->rgbmd_flag == 1)
				aw210xx->br_max = AW210XX_REG_BR05H;
			else
				aw210xx->br_max = AW210XX_REG_BR17H;
			aw210xx_brnl_and_brnh_set(aw210xx, rgb_data);
		}
		break;
	case AW21012_CHIPID:
		if (aw210xx->sdmd_flag == 1) {
			if (aw210xx->rgbmd_flag == 1)
				aw210xx->br_max = AW210XX_REG_BR01H;
			else
				aw210xx->br_max = AW210XX_REG_BR05H;
			br_data = rgb_data;
			for (i = AW210XX_REG_BR00L; i <= aw210xx->br_max; i++)
				aw210xx_i2c_write(aw210xx, i, br_data);
		} else {
			if (aw210xx->rgbmd_flag == 1)
				aw210xx->br_max = AW210XX_REG_BR03H;
			else
				aw210xx->br_max = AW210XX_REG_BR11H;
			aw210xx_brnl_and_brnh_set(aw210xx, rgb_data);
		}
		break;
	case AW21009_CHIPID:
		if (aw210xx->sdmd_flag == 1) {
			if (aw210xx->rgbmd_flag == 1)
				aw210xx->br_max = AW210XX_REG_BR01L;
			else
				aw210xx->br_max = AW210XX_REG_BR04L;
			br_data = rgb_data;
			for (i = AW210XX_REG_BR00L; i <= aw210xx->br_max; i++)
				aw210xx_i2c_write(aw210xx, i, br_data);
		} else {
			if (aw210xx->rgbmd_flag == 1)
				aw210xx->br_max = AW210XX_REG_BR02H;
			else
				aw210xx->br_max = AW210XX_REG_BR08H;
			aw210xx_brnl_and_brnh_set(aw210xx, rgb_data);
		}
		break;
	default:
		AWLOGD("%s: chip is unsupported device!\n",
		       __func__);
		return -AW210XX_CHIPID_FAILD;
	}

	return 0;
}

static int32_t aw210xx_sl_set(struct aw210xx *aw210xx, uint8_t sl_data)
{
	uint8_t sl_max = 0;
	uint8_t i = 0;

	switch (aw210xx->chipid) {
	case AW21018_CHIPID:
		sl_max = AW210XX_REG_SL17;
		break;
	case AW21012_CHIPID:
		sl_max = AW210XX_REG_SL11;
		break;
	case AW21009_CHIPID:
		sl_max = AW210XX_REG_SL08;
		break;
	default:
		AWLOGD("%s: chip is unsupported device!\n",
			__func__);
		return -AW210XX_CHIPID_FAILD;
	}
	for (i = AW210XX_REG_SL00; i <= sl_max; i++)
		aw210xx_i2c_write(aw210xx, i, sl_data);

	return 0;
}

/*****************************************************
* aw210xx debug interface set
*****************************************************/
static void aw210xx_update(struct aw210xx *aw210xx)
{
	aw210xx_i2c_write(aw210xx, AW210XX_REG_UPDATE, AW210XX_UPDATE_BR_SL);
}

void aw210xx_global_set(uint8_t data)
{
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_GCCR, data);
}

void aw210xx_allbrightness_set(uint16_t brightness_data)
{
	/* chip enable */
	aw210xx_chipen_set(true);
	/* group set disable */
	aw210xx_group_gcfg_set(g_aw210xx, false);
	/* global set */
	aw210xx_global_set(0xff);
	/* br set */
	aw210xx_br_set(g_aw210xx, brightness_data);
	/* sl set */
	aw210xx_sl_set(g_aw210xx, 0xff);
	/* update */
	aw210xx_update(g_aw210xx);
}

void aw210xx_rgbcolor_set(uint32_t rgb_reg, uint32_t rgb_addr)
{
	uint8_t br_flag = 0;

	/* chip enable */
	aw210xx_chipen_set(true);
	/* global set */
	aw210xx_global_set(AW210XX_GLOBAL_DEFAULT_SET);
	/* group set disable */
	aw210xx_group_gcfg_set(g_aw210xx, false);
	/* set sl */
	if ((rgb_addr & 0x00ff0000) != 0) {
		g_aw210xx->rgbcolor = (rgb_addr & 0x00ff0000) >> 16;
		aw210xx_i2c_write(g_aw210xx, AW210XX_REG_SL00 + rgb_reg * 3,
				  g_aw210xx->rgbcolor);
		br_flag = 0;
	} else if ((rgb_addr & 0x0000ff00) != 0) {
		g_aw210xx->rgbcolor = (rgb_addr & 0x0000ff00) >> 8;
		aw210xx_i2c_write(g_aw210xx, AW210XX_REG_SL00 + rgb_reg * 3 + 1,
				  g_aw210xx->rgbcolor);
		br_flag = 1;
	} else if ((rgb_addr & 0x000000ff) != 0) {
		g_aw210xx->rgbcolor = (rgb_addr & 0x000000ff);
		aw210xx_i2c_write(g_aw210xx, AW210XX_REG_SL00 + rgb_reg * 3 + 2,
				  g_aw210xx->rgbcolor);
		br_flag = 2;
	}
	/* br set */
	if (g_aw210xx->sdmd_flag == 1) {
		if (g_aw210xx->rgbmd_flag == 1) {
			aw210xx_i2c_write(g_aw210xx,
					  AW210XX_REG_BR00L + rgb_reg,
					  AW210XX_GLOBAL_DEFAULT_SET);
		} else {
			aw210xx_i2c_write(g_aw210xx,
					  AW210XX_REG_BR00L + rgb_reg * 3 + br_flag,
					  AW210XX_GLOBAL_DEFAULT_SET);
		}
	} else {
		if (g_aw210xx->rgbmd_flag == 1) {
			aw210xx_i2c_write(g_aw210xx,
					  AW210XX_REG_BR00L + rgb_reg * 2,
					  AW210XX_GLOBAL_DEFAULT_SET);
			aw210xx_i2c_write(g_aw210xx,
					  AW210XX_REG_BR00H + rgb_reg * 2,
					  AW210XX_GLOBAL_DEFAULT_SET);
		} else {
			aw210xx_i2c_write(g_aw210xx,
					  AW210XX_REG_BR00L + rgb_reg * 6 + br_flag * 2,
					  AW210XX_GLOBAL_DEFAULT_SET);
			aw210xx_i2c_write(g_aw210xx,
					  AW210XX_REG_BR00H + rgb_reg * 6 + br_flag * 2,
					  AW210XX_GLOBAL_DEFAULT_SET);
		}
	}
	/* update */
	aw210xx_update(g_aw210xx);
}

int32_t aw210xx_group_brightness_set(uint16_t rgb_data)
{
	uint8_t gbrl = 0;
	uint8_t gbrh = 0;

	switch (g_aw210xx->br_pwm) {
	case BR_RESOLUTION_8BIT:
		gbrl = rgb_data;
		gbrh = 0x00;
		break;
	case BR_RESOLUTION_9BIT:
		gbrl = rgb_data;
		gbrh = (rgb_data >> 8) & 0x01;
		break;
	case BR_RESOLUTION_12BIT:
		gbrl = rgb_data;
		gbrh = (rgb_data >> 8) & 0x0f;
		break;
	default:
		AWLOGD("%s: chip is unsupported device!\n",
			__func__);
		return -AW210XX_CHIPID_FAILD;
	}
	/* chip enable */
	aw210xx_chipen_set(true);
	/* global set */
	aw210xx_global_set(AW210XX_GLOBAL_DEFAULT_SET);
	/* group set enable */
	aw210xx_group_gcfg_set(g_aw210xx, true);
	/* set sl */
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_GSLR,
			  AW210XX_GLOBAL_DEFAULT_SET);
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_GSLG,
			  AW210XX_GLOBAL_DEFAULT_SET);
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_GSLB,
			  AW210XX_GLOBAL_DEFAULT_SET);
	/* set br */
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_GBRH, gbrh);
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_GBRL, gbrl);
	/* update */
	aw210xx_update(g_aw210xx);

	return 0;
}

void aw210xx_group_breath_config(void)
{
	/* chip enable */
	aw210xx_chipen_set(true);
	/* global set */
	aw210xx_global_set(AW210XX_GLOBAL_DEFAULT_SET);
	/* group set enable */
	aw210xx_group_gcfg_set(g_aw210xx, true);
	/* set sl */
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_GSLR,
			  AW210XX_GLOBAL_DEFAULT_SET);
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_GSLG,
			  AW210XX_GLOBAL_DEFAULT_SET);
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_GSLB,
			  AW210XX_GLOBAL_DEFAULT_SET);
	/* set br */
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_GBRH,
			  AW210XX_GBRH_DEFAULT_SET);
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_GBRL,
			  AW210XX_GBRL_DEFAULT_SET);
	/* breath config */
	/* rise-on-fall-down set */
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_ABMT0,
			  AW210XX_ABMT0_SET);
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_ABMT1,
			  AW210XX_ABMT1_SET);
	/* cycle num */
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_ABMT2,
			  AW210XX_ABMT2_SET);
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_ABMT3,
			  AW210XX_ABMT3_SET);
	/* abm cfg */
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_ABMCFG,
			  AW210XX_ABMCFG_SET);
	/* abm go */
	aw210xx_i2c_write(g_aw210xx, AW210XX_REG_ABMGO,
			  AW210XX_ABMGO_SET);
}

/*****************************************************
* aw210xx check chipid
*****************************************************/
static int32_t aw210xx_read_chipid(struct aw210xx *aw210xx)
{
	uint8_t cnt = 0;
	uint8_t chipid = 0;
	HAL_StatusTypeDef status;

	while (cnt < AW210XX_READ_CHIPID_RETRIES) {
		status = aw210xx_i2c_read(aw210xx, AW210XX_REG_RESET, &chipid);
		if (status == HAL_OK) {
			aw210xx->chipid = chipid;
			switch (aw210xx->chipid) {
			case AW21018_CHIPID:
				AWLOGD("%s:AW21018, read chipid = 0x%02x!!\n",
				       __func__, chipid);
				return 0;
			case AW21012_CHIPID:
				AWLOGD("%s:AW21012, read chipid = 0x%02x!!\n",
				       __func__, chipid);
					return 0;
			case AW21009_CHIPID:
				AWLOGD("%s:AW21009, read chipid = 0x%02x!!\n",
				       __func__, chipid);
					return 0;
			default:
				AWLOGD("%s: chip is unsupported device!read chipid = 0x%02x!!\n",
				       __func__, aw210xx->chipid);
				break;
			}
		}
		cnt++;
		HAL_Delay(10);
	}

	return -AW210XX_CHIPID_FAILD;
}

static void aw210xx_pin_init(struct aw210xx *aw210xx)
{
	aw210xx->hi2cx = &hi2c1;
	aw210xx->gpio_port = GPIOA;
	aw210xx->en_gpio_pin = GPIO_PIN_3;
}

static void aw210xx_led_init(struct aw210xx *aw210xx)
{
	aw210xx->sdmd_flag = 0;
	aw210xx->rgbmd_flag = 0;
	/* chip enable */
	aw210xx_chipen_set(true);
	/* sbmd enable */
	aw210xx_sbmd_set(true);
	/* rgbmd enable */
	aw210xx_rgbmd_set(true);
	/* clk_pwm selsect */
	aw210xx_clk_pwm_set(CLK_FRQ_16M);
	/* br_pwm select */
	aw210xx_br_pwm_set(BR_RESOLUTION_8BIT);
	/* global set */
	aw210xx_global_set(AW210XX_GLOBAL_DEFAULT_SET);
	/* under voltage lock out */
	aw210xx_uvlo_set(true);
	/* apse enable */
	aw210xx_apse_set(true);
}

/*****************************************************
* aw210xx init
*****************************************************/
int32_t aw210xx_init(void)
{
	int32_t ret;
	struct aw210xx *aw210xx;

	AWLOGD("%s enter, driver version: %s\n", __func__,
	       AW210XX_DRIVER_VERSION);
	aw210xx = (struct aw210xx *)malloc(sizeof(struct aw210xx));
	if (aw210xx == NULL) {
		AWLOGD("%s malloc aw210xx fail\n", __func__);
		return -AW210XX_MALLOC_FAILEDT;
	}

	g_aw210xx = aw210xx;
	aw210xx_pin_init(aw210xx);
	aw210xx_en_pull_high();

	ret = aw210xx_read_chipid(aw210xx);
	if (ret) {
		AWLOGD("%s:chip not detected!!\n", __func__);
		return ret;
	}

	AWLOGD("%s:hardware reset!!\n", __func__);
	aw210xx_hw_reset();

	AWLOGD("%s:led init!!\n", __func__);
	aw210xx_led_init(aw210xx);

	/*light effect*/
	aw210xx_effect_select(AW21018_GROUP_BREATH_LEDS_ON);

	/*open short detect*/
	aw210xx_open_detect_cfg(aw210xx);
	/*aw210xx_short_detect_cfg(aw210xx);*/
	aw210xx_open_short_read(aw210xx);
	aw210xx_open_short_dis(aw210xx);

	return 0;
}
extern void awinic_single_enter(void)
{
	aw210xx_init();
}