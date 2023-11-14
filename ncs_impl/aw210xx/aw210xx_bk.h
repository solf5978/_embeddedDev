#ifndef __AW210XX_H
#define __AW210XX_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <nrfx_twis.h>

#define AWINIC_UART_DEBUG
#ifdef AWINIC_UART_DEBUG
#define AWLOGD(format, arg...) printf(format, ##arg)
#else
#define AWLOGD(format, arg...) do {} while (0)
#endif

/*****************************************************
* define macro
*****************************************************/
#define AW210XX_I2C_ADDR                    (0x20)
#define AW210XX_I2C_RW_RETRIES              (5)
#define AW210XX_READ_CHIPID_RETRIES         (5)
#define AW210XX_REGADD_SIZE_8BIT            (1)
#define AW210XX_REGDATA_SIZE_8BIT           (1)

/*****************************************************
* register about led mode
*****************************************************/
#define AW210XX_REG_GCR                     (0x20)
#define AW210XX_REG_BR00L                   (0x21)
#define AW210XX_REG_BR00H                   (0x22)
#define AW210XX_REG_BR01L                   (0x23)
#define AW210XX_REG_BR01H                   (0x24)
#define AW210XX_REG_BR02L                   (0x25)
#define AW210XX_REG_BR02H                   (0x26)
#define AW210XX_REG_BR03L                   (0x27)
#define AW210XX_REG_BR03H                   (0x28)
#define AW210XX_REG_BR04L                   (0x29)
#define AW210XX_REG_BR04H                   (0x2A)
#define AW210XX_REG_BR05L                   (0x2B)
#define AW210XX_REG_BR05H                   (0x2C)
#define AW210XX_REG_BR06L                   (0x2D)
#define AW210XX_REG_BR06H                   (0x2E)
#define AW210XX_REG_BR07L                   (0x2F)
#define AW210XX_REG_BR07H                   (0x30)
#define AW210XX_REG_BR08L                   (0x31)
#define AW210XX_REG_BR08H                   (0x32)
#define AW210XX_REG_BR09L                   (0x33)
#define AW210XX_REG_BR09H                   (0x34)
#define AW210XX_REG_BR10L                   (0x35)
#define AW210XX_REG_BR10H                   (0x36)
#define AW210XX_REG_BR11L                   (0x37)
#define AW210XX_REG_BR11H                   (0x38)
#define AW210XX_REG_BR12L                   (0x39)
#define AW210XX_REG_BR12H                   (0x3A)
#define AW210XX_REG_BR13L                   (0x3B)
#define AW210XX_REG_BR13H                   (0x3C)
#define AW210XX_REG_BR14L                   (0x3D)
#define AW210XX_REG_BR14H                   (0x3E)
#define AW210XX_REG_BR15L                   (0x3F)
#define AW210XX_REG_BR15H                   (0x40)
#define AW210XX_REG_BR16L                   (0x41)
#define AW210XX_REG_BR16H                   (0x42)
#define AW210XX_REG_BR17L                   (0x43)
#define AW210XX_REG_BR17H                   (0x44)
#define AW210XX_REG_UPDATE                  (0x45)
#define AW210XX_REG_SL00                    (0x46)
#define AW210XX_REG_SL01                    (0x47)
#define AW210XX_REG_SL02                    (0x48)
#define AW210XX_REG_SL03                    (0x49)
#define AW210XX_REG_SL04                    (0x4A)
#define AW210XX_REG_SL05                    (0x4B)
#define AW210XX_REG_SL06                    (0x4C)
#define AW210XX_REG_SL07                    (0x4D)
#define AW210XX_REG_SL08                    (0x4E)
#define AW210XX_REG_SL09                    (0x4F)
#define AW210XX_REG_SL10                    (0x50)
#define AW210XX_REG_SL11                    (0x51)
#define AW210XX_REG_SL12                    (0x52)
#define AW210XX_REG_SL13                    (0x53)
#define AW210XX_REG_SL14                    (0x54)
#define AW210XX_REG_SL15                    (0x55)
#define AW210XX_REG_SL16                    (0x56)
#define AW210XX_REG_SL17                    (0x57)
#define AW210XX_REG_GCCR                    (0x58)
#define AW210XX_REG_PHCR                    (0x59)
#define AW210XX_REG_OSDCR                   (0x5A)
#define AW210XX_REG_OSST0                   (0x5B)
#define AW210XX_REG_OSST1                   (0x5C)
#define AW210XX_REG_OSST2                   (0x5D)
#define AW210XX_REG_OTCR                    (0x5E)
#define AW210XX_REG_SSCR                    (0x5F)
#define AW210XX_REG_UVCR                    (0x60)
#define AW210XX_REG_GCR2                    (0x61)
#define AW210XX_REG_GCR3                    (0x62)
#define AW210XX_REG_RESET                   (0x70)
#define AW210XX_REG_ABMCFG                  (0x80)
#define AW210XX_REG_ABMGO                   (0x81)
#define AW210XX_REG_ABMT0                   (0x82)
#define AW210XX_REG_ABMT1                   (0x83)
#define AW210XX_REG_ABMT2                   (0x84)
#define AW210XX_REG_ABMT3                   (0x85)
#define AW210XX_REG_GBRH                    (0x86)
#define AW210XX_REG_GBRL                    (0x87)
#define AW210XX_REG_GSLR                    (0x88)
#define AW210XX_REG_GSLG                    (0x89)
#define AW210XX_REG_GSLB                    (0x8A)
#define AW210XX_REG_GCFG                    (0x8B)

/*****************************************************
 * define register Detail
*****************************************************/
#define AW210XX_BIT_APSE_MASK               (~(1 << 7))
#define AW210XX_BIT_APSE_ENABLE             (1 << 7)
#define AW210XX_BIT_APSE_DISENA             (0 << 7)
#define AW210XX_BIT_CHIPEN_MASK             (~(1 << 0))
#define AW210XX_BIT_CHIPEN_ENABLE           (1 << 0)
#define AW210XX_BIT_CHIPEN_DISENA           (0 << 0)
#define AW210XX_BIT_UVPD_MASK               (~(1 << 1))
#define AW210XX_BIT_UVPD_ENABLE             (0 << 1)
#define AW210XX_BIT_UVPD_DISENA             (1 << 1)
#define AW210XX_BIT_UVDIS_MASK              (~(1 << 0))
#define AW210XX_BIT_UVDIS_ENABLE            (0 << 0)
#define AW210XX_BIT_UVDIS_DISENA            (1 << 0)
#define AW210XX_BIT_ABME_MASK               (~(1 << 0))
#define AW210XX_BIT_ABME_ENABLE             (1 << 0)
#define AW210XX_BIT_ABME_DISENA             (0 << 0)
#define AW210XX_BIT_GSLDIS_MASK             (~(1 << 6))
#define AW210XX_BIT_GSLDIS_ENABLE           (0 << 6)
#define AW210XX_BIT_GSLDIS_DISENA           (1 << 6)
#define AW210XX_BIT_RGBMD_MASK              (~(1 << 0))
#define AW210XX_BIT_RGBMD_ENABLE            (1 << 0)
#define AW210XX_BIT_RGBMD_DISENA            (0 << 0)
#define AW210XX_BIT_SBMD_MASK               (~(1 << 1))
#define AW210XX_BIT_SBMD_ENABLE             (1 << 1)
#define AW210XX_BIT_SBMD_DISENA             (0 << 1)
#define AW210XX_BIT_CLKFRQ_MASK             (~(7 << 4))
#define AW210XX_BIT_CLKFRQ_16MH             (0 << 4)
#define AW210XX_BIT_CLKFRQ_8MH              (1 << 4)
#define AW210XX_BIT_CLKFRQ_4MH              (2 << 4)
#define AW210XX_BIT_CLKFRQ_2MH              (3 << 4)
#define AW210XX_BIT_CLKFRQ_1MH              (4 << 4)
#define AW210XX_BIT_CLKFRQ_512KH            (5 << 4)
#define AW210XX_BIT_CLKFRQ_256KH            (6 << 4)
#define AW210XX_BIT_CLKFRQ_125KH            (7 << 4)

#define AW210XX_BIT_PWMRES_MASK             (~(3 << 1))
#define AW210XX_BIT_PWMRES_8BIT             (0 << 1)
#define AW210XX_BIT_PWMRES_9BIT             (1 << 1)
#define AW210XX_BIT_PWMRES_12BIT            (2 << 1)
#define AW210XX_BIT_PWMRES_9_AND_3_BIT      (3 << 1)

#define AW210XX_DCPWM_SET					(7 << 5)
#define AW210XX_DCPWM_CLEAN					(0x00)
#define AW210XX_DCPWM_SET_MASK				~(7 << 5)
#define AW210XX_OPEN_THRESHOLD_SET			(1 << 3)
#define AW210XX_OPEN_THRESHOLD_SET_MASK		~(1 << 3)
#define AW210XX_SHORT_THRESHOLD_SET			(1 << 2)
#define AW210XX_SHORT_THRESHOLD_SET_MASK	~(1 << 2)


/*****************************************************
 * define register data
*****************************************************/



/*****************************************************
* define return value
*****************************************************/



/***********************************************
 * define register Detail
 ***********************************************/


static uint8_t aw210xx_reg_access[AW210XX_REG_MAX] = {
	[AW210XX_REG_GCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR00L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR00H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR01L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR01H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR02L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR02H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR03L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR03H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR04L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR04H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR05L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR05H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR06L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR06H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR07L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR07H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR08L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR08H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR09L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR09H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR10L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR10H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR11L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR11H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR12L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR12H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR13L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR13H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR14L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR14H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR15L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR15H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR16L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR16H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR17L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR17H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_UPDATE] = REG_WR_ACCESS,
	[AW210XX_REG_SL00] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL01] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL02] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL03] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL04] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL05] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL06] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL07] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL08] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL09] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL10] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL11] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL12] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL13] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL14] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL15] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL16] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL17] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GCCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_PHCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_OSDCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_OSST0] = REG_RD_ACCESS,
	[AW210XX_REG_OSST1] = REG_RD_ACCESS,
	[AW210XX_REG_OSST2] = REG_RD_ACCESS,
	[AW210XX_REG_OTCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SSCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_UVCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GCR2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GCR3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_RESET] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMCFG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMGO] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMT0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMT1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMT2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMT3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GBRH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GBRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GSLR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GSLG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GSLB] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GCFG] = REG_RD_ACCESS | REG_WR_ACCESS,
};

typedef enum {
	CLK_FRQ_16M = 1,
	CLK_FRQ_8M,
	CLK_FRQ_4M,
	CLK_FRQ_2M,
	CLK_FRQ_1M,
	CLK_FRQ_512K,
	CLK_FRQ_256K,
	CLK_FRQ_125K,
} clk_pwm_t;

typedef enum {
	BR_RESOLUTION_8BIT = 1,
	BR_RESOLUTION_9BIT,
	BR_RESOLUTION_12BIT,
	BR_RESOLUTION_9_AND_3_BIT,
} br_pwm_t;

typedef enum {
	GROUP_ALL_LED_OFF = 0,
	AW21018_GROUP_ALL_LEDS_ON,
	AW21018_GROUP_RED_LEDS_ON,
	AW21018_GROUP_GREEN_LEDS_ON,
	AW21018_GROUP_BLUE_LEDS_ON,
	AW21018_GROUP_BREATH_LEDS_ON,
	AW21012_GROUP_ALL_LEDS_ON,
	AW21012_GROUP_RED_LEDS_ON,
	AW21012_GROUP_GREEN_LEDS_ON,
	AW21012_GROUP_BLUE_LEDS_ON,
	AW21012_GROUP_BREATH_LEDS_ON,
	AW21009_GROUP_ALL_LEDS_ON,
	AW21009_GROUP_RED_LEDS_ON,
	AW21009_GROUP_GREEN_LEDS_ON,
	AW21009_GROUP_BLUE_LEDS_ON,
	AW21009_GROUP_BREATH_LEDS_ON,
} effect_select_t;

struct aw210xx {
	I2C_HandleTypeDef *hi2cx;
	GPIO_TypeDef *gpio_port;
	uint8_t chipid;
	uint8_t en_flag;
	uint8_t sdmd_flag;
	uint8_t rgbmd_flag;
	uint8_t brn_h;
	uint8_t brn_l;
	uint8_t br_max;
	uint16_t en_gpio_pin;
	uint32_t rgbcolor;
	effect_select_t effect;
	br_pwm_t br_pwm;
};

typedef struct aw210xx_cfg {
	unsigned char *p;
	unsigned int count;
} AW210XX_CFG;


/************************************************
 * AW210XX Program Interface
 ************************************************/

void twi_master_init(void);
bool aw210xx_init(void);

void aw210xx_open_detect_cfg(struct aw210xx *aw210xx);
void aw210xx_short_detect_cfg(struct aw210xx *aw210xx);
void aw210xx_open_short_dis(struct aw210xx *aw210xx);
void aw210xx_open_short_read(struct aw210xx *aw210xx);

int32_t aw_twi_master_init(void);
bool aw210xx_init(void);
int32_t aw210xx_hw_reset(void);
int32_t aw210xx_en_pull_high(void);
int32_t aw210xx_en_pull_low(void);
void aw210xx_sw_reset(void);

int32_t aw210xx_get_current_effect(void);
int32_t aw210xx_set_reg(uint8_t reg_addr, uint8_t reg_data);
int32_t aw210xx_get_reg(void);
int32_t aw210xx_clk_pwm_set(clk_pwm_t clk_pwm);
int32_t aw210xx_br_pwm_set(br_pwm_t br_pwm);
void aw210xx_apse_set(bool flag);
void aw210xx_chipen_set(bool flag);
void aw210xx_uvlo_set(bool flag);
void aw210xx_gsldis_set(bool flag);
void aw210xx_rgbmd_set(bool flag);
void aw210xx_sbmd_set(bool flag);
void aw210xx_global_set(uint8_t data);

/*you can call flowing interface*/
void aw210xx_effect_select(effect_select_t effect);
void aw210xx_allbrightness_set(uint16_t brightness_data);
void aw210xx_rgbcolor_set(uint32_t rgb_reg, uint32_t rgb_addr);
int32_t aw210xx_group_brightness_set(uint16_t rgb_data);
void aw210xx_group_breath_config(void);
int32_t aw210xx_init(void);

#endif
