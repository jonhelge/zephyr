/* Pixart ADNS-3000 optical mouse sensor header
 *
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ADNS3000_H_
#define _ADNS3000_H_

#include <gpio.h>
#include <misc/util.h>

#define ADNS3000_PRODUCT_ID (0x2AU) /*!< ADNS3000 product id */
#define ADNS3000_RESET_NUMBER (0x5AU) /*!< ADNS3000 reset code */

/* ADNS3000 register addresses */
#define ADNS3000_REG_PROD_ID (0x00U) /*!< Product ID. Default value : 0x2A */
#define ADNS3000_REG_REV_ID (0x01U) /*!< Revision ID. Default value : 0x00 */
#define ADNS3000_REG_MOTION_ST (0x02U) /*!< Motion Status. Default value : 0x00 */
#define ADNS3000_REG_DELTA_X (0x03U) /*!< Lower byte of Delta_X. Default value : 0x00 */
#define ADNS3000_REG_DELTA_Y (0x04U) /*!< Lower byte of Delta_Y. Default value : 0x00 */
#define ADNS3000_REG_SQUAL (0x05U) /*!< Squal Quality. Default value : 0x00 */
#define ADNS3000_REG_SHUT_HI (0x06U) /*!< Shutter Open Time (Upper 8-bit). Default value : 0x00 */
#define ADNS3000_REG_SHUT_LO (0x07U) /*!< Shutter Open Time (Lower 8-bit). Default value : 0x64 */
#define ADNS3000_REG_PIX_MAX (0x08U) /*!< Maximum Pixel Value. Default value : 0xD0 */
#define ADNS3000_REG_PIX_ACCUM (0x09U) /*!< Average Pixel Value. Default value : 0x80 */
#define ADNS3000_REG_PIX_MIN (0x0AU) /*!< Minimum Pixel Value. Default value : 0x00 */
#define ADNS3000_REG_PIX_GRAB (0x0BU) /*!< Pixel Grabber. Default value : 0x00 */
#define ADNS3000_REG_DELTA_XY_HIGH (0x0CU) /*!< Upper 4 bits of Delta X and Y displacement. Default value : 0x00 */
#define ADNS3000_REG_MOUSE_CTRL (0x0DU) /*!< Mouse Control. Default value : 0x01 */
#define ADNS3000_REG_RUN_DOWNSHIFT (0x0EU) /*!< Run to Rest1 Time. Default value : 0x08 */
#define ADNS3000_REG_REST1_PERIOD (0x0FU) /*!< Rest1 Period. Default value : 0x01 */
#define ADNS3000_REG_REST1_DOWNSHIFT (0x10U) /*!< Rest1 to Rest2 Time. Default value : 0x1f */
#define ADNS3000_REG_REST2_PERIOD (0x11U) /*!< Rest2 Period. Default value : 0x09 */
#define ADNS3000_REG_REST2_DOWNSHIFT (0x12U) /*!< Rest2 to Rest3 Time. Default value : 0x2f */
#define ADNS3000_REG_REST3_PERIOD (0x13U) /*!< Rest3 Period. Default value : 0x31 */
#define ADNS3000_REG_PERFORMANCE (0x22U) /*!< Performance. Default value : 0x00 */
#define ADNS3000_REG_RESET (0x3aU) /*!< Reset. Default value : 0x00 */
#define ADNS3000_REG_NOT_REV_ID (0x3fU) /*!< Inverted Revision ID. Default value : 0xff */
#define ADNS3000_REG_LED_CTRL (0x40U) /*!< LED Control. Default value : 0x00 */
#define ADNS3000_REG_MOTION_CTRL (0x41U) /*!< Motion Control. Default value : 0x40 */
#define ADNS3000_REG_BURST_READ_FIRST (0x42U) /*!< Burst Read Starting Register. Default value : 0x03 */
#define ADNS3000_REG_BURST_READ_LAST (0x44U) /*!< Burst Read Ending Register. Default value : 0x09 */
#define ADNS3000_REG_REST_MODE_CONFIG (0x45U) /*!< Rest Mode Confi guration. Default value : 0x00 */
#define ADNS3000_REG_MOTION_BURST (0x63U) /*!< Burst Read. Default value : 0x00 */

/* ADNS3000 register bits */
#define ADNS3000_REG_MOUSE_CTRL_POWERDOWN (0x02U) /*!< Mouse control register powerdown bit */
#define ADNS3000_REG_MOTION_CTRL_MOT_A (0x80U) /*!< Motion control register polarity bit */
#define ADNS3000_REG_MOTION_CTRL_MOT_S (0x40U) /*!< Motion control register edge sensitivity bit */
#define ADNS3000_REG_MOUSE_CTRL_RES_EN (0x40U) /*!< Mouse control register resolution enable bit */
#define ADNS3000_REG_MOUSE_CTRL_BIT_REPORTING (0x80U) /*!< Mouse control register "number of motion bits" bit*/

#define ADNS3000_SS_PIN   23
#define ADNS3000_SCK_PIN  29
#define ADNS3000_MOSI_PIN 12
#define ADNS3000_MISO_PIN 22

/**
 * adns3000 motion output pin polarity values.
 */
typedef enum
{
  ADNS3000_MOTION_OUTPUT_POLARITY_LOW = 0,  /*!< Motion output polarity active low */
  ADNS3000_MOTION_OUTPUT_POLARITY_HIGH = 1  /*!< Motion output polarity active high */
} motion_output_polarity_t;

/**
 * Motion output pin configuration.
 */
typedef enum
{
  ADNS3000_MOTION_OUTPUT_SENSITIVITY_LEVEL = 0,  /*!< Motion output pin will be driven low/high (depending on the polarity setting) as long as there is motion data in DELTA registers */
  ADNS3000_MOTION_OUTPUT_SENSITIVITY_EDGE = 1  /*!< Motion output pin will be driven low/high (depending on the polarity setting) for 380 ns when motion is detected during rest modes */
} motion_output_sensitivity_t;

/**
 * Mouse sensor resolution values.
 */
typedef enum
{
  ADNS3000_RESOLUTION_250DPI = 1, /*!< 250 dpi resolution */
  ADNS3000_RESOLUTION_500DPI = 2, /*!< 500 dpi resolution */
  ADNS3000_RESOLUTION_1000DPI = 0, /*!< 1000 dpi resolution */
  ADNS3000_RESOLUTION_1250DPI = 3, /*!< 1250 dpi resolution */
  ADNS3000_RESOLUTION_1500DPI = 4, /*!< 1500 dpi resolution */
  ADNS3000_RESOLUTION_1750DPI = 5, /*!< 1750 dpi resolution */
  ADNS3000_RESOLUTION_2000DPI = 6 /*!< 2000 dpi resolution */
} adns3000_resolution_t;

/**
 * Mouse sensor forced mode options.
 */
typedef enum
{
  ADNS3000_MODE_NORMAL = 0, /*!< Normal operation mode */
  ADNS3000_MODE_REST1 = 1, /*!< Rest1 operation mode */
  ADNS3000_MODE_REST2 = 2, /*!< Rest2 operation mode */
  ADNS3000_MODE_REST3 = 3, /*!< Rest3 operation mode */
  ADNS3000_MODE_RUN1 = 4, /*!< Run1 operation mode */
  ADNS3000_MODE_RUN2 = 5, /*!< Run2 operation mode */
  ADNS3000_MODE_IDLE = 6 /*!< Idle operation mode */
} adns3000_mode_t;

/**
 * Mouse sensor motion reporting bits.
 */
typedef enum
{
  ADNS3000_MOTION_BITS_8 = 0,  /*!< Motion reporting uses 8 bits */
  ADNS3000_MOTION_BITS_12 = 1  /*!< Motion reporting uses 12 bits */
} adns3000_motion_bits_t;



struct adns3000_device_config {
	const char *spi_port;
#if defined(CONFIG_ADNS3000_TRIGGER)
	const char *gpio_port;
	u8_t int_pin;
#endif
	u32_t spi_freq;
	u8_t spi_slave;
};

#define ADNS3000_SAMPLE_SIZE 4
/* total buffer contains one dummy byte, needed by SPI */
#define ADNS3000_BUF_SIZE			(ADNS3000_SAMPLE_SIZE)
union adns3000_sample {
	u8_t raw[ADNS3000_BUF_SIZE];
	struct {
		u16_t x;
		u16_t y;
	} __packed;
};

struct adns3000_device_data {
	struct device *spi;
#if defined(CONFIG_ADNS3000_TRIGGER)
	struct device *gpio;
	struct gpio_callback gpio_cb;
#endif
	union adns3000_sample sample;

#ifdef CONFIG_ADNS3000_TRIGGER_OWN_THREAD
	struct k_sem sem;
#endif

#ifdef CONFIG_ADNS3000_TRIGGER_GLOBAL_THREAD
	struct k_work work;
	struct device *dev;
#endif

#ifdef CONFIG_ADNS3000_TRIGGER
	sensor_trigger_handler_t handler_drdy_mouse;
#endif

};

int adns3000_read(struct device *dev, u8_t reg_addr,
		u8_t *data, u8_t len);
int adns3000_byte_read(struct device *dev, u8_t reg_addr, u8_t *byte);
int adns3000_byte_write(struct device *dev, u8_t reg_addr, u8_t byte);
int adns3000_word_write(struct device *dev, u8_t reg_addr, u16_t word);
int adns3000_reg_field_update(struct device *dev, u8_t reg_addr,
			    u8_t pos, u8_t mask, u8_t val);
static inline int adns3000_reg_update(struct device *dev, u8_t reg_addr,
				    u8_t mask, u8_t val)
{
	return adns3000_reg_field_update(dev, reg_addr, 0, mask, val);
}
;


#define SYS_LOG_DOMAIN "ADNS3000"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SENSOR_LEVEL
#include <logging/sys_log.h>
#endif /* _ADNS_H_ */
