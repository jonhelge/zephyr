/* Bosch BMI160 inertial measurement unit driver
 *
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * http://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI160-DS000-07.pdf
 */

#include <init.h>
#include <sensor.h>
#include <spi.h>
#include <misc/byteorder.h>
#include <kernel.h>
#include <misc/__assert.h>

#include "adns3000.h"


/* Register values according to Application Note 5544 from Avago */
static const uint8_t adns3000_reset_config[][2] = 
{
	/*{register, value}*/
    {0x0D,0x81},
	{0x47,0x52},
	{0x48,0x68},
	{0x49,0x20},
	{0x6D,0x41},
	{0x6E,0xA0},
	{0x70,0x85},
	{0x71,0x64},
	{0x72,0x46},
	{0x73,0x37},
	{0x74,0x41},
	{0x75,0x28},
	{0x76,0x16},
	{0x77,0x0F},
	{0x64,0xF0},
	{0x03,0x03},
	{0x48,0x60},
};

struct adns3000_device_data adns3000_data;

static int adns3000_transceive(struct device *dev, u8_t *tx_buf,
			     u8_t tx_buf_len, u8_t *rx_buf,
			     u8_t rx_buf_len)
{
	const struct adns3000_device_config *dev_cfg = dev->config->config_info;
	struct adns3000_device_data *adns3000 = dev->driver_data;
	struct spi_config spi_cfg;

	spi_cfg.config = SPI_WORD(8);
	spi_cfg.max_sys_freq = dev_cfg->spi_freq;

	if (spi_configure(adns3000->spi, &spi_cfg) < 0) {
		printk("Cannot configure SPI bus.");
		return -EIO;
	}

	if (spi_slave_select(adns3000->spi, dev_cfg->spi_slave) < 0) {
		printk("Cannot select slave.");
		return -EIO;
	}

	return spi_transceive(adns3000->spi, tx_buf, tx_buf_len,
			      rx_buf, rx_buf_len);
}

int adns3000_read(struct device *dev, u8_t reg_addr,
		u8_t *data, u8_t len)
{
	u8_t tx[3] = {0};

	tx[0] = reg_addr & 0x7F ;

	return adns3000_transceive(dev, tx, len, data, len);
}

int adns3000_byte_read(struct device *dev, u8_t reg_addr,
		     u8_t *byte)
{
	u8_t rx_buf[2];

	if (adns3000_read(dev, reg_addr, rx_buf, 2) < 0) {
		return -EIO;
	}

	*byte = rx_buf[1];

	return 0;
}



int adns3000_byte_write(struct device *dev, u8_t reg_addr, u8_t byte)
{
	u8_t tx_buf[2] = {reg_addr | (1 << 7) , byte};

	return adns3000_transceive(dev, tx_buf, 2, NULL, 0);
}


int adns3000_reg_field_update(struct device *dev, u8_t reg_addr,
			    u8_t pos, u8_t mask, u8_t val)
{
	u8_t old_val;

	if (adns3000_byte_read(dev, reg_addr, &old_val) < 0) {
		return -EIO;
	}

	return  adns3000_byte_write(dev, reg_addr,
				  (old_val & ~mask) | ((val << pos) & mask));
}



    




static int adns3000_sample_fetch(struct device *dev, enum sensor_channel chan)
{

	struct adns3000_device_data *adns3000 = dev->driver_data;
	 u8_t delta_x; /*!< Stores REG_DELTA_X contents */
    u8_t delta_y; /*!< Stores REG_DELTA_Y contents */
    u8_t delta_xy_high; /*!< Stores REG_DELTA_XY contents which contains upper 4 bits for both delta_x and delta_y when 12 bit mode is used */

	adns3000_byte_read(dev, ADNS3000_REG_DELTA_X, &delta_x);
	adns3000_byte_read(dev, ADNS3000_REG_DELTA_Y, &delta_y);
	adns3000_byte_read(dev, ADNS3000_REG_DELTA_XY_HIGH, &delta_xy_high);

	 adns3000->sample.x = ((int16_t) (((delta_xy_high & 0xF0) << 8)  | (delta_x << 4))) / 16;
     adns3000->sample.y = ((int16_t) (((delta_xy_high & 0x0F) << 12) | (delta_y << 4))) / 16;	

	return 0;
}



static int adns3000_channel_get(struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	
		struct adns3000_device_data *adns3000 = dev->driver_data;
	switch (chan) {
		case SENSOR_CHAN_ACCEL_X:
			val->val1=adns3000->sample.x;
			return 0;
		case SENSOR_CHAN_ACCEL_Y:
			val->val1=adns3000->sample.y;
			return 0;

		default:
			SYS_LOG_DBG("Channel not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api adns3000_api = {
#ifdef CONFIG_ADNS3000_TRIGGER
	.trigger_set = adns3000_trigger_set,
#endif
	.sample_fetch = adns3000_sample_fetch,
	.channel_get = adns3000_channel_get
};

int adns3000_init(struct device *dev)
{
	const struct adns3000_device_config *cfg = dev->config->config_info;
	struct adns3000_device_data *adns3000 = dev->driver_data;
	u8_t val = 0;
	u8_t i = 0;

	printk("running init\n");
	adns3000->spi = device_get_binding((char *)cfg->spi_port);
	if (!adns3000->spi) {
		printk("SPI master controller not found: %d.\n",
			    adns3000->spi);
		return -EINVAL;
	}

	adns3000_byte_write(dev, ADNS3000_REG_RESET, ADNS3000_RESET_NUMBER);
	k_busy_wait(100);

	if (adns3000_byte_read(dev, ADNS3000_REG_PROD_ID, &val) < 0) {
		printk("Failed to read chip id.\n");
		return -EIO;
	}

	if (val != ADNS3000_PRODUCT_ID) {
		printk("Unsupported chip detected (0x%x)!\n", val);
		return -ENODEV;
	}

	for (i = 0; i < (sizeof(adns3000_reset_config) / 2); ++i)
    {
        adns3000_byte_write(dev,adns3000_reset_config[i][0], adns3000_reset_config[i][1]);
    }

	
	

	dev->driver_api = &adns3000_api;

	return 0;
}

const struct adns3000_device_config adns3000_config = {
	.spi_port = CONFIG_ADNS3000_SPI_PORT_NAME,
	.spi_freq = CONFIG_ADNS3000_SPI_BUS_FREQ,
	.spi_slave = CONFIG_ADNS3000_SLAVE,
#if defined(CONFIG_ADNS3000_TRIGGER)
	.gpio_port = CONFIG_ADNS3000_GPIO_DEV_NAME,
	.int_pin = CONFIG_ADNS3000_GPIO_PIN_NUM,
#endif
};

DEVICE_INIT(adns3000, CONFIG_ADNS3000_NAME, adns3000_init, &adns3000_data,
	    &adns3000_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY);

