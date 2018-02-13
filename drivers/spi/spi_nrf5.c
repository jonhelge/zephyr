/*
 * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spi.h>
#include <nrfx_spi.h>

#define SYS_LOG_DOMAIN "SPI NRF5"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SPI_LEVEL
#include <logging/sys_log.h>

#include "spi_context.h"

#ifdef __cplusplus
extern "C" {
#endif


struct spi_nrf5_data {
	struct spi_context ctx;
	u16_t	chunk_len;
	bool	busy;
};

struct spi_nrf5_config {
	nrfx_spi_t spi;
};

static inline struct spi_nrf5_data *get_dev_data(struct device *dev)
{
	return dev->driver_data;
}

static inline const struct spi_nrf5_config *get_dev_config(struct device *dev)
{
	return dev->config->config_info;
}

static inline nrfx_spi_frequency_t get_nrfx_spi_frequency(u32_t frequency)
{
	/* Use the highest supported frequency but not exceeding the requested
	 * one. */
	if      (frequency <  250000) { return NRFX_SPI_FREQ_125K; }
	else if (frequency <  500000) { return NRFX_SPI_FREQ_250K; }
	else if (frequency < 1000000) { return NRFX_SPI_FREQ_500K; }
	else if (frequency < 2000000) { return NRFX_SPI_FREQ_1M; }
	else if (frequency < 4000000) { return NRFX_SPI_FREQ_2M; }
	else if (frequency < 8000000) { return NRFX_SPI_FREQ_4M; }
	else			      { return NRFX_SPI_FREQ_8M; }
}

static inline nrfx_spi_mode_t get_nrfx_spi_mode(u16_t operation)
{
	if (SPI_MODE_GET(operation) & SPI_MODE_CPOL) {
		if (SPI_MODE_GET(operation) & SPI_MODE_CPHA) {
			return NRFX_SPI_MODE_3;
		} else {
			return NRFX_SPI_MODE_2;
		}
	} else {
		if (SPI_MODE_GET(operation) & SPI_MODE_CPHA) {
			return NRFX_SPI_MODE_1;
		} else {
			return NRFX_SPI_MODE_0;
		}
	}
}

static inline nrfx_spi_bit_order_t get_nrfx_spi_bit_order(u16_t operation)
{
	if (operation & SPI_TRANSFER_LSB) {
		return NRFX_SPI_BIT_ORDER_LSB_FIRST;
	} else {
		return NRFX_SPI_BIT_ORDER_MSB_FIRST;
	}
}

static int configure(const struct spi_config *spi_cfg)
{
	struct spi_nrf5_data *dev_data = get_dev_data(spi_cfg->dev);

	if (spi_cfg->operation & SPI_OP_MODE_SLAVE) {
		SYS_LOG_ERR("Slave mode is not supported");
		return -EINVAL;
	}

	if ((spi_cfg->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		SYS_LOG_ERR("Only single line mode is supported");
		return -EINVAL;
	}

	if (SPI_WORD_SIZE_GET(spi_cfg->operation) != 8) {
		SYS_LOG_ERR("Word sizes other than 8 bits are not supported");
		return -EINVAL;
	}

	if (spi_cfg->frequency < 125000) {
		SYS_LOG_ERR("Frequencies lower than 125 kHz are not supported");
		return -EINVAL;
	}

	const nrfx_spi_t *spi = &get_dev_config(spi_cfg->dev)->spi;
	nrf_spi_frequency_set(spi->p_reg,
			      get_nrfx_spi_frequency(spi_cfg->frequency));
	nrf_spi_configure(spi->p_reg,
			  get_nrfx_spi_mode(spi_cfg->operation),
			  get_nrfx_spi_bit_order(spi_cfg->operation));

	spi_context_cs_configure(&dev_data->ctx);

	return 0;
}

static void transfer_next_chunk(struct device *dev)
{
	struct spi_nrf5_data *dev_data = get_dev_data(dev);
	int error = 0;

	size_t chunk_len = spi_context_longest_current_buf(&dev_data->ctx);
	if (chunk_len > 0) {
		dev_data->chunk_len = chunk_len;

		const nrfx_spi_xfer_desc_t xfer_desc = {
			.p_tx_buffer = dev_data->ctx.tx_buf,
			.tx_length   = (dev_data->ctx.tx_buf ? chunk_len : 0),
			.p_rx_buffer = dev_data->ctx.rx_buf,
			.rx_length   = (dev_data->ctx.rx_buf ? chunk_len : 0),
		};
		nrfx_err_t result = nrfx_spi_xfer(&get_dev_config(dev)->spi,
						  &xfer_desc, 0);
		if (result == NRFX_SUCCESS) {
			return;
		}

		error = -EIO;
	}

	spi_context_cs_control(&dev_data->ctx, false);

	SYS_LOG_DBG("SPI transaction completed %s error",
		    error ? "with" : "without");

	spi_context_complete(&dev_data->ctx, error);
	dev_data->busy = false;
}

static int transceive(const struct spi_config *spi_cfg,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      struct k_poll_signal *signal)
{
	struct spi_nrf5_data *dev_data = get_dev_data(spi_cfg->dev);
	int error;

	spi_context_lock(&dev_data->ctx, asynchronous, signal);

	dev_data->ctx.config = spi_cfg;

	error = configure(spi_cfg);
	if (error == 0) {
		dev_data->busy = true;

		spi_context_buffers_setup(&dev_data->ctx, tx_bufs, rx_bufs, 1);
		spi_context_cs_control(&dev_data->ctx, true);

		transfer_next_chunk(spi_cfg->dev);

		error = spi_context_wait_for_completion(&dev_data->ctx);
	}

	spi_context_release(&dev_data->ctx, error);

	return error;
}

static int spi_nrf5_transceive(const struct spi_config *spi_cfg,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
	return transceive(spi_cfg, tx_bufs, rx_bufs, false, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_nrf5_transceive_async(const struct spi_config *spi_cfg,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
				     struct k_poll_signal *async)
{
	return transceive(spi_cfg, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_nrf5_release(const struct spi_config *spi_cfg)
{
	struct spi_nrf5_data *dev_data = get_dev_data(spi_cfg->dev);

	if (!spi_context_configured(&dev_data->ctx, spi_cfg)) {
		return -EINVAL;
	}

	if (dev_data->busy) {
		return -EBUSY;
	}

	spi_context_unlock_unconditionally(&dev_data->ctx);

	return 0;
}

static const struct spi_driver_api spi_nrf5_api = {
	.transceive = spi_nrf5_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_nrf5_transceive_async,
#endif
	.release = spi_nrf5_release,
};


static void event_handler(const nrfx_spi_evt_t *p_event, void *p_context)
{
	if (p_event->type == NRFX_SPI_EVENT_DONE) {
		struct device *dev = p_context;
		struct spi_nrf5_data *dev_data = get_dev_data(dev);

		spi_context_update_tx(&dev_data->ctx, 1, dev_data->chunk_len);
		spi_context_update_rx(&dev_data->ctx, 1, dev_data->chunk_len);

		transfer_next_chunk(dev);
	}
}

static int spi_init(struct device *dev,
		    u8_t sck_pin,
		    u8_t mosi_pin,
		    u8_t miso_pin,
		    u8_t orc)
{
	const nrfx_spi_config_t config = {
		.sck_pin	= sck_pin,
		.mosi_pin	= mosi_pin,
		.miso_pin	= miso_pin,
		.ss_pin		= NRFX_SPI_PIN_NOT_USED,
		.orc		= orc,
		/* These are default settings, configure() will set proper ones
		 * for each transfer. */
		.frequency	= NRFX_SPI_FREQ_4M,
		.mode		= NRFX_SPI_MODE_0,
		.bit_order	= NRFX_SPI_BIT_ORDER_MSB_FIRST,
	};
	nrfx_err_t result = nrfx_spi_init(&get_dev_config(dev)->spi,
					  &config,
					  event_handler,
					  dev);
	if (result != NRFX_SUCCESS) {
		assert(false);
	}

	spi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);

	return 0;
}

static void spi_isr(void *irq_handler)
{
	((nrfx_irq_handler_t)irq_handler)();
}

/* In Nordic SoCs the IRQ number assigned to a peripheral is equal to the ID
 * of the block of 0x1000 bytes in the peripheral address space assigned to
 * this peripheral. See the chapter "Peripheral interface" (sections "Peripheral
 * ID" and "Interrupts") in the product specification of a given SoC. */
#define SPI_NRF5_IRQ_NUMBER(idx)  (uint8_t)((uint32_t)NRF_SPI##idx >> 12u)

#define SPI_NRF5_DEVICE(idx)						\
	static int spi_##idx##_init(struct device *dev)			\
	{								\
		spi_init(dev,						\
			 CONFIG_SPI_##idx##_NRF5_SCK_PIN,		\
			 CONFIG_SPI_##idx##_NRF5_MOSI_PIN,		\
			 CONFIG_SPI_##idx##_NRF5_MISO_PIN,		\
			 CONFIG_SPI_##idx##_NRF5_ORC);			\
		IRQ_CONNECT(SPI_NRF5_IRQ_NUMBER(idx),			\
			    CONFIG_SPI_##idx##_IRQ_PRI,			\
			    spi_isr, nrfx_spi_##idx##_irq_handler, 0);	\
		return 0;						\
	}								\
									\
	static struct spi_nrf5_data spi_##idx##_data = {		\
		SPI_CONTEXT_INIT_LOCK(spi_##idx##_data, ctx),		\
		SPI_CONTEXT_INIT_SYNC(spi_##idx##_data, ctx),		\
		.busy = false,						\
	};								\
	static const struct spi_nrf5_config spi_##idx##_config = {	\
		.spi = NRFX_SPI_INSTANCE(idx),				\
	};								\
									\
	DEVICE_AND_API_INIT(spi_##idx, CONFIG_SPI_##idx##_NAME,		\
			    spi_##idx##_init,				\
			    &spi_##idx##_data,				\
			    &spi_##idx##_config, 			\
			    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,	\
			    &spi_nrf5_api)

#ifdef CONFIG_SPI_0_NRF5
SPI_NRF5_DEVICE(0);
#endif

#ifdef CONFIG_SPI_1_NRF5
SPI_NRF5_DEVICE(1);
#endif

#ifdef CONFIG_SPI_2_NRF5
SPI_NRF5_DEVICE(2);
#endif

#ifdef __cplusplus
}
#endif
