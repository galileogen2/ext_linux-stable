/*
 * Intel Quark board support platform driver
 *
 * Copyright(c) 2015 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Author : Dan O'Donovan <dan@emutex.com> 2015
 */

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_data/at24.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/spi.h>
#include <linux/i2c/pcf857x.h>
#include <linux/platform_data/tpm_i2c_infenion.h>

#define DRIVER_NAME		"RelianceCreek"
#define GPIO_RESTRICT_NAME_NC	"qrk-gpio-restrict-nc"
#define GPIO_RESTRICT_NAME_SC	"qrk-gpio-restrict-sc"

/* GPIO signal names from RelianceCreek board data-sheet, prefixed with GPIO_ */
#define GPIO_IRQ_SPI2UART_B_BUF		15
#define GPIO_GPIO1_RS485_IRQ		9

/* GPIO line used to reset SLB9645TT */
#define GPIO_SLB9645TT_RESET          0
/* GPIO line SLB9645TT interrupt are routed to */
#define GPIO_SLB9645TT_INT            13

static int nc_gpio_reg;
static int sc_gpio_reg;

#include "linux/platform_data/pca953x.h"
#define PCF8574_GPIO_BASE_OFFSET 16

static struct pcf857x_platform_data pcf8574_platform_data_exp1 = {
	.gpio_base = PCF8574_GPIO_BASE_OFFSET,
};

static struct pcf857x_platform_data pcf8574_platform_data_exp2 = {
	.gpio_base = PCF8574_GPIO_BASE_OFFSET + 8,
};

static struct pcf857x_platform_data pcf8574_platform_data_exp3 = {
	.gpio_base = PCF8574_GPIO_BASE_OFFSET + 16,
};

struct tpm_i2c_infenion_platform_data slb9645tt_platform_data = {
	.gpio_reset = GPIO_SLB9645TT_RESET,
	.gpio_irq = GPIO_SLB9645TT_INT,
};

/******************************************************************************
 *                        Reliance Creek i2c clients
 ******************************************************************************/
#define TMP75_ADDR				0x48
#define EEPROM_ADDR				0x50
#define PCF8574_EXP1_ADDR			0x23
#define PCF8574_EXP2_ADDR			0x21
#define PCF8574_EXP3_ADDR			0x22
#define SLB9645TT_ADDR				0x20

static struct i2c_board_info probed_i2c_tmp75;
static struct i2c_board_info probed_i2c_eeprom;
static struct i2c_board_info probed_i2c_pcf8574_exp1 = {
	.platform_data = &pcf8574_platform_data_exp1,
};
static struct i2c_board_info probed_i2c_pcf8574_exp2 = {
	.platform_data = &pcf8574_platform_data_exp2,
};
static struct i2c_board_info probed_i2c_pcf8574_exp3 = {
	.platform_data = &pcf8574_platform_data_exp3,
};
static struct i2c_board_info probed_slb9645tt = {
	.platform_data = &slb9645tt_platform_data,
};

static const unsigned short tmp75_i2c_addr[] = {
	TMP75_ADDR, I2C_CLIENT_END
};
static const unsigned short eeprom_i2c_addr[] = {
	EEPROM_ADDR, I2C_CLIENT_END
};
static const unsigned short pcf8574_exp1_i2c_addr[] = {
	PCF8574_EXP1_ADDR, I2C_CLIENT_END
};
static const unsigned short pcf8574_exp2_i2c_addr[] = {
	PCF8574_EXP2_ADDR, I2C_CLIENT_END
};
static const unsigned short pcf8574_exp3_i2c_addr[] = {
	PCF8574_EXP3_ADDR, I2C_CLIENT_END
};
static const unsigned short slb9645tt_i2c_addr[] = {
	SLB9645TT_ADDR, I2C_CLIENT_END
};

static int i2c_probe(struct i2c_adapter *adap, unsigned short addr)
{
	/* Always return success: the I2C clients are already known.  */
	return 1;
}

/******************************************************************************
 *             NXP SC16IS7XX SPI Device Platform Data
 ******************************************************************************/

static const unsigned long sc16is752_platform_data = 18432000;
static const unsigned long sc16is741_platform_data = 18432000;

/******************************************************************************
 *                 Intel Quark SPI Controller Data
 ******************************************************************************/
static struct pxa2xx_spi_chip qrk_ffrd_spi_0_cs_0 = {
	.gpio_cs = 4,
};

static struct pxa2xx_spi_chip qrk_ffrd_spi_1_cs_0 = {
	.gpio_cs = 10,
};

static struct spi_board_info spi_sc16is752_info = {
	.modalias = "sc16is752",
	.max_speed_hz = 4000000,
	.mode = SPI_MODE_0,
	.bus_num = 0,
	.chip_select = 0,
	.controller_data = &qrk_ffrd_spi_0_cs_0,
	.platform_data = &sc16is752_platform_data,
};

static struct spi_board_info spi_sc16is741_info = {
	.modalias = "sc16is74x",
	.max_speed_hz = 4000000,
	.mode = SPI_MODE_0,
	.bus_num = 1,
	.chip_select = 0,
	.controller_data = &qrk_ffrd_spi_1_cs_0,
	.platform_data = &sc16is741_platform_data,
};


static struct gpio reserved_gpios[] = {
	{
		GPIO_IRQ_SPI2UART_B_BUF,
		GPIOF_IN,
		"sc16is752-int",
	},
	{
		GPIO_GPIO1_RS485_IRQ,
		GPIOF_IN,
		"sc16is741-int",
	},
	{
		GPIO_SLB9645TT_RESET,
		GPIOF_OUT_INIT_HIGH,
		"slb96455tt-reset",
	},
	{
		GPIO_SLB9645TT_INT,
		GPIOF_IN,
		"slb96455tt-int",
	},
};

static int slb9645tt_i2c_probe(struct i2c_adapter *adap, unsigned short addr)
{
	return gpio_get_value(GPIO_SLB9645TT_RESET);
}

/**
 * intel_qrk_gpio_restrict_probe
 *
 * Make GPIOs pertaining to Firmware inaccessible by requesting them.  The
 * GPIOs are never released nor accessed by this driver.
 *
 * Registers devices which are dependent on these GPIO drivers
 */
static int intel_qrk_gpio_restrict_probe(void)
{
	struct i2c_adapter *i2c_adap;
	struct i2c_client *slb9645tt;
	int ret;

	/* Reserve GPIOs for I2C/SPI device interrupts (never released) */
	ret = gpio_request_array(reserved_gpios, ARRAY_SIZE(reserved_gpios));
	if (ret) {
		pr_err("%s: failed to request reserved gpios\n",
		       __func__);
		return ret;
	}

	/*
	 * Register on-board I2C devices
	 */
	probed_slb9645tt.irq = gpio_to_irq(GPIO_SLB9645TT_INT);

	i2c_adap = i2c_get_adapter(0);
	if (!i2c_adap) {
		pr_info("%s: i2c adapter not ready yet. Deferring..\n",
			__func__);
		ret = -EPROBE_DEFER;
		goto err;
	}

	strlcpy(probed_slb9645tt.type, "slb9645tt", I2C_NAME_SIZE);
	slb9645tt = i2c_new_probed_device(i2c_adap, &probed_slb9645tt,
					  slb9645tt_i2c_addr,
					  slb9645tt_i2c_probe);
	i2c_put_adapter(i2c_adap);

	if (!slb9645tt) {
		pr_err("%s: can't probe slb9645tt\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	/*
	 * Register on-board SPI devices
	 */
	spi_sc16is752_info.irq = gpio_to_irq(GPIO_IRQ_SPI2UART_B_BUF);
	ret = spi_register_board_info(&spi_sc16is752_info, 1);
	if (ret) {
		pr_err("%s: Failed to register sc16is752 SPI device\n",
		       __func__);
		goto err;
	}

	spi_sc16is741_info.irq = gpio_to_irq(GPIO_GPIO1_RS485_IRQ);
	ret = spi_register_board_info(&spi_sc16is741_info, 1);
	if (ret) {
		pr_err("%s: Failed to register sc16is741 SPI device\n",
		       __func__);
		goto err;
	}

	return 0;
err:
	gpio_free_array(reserved_gpios, ARRAY_SIZE(reserved_gpios));
	return ret;
}

/**
 * intel_qrk_gpio_restrict_probe_nc
 *
 * Make GPIOs pertaining to Firmware inaccessible by requesting them.  The
 * GPIOs are never released nor accessed by this driver.
 */
static int intel_qrk_gpio_restrict_probe_nc(struct platform_device *pdev)
{
	int ret;
	nc_gpio_reg = 1;

	if (nc_gpio_reg == 1 && sc_gpio_reg == 1) {
		ret = intel_qrk_gpio_restrict_probe();
		if (ret)
			return ret;
	}
	return 0;
}

/**
 * intel_qrk_gpio_restrict_probe_sc
 *
 * Make GPIOs pertaining to Firmware inaccessible by requesting them.  The
 * GPIOs are never released nor accessed by this driver.
 */
static int intel_qrk_gpio_restrict_probe_sc(struct platform_device *pdev)
{
	int ret;
	sc_gpio_reg = 1;

	if (nc_gpio_reg == 1 && sc_gpio_reg == 1) {
		ret = intel_qrk_gpio_restrict_probe();
		if (ret)
			return ret;
	}
	return 0;
}

static struct platform_driver gpio_restrict_pdriver_nc = {
	.driver		= {
		.name	= GPIO_RESTRICT_NAME_NC,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_gpio_restrict_probe_nc,
};

static struct platform_driver gpio_restrict_pdriver_sc = {
	.driver		= {
		.name	= GPIO_RESTRICT_NAME_SC,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_gpio_restrict_probe_sc,
};

/**
 * intel_qrk_i2c_add_onboard_devs
 *
 * @return 0 on success or standard errnos on failure
 *
 * Registers onboard I2c device(s) present on the Reliance Creek platform
 */
static int intel_qrk_i2c_add_onboard_devs(void)
{
	int ret = 0;
	struct i2c_adapter *i2c_adap = NULL;
	struct i2c_client *client = NULL;

	i2c_adap = i2c_get_adapter(0);
	if (NULL == i2c_adap) {
		pr_info("%s: i2c adapter not ready yet. Deferring..\n",
			__func__);
		return -EPROBE_DEFER;
	}

	/*
	 * Register on-board I2C devices
	 */
	strlcpy(probed_i2c_tmp75.type, "tmp75", I2C_NAME_SIZE);
	client = i2c_new_probed_device(i2c_adap, &probed_i2c_tmp75,
				       tmp75_i2c_addr, i2c_probe);
	if (client == NULL) {
		pr_err("%s: Failed to probe tmp75 I2C device\n", __func__);
		ret = -ENODEV;
		goto end;
	}
	strlcpy(probed_i2c_eeprom.type, "24c64", I2C_NAME_SIZE);
	client = i2c_new_probed_device(i2c_adap, &probed_i2c_eeprom,
				       eeprom_i2c_addr, i2c_probe);
	if (client == NULL) {
		pr_err("%s: Failed to probe 24c64 I2C device\n", __func__);
		ret = -ENODEV;
		goto end;
	}
	strlcpy(probed_i2c_pcf8574_exp3.type, "pcf8574", I2C_NAME_SIZE);
	client = i2c_new_probed_device(i2c_adap, &probed_i2c_pcf8574_exp3,
				       pcf8574_exp3_i2c_addr, i2c_probe);
	if (client == NULL) {
		pr_err("%s: Failed to probe pcf8574 I2C device\n", __func__);
		ret = -ENODEV;
		goto end;
	}

end:
	i2c_put_adapter(i2c_adap);

	return ret;
}

/**
 * intel_qrk_i2c_add_onboard_devs
 *
 * @return 0 on success or standard errnos on failure
 *
 * Registers I2c device(s) present on the Reliance Creek Sensor Interface Board
 */
static int intel_qrk_i2c_add_sib_devs(void)
{
	int ret = 0;
	struct i2c_adapter *i2c_adap = NULL;
	struct i2c_client *client = NULL;

	i2c_adap = i2c_get_adapter(0);
	if (NULL == i2c_adap) {
		pr_info("%s: i2c adapter not ready yet. Deferring..\n",
			__func__);
		return -EPROBE_DEFER;
	}

	/*
	 * Register I2C devices on the Sensor Interface Board
	 */
	strlcpy(probed_i2c_pcf8574_exp1.type, "pcf8574", I2C_NAME_SIZE);
	client = i2c_new_probed_device(i2c_adap, &probed_i2c_pcf8574_exp1,
				       pcf8574_exp1_i2c_addr, i2c_probe);
	if (client == NULL) {
		pr_err("%s: Failed to probe pcf8574 I2C device\n", __func__);
		ret = -ENODEV;
		goto end;
	}
	strlcpy(probed_i2c_pcf8574_exp2.type, "pcf8574", I2C_NAME_SIZE);
	client = i2c_new_probed_device(i2c_adap, &probed_i2c_pcf8574_exp2,
				       pcf8574_exp2_i2c_addr, i2c_probe);
	if (client == NULL) {
		pr_err("%s: Failed to probe pcf8574 I2C device\n", __func__);
		ret = -ENODEV;
		goto end;
	}

end:
	i2c_put_adapter(i2c_adap);

	return ret;
}

static int intel_qrk_plat_reliance_creek_probe(struct platform_device *pdev)
{
	int ret = 0;

	/* Register on-board I2C devices (common to all SKUs) */
	ret = intel_qrk_i2c_add_onboard_devs();
	if (ret)
		return ret;

	/* Register I2C devices on Sensor Interface Board
	 * Applicable to Reliance Creek SPU board only
	 */
	if (pdev->id_entry && pdev->id_entry->driver_data) {
		ret = intel_qrk_i2c_add_sib_devs();
		if (ret)
			return ret;
	}

	/* Note that we register I2C devices first and everything else later
	 * because the I2C reg functions may return -EPROBE_DEFER
	 * and the following functions are not written to be idempotent
	 */
	ret = platform_driver_register(&gpio_restrict_pdriver_nc);
	if (ret)
		return ret;
	ret = platform_driver_register(&gpio_restrict_pdriver_sc);
	if (ret)
		return ret;


	return 0;
}

static int intel_qrk_plat_reliance_creek_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_device_id intel_qrk_reliance_creek_ids[] = {
	{
		.name = "RelianceCreek",
		.driver_data = 0,
	},
	{
		.name = "RelianceCreekSPU",
		.driver_data = 1,
	},
};
MODULE_DEVICE_TABLE(platform, intel_qrk_reliance_creek_ids);

static struct platform_driver intel_qrk_reliance_creek_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_plat_reliance_creek_probe,
	.remove		= intel_qrk_plat_reliance_creek_remove,
	.id_table	= intel_qrk_reliance_creek_ids,
};

module_platform_driver(intel_qrk_reliance_creek_driver);

MODULE_AUTHOR("Dan O'Donovan <dan@emutex.com>");
MODULE_DESCRIPTION("Reliance Creek BSP Data");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:"DRIVER_NAME);
