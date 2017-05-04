/*
 * SC16IS7XX tty serial driver (SPI bus)
 *
 * Copyright (C) 2014 GridPoint
 * Author: Jon Ringle <jringle@gridpoint.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "sc16is7xx.h"

static int sc16is7xx_spi_probe(struct spi_device *spi)
{
	unsigned long flags = 0;
	struct regmap *regmap;
	struct sc16is7xx_devtype *devtype;
	struct regmap_config regcfg;

	if (spi->dev.of_node) {
		const struct of_device_id *of_id =
			of_match_device(sc16is7xx_dt_ids, &spi->dev);
		if (of_id == NULL) {
			dev_err(&spi->dev, "Error getting device id!\n");
			return -1;
		}
		devtype = (struct sc16is7xx_devtype *)of_id->data;
	} else {
		const struct spi_device_id *id = spi_get_device_id(spi);
		devtype = (struct sc16is7xx_devtype *)id->driver_data;
	}
	_dev_info(&spi->dev, "device type: %s\n", devtype->name);
	flags = IRQF_TRIGGER_LOW;

	sc16is7xx_regmap_config_init(&regcfg, devtype->nr_uart);

	regmap = devm_regmap_init_spi(spi, &regcfg);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initialising the SPI regmap!\n");
		return -1;
	}

	return sc16is7xx_probe(&spi->dev, devtype, regmap, spi->irq, flags);
}

static int sc16is7xx_spi_remove(struct spi_device *spi)
{
	return sc16is7xx_remove(&spi->dev);
}

static const struct spi_device_id sc16is7xx_spi_id_table[] = {
	{ "sc16is74x",	(kernel_ulong_t)&sc16is74x_devtype, },
	{ "sc16is750",	(kernel_ulong_t)&sc16is750_devtype, },
	{ "sc16is752",	(kernel_ulong_t)&sc16is752_devtype, },
	{ "sc16is760",	(kernel_ulong_t)&sc16is760_devtype, },
	{ "sc16is762",	(kernel_ulong_t)&sc16is762_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(spi, sc16is7xx_spi_id_table);

static struct spi_driver sc16is7xx_spi_driver = {
	.driver = {
		.name		= SC16IS7XX_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(sc16is7xx_dt_ids),
	},
	.probe			= sc16is7xx_spi_probe,
	.remove			= sc16is7xx_spi_remove,
	.id_table		= sc16is7xx_spi_id_table,
};
module_spi_driver(sc16is7xx_spi_driver);

MODULE_AUTHOR("Bogdan Pricop <bogdan.pricop@emutex.com>");
MODULE_DESCRIPTION("SC16IS7XX tty serial driver over SPI bus");
MODULE_LICENSE("GPL v2");
