// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * cs35l45-i2c.c -- CS35L45 I2C driver
 *
 * Copyright 2019 Cirrus Logic, Inc.
 *
 * Author: James Schulman <james.schulman@cirrus.com>
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>

#include "wm_adsp.h"
#include "cs35l45.h"
#include <sound/cs35l45.h>

#ifdef ASUS_SAKE_PROJECT
extern int register_receiver_dai_name(struct device *dev, int i2cbus, int addr);
extern int register_speaker_dai_name(struct device *dev, int i2cbus, int addr);
#endif

static struct regmap_config cs35l45_regmap = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = CS35L45_REGSTRIDE,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = CS35L45_LASTREG,
	.reg_defaults = cs35l45_reg,
	.num_reg_defaults = ARRAY_SIZE(cs35l45_reg),
	.volatile_reg = cs35l45_volatile_reg,
	.readable_reg = cs35l45_readable_reg,
	.cache_type = REGCACHE_RBTREE,
};

static int cs35l45_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct cs35l45_private *cs35l45;
	struct device *dev = &client->dev;
	int ret;
	int i = 0, retry_count = 100; /* ASUS_BSP Paul +++ */

	/* ASUS_BSP Paul +++ */
	if (of_property_read_bool(dev->of_node, "asus,dummy")) {
		dev_err(dev, "dummy driver for EVB");
		return 0;
	}
	/* ASUS_BSP Paul --- */

	cs35l45 = devm_kzalloc(dev, sizeof(struct cs35l45_private), GFP_KERNEL);
	if (cs35l45 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, cs35l45);
	cs35l45->regmap = devm_regmap_init_i2c(client, &cs35l45_regmap);
	if (IS_ERR(cs35l45->regmap)) {
		ret = PTR_ERR(cs35l45->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	cs35l45->dev = dev;
	cs35l45->irq = client->irq;
	cs35l45->wksrc = CS35L45_WKSRC_I2C;
	cs35l45->i2c_addr = client->addr;

#ifdef ASUS_SAKE_PROJECT
	if(client->addr == 48){//48=0x30
		register_receiver_dai_name(dev, client->adapter->nr, client->addr);
	}else if(client->addr == 49){//49=0x31
		register_speaker_dai_name(dev, client->adapter->nr, client->addr);
	}
#endif

	ret = cs35l45_probe(cs35l45);
	if (ret < 0) {
		dev_err(dev, "Failed device probe: %d\n", ret);
		return ret;
	}

#if 0 /* ASUS_BSP Paul +++ */
	usleep_range(2000, 2100);

	ret = cs35l45_initialize(cs35l45);
#else
	do {
		usleep_range(2000, 2100);
		ret = cs35l45_initialize(cs35l45);
		i++;
	} while (ret < 0 && i < retry_count);
#endif /* ASUS_BSP Paul --- */

	if (ret < 0) {
		dev_err(dev, "Failed device initialization: %d , still return status OK for factory special usecase\n", ret); //Austin+++
		//return ret;
	}

	return 0;
}

static int cs35l45_i2c_remove(struct i2c_client *client)
{
	struct cs35l45_private *cs35l45 = i2c_get_clientdata(client);

	return cs35l45_remove(cs35l45);
}

static const struct of_device_id cs35l45_of_match[] = {
	{.compatible = "cirrus,cs35l45"},
	{},
};
MODULE_DEVICE_TABLE(of, cs35l45_of_match);

static const struct i2c_device_id cs35l45_id_i2c[] = {
	{"cs35l45", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cs35l45_id_i2c);

static struct i2c_driver cs35l45_i2c_driver = {
	.driver = {
		.name		= "cs35l45",
		.of_match_table = cs35l45_of_match,
	},
	.id_table	= cs35l45_id_i2c,
	.probe		= cs35l45_i2c_probe,
	.remove		= cs35l45_i2c_remove,
};
module_i2c_driver(cs35l45_i2c_driver);

MODULE_DESCRIPTION("I2C CS35L45 driver");
MODULE_AUTHOR("James Schulman, Cirrus Logic Inc, <james.schulman@cirrus.com>");
MODULE_LICENSE("GPL");
