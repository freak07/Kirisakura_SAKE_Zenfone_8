#include <linux/regulator/consumer.h>

#include "asus_module.h"

static struct regulator_info laser_reg[] = {
	{NULL, "laser_vdd", 3100000, 2800000, 200000, false},
	{NULL, "laser_bus", 1800000, 1800000, 200000, false},
};

int32_t _regulator_init(struct device *dev)
{
	int rc = 0, idx = 0;

	if(!dev)
		return -EINVAL;

	for(idx = 0; idx < ARRAY_SIZE(laser_reg); idx++) {
		laser_reg[idx].reg = regulator_get(dev, laser_reg[idx].name);
		if(IS_ERR_OR_NULL(laser_reg[idx].reg)) {
			if(PTR_ERR(laser_reg[idx].reg))
				err("%s NULL pointer", laser_reg[idx].name);
			rc = -EINVAL;
			err("Failed to get regulator '%s'", laser_reg[idx].name);
			goto error;
		}
		rc = regulator_set_voltage(laser_reg[idx].reg, laser_reg[idx].min_volt, laser_reg[idx].max_volt);
		if(rc) {
			err("Failed to set voltage to %u~%u for '%s' regulator, err = %d",
				laser_reg[idx].min_volt, laser_reg[idx].max_volt, laser_reg[idx].name, rc);
			goto error;
		}
		rc = regulator_set_load(laser_reg[idx].reg, 0);
		if(rc) {
			err("Failed to set load current to %u for '%s' regulaotr, err = %d",
				0, laser_reg[idx].name, rc);
			regulator_set_voltage(laser_reg[idx].reg, 0,  laser_reg[idx].max_volt);
			goto error;
		}
		log("Laser regulator '%s' initial finish!!", laser_reg[idx].name);
	}

	return 0;
error:
	for(idx -= 1; idx >= 0; idx--) {
		regulator_set_voltage(laser_reg[idx].reg, 0, laser_reg[idx].max_volt);
		regulator_put(laser_reg[idx].reg);
		laser_reg[idx].reg = NULL;
	}
	return rc;
}

int32_t _regulator_enable(void)
{
	int rc = 0, idx = 0;

	for(idx = 0; idx < ARRAY_SIZE(laser_reg); idx++) {
		if(laser_reg[idx].reg == NULL) {
			rc = -EINVAL;
			err("The regulator '%s' is NULL.", laser_reg[idx].name);
			goto error;
		}
		rc = regulator_set_load(laser_reg[idx].reg, laser_reg[idx].load_curr);
                if(rc) {
                        err("Failed to set load to %u for '%s' regulator, err = %d",
				laser_reg[idx].load_curr, laser_reg[idx].name, rc);
                        goto error;
                }
		rc = regulator_enable(laser_reg[idx].reg);
		if(rc) {
			err("Failed to enable '%s' regulator, err = %d", laser_reg[idx].name, rc);
			regulator_set_load(laser_reg[idx].reg, 0);
			goto error;
		}
	}
	log("Laser regulator enable finish!!");

	return 0;
error:
	for(idx -= 1; idx > 0; idx--) {
		regulator_set_load(laser_reg[idx].reg, 0);
		regulator_disable(laser_reg[idx].reg);
	}
	return rc;
}

int32_t _regulator_disable(void)
{
	int rc = 0, idx = 0;

	for(idx = 0; idx < ARRAY_SIZE(laser_reg); idx++) {
		if(laser_reg[idx].reg == NULL) {
			rc = -EINVAL;
			err("The regulator '%s' is NULL.\n", laser_reg[idx].name);
			goto error;
		}
		rc = regulator_set_load(laser_reg[idx].reg, 0);
		if(rc) {
			err("Failed to set load to %u for '%s' regulator, err = %d",
				0, laser_reg[idx].name, rc);
			goto error;
		}
		rc = regulator_disable(laser_reg[idx].reg);
		if(rc) {
			err("Failed to disable '%s' regulator, err = %d", laser_reg[idx].name, rc);
			goto error;
		}
	}
	log("Laser regulator disable finish!!");

	return 0;
error:
	return rc;
}

int32_t _regulator_remove(void)
{
	int rc = 0, idx = 0;

	for(idx = 0; idx < ARRAY_SIZE(laser_reg); idx++) {
		if(laser_reg[idx].reg == NULL) {
			err("The regulator '%s' is NULL.", laser_reg[idx].name);
			continue;
		}
		regulator_disable(laser_reg[idx].reg);
		regulator_set_load(laser_reg[idx].reg, 0);
		regulator_set_voltage(laser_reg[idx].reg, 0, laser_reg[idx].max_volt);
		regulator_put(laser_reg[idx].reg);
		laser_reg[idx].reg = NULL;
	}

	return rc;
}

int32_t asus_laser_init(struct device *dev)
{
        int ret = 0;

        _regulator_init(dev);

        return ret;
}

int32_t asus_laser_enable(void)
{
	int rc = 0;

	rc = _regulator_enable();

	return rc;
}

int32_t asus_laser_disable(void)
{
	int rc = 0;

	rc = _regulator_disable();

	return rc;
}

int32_t asus_laser_remove(void)
{
	int rc = 0;

	rc = _regulator_remove();

	return rc;
}

//---------------------------------------------------------//

static struct spi_geni_qcom_ctrl_data delay_params;

int32_t asus_set_spi_clk_delay(struct spi_device *spi, u32 cs_clk_delay, u32 inter_words_delay)
{
	int rc = 0;
	struct spi_geni_qcom_ctrl_data *delay_params_debug = NULL;

	delay_params.spi_cs_clk_delay = cs_clk_delay;		// Num of clk cycles to wait after asserting CS before starting txfer
	delay_params.spi_inter_words_delay = inter_words_delay;	// Num of clk cycles to wait between SPI words MOSI

	spi->controller_data = &delay_params;
	if(spi->controller_data) {
		delay_params_debug = (struct spi_geni_qcom_ctrl_data *) spi->controller_data;
		log("Set spi_cs_clk_delay:%u", delay_params_debug->spi_cs_clk_delay);
		log("Set spi_inter_words_delay:%u", delay_params_debug->spi_inter_words_delay);
	}
	else {
		err("spi->controller_data is NULL");
		return -1;
	}

	return rc;	
}
