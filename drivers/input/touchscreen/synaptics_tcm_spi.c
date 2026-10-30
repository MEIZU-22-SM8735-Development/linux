// SPDX-License-Identifier: GPL-2.0-only
/*
 * Synaptics TouchComm (TCM) touchscreen driver, SPI bus glue.
 *
 * This part is bus agnostic: synaptics_tcm_spi.c provides the SPI read/write
 * pair plus the power and reset wiring, while the core owns the protocol, the
 * interrupt handler and the moment the interrupt is enabled.
 *
 * The Meizu 22 (SM8735) wires the part to QUPV3 wrapper 1 serial engine 4,
 * which the bootloader already leaves programmed as SPI, chip select 0 at
 * 15 MHz.  The interrupt is TLMM GPIO 176 (level low, the vendor node's
 * "interrupts = <176 0x2008>" is IRQF_ONESHOT | IRQ_TYPE_LEVEL_LOW) and the
 * reset line is TLMM GPIO 189, asserted low.
 *
 * References to "vendor" are to vendor_dlkm/lib/modules/synaptics_tcm2_ts.ko;
 * its .text is an ET_REL section, so file offset = sh_offset + st_value.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include "synaptics_tcm.h"

/* Vendor delays, from the live device tree of this board:
 * synaptics,reset-active-ms = 20, synaptics,reset-delay-ms = 200,
 * synaptics,power-delay-ms = 50.
 */
#define TCM_RESET_ASSERT_MS	20
#define TCM_RESET_RELEASE_MS	200
#define TCM_REGULATOR_DELAY_US	5000
#define TCM_REGULATOR_OFF_US	10000

/* Vendor syna_spi_power_on(): the live device tree's synaptics,power-delay-ms. */
#define TCM_POWER_DELAY_MS	50
/* Rails down and up again, for the cold boot described at the call site. */
#define TCM_POWER_CYCLE_MS	20

struct synaptics_tcm_spi {
	struct synaptics_tcm tcm;
	struct spi_device *spi;
	struct regulator *vdd;
	struct regulator *avdd;
	struct gpio_desc *reset_gpio;
	u8 *dummy_tx;
};

static int synaptics_tcm_spi_write(struct synaptics_tcm *tcm, const void *buf,
				   size_t count)
{
	struct synaptics_tcm_spi *priv;

	priv = container_of(tcm, struct synaptics_tcm_spi, tcm);

	return spi_write(priv->spi, buf, count);
}

/*
 * Reads are clocked with 0xff dummy bytes in full duplex, which is what the
 * vendor syna_spi_read() (.text+0x1342c) does: it fills the transmit buffer
 * with 0xff and runs one spi_sync().  The vendor only splits the transfer
 * into one byte per spi_transfer when "synaptics,spi-byte-delay-us" is set;
 * this board sets it to 0, so a single transfer is used.
 */
static int synaptics_tcm_spi_read(struct synaptics_tcm *tcm, void *buf,
				  size_t count)
{
	struct synaptics_tcm_spi *priv;
	struct spi_transfer transfer = {
		.tx_buf = NULL,
		.rx_buf = buf,
		.len = count,
	};
	struct spi_message message;

	priv = container_of(tcm, struct synaptics_tcm_spi, tcm);

	if (count > TCM_MAX_TRANSFER)
		return -EINVAL;

	transfer.tx_buf = priv->dummy_tx;

	spi_message_init(&message);
	spi_message_add_tail(&transfer, &message);

	return spi_sync(priv->spi, &message);
}

static const struct synaptics_tcm_ops synaptics_tcm_spi_ops = {
	.read = synaptics_tcm_spi_read,
	.write = synaptics_tcm_spi_write,
};

static int synaptics_tcm_spi_power_on(struct synaptics_tcm_spi *priv)
{
	int error;

	/* Vendor syna_spi_power_on() (.text+0x139e4) brings up avdd first,
	 * waits about 5 ms, then vdd; power down is vdd, 10 ms, avdd.
	 */
	if (priv->avdd) {
		error = regulator_enable(priv->avdd);
		if (error)
			return error;
		usleep_range(TCM_REGULATOR_DELAY_US,
			     TCM_REGULATOR_DELAY_US + 100);
	}

	if (priv->vdd) {
		error = regulator_enable(priv->vdd);
		if (error) {
			if (priv->avdd)
				regulator_disable(priv->avdd);
			return error;
		}
		usleep_range(TCM_REGULATOR_DELAY_US,
			     TCM_REGULATOR_DELAY_US + 100);
	}

	return 0;
}

static void synaptics_tcm_spi_power_off(struct synaptics_tcm_spi *priv)
{
	if (priv->vdd)
		regulator_disable(priv->vdd);
	usleep_range(TCM_REGULATOR_OFF_US, TCM_REGULATOR_OFF_US + 100);
	if (priv->avdd)
		regulator_disable(priv->avdd);
}

/*
 * Vendor syna_spi_hw_reset() (.text+0x13cf8) drives the line to
 * synaptics,reset-on-state (0 on this board, so reset is asserted low),
 * sleeps reset-active-ms and then releases it for reset-delay-ms.  The
 * device tree must describe the line as GPIO_ACTIVE_LOW.
 */
static void synaptics_tcm_spi_hw_reset(struct synaptics_tcm_spi *priv)
{
	if (!priv->reset_gpio)
		return;

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	msleep(TCM_RESET_ASSERT_MS);
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	msleep(TCM_RESET_RELEASE_MS);
}

static int synaptics_tcm_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct synaptics_tcm_spi *priv;
	const char *firmware_name = "synaptics/firmware.img";
	int error;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->spi = spi;
	priv->tcm.dev = dev;
	priv->tcm.ops = &synaptics_tcm_spi_ops;
	priv->tcm.irq = spi->irq;
	mutex_init(&priv->tcm.msg_lock);
	spi_set_drvdata(spi, priv);

	priv->dummy_tx = devm_kzalloc(dev, TCM_MAX_TRANSFER, GFP_KERNEL);
	if (!priv->dummy_tx)
		return -ENOMEM;
	memset(priv->dummy_tx, 0xff, TCM_MAX_TRANSFER);

	priv->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(priv->vdd))
		return dev_err_probe(dev, PTR_ERR(priv->vdd),
				     "cannot get vdd supply\n");

	priv->avdd = devm_regulator_get_optional(dev, "avdd");
	if (IS_ERR(priv->avdd))
		return dev_err_probe(dev, PTR_ERR(priv->avdd),
				     "cannot get avdd supply\n");

	if (!priv->vdd)
		dev_warn(dev, "no vdd supply, the part may stay unpowered\n");

	/* Held in reset while the rails come up, as the vendor does. */
	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "cannot get reset GPIO\n");

	/*
	 * Cold boot the part, the way the vendor's syna_spi_power_on/off pair
	 * does it: rails off, then avdd, 5 ms, vdd, then msleep(power_delay_ms)
	 * -- which the live device tree sets to 50 -- and only then the reset
	 * pulse.  A reset pulse on its own is not the same thing, and the
	 * difference is measurable on this board: the one time the controller
	 * was seen announcing itself (its attention line driven low) was after
	 * its first power-up, and never again after a reset-only cycle.  The
	 * first power_on() is what makes the following power_off() a balanced
	 * call rather than an unbalanced disable.
	 */
	error = synaptics_tcm_spi_power_on(priv);
	if (error) {
		dev_err(dev, "cannot power on: %d\n", error);
		return error;
	}
	msleep(TCM_POWER_CYCLE_MS);
	synaptics_tcm_spi_power_off(priv);
	msleep(TCM_POWER_CYCLE_MS);

	error = synaptics_tcm_spi_power_on(priv);
	if (error) {
		dev_err(dev, "cannot power on: %d\n", error);
		return error;
	}

	/* Vendor syna_spi_power_on() ends with msleep(power_delay_ms). */
	msleep(TCM_POWER_DELAY_MS);

	synaptics_tcm_spi_hw_reset(priv);

	device_property_read_string(dev, "firmware-name", &firmware_name);

	/* The interrupt itself is requested by the core, which also knows
	 * when the touch report has been enabled and the part starts asserting
	 * it; requesting it here as well would fail with -EBUSY.
	 */
	if (spi->irq <= 0)
		dev_warn(dev, "no interrupt, no touch events will be reported\n");

	error = synaptics_tcm_probe(&priv->tcm, firmware_name);
	if (error)
		goto err_power;

	return 0;

err_power:
	synaptics_tcm_spi_power_off(priv);

	return error;
}

static void synaptics_tcm_spi_remove(struct spi_device *spi)
{
	struct synaptics_tcm_spi *priv = spi_get_drvdata(spi);

	if (!priv)
		return;

	if (priv->tcm.irq > 0)
		disable_irq(priv->tcm.irq);

	synaptics_tcm_remove(&priv->tcm);
	synaptics_tcm_spi_power_off(priv);
}

static int __maybe_unused synaptics_tcm_spi_suspend(struct device *dev)
{
	struct synaptics_tcm_spi *priv = dev_get_drvdata(dev);

	return synaptics_tcm_suspend(&priv->tcm);
}

static int __maybe_unused synaptics_tcm_spi_resume(struct device *dev)
{
	struct synaptics_tcm_spi *priv = dev_get_drvdata(dev);

	return synaptics_tcm_resume(&priv->tcm);
}

static DEFINE_SIMPLE_DEV_PM_OPS(synaptics_tcm_spi_pm_ops,
				synaptics_tcm_spi_suspend,
				synaptics_tcm_spi_resume);

static const struct of_device_id synaptics_tcm_spi_of_match[] = {
	{ .compatible = "synaptics,tcm-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, synaptics_tcm_spi_of_match);

/* The vendor module's modinfo carries "alias=spi:synaptics_tcm_spi". */
static const struct spi_device_id synaptics_tcm_spi_id[] = {
	{ "synaptics_tcm_spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, synaptics_tcm_spi_id);

static struct spi_driver synaptics_tcm_spi_driver = {
	.probe = synaptics_tcm_spi_probe,
	.remove = synaptics_tcm_spi_remove,
	.id_table = synaptics_tcm_spi_id,
	.driver = {
		.name = "synaptics_tcm_spi",
		.of_match_table = synaptics_tcm_spi_of_match,
		.pm = pm_ptr(&synaptics_tcm_spi_pm_ops),
	},
};
module_spi_driver(synaptics_tcm_spi_driver);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TouchComm (TCM) SPI touchscreen");
MODULE_LICENSE("GPL");
