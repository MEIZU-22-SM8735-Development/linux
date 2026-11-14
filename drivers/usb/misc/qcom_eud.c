// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/extcon-provider.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/usb/role.h>
#include <linux/firmware/qcom/qcom_scm.h>

#define EUD_REG_INT1_EN_MASK	0x0024
#define EUD_REG_INT_STATUS_1	0x0044
#define EUD_REG_CTL_OUT_1	0x0074
#define EUD_REG_VBUS_INT_CLR	0x0080
#define EUD_REG_CSR_EUD_EN	0x1014
#define EUD_REG_SW_ATTACH_DET	0x1018
#define EUD_REG_EUD_EN2		0x0000

#define EUD_ENABLE		BIT(0)
#define EUD_INT_PET_EUD		BIT(0)
#define EUD_INT_VBUS		BIT(2)
#define EUD_INT_SAFE_MODE	BIT(4)
#define EUD_INT_ALL		(EUD_INT_VBUS | EUD_INT_SAFE_MODE)

static const unsigned int eud_extcon_cables[] = {
	EXTCON_USB,
	EXTCON_CHG_USB_SDP,
	EXTCON_JIG,
	EXTCON_NONE,
};

struct eud_chip {
	struct device			*dev;
	struct usb_role_switch		*role_sw;
	struct extcon_dev		*extcon;
	void __iomem			*base;
	phys_addr_t			mode_mgr;
	unsigned int			int_status;
	int				irq;
	bool				enabled;
	bool				usb_attached;
};

static bool eud_hw_enabled(struct eud_chip *priv)
{
	u32 value;

	if (qcom_scm_io_readl(priv->mode_mgr + EUD_REG_EUD_EN2, &value))
		return false;

	return value & EUD_ENABLE;
}

static int enable_eud(struct eud_chip *priv)
{
	int ret;

	ret = qcom_scm_io_writel(priv->mode_mgr + EUD_REG_EUD_EN2, 1);
	if (ret)
		return ret;

	writel(EUD_ENABLE, priv->base + EUD_REG_CSR_EUD_EN);
	writel(EUD_INT_VBUS | EUD_INT_SAFE_MODE,
			priv->base + EUD_REG_INT1_EN_MASK);
	extcon_set_state_sync(priv->extcon, EXTCON_USB, true);
	extcon_set_state_sync(priv->extcon, EXTCON_CHG_USB_SDP, true);

	return usb_role_switch_set_role(priv->role_sw, USB_ROLE_DEVICE);
}

static int disable_eud(struct eud_chip *priv)
{
	int ret;

	ret = qcom_scm_io_writel(priv->mode_mgr + EUD_REG_EUD_EN2, 0);
	if (ret)
		return ret;

	writel(0, priv->base + EUD_REG_CSR_EUD_EN);
	extcon_set_state_sync(priv->extcon, EXTCON_USB, false);
	extcon_set_state_sync(priv->extcon, EXTCON_CHG_USB_SDP, false);
	return 0;
}

static ssize_t enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct eud_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", chip->enabled);
}

static ssize_t enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct eud_chip *chip = dev_get_drvdata(dev);
	bool enable;
	int ret;

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	if (enable) {
		ret = enable_eud(chip);
		if (!ret)
			chip->enabled = enable;
		else
			disable_eud(chip);

	} else {
		ret = disable_eud(chip);
	}

	return ret < 0 ? ret : count;
}

static DEVICE_ATTR_RW(enable);

static struct attribute *eud_attrs[] = {
	&dev_attr_enable.attr,
	NULL,
};
ATTRIBUTE_GROUPS(eud);

static void usb_attach_detach(struct eud_chip *chip)
{
	u32 reg;

	/* read ctl_out_1[4] to find USB attach or detach event */
	reg = readl(chip->base + EUD_REG_CTL_OUT_1);
	chip->usb_attached = reg & EUD_INT_SAFE_MODE;
	extcon_set_state_sync(chip->extcon, EXTCON_USB, chip->usb_attached);
	extcon_set_state_sync(chip->extcon, EXTCON_CHG_USB_SDP,
				 chip->usb_attached);
}

static void pet_eud(struct eud_chip *chip)
{
	u32 reg;
	int ret;

	/* When the EUD_INT_PET_EUD in SW_ATTACH_DET is set, the cable has been
	 * disconnected and we need to detach the pet to check if EUD is in safe
	 * mode before attaching again.
	 */
	reg = readl(chip->base + EUD_REG_SW_ATTACH_DET);
	if (reg & EUD_INT_PET_EUD) {
		/* Detach & Attach pet for EUD */
		writel(0, chip->base + EUD_REG_SW_ATTACH_DET);
		/* Delay to make sure detach pet is done before attach pet */
		ret = readl_poll_timeout(chip->base + EUD_REG_SW_ATTACH_DET,
					reg, (reg == 0), 1, 100);
		if (ret) {
			dev_err(chip->dev, "Detach pet failed\n");
			return;
		}
	}
	/* Attach pet for EUD */
	writel(EUD_INT_PET_EUD, chip->base + EUD_REG_SW_ATTACH_DET);
}

static irqreturn_t handle_eud_irq(int irq, void *data)
{
	struct eud_chip *chip = data;
	u32 reg;

	reg = readl(chip->base + EUD_REG_INT_STATUS_1);
	if (reg & EUD_INT_VBUS) {
		usb_attach_detach(chip);
		return IRQ_WAKE_THREAD;
	}
	if (reg & EUD_INT_SAFE_MODE) {
		pet_eud(chip);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t handle_eud_irq_thread(int irq, void *data)
{
	struct eud_chip *chip = data;
	int ret;

	if (chip->usb_attached)
		ret = usb_role_switch_set_role(chip->role_sw, USB_ROLE_DEVICE);
	else
		ret = usb_role_switch_set_role(chip->role_sw, USB_ROLE_HOST);
	if (ret)
		dev_err(chip->dev, "failed to set role switch\n");

	/* set and clear vbus_int_clr[0] to clear interrupt */
	writel(BIT(0), chip->base + EUD_REG_VBUS_INT_CLR);
	writel(0, chip->base + EUD_REG_VBUS_INT_CLR);

	return IRQ_HANDLED;
}

static void eud_role_switch_release(void *data)
{
	struct eud_chip *chip = data;

	usb_role_switch_put(chip->role_sw);
}

static int eud_probe(struct platform_device *pdev)
{
	struct eud_chip *chip;
	struct resource *res;
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;

	chip->role_sw = usb_role_switch_get(&pdev->dev);
	if (IS_ERR(chip->role_sw))
		return dev_err_probe(chip->dev, PTR_ERR(chip->role_sw),
					"failed to get role switch\n");

	chip->extcon = devm_extcon_dev_allocate(chip->dev, eud_extcon_cables);
	if (IS_ERR(chip->extcon))
		return dev_err_probe(chip->dev, PTR_ERR(chip->extcon),
					"failed to allocate extcon\n");

	ret = devm_extcon_dev_register(chip->dev, chip->extcon);
	if (ret)
		return dev_err_probe(chip->dev, ret, "failed to register extcon\n");

	ret = devm_add_action_or_reset(chip->dev, eud_role_switch_release, chip);
	if (ret)
		return ret;

	chip->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(chip->base))
		return PTR_ERR(chip->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return -ENODEV;
	chip->mode_mgr = res->start;

	chip->irq = platform_get_irq(pdev, 0);
	if (chip->irq < 0)
		return chip->irq;

	ret = devm_request_threaded_irq(&pdev->dev, chip->irq, handle_eud_irq,
			handle_eud_irq_thread, IRQF_ONESHOT, NULL, chip);
	if (ret)
		return dev_err_probe(chip->dev, ret, "failed to allocate irq\n");

	enable_irq_wake(chip->irq);

	/* Match the factory driver: inherit a secure EUD state left by the
	 * bootloader, but do not enable the block from Linux by default. */
	if (eud_hw_enabled(chip)) {
		writel(EUD_ENABLE, chip->base + EUD_REG_CSR_EUD_EN);
		writel(EUD_INT_VBUS | EUD_INT_SAFE_MODE,
			chip->base + EUD_REG_INT1_EN_MASK);
		extcon_set_state_sync(chip->extcon, EXTCON_USB, true);
		extcon_set_state_sync(chip->extcon, EXTCON_CHG_USB_SDP, true);
		chip->enabled = true;
	}

	platform_set_drvdata(pdev, chip);

	return 0;
}

static void eud_remove(struct platform_device *pdev)
{
	struct eud_chip *chip = platform_get_drvdata(pdev);

	if (chip->enabled)
		disable_eud(chip);

	device_init_wakeup(&pdev->dev, false);
	disable_irq_wake(chip->irq);
}

static const struct of_device_id eud_dt_match[] = {
	{ .compatible = "qcom,eud" },
	{ }
};
MODULE_DEVICE_TABLE(of, eud_dt_match);

static struct platform_driver eud_driver = {
	.probe	= eud_probe,
	.remove = eud_remove,
	.driver	= {
		.name = "qcom_eud",
		.dev_groups = eud_groups,
		.of_match_table = eud_dt_match,
	},
};
module_platform_driver(eud_driver);

MODULE_DESCRIPTION("QTI EUD driver");
MODULE_LICENSE("GPL v2");
