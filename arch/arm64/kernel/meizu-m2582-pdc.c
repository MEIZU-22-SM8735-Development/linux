// SPDX-License-Identifier: GPL-2.0-only
/*
 * Meizu 22 (M2582): the WCN's PDC init table, sent to the AOP before the
 * Bluetooth/WLAN chip's rails come up.
 *
 * The vendor's Bluetooth power driver does more than enable rails.  Its
 * bt_aop_pdc_reconfig() (.text+0x30ac, 144 bytes, in the phone's own
 * vendor_dlkm/lib/modules/btpower.ko) reads a qcom,pdc_init_table property off
 * its device tree node and sends every entry to the AOP over QMP.  The strings
 * in that module say exactly what it does and are the reason this file exists:
 *
 *	"qcom,pdc_init_table"                      the property it reads
 *	"Failed to get PDC Init Table"
 *	"PDC Init table configured"                the success path
 *	"Failed to reconfig BT WLAN PDC, err = %d"
 *	"{class: wlan_pdc, res: %s.%s, %s: %d}"    the message it builds
 *	"failed to get qmp" / "Failed to send AOP qmp xmsg: %s"
 *
 * Note what that template does *not* contain: the device tree entries carry an
 * "ss: rf" / "ss: bb" field, but the message put on the wire does not -- the
 * property is the declarative form, and the driver re-emits class, res and the
 * single value field.  This file reproduces that rather than sending the device
 * tree string verbatim, because the wire format is the one the vendor itself
 * exercised on this phone.
 *
 * What the table is for: class "wlan_pdc" is the WCN's power domain
 * controller.  The entries set the s3b supervision thresholds (upval 976,
 * dwnval 616 mV), release the chip's memory-domain retention (the s1g, l18b,
 * b1b and l4k ".m" domains, enable 0), set l2g.m dwnval 3, and finally enable
 * the PDC.  Without it the WCN's internal power domains are in an undefined
 * state while the chip is powered and released from reset, which is the shape
 * of the failure this board shows: it goes down about a second after the
 * Bluetooth probe with no kernel message at all.
 *
 * The table below is the WLAN node's superset, from the phone's own runtime
 * device tree (artifacts/meizu22/android-live-ON/live-fdt.dts:34673).  The
 * Bluetooth node carries a three-entry subset of it (:34455).  On Android both
 * cnss2 and btpower run and both tables are applied; a Bluetooth-only boot of
 * ours has applied neither, so the superset is the one to send.
 *
 * Ordering is the point of this driver, and it is enforced structurally --
 * but NOT by fw_devlink, which cannot help here: its link creation runs in an
 * init-time pass over the tree, long before the QUP UART driver creates this
 * serdev child, so there is nothing for it to order at that point.  What
 * actually orders the two:
 *
 *  - serdev_drv_probe() calls dev_pm_domain_attach() before the client
 *    driver's probe (drivers/tty/serdev/core.c:398-408), and the generic power
 *    domain core answers -EPROBE_DEFER while the provider is not registered
 *    (drivers/pmdomain/core.c:3161).  This driver registers its (no-op,
 *    always-on) domain LAST, after the table has been sent, and hci_qca raises
 *    the controller's rails inside its own probe.  So the Bluetooth device
 *    cannot probe, and its rails cannot come up, before the table is on its
 *    way to the AOP.
 *    Do NOT remove the power-domains line on the Bluetooth node as
 *    "redundant": it is the only ordering mechanism between two otherwise
 *    unrelated platform drivers, and deleting it silently removes the
 *    guarantee.
 *  - The same reference makes the Bluetooth device defer rather than probe if
 *    this node is missing or gated off, which is the intended behaviour: its
 *    rails must not be raised without this table.
 *  - qmp_get() returns -EPROBE_DEFER until aoss_qmp has probed
 *    (drivers/soc/qcom/qcom_aoss.c:443-474), so the AOP side is ordered the
 *    same way, and a failure to reach the AOP is never mistaken for success.
 *
 * The rails themselves are already always-on before this runs (the WiFi work
 * made s1b/s2b/s3b always-on), which matches Android, where cnss2 raises the
 * shared rails before btpower sends its table.  The property that matters is
 * therefore "the table is sent before the Bluetooth probe", not "before the
 * rails".
 *
 * The node carries m2582,feature = "bt", the project's usual gate, so a boot
 * that does not ask for Bluetooth leaves the whole step out.
 */

#include <linux/err.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/soc/qcom/qcom_aoss.h>
#include <linux/string.h>

#define M2582_PDC_MAX_ENTRIES	16
#define M2582_PDC_ENTRY_MAX	128
/*
 * One message at a time, and never longer than what the mailbox can carry:
 * qmp_send() formats into a QMP_MSG_LEN (64, drivers/soc/qcom/qcom_aoss.c:48)
 * buffer, so a message that fits here but not there would be truncated inside
 * qmp_send() instead of being rejected here.  The longest entry in the table
 * this board ships is 43 characters.
 */
#define M2582_PDC_MSG_MAX	64

/*
 * The domain carries no state of its own: everything it exists for happens in
 * the probe below, before the Bluetooth device is allowed to probe.  It is
 * always on so that the power domain core never tries to power it down, and so
 * that attaching to it cannot fail the Bluetooth probe for an unrelated
 * reason.
 */
static struct generic_pm_domain m2582_pdc_domain = {
	.name = "m2582-pdc-init",
	.flags = GENPD_FLAG_ALWAYS_ON,
};

/*
 * Turn one table entry, as the vendor's device tree writes it:
 *
 *	" {class: wlan_pdc, ss: rf, res: s3b.v, upval: 976}"
 *
 * into what goes on the wire:
 *
 *	"{class: wlan_pdc, res: s3b.v, upval: 976}"
 *
 * Returns 0, or -EINVAL if the entry is not shaped like that.
 */
static int m2582_pdc_build_message(const char *entry, char *msg, size_t msg_len)
{
	char buf[M2582_PDC_ENTRY_MAX];
	const char *class = NULL, *res = NULL, *key = NULL;
	unsigned int value = 0;
	bool value_seen = false;
	char *p, *field, *brace;

	if (strscpy(buf, entry, sizeof(buf)) < 0)
		return -EINVAL;

	/* The entry is written as "{key: value, key: value}": the braces
	 * belong to the declarative form, not to the first field name or the
	 * last value.
	 */
	p = strim(buf);
	if (*p == '{')
		p = strim(p + 1);

	brace = strrchr(p, '}');
	if (brace)
		*brace = '\0';

	while ((field = strsep(&p, ","))) {
		const char *name, *val;
		char *colon;

		field = strim(field);
		if (!*field)		/* a stray separator */
			continue;

		colon = strchr(field, ':');
		if (!colon)
			return -EINVAL;
		*colon = '\0';

		name = strim(field);
		val = strim(colon + 1);

		if (!strcmp(name, "class")) {
			class = val;
		} else if (!strcmp(name, "res")) {
			res = val;
		} else if (!strcmp(name, "ss")) {
			/* Declarative only: the vendor parses this field and
			 * does not put it on the wire.
			 */
		} else if (*val) {
			/* The one field carrying a number. */
			if (kstrtouint(val, 10, &value))
				return -EINVAL;
			key = name;
			value_seen = true;
		}
	}

	if (!class || !res || !key || !value_seen)
		return -EINVAL;

	/* The vendor's template is "{class: %s, res: %s.%s, %s: %d}", whose
	 * %s.%s is the res field split at the dot; the text it produces is what
	 * the entry already spells, so res is kept whole.
	 */
	if (snprintf(msg, msg_len, "{class: %s, res: %s, %s: %u}",
		     class, res, key, value) >= msg_len)
		return -EINVAL;

	return 0;
}

static int m2582_pdc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *table[M2582_PDC_MAX_ENTRIES];
	struct qmp *qmp;
	char msg[M2582_PDC_MSG_MAX];
	int entries, i, ret;

	entries = device_property_string_array_count(dev,
						     "qcom,pdc-init-table");
	if (entries < 0) {
		/* No table is not a reason to fail: the Bluetooth node defers
		 * on this driver having probed, so returning an error here
		 * would keep it out of the boot entirely.  Say so loudly and
		 * register the provider anyway.
		 */
		dev_err(dev, "failed to get PDC Init Table: %d\n", entries);
		entries = 0;
	} else if (entries > M2582_PDC_MAX_ENTRIES) {
		dev_err(dev, "PDC init table too long: %d entries\n", entries);
		return -EINVAL;
	}

	qmp = qmp_get(dev);
	if (IS_ERR(qmp)) {
		/* -EPROBE_DEFER is the normal path when aoss_qmp has not probed
		 * yet, and it is what orders this step after the AOP.
		 */
		return dev_err_probe(dev, PTR_ERR(qmp), "failed to get qmp\n");
	}

	if (entries) {
		ret = device_property_read_string_array(dev,
							"qcom,pdc-init-table",
							table, entries);
		if (ret < 0) {
			dev_err(dev, "failed to read the PDC Init Table: %d\n",
				ret);
			goto out_put;
		}
	}

	for (i = 0; i < entries; i++) {
		ret = m2582_pdc_build_message(table[i], msg, sizeof(msg));
		if (ret) {
			dev_err(dev, "PDC init entry %d is malformed: \"%s\"\n",
				i, table[i]);
			goto out_put;
		}

		/* qmp_send() takes a format string, so the built message is
		 * never passed as one.
		 */
		ret = qmp_send(qmp, "%s", msg);
		if (ret) {
			dev_err(dev, "failed to reconfig BT WLAN PDC, err = %d\n",
				ret);
			goto out_put;
		}

		dev_dbg(dev, "PDC entry %d: %s\n", i, msg);
	}

	/*
	 * "configured" means the mailbox took every message.  qmp_send() waits
	 * for the message RAM to be consumed and reports the transport result,
	 * but it never inspects an acknowledgment value, so an AOP that
	 * accepted the message and then ignored it is indistinguishable from
	 * success in this log.  The evidence that this step worked is
	 * behavioural: whether the Bluetooth hard reset is gone.
	 */
	dev_info(dev, "PDC Init table configured (%d entries)\n", entries);

	/*
	 * Registered last: this is the reference the Bluetooth node's
	 * power-domains resolves to, and the table above has to be on its way
	 * to the AOP before anything can start the Bluetooth probe.
	 */
	ret = of_genpd_add_provider_simple(dev->of_node, &m2582_pdc_domain);
	if (ret) {
		dev_err(dev, "failed to add the power domain provider: %d\n",
			ret);
		goto out_put;
	}

	platform_set_drvdata(pdev, qmp);

	return 0;

out_put:
	qmp_put(qmp);
	return ret;
}

static void m2582_pdc_remove(struct platform_device *pdev)
{
	of_genpd_del_provider(pdev->dev.of_node);
	qmp_put(platform_get_drvdata(pdev));
}

static const struct of_device_id m2582_pdc_of_match[] = {
	{ .compatible = "meizu,m2582-pdc-init" },
	{}
};
MODULE_DEVICE_TABLE(of, m2582_pdc_of_match);

static struct platform_driver m2582_pdc_driver = {
	.probe = m2582_pdc_probe,
	.remove = m2582_pdc_remove,
	.driver = {
		.name = "m2582-pdc-init",
		.of_match_table = m2582_pdc_of_match,
	},
};
module_platform_driver(m2582_pdc_driver);

MODULE_DESCRIPTION("Meizu 22 (M2582) WCN PDC init table over AOP/QMP");
MODULE_LICENSE("GPL");
