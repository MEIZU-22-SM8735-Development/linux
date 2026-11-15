// SPDX-License-Identifier: GPL-2.0-only
/*
 * Configure the WCN PDC through AOP QMP before Bluetooth/WLAN probe.
 * Factory DT entries include a declarative "ss" field which is omitted
 * from wire messages. The always-on power domain orders consumers after
 * all QMP sends have completed.
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

/* The domain enforces probe ordering and has no runtime power state. */
static struct generic_pm_domain m2582_pdc_domain = {
	.name = "m2582-pdc-init",
	.flags = GENPD_FLAG_ALWAYS_ON,
};

/*
 * Convert factory PDC entries to QMP messages, omitting the declarative
 * ss field. Reject malformed entries before sending them to AOP.
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
	}

	/* Publish the provider only after all PDC messages have been sent. */
	ret = pm_genpd_init(&m2582_pdc_domain, NULL, false);
	if (ret)
		goto out_put;

	ret = of_genpd_add_provider_simple(dev->of_node, &m2582_pdc_domain);
	if (ret) {
		dev_err(dev, "failed to add the power domain provider: %d\n",
			ret);
		pm_genpd_remove(&m2582_pdc_domain);
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
	pm_genpd_remove(&m2582_pdc_domain);
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
