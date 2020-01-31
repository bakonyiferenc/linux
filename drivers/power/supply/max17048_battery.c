/*
 *  max17048_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2012 Nvidia Cooperation
 *  Chandler Zhang <chazhang@nvidia.com>
 *  Syed Rafiuddin <srafiuddin@nvidia.com>
 *
 *  Copyright (C) 2013 LGE Inc.
 *  ChoongRyeol Lee <choongryeol.lee@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/power/max17048_battery.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/suspend.h>
#include <linux/syscalls.h>
#include <linux/completion.h>

#define MODE_REG      0x06
#define VCELL_REG     0x02
#define SOC_REG       0x04
#define VERSION_REG   0x08
#define HIBRT_REG     0x0A
#define CONFIG_REG    0x0C
#define VALRT_REG     0x14
#define CRATE_REG     0x16
#define VRESET_REG    0x18
#define STATUS_REG    0x1A

#define CFG_ALRT_MASK    0x0020
#define CFG_ATHD_MASK    0x001F
#define CFG_ALSC_MASK    0x0040
#define CFG_RCOMP_MASK    0xFF00
#define CFG_RCOMP_SHIFT    8
#define CFG_ALSC_SHIFT   6
#define STAT_RI_MASK     0x0100
#define STAT_CLEAR_MASK  0xFF00

#define MAX17048_VERSION_11    0x11
#define MAX17048_VERSION_12    0x12

struct max17048_chip {
	struct i2c_client *client;
	struct power_supply *batt_psy;
	struct power_supply *ac_psy;
	struct max17048_platform_data *pdata;
	struct dentry *dent;
	struct notifier_block pm_notifier;
	struct delayed_work monitor_work;
	int status;
	s16 crate;
	int vcell;
	int soc;
	int capacity_level;
	struct mutex lock;
	int alert_gpio;
	int alert_irq;
	int alert_threshold;
	int max_mvolt;
	int min_mvolt;
	int full_soc;
	int empty_soc;
	int batt_tech;
	int fcc_mah;
	int voltage;
	int chg_state;
	int batt_health;
	int poll_interval_ms;
};

static struct max17048_chip *ref;
struct completion monitor_work_done;
static int max17048_clear_interrupt(struct max17048_chip *chip);
static int max17048_get_prop_status(struct max17048_chip *chip);

static int bound_check(int max, int min, int val)
{
	val = max(min, val);
	val = min(max, val);
	return val;
}

static int max17048_write_word(struct i2c_client *client, int reg, u16 value)
{
	int ret;

	ret = i2c_smbus_write_word_data(client, reg, swab16(value));
	if (ret < 0)
		dev_err(&client->dev, "%s(): Failed in writing register"
					"0x%02x err %d\n", __func__, reg, ret);

	return ret;
}

static int max17048_read_word(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "%s(): Failed in reading register"
					"0x%02x err %d\n", __func__, reg, ret);
	else
		ret = (int)swab16((uint16_t)(ret & 0x0000ffff));

	return ret;
}

static int max17048_masked_write_word(struct i2c_client *client, int reg,
			       u16 mask, u16 val)
{
	s32 rc;
	u16 temp;

	temp = max17048_read_word(client, reg);
	if (temp < 0) {
		pr_err("max17048_read_word failed: reg=%03X, rc=%d\n",
				reg, temp);
		return temp;
	}

	if ((temp & mask) == (val & mask))
		return 0;

	temp &= ~mask;
	temp |= val & mask;
	rc = max17048_write_word(client, reg, temp);
	if (rc) {
		pr_err("max17048_write_word failed: reg=%03X, rc=%d\n",
				reg, rc);
		return rc;
	}

	return 0;
}

#ifdef CONFIG_DEBUG_FS
struct debug_reg {
	char  *name;
	u8  reg;
};

#define MAX17048_DEBUG_REG(x) {#x, x##_REG}

static struct debug_reg max17048_debug_regs[] = {
	MAX17048_DEBUG_REG(MODE),
	MAX17048_DEBUG_REG(VCELL),
	MAX17048_DEBUG_REG(SOC),
	MAX17048_DEBUG_REG(VERSION),
	MAX17048_DEBUG_REG(HIBRT),
	MAX17048_DEBUG_REG(CONFIG),
	MAX17048_DEBUG_REG(VALRT),
	MAX17048_DEBUG_REG(CRATE),
	MAX17048_DEBUG_REG(VRESET),
	MAX17048_DEBUG_REG(STATUS),
};

static int set_reg(void *data, u64 val)
{
	u32 addr = (u32) data;
	int ret;
	struct i2c_client *client = ref->client;

	ret = max17048_write_word(client, addr, (u16) val);

	return ret;
}

static int get_reg(void *data, u64 *val)
{
	u32 addr = (u32) data;
	int ret;
	struct i2c_client *client = ref->client;

	ret = max17048_read_word(client, addr);
	if (ret < 0)
		return ret;

	*val = ret;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%04llx\n");

#endif /* CONFIG_DEBUG_FS */

/* Using Quickstart instead of reset for Power Test
*  DO NOT USE THIS COMMAND ANOTHER SCENE.
*/
static int max17048_set_reset(struct max17048_chip *chip)
{
	max17048_write_word(chip->client, MODE_REG, 0x4000);
	pr_info("%s: Reset (Quickstart)\n", __func__);
	return 0;
}

static int max17048_get_capacity_from_soc(struct max17048_chip *chip)
{
	u8 buf[2];
	int batt_soc = 0;

	buf[0] = (chip->soc & 0x0000FF00) >> 8;
	buf[1] = (chip->soc & 0x000000FF);

	pr_debug("%s: SOC raw = 0x%x%x\n", __func__, buf[0], buf[1]);

	batt_soc = (((int)buf[0]*256)+buf[1])*19531; /* 0.001953125 */
	batt_soc = (batt_soc - (chip->empty_soc * 1000000))
			/ ((chip->full_soc - chip->empty_soc) * 10000);

	batt_soc = bound_check(100, 0, batt_soc);

	return batt_soc;
}

static int max17048_get_status(struct max17048_chip *chip)
{
	int status;

	status = max17048_read_word(chip->client, STATUS_REG);
	if (status < 0) {
		pr_err("%s: err %d\n", __func__, status);
		return status;
	} else {
		chip->status = status >> 8;
	}

	return 0;
}
/*
static int max17048_get_crate(struct max17048_chip *chip)
{
	int crate;

	crate = max17048_read_word(chip->client, CRATE_REG);
	if (crate < 0) {
		pr_err("%s: err %d\n", __func__, crate);
		return crate;
	} else {
		chip->crate = crate;
	}

	return 0;
}
*/
static int max17048_get_vcell(struct max17048_chip *chip)
{
	int vcell;

	vcell = max17048_read_word(chip->client, VCELL_REG);
	if (vcell < 0) {
		pr_err("%s: err %d\n", __func__, vcell);
		return vcell;
	} else {
		chip->vcell = vcell >> 4;
		chip->voltage = (chip->vcell * 5) >> 2;
	}

	return 0;
}

static int max17048_get_soc(struct max17048_chip *chip)
{
	int soc;

	soc = max17048_read_word(chip->client, SOC_REG);
	if (soc < 0) {
		pr_err("%s: err %d\n", __func__, soc);
		return soc;
	} else {
		chip->soc = soc;
		chip->capacity_level =
			max17048_get_capacity_from_soc(chip);
	}

	return 0;
}

static uint16_t max17048_get_version(struct max17048_chip *chip)
{
	return (uint16_t) max17048_read_word(chip->client, VERSION_REG);
}

static void max17048_work(struct work_struct *work)
{
	struct max17048_chip *chip =
		container_of(work, struct max17048_chip, monitor_work.work);
	int ret = 0;

	mutex_lock(&chip->lock);

	pr_debug("%s.\n", __func__);
	max17048_get_status(chip);
	max17048_get_vcell(chip);
	max17048_get_soc(chip);
	complete_all(&monitor_work_done);

	ret = max17048_clear_interrupt(chip);
	if (ret < 0)
		pr_err("%s : error clear alert irq register.\n", __func__);

	if (chip->capacity_level == 0) {
		schedule_delayed_work(&chip->monitor_work,
				msecs_to_jiffies(chip->poll_interval_ms));
	} else {
		pr_info("%s: rsoc=0x%04X rvcell=0x%04X soc=%d"\
			" v_mv=%d\n", __func__,
				chip->soc, chip->vcell,
				chip->capacity_level, chip->voltage);
	}

	mutex_unlock(&chip->lock);
}
static irqreturn_t max17048_interrupt_handler(int irq, void *data)
{
	struct max17048_chip *chip = data;

	pr_debug("%s : interupt occured\n", __func__);
	schedule_delayed_work(&chip->monitor_work, 0);

	return IRQ_HANDLED;
}

static int max17048_clear_interrupt(struct max17048_chip *chip)
{
	int ret;

	pr_debug("%s.\n", __func__);

	ret = max17048_masked_write_word(chip->client,
			CONFIG_REG, CFG_ALRT_MASK, 0);
	if (ret < 0) {
		pr_err("%s: failed to clear alert status bit\n", __func__);
		return ret;
	}

	ret = max17048_masked_write_word(chip->client,
			STATUS_REG, STAT_CLEAR_MASK, 0);
	if (ret < 0) {
		pr_err("%s: failed to clear status reg\n", __func__);
		return ret;
	}

	return 0;
}

static int max17048_set_athd_alert(struct max17048_chip *chip, int level)
{
	int ret;

	pr_debug("%s.\n", __func__);

	level = bound_check(32, 1, level);
	level = 32 - level;

	ret = max17048_masked_write_word(chip->client,
			CONFIG_REG, CFG_ATHD_MASK, level);
	if (ret < 0)
		pr_err("%s: failed to set athd alert\n", __func__);

	return ret;
}

static int max17048_set_alsc_alert(struct max17048_chip *chip, bool enable)
{
	int ret;
	u16 val;

	pr_debug("%s. with %d\n", __func__, enable);

	val = (u16)(!!enable << CFG_ALSC_SHIFT);

	ret = max17048_masked_write_word(chip->client,
			CONFIG_REG, CFG_ALSC_MASK, val);
	if (ret < 0)
		pr_err("%s: failed to set alsc alert\n", __func__);

	return ret;
}

static ssize_t max17048_show_status(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17048_chip *chip = i2c_get_clientdata(client);

	if (!chip)
		return -ENODEV;

        return sprintf(buf, "%s\n", "TODO");
}

static ssize_t max17048_store_status(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17048_chip *chip = i2c_get_clientdata(client);

	if (!chip)
		return -ENODEV;

	if (strncmp(buf, "reset", 5) == 0) {
		max17048_set_reset(chip);
		schedule_delayed_work(&chip->monitor_work, 0);
	} else {
		return -EINVAL;
	}

	return count;
}
DEVICE_ATTR(fuelrst, 0664, max17048_show_status, max17048_store_status);

static int max17048_parse_dt(struct device *dev,
		struct max17048_chip *chip)
{
	struct device_node *dev_node = dev->of_node;
	int ret = 0;

	chip->alert_gpio = of_get_named_gpio(dev_node,
			"max17048,alert_gpio", 0);
	if (chip->alert_gpio < 0) {
		pr_err("failed to get stat-gpio.\n");
		ret = chip->alert_gpio;
		goto out;
	}
	ret = of_property_read_u32(dev_node, "max17048,alert_threshold",
				&chip->alert_threshold);
	if (ret) {
		pr_err("%s: failed to read alert_threshold\n", __func__);
		goto out;
	}

	ret = of_property_read_u32(dev_node, "max17048,max-mvolt",
				   &chip->max_mvolt);
	if (ret) {
		pr_err("%s: failed to read max voltage\n", __func__);
		goto out;
	}

	ret = of_property_read_u32(dev_node, "max17048,min-mvolt",
				   &chip->min_mvolt);
	if (ret) {
		pr_err("%s: failed to read min voltage\n", __func__);
		goto out;
	}

	ret = of_property_read_u32(dev_node, "max17048,full-soc",
				   &chip->full_soc);
	if (ret) {
		pr_err("%s: failed to read full soc\n", __func__);
		goto out;
	}

	ret = of_property_read_u32(dev_node, "max17048,empty-soc",
				   &chip->empty_soc);
	if (ret) {
		pr_err("%s: failed to read empty soc\n", __func__);
		goto out;
	}

	ret = of_property_read_u32(dev_node, "max17048,batt-tech",
				   &chip->batt_tech);
	if (ret) {
		pr_err("%s: failed to read batt technology\n", __func__);
		goto out;
	}

	ret = of_property_read_u32(dev_node, "max17048,fcc-mah",
				   &chip->fcc_mah);
	if (ret) {
		pr_err("%s: failed to read batt fcc\n", __func__);
		goto out;
	}

	pr_info("%s: alert_thres = %d full_soc = %d empty_soc = %d\n",
			__func__, chip->alert_threshold,
			chip->full_soc, chip->empty_soc);

out:
	return ret;
}

static int max17048_get_prop_status(struct max17048_chip *chip)
{
	int ret;
	int cap;

	mutex_lock(&chip->lock);
	cap = chip->capacity_level;
	mutex_unlock(&chip->lock);

        ret = power_supply_am_i_supplied(chip->batt_psy);
        if (ret < 0 || cap == -EINVAL) 
                return POWER_SUPPLY_STATUS_UNKNOWN;
        if (ret == 0) 
                return POWER_SUPPLY_STATUS_DISCHARGING;
	if (cap >= 95)
		return POWER_SUPPLY_STATUS_FULL;
	return POWER_SUPPLY_STATUS_CHARGING;
/* Using CRATE_REG for charging detecion is not fast enough.
	if (chip->capacity_level == -EINVAL)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	mutex_lock(&chip->lock);
	max17048_get_crate(chip);
	mutex_unlock(&chip->lock);
	if (chip->crate < 0) {
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
	if (chip->capacity_level >= 95) {
		return POWER_SUPPLY_STATUS_FULL;
	}
	if (chip->crate > 0) {
		return POWER_SUPPLY_STATUS_CHARGING;
	}
	return POWER_SUPPLY_STATUS_UNKNOWN;
*/
}

static int max17048_get_prop_health(struct max17048_chip *chip)
{
	return POWER_SUPPLY_HEALTH_UNKNOWN; // TODO
}

static int max17048_get_prop_vbatt_uv(struct max17048_chip *chip)
{
	mutex_lock(&chip->lock); // Perhaps not needed.
	max17048_get_vcell(chip);
	mutex_unlock(&chip->lock);
	return chip->voltage * 1000;
}

static int max17048_get_prop_present(struct max17048_chip *chip)
{
	/*FIXME - need to implement */
	return true;
}

static int max17048_get_prop_capacity(struct max17048_chip *chip)
{
	int ret;

	if (chip->capacity_level == -EINVAL)
		return -EINVAL;

	ret = wait_for_completion_timeout(&monitor_work_done,
					msecs_to_jiffies(500));
	if (!ret)
		pr_err("%s: timeout monitor work done\n", __func__);

	return chip->capacity_level;
}

static enum power_supply_property max17048_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static int max17048_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17048_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = max17048_get_prop_status(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = max17048_get_prop_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = max17048_get_prop_present(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = chip->batt_tech;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->max_mvolt * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->min_mvolt * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = max17048_get_prop_vbatt_uv(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = max17048_get_prop_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->fcc_mah * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void max17048_external_power_changed(struct power_supply *psy)
{
	struct max17048_chip *chip = power_supply_get_drvdata(psy);
	int chg_state;
	int batt_health;

	chg_state = max17048_get_prop_status(chip);
	batt_health = max17048_get_prop_health(chip);

	if ((chip->chg_state ^ chg_state)
			||(chip->batt_health ^ batt_health)) {
		chip->chg_state = chg_state;
		chip->batt_health = batt_health;
		pr_info("%s: power supply changed state = %d health = %d",
					__func__, chg_state, batt_health);
		power_supply_changed(psy);
	}
}

static int max17048_create_debugfs_entries(struct max17048_chip *chip)
{
#ifdef CONFIG_DEBUG_FS
	int i;

	chip->dent = debugfs_create_dir("max17048", NULL);
	if (IS_ERR(chip->dent)) {
		pr_err("max17048 driver couldn't create debugfs dir\n");
		return -EFAULT;
	}

	for (i = 0 ; i < ARRAY_SIZE(max17048_debug_regs) ; i++) {
		char *name = max17048_debug_regs[i].name;
		u32 reg = max17048_debug_regs[i].reg;
		struct dentry *file;

		file = debugfs_create_file(name, 0644, chip->dent,
					(void *) reg, &reg_fops);
		if (IS_ERR(file)) {
			pr_err("debugfs_create_file %s failed.\n", name);
			return -EFAULT;
		}
	}
#endif /* CONFIG_DEBUG_FS */
	return 0;
}

static void max17048_remove_debugfs_entries(struct max17048_chip *chip)
{
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(chip->dent);
#endif
}

static int max17048_hw_init(struct max17048_chip *chip)
{
	int ret;

	ret = max17048_masked_write_word(chip->client,
			STATUS_REG, STAT_RI_MASK, 0);
	if (ret) {
		pr_err("%s: failed to clear ri bit\n", __func__);
		return ret;
	}

	ret = max17048_set_athd_alert(chip, chip->alert_threshold);
	if (ret) {
		pr_err("%s: failed to set athd alert threshold\n", __func__);
		return ret;
	}
/*
	ret = max17048_set_alsc_alert(chip, true);
	if (ret) {
		pr_err("%s: failed to set alsc alert\n", __func__);
		return ret;
	}
*/
	return 0;
}

static int max17048_pm_notifier(struct notifier_block *notifier,
			unsigned long pm_event, void *unused)
{
	struct max17048_chip *chip = container_of(notifier,
				struct max17048_chip, pm_notifier);

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		max17048_set_alsc_alert(chip, false);
		cancel_delayed_work_sync(&chip->monitor_work);
		break;
	case PM_POST_SUSPEND:
		reinit_completion(&monitor_work_done);
		schedule_delayed_work(&chip->monitor_work,
					msecs_to_jiffies(200));
		max17048_set_alsc_alert(chip, true);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static const struct power_supply_desc max17048_psy_desc = {
	.name		= "max17048-battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= max17048_get_property,
	.external_power_changed	= max17048_external_power_changed,
	.properties	= max17048_battery_props,
	.num_properties	= ARRAY_SIZE(max17048_battery_props),
};

static int max17048_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct max17048_chip *chip;
	const struct power_supply_desc *max17048_desc = &max17048_psy_desc;
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	int ret;
	uint16_t version;

	pr_info("%s: start\n", __func__);

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_WORD_DATA)) {
		pr_err("%s: i2c_check_functionality fail\n", __func__);
		return -EIO;
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	if (&client->dev.of_node) {
		ret = max17048_parse_dt(&client->dev, chip);
		if (ret) {
			pr_err("%s: failed to parse dt\n", __func__);
			goto  error;
		}
	} else {
		chip->pdata = client->dev.platform_data;
	}

	i2c_set_clientdata(client, chip);
	psy_cfg.drv_data = chip;
	psy_cfg.of_node = dev->of_node;

	version = max17048_get_version(chip);
	dev_info(&client->dev, "MAX17048 Fuel-Gauge Ver 0x%x\n", version);
	if (version != MAX17048_VERSION_11 &&
	    version != MAX17048_VERSION_12) {
		pr_err("%s: Not supported version: 0x%x\n", __func__,
				version);
		ret = -ENODEV;
		goto error;
	}
	init_completion(&monitor_work_done);
	chip->capacity_level = -EINVAL;
	ref = chip;
	chip->batt_psy = devm_power_supply_register(dev, max17048_desc,
						   &psy_cfg);
	if (IS_ERR(chip->batt_psy)) {
		dev_err(&client->dev, "failed: power supply register\n");
		ret = PTR_ERR(chip->batt_psy);
		goto error;
	}

	mutex_init(&chip->lock);
	INIT_DELAYED_WORK(&chip->monitor_work, max17048_work);

	ret = devm_gpio_request_one(dev, chip->alert_gpio, GPIOF_DIR_IN,
				"max17048_alert");
	if (ret) {
		pr_err("%s: GPIO Request Failed : return %d\n",
				__func__, ret);
		goto error;
	}

	chip->alert_irq = gpio_to_irq(chip->alert_gpio);
	if (chip->alert_irq < 0) {
		pr_err("%s: failed to get alert irq\n", __func__);
		goto error;
	}

	ret = devm_request_threaded_irq(dev, chip->alert_irq, NULL,
				max17048_interrupt_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"MAX17048_Alert", chip);
	if (ret) {
		pr_err("%s: IRQ Request Failed : return %d\n",
				__func__, ret);
		goto error;
	}

	ret = enable_irq_wake(chip->alert_irq);
	if (ret) {
		pr_err("%s: set irq to wakeup source failed.\n", __func__);
		goto error;
	}

	disable_irq(chip->alert_irq);

	ret = device_create_file(dev, &dev_attr_fuelrst);
	if (ret) {
		pr_err("%s: fuelrst creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_fuelrst;
	}

	ret = max17048_create_debugfs_entries(chip);
	if (ret) {
		pr_err("max17048_create_debugfs_entries failed\n");
		goto err_create_debugfs;
	}

	ret = max17048_hw_init(chip);
	if (ret) {
		pr_err("%s: failed to init hw.\n", __func__);
		goto err_hw_init;
	}

	chip->pm_notifier.notifier_call = max17048_pm_notifier;
	ret = register_pm_notifier(&chip->pm_notifier);
	if (ret) {
		pr_err("%s: failed to register pm notifier\n", __func__);
		goto err_hw_init;
	}

	schedule_delayed_work(&chip->monitor_work, 0);
	enable_irq(chip->alert_irq);

	pr_info("%s: done\n", __func__);
	return 0;

err_hw_init:
	max17048_remove_debugfs_entries(chip);
err_create_debugfs:
	device_remove_file(&client->dev, &dev_attr_fuelrst);
err_create_file_fuelrst:
	disable_irq_wake(chip->alert_irq);
error:
	ref = NULL;
	return ret;
}

static int max17048_remove(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	unregister_pm_notifier(&chip->pm_notifier);
	max17048_remove_debugfs_entries(chip);
	device_remove_file(&client->dev, &dev_attr_fuelrst);
	disable_irq_wake(chip->alert_irq);
	ref = NULL;

	return 0;
}

static struct of_device_id max17048_match_table[] = {
	{ .compatible = "maxim,max17048", },
	{ },
};

static const struct i2c_device_id max17048_id[] = {
	{ "max17048", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max17048_id);

static struct i2c_driver max17048_i2c_driver = {
	.driver	= {
		.name = "max17048",
		.owner = THIS_MODULE,
		.of_match_table = max17048_match_table,
	},
	.probe = max17048_probe,
	.remove = max17048_remove,
	.id_table = max17048_id,
};

static int __init max17048_init(void)
{
	return i2c_add_driver(&max17048_i2c_driver);
}
module_init(max17048_init);

static void __exit max17048_exit(void)
{
	i2c_del_driver(&max17048_i2c_driver);
}
module_exit(max17048_exit);

MODULE_DESCRIPTION("MAX17048 Fuel Gauge");
MODULE_LICENSE("GPL");
