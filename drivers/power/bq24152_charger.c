/*
 * bq2415x charger driver
 *
 * Copyright (C) 2011-2012  Pali Rohár <pali.rohar@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * Datasheets:
 * http://www.ti.com/product/bq24150
 * http://www.ti.com/product/bq24150a
 * http://www.ti.com/product/bq24152
 * http://www.ti.com/product/bq24153
 * http://www.ti.com/product/bq24153a
 * http://www.ti.com/product/bq24155
 */
//#define DEBUG
#define pr_fmt(fmt)	"Ti-CHARGER: %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/power/bq24152_charger.h>

#ifdef CONFIG_HUAWEI_DSM
#include <linux/dsm_pub.h>
#endif
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#include <linux/power/bq27510_bms.h>
#include <linux/batterydata-lib.h>
#define STATUS_FULL 4
jeita_spec batt_jeita_param = 
{
	.normal	= {100,420,1250,4350},
	.hot.t_high = -1
};

static bool user_ctl_status = true;
static jeita_entry *jeita_now = &batt_jeita_param.normal;
static jeita_entry *jeita_last = &batt_jeita_param.normal;
static int migration = 0;
extern int hot_design_current;

static void bq2415x_set_appropriate_jeita(struct bq2415x_device *bq);
static void select_jeita(struct bq2415x_device *bq);
extern int hot_design_current;
/* timeout for resetting chip timer */
#define BQ2415X_TIMER_TIMEOUT		2
#define HYSTERTSIS_TIME 20  //sec
#define USB_CURRENT_LIMIT	540
#define USB_CHARGE_CURRENT	550
#define CURRENT_LIMIT_INIT		100
#define CURRENT_LIMIT_STEP_ONE	500
#define CURRENT_LIMIT_STEP_TWO	800
#define CURRENT_THRESHOLD_MA	1800
#define MAX_CHARGE_CURRENT	1250
#define BATTERY_VOL_THRESHOLD	3750
#define BQ24152_REG_0_FAULT_MASK	0x07
#define BQ24152_REG_0_STAT_MASK		0x30
#define BQ24152_CHG_STAT_FAULT	3
#define POOR_INPUT_FAULT_STATUS	3

#define BQ2415X_REG_STATUS		0x00
#define BQ2415X_REG_CONTROL		0x01
#define BQ2415X_REG_VOLTAGE		0x02
#define BQ2415X_REG_VENDER		0x03
#define BQ2415X_REG_CURRENT		0x04

/* reset state for all registers */
#define BQ2415X_RESET_STATUS		BIT(6)
#define BQ2415X_RESET_CONTROL		(BIT(4)|BIT(5))
#define BQ2415X_RESET_VOLTAGE		(BIT(1)|BIT(3))
#define BQ2415X_RESET_CURRENT		(BIT(0)|BIT(3)|BIT(7))

/* status register */
#define BQ2415X_BIT_TMR_RST		7
#define BQ2415X_BIT_OTG			7
#define BQ2415X_BIT_EN_STAT		6
#define BQ2415X_MASK_STAT		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_STAT		4
#define BQ2415X_BIT_BOOST		3
#define BQ2415X_MASK_FAULT		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_FAULT		0

/* control register */
#define BQ2415X_MASK_LIMIT		(BIT(6)|BIT(7))
#define BQ2415X_SHIFT_LIMIT		6
#define BQ2415X_MASK_VLOWV		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_VLOWV		4
#define BQ2415X_BIT_TE			3
#define BQ2415X_BIT_CE			2
#define BQ2415X_BIT_HZ_MODE		1
#define BQ2415X_BIT_OPA_MODE		0

/* voltage register */
#define BQ2415X_MASK_VO		(BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VO		2
#define BQ2415X_BIT_OTG_PL		1
#define BQ2415X_BIT_OTG_EN		0

/* vender register */
#define BQ2415X_MASK_VENDER		(BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VENDER		5
#define BQ2415X_MASK_PN			(BIT(3)|BIT(4))
#define BQ2415X_SHIFT_PN		3
#define BQ2415X_MASK_REVISION		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_REVISION		0

/* current register */
#define BQ2415X_MASK_RESET		BIT(7)
#define BQ2415X_MASK_VI_CHRG		(BIT(4)|BIT(5)|BIT(6))
#define BQ2415X_SHIFT_VI_CHRG		4
/* N/A					BIT(3) */
#define BQ2415X_MASK_VI_TERM		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_VI_TERM		0


//move code to bq24152_charger.h

static char *bq2415x_chip_name[] = {
	"unknown",
	"bq24150",
	"bq24150a",
	"bq24151",
	"bq24151a",
	"bq24152",
	"bq24153",
	"bq24153a",
	"bq24155",
	"bq24156",
	"bq24156a",
	"bq24158",
};

//move code to bq24152_charger.h

/* each registered chip must have unique id */
static DEFINE_IDR(bq2415x_id);

static DEFINE_MUTEX(bq2415x_id_mutex);
static DEFINE_MUTEX(bq2415x_timer_mutex);
static DEFINE_MUTEX(bq2415x_i2c_mutex);

#ifdef CONFIG_HUAWEI_DSM
extern struct dsm_client *charger_dclient;
extern struct bq2415x_device *bq_device;
extern struct qpnp_lbc_chip *g_lbc_chip;
void bq2415x_dump_regs(struct dsm_client *dclient);
extern int dump_registers_and_adc(struct dsm_client *dclient, struct qpnp_lbc_chip *chip, int type);
#endif
extern int is_usb_chg_exist(void);
extern int qpnp_lbc_is_in_vin_min_loop(struct qpnp_lbc_chip *chip);

static int poor_input_enable = 0;

/**** i2c read functions ****/

/* read value from register */
static int bq2415x_i2c_read(struct bq2415x_device *bq, u8 reg)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[2];
	u8 val;
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &val;
	msg[1].len = sizeof(val);

	mutex_lock(&bq2415x_i2c_mutex);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&bq2415x_i2c_mutex);

	if (ret < 0)
		return ret;

	return val;
}

#ifdef CONFIG_HUAWEI_DSM
void bq2415x_dump_regs(struct dsm_client *dclient)
{
	int ret = 0;
	u8 reg = 0;

	dsm_client_record(dclient, "[BQ24152] regs:\n");
	pr_info("[BQ24152] regs:\n");
	if(bq_device){
		for(reg = BQ2415X_REG_STATUS; reg <= BQ2415X_REG_CURRENT; reg++){
			ret = bq2415x_i2c_read(bq_device, reg);
			dsm_client_record(dclient, "0x%x, 0x%x\n", reg, ret);
			pr_info("0x%x, 0x%x\n", reg, ret);
		}
	}
}
#endif

/*===========================================
FUNCTION: get_bq2415x_reg_values
DESCRIPTION: to get bq2415x Reg0 to Reg4 values for NFF log feature
IPNUT: reg number
RETURN:	a int value, the value of bq2415x reg
=============================================*/
int get_bq2415x_reg_values(u8 reg)
{
	int ret = 0;
	if(bq_device){
		ret = bq2415x_i2c_read(bq_device, reg);
		return ret;
	}else{
		pr_info("bq_device is not init, do nothing!\n");
		return -1;
	}
}
EXPORT_SYMBOL(get_bq2415x_reg_values);

/* read value from register, apply mask and right shift it */
static int bq2415x_i2c_read_mask(struct bq2415x_device *bq, u8 reg,
				 u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = bq2415x_i2c_read(bq, reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}

/* read value from register and return one specified bit */
static int bq2415x_i2c_read_bit(struct bq2415x_device *bq, u8 reg, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return bq2415x_i2c_read_mask(bq, reg, BIT(bit), bit);
}

/**** i2c write functions ****/

/* write value to register */
static int bq2415x_i2c_write(struct bq2415x_device *bq, u8 reg, u8 val)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;

	data[0] = reg;
	data[1] = val;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = ARRAY_SIZE(data);

	mutex_lock(&bq2415x_i2c_mutex);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&bq2415x_i2c_mutex);

	/* i2c_transfer returns number of messages transferred */
	if (ret < 0)
		return ret;
	else if (ret != 1)
		return -EIO;

	return 0;
}

/* read value from register, change it with mask left shifted and write back */
static int bq2415x_i2c_write_mask(struct bq2415x_device *bq, u8 reg, u8 val,
				  u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = bq2415x_i2c_read(bq, reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val << shift;

	return bq2415x_i2c_write(bq, reg, ret);
}

/* change only one bit in register */
static int bq2415x_i2c_write_bit(struct bq2415x_device *bq, u8 reg,
				 bool val, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return bq2415x_i2c_write_mask(bq, reg, val, BIT(bit), bit);
}

/**** global functions ****/

/* exec command function */
static int bq2415x_exec_command(struct bq2415x_device *bq,
				enum bq2415x_command command)
{
	switch (command) {
	case BQ2415X_TIMER_RESET:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS,
				1, BQ2415X_BIT_TMR_RST);
	case BQ2415X_OTG_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_OTG);
	case BQ2415X_STAT_PIN_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_STAT_PIN_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS, 1,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_STAT_PIN_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS, 0,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_CHARGE_STATUS:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_STATUS,
				BQ2415X_MASK_STAT, BQ2415X_SHIFT_STAT);
	case BQ2415X_BOOST_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_BOOST);
	case BQ2415X_FAULT_STATUS:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_STATUS,
			BQ2415X_MASK_FAULT, BQ2415X_SHIFT_FAULT);

	case BQ2415X_CHARGE_TERMINATION_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_TE);
	case BQ2415X_CHARGE_TERMINATION_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_TE);
	case BQ2415X_CHARGE_TERMINATION_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_TE);
	case BQ2415X_CHARGER_STATUS:
		return !bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
			BQ2415X_BIT_CE);
	case BQ2415X_CHARGER_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_CE);
	case BQ2415X_CHARGER_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_CE);
	case BQ2415X_HIGH_IMPEDANCE_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_HZ_MODE);
	case BQ2415X_HIGH_IMPEDANCE_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_HZ_MODE);
	case BQ2415X_HIGH_IMPEDANCE_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_HZ_MODE);
	case BQ2415X_BOOST_MODE_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_OPA_MODE);
	case BQ2415X_BOOST_MODE_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_OPA_MODE);
	case BQ2415X_BOOST_MODE_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_OPA_MODE);

	case BQ2415X_OTG_LEVEL:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_VOLTAGE,
				BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_ACTIVATE_HIGH:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				1, BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_ACTIVATE_LOW:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				0, BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_PIN_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_VOLTAGE,
				BQ2415X_BIT_OTG_EN);
	case BQ2415X_OTG_PIN_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				1, BQ2415X_BIT_OTG_EN);
	case BQ2415X_OTG_PIN_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				0, BQ2415X_BIT_OTG_EN);

	case BQ2415X_VENDER_CODE:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
			BQ2415X_MASK_VENDER, BQ2415X_SHIFT_VENDER);
	case BQ2415X_PART_NUMBER:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
				BQ2415X_MASK_PN, BQ2415X_SHIFT_PN);
	case BQ2415X_REVISION:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
			BQ2415X_MASK_REVISION, BQ2415X_SHIFT_REVISION);
	}
	return -EINVAL;
}

/* detect chip type */
static enum bq2415x_chip bq2415x_detect_chip(struct bq2415x_device *bq)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	int ret = bq2415x_exec_command(bq, BQ2415X_PART_NUMBER);

	if (ret < 0)
		return ret;

	switch (client->addr) {
	case 0x6b:
		switch (ret) {
		case 0:
			if (bq->chip == BQ24151A)
				return bq->chip;
			else
				return BQ24151;
		case 1:
			if (bq->chip == BQ24150A ||
				bq->chip == BQ24152 ||
				bq->chip == BQ24155)
				return bq->chip;
			else
				return BQ24150;
		case 2:
			if (bq->chip == BQ24153A)
				return bq->chip;
			else
				return BQ24153;
		default:
			return BQUNKNOWN;
		}
		break;

	case 0x6a:
		switch (ret) {
		case 0:
			if (bq->chip == BQ24156A)
				return bq->chip;
			else
				return BQ24156;
		case 2:
			return BQ24158;
		default:
			return BQUNKNOWN;
		}
		break;
	}

	return BQUNKNOWN;
}

/* detect chip revision */
static int bq2415x_detect_revision(struct bq2415x_device *bq)
{
	int ret = bq2415x_exec_command(bq, BQ2415X_REVISION);
	int chip = bq2415x_detect_chip(bq);

	if (ret < 0 || chip < 0)
		return -1;

	switch (chip) {
	case BQ24150:
	case BQ24150A:
	case BQ24151:
	case BQ24151A:
	case BQ24152:
		if (ret >= 0 && ret <= 3)
			return ret;
		else
			return -1;
	case BQ24153:
	case BQ24153A:
	case BQ24156:
	case BQ24156A:
	case BQ24158:
		if (ret == 3)
			return 0;
		else if (ret == 1)
			return 1;
		else
			return -1;
	case BQ24155:
		if (ret == 3)
			return 3;
		else
			return -1;
	case BQUNKNOWN:
		return -1;
	}

	return -1;
}

/* return chip vender code */
static int bq2415x_get_vender_code(struct bq2415x_device *bq)
{
	int ret;

	ret = bq2415x_exec_command(bq, BQ2415X_VENDER_CODE);
	if (ret < 0)
		return 0;

	/* convert to binary */
	return (ret & 0x1) +
	       ((ret >> 1) & 0x1) * 10 +
	       ((ret >> 2) & 0x1) * 100;
}

/* reset all chip registers to default state */
static void bq2415x_reset_chip(struct bq2415x_device *bq)
{
	bq2415x_i2c_write(bq, BQ2415X_REG_CURRENT, BQ2415X_RESET_CURRENT);
	bq2415x_i2c_write(bq, BQ2415X_REG_VOLTAGE, BQ2415X_RESET_VOLTAGE);
	bq2415x_i2c_write(bq, BQ2415X_REG_CONTROL, BQ2415X_RESET_CONTROL);
	bq2415x_i2c_write(bq, BQ2415X_REG_STATUS, BQ2415X_RESET_STATUS);
	bq->timer_error = NULL;
}

/**** properties functions ****/

/* set current limit in mA */
static int bq2415x_set_current_limit(struct bq2415x_device *bq, int mA)
{
	int val;

	if (mA <= 100)
		val = 0;
	else if (mA <= USB_CURRENT_LIMIT)
		val = 1;
	else if (mA <= 800)
		val = 2;
	else
		val = 3;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CONTROL, val,
			BQ2415X_MASK_LIMIT, BQ2415X_SHIFT_LIMIT);
}

/* get current limit in mA */
static int bq2415x_get_current_limit(struct bq2415x_device *bq)
{
	int ret;

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CONTROL,
			BQ2415X_MASK_LIMIT, BQ2415X_SHIFT_LIMIT);
	if (ret < 0)
		return ret;
	else if (ret == 0)
		return 100;
	else if (ret == 1)
		return 500;
	else if (ret == 2)
		return 800;
	else if (ret == 3)
		return 1800;
	return -EINVAL;
}

/* set weak battery voltage in mV */
static int bq2415x_set_weak_battery_voltage(struct bq2415x_device *bq, int mV)
{
	int val;

	/* round to 100mV */
	if (mV <= 3400 + 50)
		val = 0;
	else if (mV <= 3500 + 50)
		val = 1;
	else if (mV <= 3600 + 50)
		val = 2;
	else
		val = 3;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CONTROL, val,
			BQ2415X_MASK_VLOWV, BQ2415X_SHIFT_VLOWV);
}

/* get weak battery voltage in mV */
static int bq2415x_get_weak_battery_voltage(struct bq2415x_device *bq)
{
	int ret;

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CONTROL,
			BQ2415X_MASK_VLOWV, BQ2415X_SHIFT_VLOWV);
	if (ret < 0)
		return ret;
	return 100 * (34 + ret);
}

/* set battery regulation voltage in mV */
static int bq2415x_set_battery_regulation_voltage(struct bq2415x_device *bq,
						  int mV)
{
	int val = (mV/10 - 350) / 2;

	if (val < 0)
		val = 0;
	else if (val > 47)
		return -EINVAL;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_VOLTAGE, val,
			BQ2415X_MASK_VO, BQ2415X_SHIFT_VO);
}

/* get battery regulation voltage in mV */
static int bq2415x_get_battery_regulation_voltage(struct bq2415x_device *bq)
{
	int ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_VOLTAGE,
			BQ2415X_MASK_VO, BQ2415X_SHIFT_VO);

	if (ret < 0)
		return ret;
	return 10 * (350 + 2*ret);
}

/* set charge current in mA (platform data must provide resistor sense) */
static int bq2415x_set_charge_current(struct bq2415x_device *bq, int mA)
{
	int val;

	if (bq->init_data.resistor_sense <= 0)
		return -ENOSYS;

	val = (mA * bq->init_data.resistor_sense - 37400) / 6800;
	if (val < 0)
		val = 0;
	else if (val > 7)
		val = 7;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CURRENT, val,
			BQ2415X_MASK_VI_CHRG | BQ2415X_MASK_RESET,
			BQ2415X_SHIFT_VI_CHRG);
}

/* get charge current in mA (platform data must provide resistor sense) */
static int bq2415x_get_charge_current(struct bq2415x_device *bq)
{
	int ret;

	if (bq->init_data.resistor_sense <= 0)
		return -ENOSYS;

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CURRENT,
			BQ2415X_MASK_VI_CHRG, BQ2415X_SHIFT_VI_CHRG);
	if (ret < 0)
		return ret;
	return (37400 + 6800*ret) / bq->init_data.resistor_sense;
}

/* set termination current in mA (platform data must provide resistor sense) */
static int bq2415x_set_termination_current(struct bq2415x_device *bq, int mA)
{
	int val;

	if (bq->init_data.resistor_sense <= 0)
		return -ENOSYS;

	val = (mA * bq->init_data.resistor_sense - 3400) / 3400;
	if (val < 0)
		val = 0;
	else if (val > 7)
		val = 7;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CURRENT, val,
			BQ2415X_MASK_VI_TERM | BQ2415X_MASK_RESET,
			BQ2415X_SHIFT_VI_TERM);
}

/* get termination current in mA (platform data must provide resistor sense) */
static int bq2415x_get_termination_current(struct bq2415x_device *bq)
{
	int ret;

	if (bq->init_data.resistor_sense <= 0)
		return -ENOSYS;

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CURRENT,
			BQ2415X_MASK_VI_TERM, BQ2415X_SHIFT_VI_TERM);
	if (ret < 0)
		return ret;
	return (3400 + 3400*ret) / bq->init_data.resistor_sense;
}

/* set default value of property */
#define bq2415x_set_default_value(bq, prop) \
	do { \
		int ret = 0; \
		if (bq->init_data.prop != -1) \
			ret = bq2415x_set_##prop(bq, bq->init_data.prop); \
		if (ret < 0) \
			return ret; \
	} while (0)

/* set default values of all properties */
static int bq2415x_set_defaults(struct bq2415x_device *bq)
{
	int rc = 0;

	rc = bq2415x_exec_command(bq, BQ2415X_STAT_PIN_DISABLE);
	if(rc < 0)
	{
		pr_info("disable pin failed\n");
		return rc;
	}

	rc = bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_DISABLE);
	if(rc < 0)
	{
		pr_info("boost mode set failed\n");
		return rc;
	}
	rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
	if(rc < 0)
	{
		pr_info("charger disable set failed\n");
		return rc;
	}

	rc = bq2415x_exec_command(bq, BQ2415X_CHARGE_TERMINATION_DISABLE);
	if(rc < 0)
	{
		pr_info("charge termination disable set failed\n");
		return rc;
	}

	bq2415x_set_default_value(bq, current_limit);
	bq2415x_set_default_value(bq, weak_battery_voltage);
	bq2415x_set_default_value(bq, battery_regulation_voltage);

	if (bq->init_data.resistor_sense > 0) {
		bq2415x_set_default_value(bq, charge_current);
		bq2415x_set_default_value(bq, termination_current);
		/* disable termination of charger */
		rc = bq2415x_exec_command(bq, BQ2415X_CHARGE_TERMINATION_DISABLE);
		if(rc < 0)
		{
			pr_info("charge termination enable set failed\n");
			return rc;
		}
	}

	rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
	if(rc < 0)
	{
		pr_info("charge enable set failed\n");
		return rc;
	}
	return 0;
}

/**** charger mode functions ****/

/* set charger mode */
static int bq2415x_set_mode(struct bq2415x_device *bq, enum bq2415x_mode mode)
{
	int ret = 0;
	int charger = 0;
	int boost = 0;

	if (mode == BQ2415X_MODE_BOOST)
		boost = 1;
	else if (mode != BQ2415X_MODE_OFF)
		charger = 1;

	if (!charger)
		ret = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);

	if (!boost)
		ret = bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_DISABLE);

	if (ret < 0)
		return ret;

	switch (mode) {
	case BQ2415X_MODE_OFF:
		dev_dbg(bq->dev, "changing mode to: Offline\n");
		ret = bq2415x_set_current_limit(bq, 100);
		break;
	case BQ2415X_MODE_NONE:
		dev_dbg(bq->dev, "changing mode to: N/A\n");
		ret = bq2415x_set_current_limit(bq, 100);
		break;
	case BQ2415X_MODE_HOST_CHARGER:
		dev_dbg(bq->dev, "changing mode to: Host/HUB charger\n");
		ret = bq2415x_set_current_limit(bq, 500);
		break;
	case BQ2415X_MODE_DEDICATED_CHARGER:
		dev_dbg(bq->dev, "changing mode to: Dedicated charger\n");
		ret = bq2415x_set_current_limit(bq, 1800);
		break;
	case BQ2415X_MODE_BOOST: /* Boost mode */
		dev_dbg(bq->dev, "changing mode to: Boost\n");
		ret = bq2415x_set_current_limit(bq, 100);
		break;
	}

	if (ret < 0)
		return ret;

	if (charger)
		ret = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
	else if (boost)
		ret = bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_ENABLE);

	if (ret < 0)
		return ret;

	bq2415x_set_default_value(bq, weak_battery_voltage);
	bq2415x_set_default_value(bq, battery_regulation_voltage);

	bq->mode = mode;
	sysfs_notify(&bq->charger.dev->kobj, NULL, "mode");

	return 0;

}

/*===========================================
FUNCTION: get_bq2415x_fault_status
DESCRIPTION: to get bq2415x Reg0 fault bits value
IPNUT: none
RETURN:	a int value, the error status of bq2415x
=============================================*/
int get_bq2415x_fault_status(void)
{
	int error = 0;
	if(bq_device == NULL)
	{
		pr_info("%s:device not init,do nothing!\n",__func__);
		return -EINVAL;
	}
	error = bq2415x_exec_command(bq_device, BQ2415X_FAULT_STATUS);
	if (error < 0) {
		pr_err ("Unknown error\n");
	}
	return error;
}
EXPORT_SYMBOL(get_bq2415x_fault_status);

/*===========================================
FUNCTION: is_bq24152_in_boost_mode
DESCRIPTION:
IPNUT: none
RETURN:	a int value, 1 means bq24152 is in boost mode, 0 means charger mode
=============================================*/
int is_bq24152_in_boost_mode(void)
{
	int ret = 0;
	if(bq_device == NULL)
	{
		pr_info("%s:device not init,do nothing!\n",__func__);
		return -EINVAL;
	}
	ret = bq2415x_exec_command(bq_device, BQ2415X_BOOST_STATUS);
	if (ret < 0)
		return ret;
	if(0 == ret)
		return 0;
	if(1 == ret)
		return 1;
	return 0;
}
EXPORT_SYMBOL(is_bq24152_in_boost_mode);

void notify_bq24152_to_control_otg(bool enable)
{
	int mode = 0;
	if(bq_device == NULL)
	{
		pr_info("%s:device not init,do nothing!\n",__func__);
		return;
	}
	mode = is_bq24152_in_boost_mode();
	if(enable){
		pr_info("bq2415x_set_mode: boost mode, enable=%d\n", enable);
		if(1 == mode)
			return;
		bq2415x_set_mode(bq_device, BQ2415X_MODE_BOOST);
	}else{
		pr_info("bq2415x_set_mode: normal mode, enable=%d\n", enable);
		if(0 == mode)
			return;
		bq2415x_set_mode(bq_device, BQ2415X_MODE_OFF);
	}
}
EXPORT_SYMBOL(notify_bq24152_to_control_otg);

/* hook function called by other driver which set reported mode */
static void bq2415x_hook_function(enum bq2415x_mode mode, void *data)
{
	struct bq2415x_device *bq = data;

	if (!bq)
		return;

	dev_dbg(bq->dev, "hook function was called\n");
	bq->reported_mode = mode;

	/* if automode is not enabled do not tell about reported_mode */
	if (bq->automode < 1)
		return;

	sysfs_notify(&bq->charger.dev->kobj, NULL, "reported_mode");
	bq2415x_set_mode(bq, bq->reported_mode);

}

/**** timer functions ****/

/* enable/disable auto resetting chip timer */
static void bq2415x_set_autotimer(struct bq2415x_device *bq, int state)
{
	mutex_lock(&bq2415x_timer_mutex);

	if (bq->autotimer == state) {
		mutex_unlock(&bq2415x_timer_mutex);
		return;
	}

	bq->autotimer = state;

	if (state) {
		schedule_delayed_work(&bq->work, BQ2415X_TIMER_TIMEOUT * HZ);
		bq2415x_exec_command(bq, BQ2415X_TIMER_RESET);
		bq->timer_error = NULL;
	} else {
		cancel_delayed_work_sync(&bq->work);
	}

	mutex_unlock(&bq2415x_timer_mutex);
}

/* called by bq2415x_timer_work on timer error */
#ifndef CONFIG_HUAWEI_KERNEL
static void bq2415x_timer_error(struct bq2415x_device *bq, const char *msg)
{
	bq->timer_error = msg;
	sysfs_notify(&bq->charger.dev->kobj, NULL, "timer");
	dev_err(bq->dev, "%s\n", msg);
	if (bq->automode > 0)
		bq->automode = 0;
	bq2415x_set_mode(bq, BQ2415X_MODE_OFF);
	bq2415x_set_autotimer(bq, 0);
}
#endif

/* to check whether charge is done by gasgauge */
static void bq2451x_check_charge_status(struct bq2415x_device *bq)
{
	int battery_full = 0;
	int rc;

	if((bq_device == NULL) || (g_battery_measure_by_bq27510_device == NULL))
	{
		pr_debug("TI chip uninitialized\n");
		return;
	}

	//hot temperature
	if(bq->charge_disable)
	{
		return;
	}

	//check if charger is online
	if(!is_usb_chg_exist())
	{
		bq->charge_done_flag = 0;
		return;
	}

	//get battery full status from gasgauge
	battery_full = is_bq27510_battery_full(g_battery_measure_by_bq27510_device);
	if(battery_full)
	{
		if( g_battery_measure_by_bq27510_device->capacity == CAPACITY_FULL)
		{
			rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
			if(rc < 0)
			{
				pr_info("disable charger failed\n");
				return;
			}
			bq->charge_done_flag = 1;
			pr_debug("charge full, stop charege!\n");
		}
	}
	else
	{
		//check hzmode for running test
		rc = bq2415x_exec_command(bq, BQ2415X_HIGH_IMPEDANCE_STATUS);
		if(rc < 0)
		{
			pr_info("get hzmode failed\n");
			return;
		}
		else if(rc == 0)
		{
			//if battery is not full, enable charge
			rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
			if(rc < 0)
			{
				pr_info("charge enable set failed\n");
				return;
			}
		}
		else
		{
			pr_debug("High impedance mode!\n");
			bq->charge_done_flag = 0;
		}
	}
}

/* delayed work function for auto resetting chip timer */
static void bq2415x_timer_work(struct work_struct *work)
{
	struct bq2415x_device *bq = container_of(work, struct bq2415x_device,
						 work.work);
	int ret;
	int error;
	int boost;
	int now_charge_status = -1;
	static int last_charge_status = -1;
	ret = bq2415x_exec_command(bq, BQ2415X_TIMER_RESET);
	if (ret < 0) {
		pr_info("Resetting timer failed %d\n",ret);
	}

	boost = bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_STATUS);
	if (boost < 0) {
		pr_info("Unknown boost error %d\n",boost);
	}

	error = bq2415x_exec_command(bq, BQ2415X_FAULT_STATUS);
	if (error < 0) {
		pr_info("Unknown error %d\n",error);
	}
#ifdef CONFIG_HUAWEI_DSM
	if(boost && error){
		pr_err("find boost mode fault! such as overload(OTG OCP), VBUS OVP, VBUS < UVLO,"
			"battery OVP, thermal shutdown and so on\n");
		dump_registers_and_adc(charger_dclient,g_lbc_chip,DSM_CHARGER_BQ_BOOST_FAULT_ERROR_NO);
	}

	if(!boost && error){
		pr_err("find charge mode fault! such as poor input source, VBUS OVP, battery is too"
			"low, timer fault, thermal shutdown and so on\n");
		dump_registers_and_adc(charger_dclient,g_lbc_chip,DSM_CHARGER_BQ_NORMAL_FAULT_ERROR_NO);
	}
#endif

	if (boost) {
		switch (error) {
		/* Non fatal errors, chip is OK */
		case 0: /* No error */
			break;
		case 6: /* Timer expired */
			dev_err(bq->dev, "Timer expired\n");
			break;
		case 3: /* Battery voltage too low */
			dev_err(bq->dev, "Battery voltage to low\n");
			break;

		/* Fatal errors, disable and reset chip */
		case 1: /* Overvoltage protection (chip fried) */
			pr_info("Overvoltage protection (chip fried)\n");
			break;
		case 2: /* Overload */
			pr_info("Overload\n");
			break;
		case 4: /* Battery overvoltage protection */
			pr_info("Battery overvoltage protection\n");
			break;
		case 5: /* Thermal shutdown (too hot) */
			pr_info("Thermal shutdown (too hot)\n");
			break;
		case 7: /* N/A */
			pr_info("Unknown error\n");
			break;
		}
	} else {
		switch (error) {
		/* Non fatal errors, chip is OK */
		case 0: /* No error */
			break;
		case 2: /* Sleep mode */
			dev_err(bq->dev, "Sleep mode\n");
			break;
		case 3: /* Poor input source */
			dev_err(bq->dev, "Poor input source\n");
			break;
		case 6: /* Timer expired */
			dev_err(bq->dev, "Timer expired\n");
			break;
		case 7: /* No battery */
			dev_err(bq->dev, "No battery\n");
			break;

		/* Fatal errors, disable and reset chip */
		case 1: /* Overvoltage protection (chip fried) */
			pr_info("Overvoltage protection (chip fried)\n");
			break;
		case 4: /* Battery overvoltage protection */
			pr_info("Battery overvoltage protection\n");
			break;
		case 5: /* Thermal shutdown (too hot) */
			pr_info("Thermal shutdown (too hot)\n");
			break;
		}
	}
	bq2415x_set_appropriate_jeita(bq);

	bq2451x_check_charge_status(bq);

	now_charge_status = bq2415x_exec_command(bq, BQ2415X_CHARGE_STATUS);
	if(bq->charge_done_flag)
	{
		now_charge_status = STATUS_FULL;
	}
	//update charge status so that charge icon change.
	if(now_charge_status != last_charge_status)
	{
		power_supply_changed(&bq->charger);
	}
	last_charge_status = now_charge_status;

	schedule_delayed_work(&bq->work, BQ2415X_TIMER_TIMEOUT * HZ);
}

static void bq2415x_iusb_work(struct work_struct *work)
{
	int rc = 0;
	struct bq2415x_device *bq = container_of(work, struct bq2415x_device,
						 iusb_work);
	pr_info("usb current is %d\n",bq->iusb_limit);
	rc = bq2415x_set_current_limit(bq,bq->iusb_limit);
	if(rc < 0)
	{
		pr_info("current limit setting failed\n");
		return;
	}
	if(bq->iusb_limit == 0 ){
		rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
		if(rc < 0){
			pr_info("disable charger failed\n");
			return;
		}
	}else if(bq->iusb_limit > USB_CURRENT_LIMIT){
		rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
		if(rc < 0){
			pr_info("charge enable set failed\n");
			return;
		}
		if(jeita_now->i_max){
			rc = bq2415x_set_charge_current(bq, min(jeita_now->i_max,hot_design_current));
			if(rc < 0){
				pr_info("set charge current failed\n");
				return;
			}
		}
	}else{
		rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
		if(rc < 0){
			pr_info("charge enable set failed\n");
			return;
		}
		rc = bq2415x_set_charge_current(bq, USB_CHARGE_CURRENT);
		if(rc < 0){
			pr_info("set charge current failed\n");
			return;
		}
	}
}

/*===========================================
FUNCTION: increase_usb_ma_step
DESCRIPTION: to increase usb current step by step
IPNUT:	bq2415x_device *bq
RETURN:	a int value: 0 means sucessful
=============================================*/
static int increase_usb_ma_step(struct bq2415x_device *bq)
{
	int usb_ma;
	int rc = 0;
	usb_ma = bq2415x_get_current_limit(bq);
	if(usb_ma < 0){
		rc = -EINVAL;
		return rc;
	}
	if(CURRENT_LIMIT_INIT == usb_ma){
		usb_ma = CURRENT_LIMIT_STEP_ONE;
	}else if(CURRENT_LIMIT_STEP_ONE == usb_ma){
		usb_ma = CURRENT_LIMIT_STEP_TWO;
	}else if(CURRENT_LIMIT_STEP_TWO == usb_ma){
		usb_ma = CURRENT_THRESHOLD_MA;
	}else{
		usb_ma = CURRENT_THRESHOLD_MA;
	}

	rc = bq2415x_set_current_limit(bq, usb_ma);
	if(rc < 0)
	{
		pr_info("current limit setting failed\n");
		return rc;
	}

	return rc;
}

/*===========================================
FUNCTION: bq2415x_usb_low_power_work
DESCRIPTION: to ajust current limit for low power(poor input) charger
IPNUT:	bq2415x_device *bq
RETURN:	N/A
=============================================*/
static void bq2415x_usb_low_power_work(struct work_struct *work)
{
	struct bq2415x_device *bq = container_of(work, struct bq2415x_device,
						 lower_power_charger_work);
	int usb_ma, usb_present, rc ;
	int vbatt = 0;
	int chg_status = 0, chg_fault = 0;
	u8 reg_val = 0;
	union power_supply_propval val = {0};

	if(g_lbc_chip == NULL){
		pr_info("not init, do nothing!\n");
		return;
	}

	/* set usb_init_ma is 100 */
	usb_ma = CURRENT_LIMIT_INIT;
	rc = bq2415x_set_current_limit(bq, usb_ma);
	if(rc < 0)
	{
		pr_err("current limit setting failed\n");
		return;
	}

	rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
	if(rc < 0){
		pr_err("bq2415x enable charging failed\n");
		return;
	}

	/*Set output current for DCP charger*/
	rc = bq2415x_set_charge_current(bq, min(MAX_CHARGE_CURRENT, hot_design_current));
	if(rc < 0){
		pr_err("set charge current failed\n");
		return;
	}

	/*G760 va board Reg0 charge status update a little slowly, need at least 2 seconds */
	if(!poor_input_enable){
		msleep(2500);
	}

	usb_present = is_usb_chg_exist();

	/* Get battery voltage in mV*/
	if (bq->ti_bms_psy){
		rc = bq->ti_bms_psy->get_property(bq->ti_bms_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	}
	vbatt = val.intval /1000;

	/* increase ma by step */
	while(usb_ma < CURRENT_THRESHOLD_MA && usb_present)
	{
		if(poor_input_enable){
			pr_info("poor input charger enable!\n");
			if(vbatt > BATTERY_VOL_THRESHOLD){
				bq2415x_set_current_limit(bq, CURRENT_LIMIT_STEP_ONE);
			}else{
				bq2415x_set_current_limit(bq, CURRENT_LIMIT_INIT);
			}
			poor_input_enable = 0;
			break;
		}
		/* Increase the current limit by step, such as 100mA to 500mA,*/
		/* 500mA to 800 mA, 800mA to no limit*/
		increase_usb_ma_step(bq);

		/* Read Reg0 bit0 to bit3(fault status) and bit 4 to bit5(chg_sts) */
		reg_val = bq2415x_i2c_read(bq, BQ2415X_REG_STATUS);
		chg_status = (reg_val & BQ24152_REG_0_STAT_MASK) >> 4;
		chg_fault = reg_val & BQ24152_REG_0_FAULT_MASK;

		pr_info("reg_val = %x, chg_status = %d, chg_fault = %d\n",
				reg_val, chg_status, chg_fault);

		if(qpnp_lbc_is_in_vin_min_loop(g_lbc_chip)){
			pr_info("charger is in vin_min loop!\n");
			if(vbatt > BATTERY_VOL_THRESHOLD){
				bq2415x_set_current_limit(bq, CURRENT_LIMIT_STEP_ONE);
			}else{
				bq2415x_set_current_limit(bq, CURRENT_LIMIT_INIT);
			}
			break;
		}

		if((BQ24152_CHG_STAT_FAULT == chg_status)
			|| (POOR_INPUT_FAULT_STATUS == chg_fault)){
			pr_info("find chg state fault or poor input fault!\n");
			poor_input_enable = 1;
			if(vbatt > BATTERY_VOL_THRESHOLD){
				bq2415x_set_current_limit(bq, CURRENT_LIMIT_STEP_ONE);
			}else{
				bq2415x_set_current_limit(bq, CURRENT_LIMIT_INIT);
			}
			break;
		}

		usb_present = is_usb_chg_exist();
		/* Get the current limit */
		usb_ma = bq2415x_get_current_limit(bq);

		msleep(50);

	}

}

/**** power supply interface code ****/

static enum power_supply_property bq2415x_power_supply_props[] = {
	/* TODO: maybe add more power supply properties */
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static int bq2415x_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	int ret = 0;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq2415x_exec_command(bq, BQ2415X_CHARGE_STATUS);
		if (ret < 0)
			return ret;
		else if (ret == 0) /* Ready */
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (ret == 1) /* Charge in progress */
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (ret == 2) /* Charge done */
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

		if(bq->charge_done_flag)
		{
			val->intval = POWER_SUPPLY_STATUS_FULL;
		}
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->model;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (bq->ti_bms_psy)
			ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_TEMP,val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (bq->ti_bms_psy)
			ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_VOLTAGE_NOW,val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (bq->ti_bms_psy)
			ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_CURRENT_NOW,val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (bq->ti_bms_psy)
			ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_HEALTH,val);
		//if ti fuel gauge is not ready,val return UNKNOWN.
		else
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if(bq->qcom_charger_psy)
			ret = bq->qcom_charger_psy->get_property(bq->qcom_charger_psy,POWER_SUPPLY_PROP_PRESENT,val);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval= bq2415x_exec_command(bq,BQ2415X_CHARGER_STATUS);
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if(bq->ti_bms_psy)
			ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_CAPACITY,val);
		if(bq->charge_done_flag)
		{
			val->intval = CAPACITY_FULL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if(bq->ti_bms_psy)
			ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int bq2415x_power_supply_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
								charger);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		bq->iusb_limit = val->intval;
		if(bq->iusb_limit <= USB_CURRENT_LIMIT){
			schedule_work(&bq->iusb_work);
		}else{
			schedule_work(&bq->lower_power_charger_work);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if(val->intval)
			rc = bq2415x_exec_command(bq, BQ2415X_HIGH_IMPEDANCE_DISABLE);
		else
			rc = bq2415x_exec_command(bq, BQ2415X_HIGH_IMPEDANCE_ENABLE);
		user_ctl_status = val->intval;
		break;
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_COOL_TEMP:
	case POWER_SUPPLY_PROP_WARM_TEMP:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	default:
		return -EINVAL;
	}
	return rc;
}

static int bq2415x_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_COOL_TEMP:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
	case POWER_SUPPLY_PROP_WARM_TEMP:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static void bq2415x_external_power_changed(struct power_supply *psy)
{
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
								charger);
	unsigned long flags;
	spin_lock_irqsave(&bq->ibat_change_lock, flags);
	
	if (!bq->ti_bms_psy)
		bq->ti_bms_psy = power_supply_get_by_name("ti-bms");
	
	if (!bq->qcom_charger_psy)
		bq->qcom_charger_psy = power_supply_get_by_name("battery");

	spin_unlock_irqrestore(&bq->ibat_change_lock, flags);

}

static char *bq2415x_supplied_to[] = {
	"ti-bms",
};
static int bq2415x_power_supply_init(struct bq2415x_device *bq)
{
	int ret;
	int chip;
	char revstr[8];

	bq->charger.name = "ti-charger";
	bq->charger.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->charger.properties = bq2415x_power_supply_props;
	bq->charger.num_properties = ARRAY_SIZE(bq2415x_power_supply_props);
	bq->charger.get_property = bq2415x_power_supply_get_property;
	bq->charger.set_property = bq2415x_power_supply_set_property;
	bq->charger.property_is_writeable = bq2415x_property_is_writeable;
	bq->charger.external_power_changed = bq2415x_external_power_changed;
	bq->charger.supplied_to = bq2415x_supplied_to;
	bq->charger.num_supplicants =ARRAY_SIZE(bq2415x_supplied_to);

	ret = bq2415x_detect_chip(bq);
	if (ret < 0)
		chip = BQUNKNOWN;
	else
		chip = ret;

	ret = bq2415x_detect_revision(bq);
	if (ret < 0)
		strncpy(revstr, "unknown",sizeof revstr);
	else
		snprintf(revstr,sizeof revstr, "1.%d", ret);

	bq->model = kasprintf(GFP_KERNEL,
				"chip %s, revision %s, vender code %.3d",
				bq2415x_chip_name[chip], revstr,
				bq2415x_get_vender_code(bq));
	if (!bq->model) {
		dev_err(bq->dev, "failed to allocate model name\n");
		return -ENOMEM;
	}

	ret = power_supply_register(bq->dev, &bq->charger);
	if (ret) {
		kfree(bq->model);
		return ret;
	}

	return 0;
}

static void bq2415x_power_supply_exit(struct bq2415x_device *bq)
{
	bq->autotimer = 0;
	if (bq->automode > 0)
		bq->automode = 0;
	cancel_delayed_work_sync(&bq->work);
	power_supply_unregister(&bq->charger);
	kfree(bq->model);
}

/**** additional sysfs entries for power supply interface ****/

/* show *_status entries */
static ssize_t bq2415x_sysfs_show_status(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						charger);
	enum bq2415x_command command;
	int ret;

	if (strcmp(attr->attr.name, "otg_status") == 0)
		command = BQ2415X_OTG_STATUS;
	else if (strcmp(attr->attr.name, "charge_status") == 0)
		command = BQ2415X_CHARGE_STATUS;
	else if (strcmp(attr->attr.name, "boost_status") == 0)
		command = BQ2415X_BOOST_STATUS;
	else if (strcmp(attr->attr.name, "fault_status") == 0)
		command = BQ2415X_FAULT_STATUS;
	else
		return -EINVAL;

	ret = bq2415x_exec_command(bq, command);
	if (ret < 0)
		return ret;
	return snprintf(buf, PAGE_SIZE,"%d\n", ret);
}

/*
 * set timer entry:
 *    auto - enable auto mode
 *    off - disable auto mode
 *    (other values) - reset chip timer
 */
static ssize_t bq2415x_sysfs_set_timer(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						charger);
	int ret = 0;

	if (strncmp(buf, "auto", 4) == 0)
		bq2415x_set_autotimer(bq, 1);
	else if (strncmp(buf, "off", 3) == 0)
		bq2415x_set_autotimer(bq, 0);
	else
		ret = bq2415x_exec_command(bq, BQ2415X_TIMER_RESET);

	if (ret < 0)
		return ret;
	return count;
}

/* show timer entry (auto or off) */
static ssize_t bq2415x_sysfs_show_timer(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);

	if (bq->timer_error)
		return snprintf(buf, PAGE_SIZE,"%s\n", bq->timer_error);

	if (bq->autotimer)
		return snprintf(buf, PAGE_SIZE,"auto\n");
	return snprintf(buf, PAGE_SIZE,"off\n");
}

/*
 * set mode entry:
 *    auto - if automode is supported, enable it and set mode to reported
 *    none - disable charger and boost mode
 *    host - charging mode for host/hub chargers (current limit 500mA)
 *    dedicated - charging mode for dedicated chargers (unlimited current limit)
 *    boost - disable charger and enable boost mode
 */
static ssize_t bq2415x_sysfs_set_mode(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	enum bq2415x_mode mode;
	int ret = 0;

	if (strncmp(buf, "auto", 4) == 0) {
		if (bq->automode < 0)
			return -ENOSYS;
		bq->automode = 1;
		mode = bq->reported_mode;
	} else if (strncmp(buf, "off", 3) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_OFF;
	} else if (strncmp(buf, "none", 4) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_NONE;
	} else if (strncmp(buf, "host", 4) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_HOST_CHARGER;
	} else if (strncmp(buf, "dedicated", 9) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_DEDICATED_CHARGER;
	} else if (strncmp(buf, "boost", 5) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_BOOST;
	} else if (strncmp(buf, "reset", 5) == 0) {
		bq2415x_reset_chip(bq);
		bq2415x_set_defaults(bq);
		if (bq->automode <= 0)
			return count;
		bq->automode = 1;
		mode = bq->reported_mode;
	} else {
		return -EINVAL;
	}

	ret = bq2415x_set_mode(bq, mode);
	if (ret < 0)
		return ret;
	return count;
}

/* show mode entry (auto, none, host, dedicated or boost) */
static ssize_t bq2415x_sysfs_show_mode(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						charger);
	ssize_t ret = 0;

	if (bq->automode > 0)
		ret += snprintf(buf+ret, PAGE_SIZE, "auto (");

	switch (bq->mode) {
	case BQ2415X_MODE_OFF:
		ret += snprintf(buf+ret, PAGE_SIZE, "off");
		break;
	case BQ2415X_MODE_NONE:
		ret += snprintf(buf+ret, PAGE_SIZE, "none");
		break;
	case BQ2415X_MODE_HOST_CHARGER:
		ret += snprintf(buf+ret, PAGE_SIZE, "host");
		break;
	case BQ2415X_MODE_DEDICATED_CHARGER:
		ret += snprintf(buf+ret, PAGE_SIZE, "dedicated");
		break;
	case BQ2415X_MODE_BOOST:
		ret += snprintf(buf+ret, PAGE_SIZE, "boost");
		break;
	}

	if (bq->automode > 0)
		ret += snprintf(buf+ret, PAGE_SIZE, ")");

	ret += snprintf(buf+ret, PAGE_SIZE,"\n");
	return ret;
}
/* show reported_mode entry (none, host, dedicated or boost) */
static ssize_t bq2415x_sysfs_show_reported_mode(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);

	if (bq->automode < 0)
		return -EINVAL;

	switch (bq->reported_mode) {
	case BQ2415X_MODE_OFF:
		return snprintf(buf, PAGE_SIZE,"off\n");
	case BQ2415X_MODE_NONE:
		return snprintf(buf, PAGE_SIZE,"none\n");
	case BQ2415X_MODE_HOST_CHARGER:
		return snprintf(buf, PAGE_SIZE,"host\n");
	case BQ2415X_MODE_DEDICATED_CHARGER:
		return snprintf(buf, PAGE_SIZE,"dedicated\n");
	case BQ2415X_MODE_BOOST:
		return snprintf(buf, PAGE_SIZE,"boost\n");
	}

	return -EINVAL;
}

	//remove redundant code
/* print value of chip register, format: 'register=value' */
static ssize_t bq2415x_sysfs_print_reg(struct bq2415x_device *bq,
				       u8 reg,
				       char *buf)
{
	int ret = bq2415x_i2c_read(bq, reg);

	if (ret < 0)
		return snprintf(buf, PAGE_SIZE, "%#.2x=error %d\n", reg, ret);	
	return snprintf(buf, PAGE_SIZE, "%#.2x ", ret);
}

/* show all raw values of chip register, format per line: 'register=value' */
static ssize_t bq2415x_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	ssize_t ret = 0;

	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_STATUS, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_CONTROL, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_VOLTAGE, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_VENDER, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_CURRENT, buf+ret);
	ret += snprintf(buf+ret, PAGE_SIZE,"\n");
	return ret;
}

/* set current and voltage limit entries (in mA or mV) */
static ssize_t bq2415x_sysfs_set_limit(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	long val;
	int ret;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	if (strcmp(attr->attr.name, "current_limit") == 0)
		ret = bq2415x_set_current_limit(bq, val);
	else if (strcmp(attr->attr.name, "weak_battery_voltage") == 0)
		ret = bq2415x_set_weak_battery_voltage(bq, val);
	else if (strcmp(attr->attr.name, "battery_regulation_voltage") == 0)
		ret = bq2415x_set_battery_regulation_voltage(bq, val);
	else if (strcmp(attr->attr.name, "charge_current") == 0)
		ret = bq2415x_set_charge_current(bq, val);
	else if (strcmp(attr->attr.name, "termination_current") == 0)
		ret = bq2415x_set_termination_current(bq, val);
	else
		return -EINVAL;

	if (ret < 0)
		return ret;
	return count;
}

/* show current and voltage limit entries (in mA or mV) */
static ssize_t bq2415x_sysfs_show_limit(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	int ret;

	if (strcmp(attr->attr.name, "current_limit") == 0)
		ret = bq2415x_get_current_limit(bq);
	else if (strcmp(attr->attr.name, "weak_battery_voltage") == 0)
		ret = bq2415x_get_weak_battery_voltage(bq);
	else if (strcmp(attr->attr.name, "battery_regulation_voltage") == 0)
		ret = bq2415x_get_battery_regulation_voltage(bq);
	else if (strcmp(attr->attr.name, "charge_current") == 0)
		ret = bq2415x_get_charge_current(bq);
	else if (strcmp(attr->attr.name, "termination_current") == 0)
		ret = bq2415x_get_termination_current(bq);
	else
		return -EINVAL;

	if (ret < 0)
		return ret;
	return snprintf(buf, PAGE_SIZE,"%d\n", ret);
}

/* set *_enable entries */
static ssize_t bq2415x_sysfs_set_enable(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	enum bq2415x_command command;
	long val;
	int ret;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	if (strcmp(attr->attr.name, "charge_termination_enable") == 0)
		command = val ? BQ2415X_CHARGE_TERMINATION_ENABLE :
			BQ2415X_CHARGE_TERMINATION_DISABLE;
	else if (strcmp(attr->attr.name, "high_impedance_enable") == 0)
		command = val ? BQ2415X_HIGH_IMPEDANCE_ENABLE :
			BQ2415X_HIGH_IMPEDANCE_DISABLE;
	else if (strcmp(attr->attr.name, "otg_pin_enable") == 0)
		command = val ? BQ2415X_OTG_PIN_ENABLE :
			BQ2415X_OTG_PIN_DISABLE;
	else if (strcmp(attr->attr.name, "stat_pin_enable") == 0)
		command = val ? BQ2415X_STAT_PIN_ENABLE :
			BQ2415X_STAT_PIN_DISABLE;
	else
		return -EINVAL;

	ret = bq2415x_exec_command(bq, command);
	if (ret < 0)
		return ret;
	return count;
}

/* show *_enable entries */
static ssize_t bq2415x_sysfs_show_enable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	enum bq2415x_command command;
	int ret;

	if (strcmp(attr->attr.name, "charge_termination_enable") == 0)
		command = BQ2415X_CHARGE_TERMINATION_STATUS;
	else if (strcmp(attr->attr.name, "high_impedance_enable") == 0)
		command = BQ2415X_HIGH_IMPEDANCE_STATUS;
	else if (strcmp(attr->attr.name, "otg_pin_enable") == 0)
		command = BQ2415X_OTG_PIN_STATUS;
	else if (strcmp(attr->attr.name, "stat_pin_enable") == 0)
		command = BQ2415X_STAT_PIN_STATUS;
	else
		return -EINVAL;

	ret = bq2415x_exec_command(bq, command);
	if (ret < 0)
		return ret;
	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static DEVICE_ATTR(current_limit, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_limit, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(weak_battery_voltage, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_limit, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(battery_regulation_voltage, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_limit, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_limit, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_limit, bq2415x_sysfs_set_limit);

static DEVICE_ATTR(charge_termination_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_enable, bq2415x_sysfs_set_enable);
static DEVICE_ATTR(high_impedance_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_enable, bq2415x_sysfs_set_enable);
static DEVICE_ATTR(otg_pin_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_enable, bq2415x_sysfs_set_enable);
static DEVICE_ATTR(stat_pin_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_enable, bq2415x_sysfs_set_enable);

static DEVICE_ATTR(reported_mode, S_IRUGO,
		bq2415x_sysfs_show_reported_mode, NULL);
static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_mode, bq2415x_sysfs_set_mode);
static DEVICE_ATTR(timer, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_timer, bq2415x_sysfs_set_timer);

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_registers, NULL);

static DEVICE_ATTR(otg_status, S_IRUGO, bq2415x_sysfs_show_status, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, bq2415x_sysfs_show_status, NULL);
static DEVICE_ATTR(boost_status, S_IRUGO, bq2415x_sysfs_show_status, NULL);
static DEVICE_ATTR(fault_status, S_IRUGO, bq2415x_sysfs_show_status, NULL);

static struct attribute *bq2415x_sysfs_attributes[] = {
	/*
	 * TODO: some (appropriate) of these attrs should be switched to
	 * use power supply class props.
	 */
	&dev_attr_current_limit.attr,
	&dev_attr_weak_battery_voltage.attr,
	&dev_attr_battery_regulation_voltage.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_termination_current.attr,

	&dev_attr_charge_termination_enable.attr,
	&dev_attr_high_impedance_enable.attr,
	&dev_attr_otg_pin_enable.attr,
	&dev_attr_stat_pin_enable.attr,

	&dev_attr_reported_mode.attr,
	&dev_attr_mode.attr,
	&dev_attr_timer.attr,

	&dev_attr_registers.attr,

	&dev_attr_otg_status.attr,
	&dev_attr_charge_status.attr,
	&dev_attr_boost_status.attr,
	&dev_attr_fault_status.attr,
	NULL,
};

static void select_jeita(struct bq2415x_device *bq)
{
	int counter = 0;
	jeita_entry *jeita_entry = NULL;
	union power_supply_propval val = {0};

	if(migration)
	{
		pr_debug("migration,ignore this loop %d\n",migration);
		migration--;
		if(migration)
			return;
	}

	if(batt_jeita_param.hot.t_high == -1)
	{
		pr_info("waiting for linear charger load battery data ...\n");
		return;
	}

	if(batt_jeita_param.hot.t_high == INT_MAX)
	{
		int i = 0;
		struct jeita_entry *loop = (struct jeita_entry *)&batt_jeita_param;
		
		for(;i < ZONE_MAX;i++)
		{
			pr_info("jeita [%d %d %d %d]\n",loop->t_low,loop->t_high,
				loop->i_max,loop->v_max);
			loop++;
		}
		batt_jeita_param.hot.t_high = INT_MAX-1;
	}
	
	//get battery temperature
	if(bq->ti_bms_psy)
	{
		bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_TEMP,&val);
	}
	else
	{
		val.intval = 250;
	}

	jeita_entry = (struct jeita_entry *)&batt_jeita_param;

	pr_debug("current temperature is %d\n",val.intval);
	
	for(counter = 0; counter < ZONE_MAX; counter++)
	{
		if(is_between(jeita_entry[counter].t_low,
			jeita_entry[counter].t_high,val.intval))
			break;
	}
	if(counter < ZONE_MAX){
		jeita_last = jeita_now;
		jeita_now = jeita_entry+counter;
		pr_debug("select param [%d %d %d %d]\n",jeita_now->t_low,jeita_now->t_high,jeita_now->i_max,jeita_now->v_max);
	}
}

static void bq2415x_set_appropriate_jeita(struct bq2415x_device *bq)
{
	int ret = 0;
	
	select_jeita(bq);
	
	if(jeita_last != jeita_now && !migration)
	{
		pr_debug("need jeita adjust\n");
		if(!jeita_now->i_max && (bq2415x_exec_command(bq, BQ2415X_CHARGE_STATUS) == 1))
		{
			ret = bq2415x_exec_command(bq,BQ2415X_CHARGER_DISABLE);
			if(ret < 0){
				pr_err("bq2415x enable charging failed\n");
				return;
			}
			bq->charge_disable = true;
			pr_debug("temperature is hot or cold,stop charging\n");
			return;
		}

		if(jeita_now->i_max && bq->charge_disable && user_ctl_status)
		{
			ret = bq2415x_exec_command(bq,BQ2415X_CHARGER_ENABLE);
			if(ret < 0){
				pr_err("bq2415x enable charging failed\n");
				return;
			}
			bq->charge_disable = false;
			pr_debug("ti charging enabled\n");
		}

		if(!bq->charge_disable)
		{
			ret = bq2415x_set_charge_current(bq, min(jeita_now->i_max,hot_design_current));
			if(ret < 0){
				pr_err("set charge current failed\n");
				return;
			}
			ret = bq2415x_set_battery_regulation_voltage(bq,jeita_now->v_max);
			if(ret < 0){
				pr_err("set battery regulation voltage failed\n");
				return;
			}
			pr_debug("adjust current %d voltage %d\n",min(jeita_now->i_max,hot_design_current),jeita_now->v_max);
		}
		migration = (HYSTERTSIS_TIME/BQ2415X_TIMER_TIMEOUT);
	}
}
static const struct attribute_group bq2415x_sysfs_attr_group = {
	.name = "ti-charger-prop",
	.attrs = bq2415x_sysfs_attributes,
};

static int bq2415x_sysfs_init(struct bq2415x_device *bq)
{
	return sysfs_create_group(&bq->charger.dev->kobj,
			&bq2415x_sysfs_attr_group);
}

static void bq2415x_sysfs_exit(struct bq2415x_device *bq)
{
	sysfs_remove_group(&bq->charger.dev->kobj, &bq2415x_sysfs_attr_group);
}

/* main bq2415x probe function */
static int bq2415x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	int num;
	char *name;
	struct bq2415x_device *bq;
	struct device_node *np = client->dev.of_node;
	struct power_supply *usb_psy;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}
	/* Get new ID for the new device */
	mutex_lock(&bq2415x_id_mutex);
	num = idr_alloc(&bq2415x_id, client, 0, 0, GFP_KERNEL);
	mutex_unlock(&bq2415x_id_mutex);
	if (num < 0)
		return num;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		ret = -ENOMEM;
		goto error_1;
	}

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "failed to allocate device data\n");
		ret = -ENOMEM;
		goto error_2;
	}

	i2c_set_clientdata(client, bq);

	bq->usb_psy = usb_psy;
	bq->id = num;
	bq->dev = &client->dev;
	bq->chip = id->driver_data;
	bq->name = name;
	bq->mode = BQ2415X_MODE_OFF;
	bq->reported_mode = BQ2415X_MODE_OFF;
	bq->autotimer = 0;
	bq->automode = 0;
	bq->charge_done_flag = 0;
	bq->charge_disable = false;
	spin_lock_init(&bq->ibat_change_lock);

	ret = of_property_read_u32(np, "ti,current-limit",
			&bq->init_data.current_limit);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ti,weak-battery-voltage",
			&bq->init_data.weak_battery_voltage);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ti,battery-regulation-voltage",
			&bq->init_data.battery_regulation_voltage);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ti,charge-current",
			&bq->init_data.charge_current);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ti,termination-current",
			&bq->init_data.termination_current);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ti,resistor-sense",
			&bq->init_data.resistor_sense);
	if (ret)
		return ret;

	bq2415x_reset_chip(bq);

	ret = bq2415x_power_supply_init(bq);
	if (ret) {
		dev_err(bq->dev, "failed to register power supply: %d\n", ret);
		goto error_2;
	}

	ret = bq2415x_sysfs_init(bq);
	if (ret) {
		dev_err(bq->dev, "failed to create sysfs entries: %d\n", ret);
		goto error_3;
	}

	ret = bq2415x_set_defaults(bq);
	if (ret) {
		dev_err(bq->dev, "failed to set default values: %d\n", ret);
		goto error_4;
	}

	if (bq->init_data.set_mode_hook) {
		if (bq->init_data.set_mode_hook(
				bq2415x_hook_function, bq)) {
			bq->automode = 1;
			bq2415x_set_mode(bq, bq->reported_mode);
			dev_info(bq->dev, "automode enabled\n");
		} else {
			bq->automode = -1;
			dev_info(bq->dev, "automode failed\n");
		}
	} else {
		bq->automode = -1;
		dev_info(bq->dev, "automode not supported\n");
	}

	INIT_DELAYED_WORK(&bq->work, bq2415x_timer_work);
	bq2415x_set_autotimer(bq, 1);
	INIT_WORK(&bq->iusb_work,bq2415x_iusb_work);
	INIT_WORK(&bq->lower_power_charger_work,
			bq2415x_usb_low_power_work);
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_CHARGER);
#endif
	dev_info(bq->dev, "driver registered\n");
	return 0;

error_4:
	bq2415x_sysfs_exit(bq);
error_3:
	bq2415x_power_supply_exit(bq);
error_2:
	kfree(name);
error_1:
	mutex_lock(&bq2415x_id_mutex);
	idr_remove(&bq2415x_id, num);
	mutex_unlock(&bq2415x_id_mutex);

	return ret;
}

/* main bq2415x remove function */

static int bq2415x_remove(struct i2c_client *client)
{
	struct bq2415x_device *bq = i2c_get_clientdata(client);

	if (bq->init_data.set_mode_hook)
		bq->init_data.set_mode_hook(NULL, NULL);
	cancel_work_sync(&bq->lower_power_charger_work);
	bq2415x_sysfs_exit(bq);
	bq2415x_power_supply_exit(bq);

	bq2415x_reset_chip(bq);

	mutex_lock(&bq2415x_id_mutex);
	idr_remove(&bq2415x_id, bq->id);
	mutex_unlock(&bq2415x_id_mutex);

	dev_info(bq->dev, "driver unregistered\n");

	kfree(bq->name);

	return 0;
}
#if 0
static int bq24152_suspend(struct i2c_client *client,pm_message_t state)
{
    struct bq2415x_device *bq = i2c_get_clientdata(client);
    cancel_delayed_work(&bq->work);
    return 0;
}

static int bq24152_resume(struct i2c_client *client)
{
    struct bq2415x_device *bq = i2c_get_clientdata(client);
    schedule_delayed_work(&bq->work, 0);
    return 0;
}
#endif

static const struct i2c_device_id bq2415x_i2c_id_table[] = {
       { "bq2415x", BQUNKNOWN },
       { "bq24150", BQ24150 },
       { "bq24150a", BQ24150A },
       { "bq24151", BQ24151 },
       { "bq24151a", BQ24151A },
       { "bq24152", BQ24152 },
       { "bq24153", BQ24153 },
       { "bq24153a", BQ24153A },
       { "bq24155", BQ24155 },
       { "bq24156", BQ24156 },
       { "bq24156a", BQ24156A },
       { "bq24158", BQ24158 },
       {},
};

static struct of_device_id bq2419x_charger_match_table[] =
{
   { .compatible = "ti,bq24152",},
};


static struct i2c_driver bq2415x_driver = {
	.driver = {
		.name = "ti,bq24152",			
        .of_match_table = bq2419x_charger_match_table,
	},
	.probe = bq2415x_probe,
	.remove = bq2415x_remove,
#if 0
	.suspend = bq24152_suspend,
	.resume = bq24152_resume,
#endif
	.id_table = bq2415x_i2c_id_table,
};
module_i2c_driver(bq2415x_driver);

MODULE_AUTHOR("Pali Rohár <pali.rohar@gmail.com>");
MODULE_DESCRIPTION("bq2415x charger driver");
MODULE_LICENSE("GPL");
