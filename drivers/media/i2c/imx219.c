/*
 * Copyright 2016 Alban Bedel <alban.bedel@avionic-design.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define EXPOSURE_MIN 1
#define EXPOSURE_MAX 199000
#define EXPOSURE_DEFAULT 100000
#define GAIN_MIN 0
#define GAIN_MAX 224
#define GAIN_DEFAULT 100
#define WAKE_UP_DURATION 5

#define IMX219_ANA_GAIN_GLOBAL 0x157
#define IMX219_COARSE_INT_TIME_HI 0x15A
#define IMX219_COARSE_INT_TIME_LO 0x15B

static const struct regulator_bulk_data imx219_regulators[] = {
	{ "vdd" },
};

struct imx219_mode {
	struct v4l2_mbus_framefmt framefmt;
	const struct reg_default *regs;
	unsigned num_regs;
	int line_length_ns;
};

struct imx219 {
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_subdev subdev;

	struct regmap *regmap;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data regulators[ARRAY_SIZE(imx219_regulators)];

	const struct imx219_mode *mode;
};

static const struct regmap_range imx219_regmap_rw_ranges[] = {
	/* Device ID */
	regmap_reg_range(0x0000, 0x000F),
	regmap_reg_range(0x0100, 0x03FF),
	regmap_reg_range(0x3000, 0x30FF),
	regmap_reg_range(0x4500, 0x47FF),
	/* Test pattern generator */
	regmap_reg_range(0x0600, 0x0627),
};

static const struct regmap_access_table imx219_regmap_access = {
	.yes_ranges = imx219_regmap_rw_ranges,
	.n_yes_ranges = ARRAY_SIZE(imx219_regmap_rw_ranges),
};

static const struct regmap_config imx219_regmap_config = {
	.reg_bits = 16,
	.reg_stride = 1,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
	.max_register = 0xffff,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.rd_table = &imx219_regmap_access,
	.wr_table = &imx219_regmap_access,
};

/* Modes taken from the Broadcom Lollipop Wear Kernel release:
 * https://android.googlesource.com/kernel/bcm/+/
 * android-bcm-tetra-3.10-lollipop-wear-release/
 * drivers/media/video/imx219.c */
const struct reg_default regs_1920x1080_p48[] = {
	{0x0160, 0x04},
	{0x0161, 0x59},
	{0x0162, 0x0D},
	{0x0163, 0x78},
	{0x0164, 0x02},
	{0x0165, 0xA8},
	{0x0166, 0x0A},
	{0x0167, 0x27},
	{0x0168, 0x02},
	{0x0169, 0xB4},
	{0x016A, 0x06},
	{0x016B, 0xEB},
	{0x016C, 0x07},
	{0x016D, 0x80},
	{0x016E, 0x04},
	{0x016F, 0x38},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x00},
	{0x0175, 0x00},
	{0x018C, 0x0A},
	{0x018D, 0x0A},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x030C, 0x00},
	{0x030D, 0x72},
};

const struct reg_default regs_3280x2464_p15[] = {
	{0x0160, 0x09},
	{0x0161, 0xC8},
	{0x0162, 0x0D},
	{0x0163, 0x78},
	{0x0164, 0x00},
	{0x0165, 0x00},
	{0x0166, 0x0C},
	{0x0167, 0xCF},
	{0x0168, 0x00},
	{0x0169, 0x00},
	{0x016A, 0x09},
	{0x016B, 0x9F},
	{0x016C, 0x0C},
	{0x016D, 0xD0},
	{0x016E, 0x09},
	{0x016F, 0xA0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0172, 0x03},
	{0x0174, 0x00},
	{0x0175, 0x00},
	{0x018C, 0x0A},
	{0x018D, 0x0A},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x2B},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x030C, 0x00},
	{0x030D, 0x55},
};

#define IMX219_FRAMEFMT(w, h)			\
{						\
	.code = V4L2_MBUS_FMT_SRGGB10_1X10,	\
	.colorspace = V4L2_COLORSPACE_SRGB,	\
	.field = V4L2_FIELD_NONE,		\
	.width = w,				\
	.height = h,				\
}

const struct imx219_mode imx219_modes[] = {
	{
		.framefmt = IMX219_FRAMEFMT(1920, 1080),
		.regs = regs_1920x1080_p48,
		.num_regs = ARRAY_SIZE(regs_1920x1080_p48),
		.line_length_ns = 18915,
	},
	{ /* HACK: We use a width of 3264 instead of 3280 because
	     the tegra VI doesn't cope with the resulting WC alignement.
	     See https://chromium.googlesource.com/chromiumos/third_party/kernel/+/abb13dc */
		.framefmt = IMX219_FRAMEFMT(3264, 2464),
		.regs = regs_3280x2464_p15,
		.num_regs = ARRAY_SIZE(regs_3280x2464_p15),
		.line_length_ns = 26624,
	},
};

const struct reg_default regs_imx219_init[] = {
	/* Enable access to address over 0x3000 */
	{0x30EB, 0x05},
	{0x30EB, 0x0C},
	{0x300A, 0xFF},
	{0x300B, 0xFF},
	{0x30EB, 0x05},
	{0x30EB, 0x09},
	/* Set CSI mode to 2 lanes */
	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012A, 0x18},
	{0x012B, 0x00},
	/* CSI timings */
	{0x455E, 0x00},
	{0x471E, 0x4B},
	{0x4767, 0x0F},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47B4, 0x14},
	{0x4713, 0x30},
	{0x478B, 0x10},
	{0x478F, 0x10},
	{0x4797, 0x0E},
	{0x479B, 0x0E},
};

static int imx219_set_exposure(struct imx219 *imx219, int exp_value)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->subdev);
	int new_exp_value;
	int err;

	if (exp_value > EXPOSURE_MAX || exp_value < EXPOSURE_MIN) {
		dev_err(&client->dev, "wrong exposure value\n");
		return -EINVAL;
	}

	new_exp_value = (1000 * exp_value) / imx219->mode->line_length_ns;

	err = regmap_write(imx219->regmap, IMX219_COARSE_INT_TIME_HI,
				(new_exp_value >> 8) & 0xFF);
	if (err) {
		dev_err(&client->dev, "failed to write exposure_hi\n");
		return err;
	}

	err = regmap_write(imx219->regmap, IMX219_COARSE_INT_TIME_LO,
			new_exp_value & 0xFF);
	if (err)
		dev_err(&client->dev, "failed to write exposure_lo\n");

	return err;
}

static int imx219_set_gain(struct imx219 *imx219, int gain_value)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->subdev);
	int err;

	if (gain_value > GAIN_MAX || gain_value < GAIN_MIN) {
		dev_err(&client->dev, "wrong gain value\n");
		return -EINVAL;
	}

	err = regmap_write(imx219->regmap, IMX219_ANA_GAIN_GLOBAL, gain_value);
	if (err)
		dev_err(&client->dev, "failed to write gain\n");

	return err;
}

static int imx219_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx219 *imx219 = container_of(ctrl->handler, struct imx219,
						ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->subdev);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = imx219_set_exposure(imx219, ctrl->val);
		break;

	case V4L2_CID_GAIN:
		ret = imx219_set_gain(imx219, ctrl->val);
		break;

	default:
		dev_err(&client->dev, "Invalid control id\n");
		return -EINVAL;
	}

	return ret;
}

static int imx219_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index > 0)
		return -EINVAL;

	*code = V4L2_MBUS_FMT_SRGGB10_1X10;

	return 0;
}

static const struct imx219_mode *imx219_get_mode(
	struct v4l2_mbus_framefmt *fmt)
{
	unsigned i;

	/* Find a mode at least as large as requested */
	for (i = 0; i < ARRAY_SIZE(imx219_modes); i++) {
		const struct v4l2_mbus_framefmt *mode_fmt =
			&imx219_modes[i].framefmt;

		if (mode_fmt->width < fmt->width)
			continue;
		if (mode_fmt->height < fmt->height)
			continue;

		return &imx219_modes[i];
	}

	/* Fallback on the largest available mode */
	return &imx219_modes[ARRAY_SIZE(imx219_modes) - 1];
}

static int imx219_try_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	const struct imx219_mode *mode = imx219_get_mode(fmt);

	*fmt = mode->framefmt;

	return 0;
}

static int imx219_g_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	struct imx219 *priv = container_of(sd, struct imx219, subdev);

	*fmt = priv->mode->framefmt;

	return 0;
}

static int imx219_s_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	struct imx219 *priv = container_of(sd, struct imx219, subdev);
	const struct imx219_mode *mode = imx219_get_mode(fmt);
	int err;

	/* Set the mode only if needed */
	if (mode == priv->mode)
		return 0;

	/* Set the registers */
	err = regmap_multi_reg_write(priv->regmap, mode->regs, mode->num_regs);
	if (err)
		return err;

	/* And save */
	priv->mode = mode;

	return 0;
}

static int imx219_g_mbus_config(struct v4l2_subdev *sd,
		struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_2_LANE |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK |
		V4L2_MBUS_CSI2_CHANNEL_0;

	return 0;
}

static int imx219_s_stream(struct v4l2_subdev *sd, int on)
{
	struct imx219 *priv = container_of(sd, struct imx219, subdev);

	return regmap_write(priv->regmap, 0x100, on ? 1 : 0);
}

static int imx219_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *id)
{
	struct imx219 *priv = container_of(sd, struct imx219, subdev);
	u8 desc[0xF];
	int err;

	err = regmap_bulk_read(priv->regmap, 0, desc, sizeof(desc));
	if (err)
		return err;

	id->ident = ((u32)desc[0] << 8) | desc[1];
	id->revision = ((u32)desc[0xD] << 8) | desc[0xE];

	return 0;
}

static int imx219_power_up(struct imx219 *priv, struct i2c_client *client)
{
	int err;

	err = regulator_bulk_enable(ARRAY_SIZE(priv->regulators),
				priv->regulators);
	if (err) {
		dev_err(&client->dev, "failed to enable regulators\n");
		return err;
	}

	if (priv->reset_gpio) {
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		msleep(WAKE_UP_DURATION);
	}

	return err;
}

static void imx219_power_off(struct imx219 *priv, struct i2c_client *client)
{
	if (priv->reset_gpio)
		gpiod_set_value_cansleep(priv->reset_gpio, 1);

	/* Ignore errors here as we can't recover */
	regulator_bulk_disable(ARRAY_SIZE(priv->regulators), priv->regulators);
}

static int imx219_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx219 *priv = container_of(sd, struct imx219, subdev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	if (!on)
		goto power_off;

	err = imx219_power_up(priv, client);
	if (err)
		return err;

	/* Set the basic settings */
	err = regmap_multi_reg_write(priv->regmap, regs_imx219_init,
				ARRAY_SIZE(regs_imx219_init));
	if (err) {
		dev_err(&client->dev, "failed to set init settings\n");
		goto power_off;
	}

	/* And the current mode */
	err = regmap_multi_reg_write(priv->regmap, priv->mode->regs,
				priv->mode->num_regs);
	if (err) {
		dev_err(&client->dev, "failed to set mode settings\n");
		goto power_off;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,"Failed to setup control\n");
		goto power_off;
	}

	return 0;

power_off:
	imx219_power_off(priv, client);
	return err;
}

static struct v4l2_subdev_video_ops imx219_subdev_video_ops = {
	.s_mbus_fmt = imx219_s_fmt,
	.g_mbus_fmt = imx219_g_fmt,
	.try_mbus_fmt = imx219_try_fmt,
	.enum_mbus_fmt = imx219_enum_fmt,
	.g_mbus_config = imx219_g_mbus_config,
	.s_stream = imx219_s_stream,
};

static const struct v4l2_ctrl_ops imx219_ctrl_ops = {
	.s_ctrl = imx219_s_ctrl,
};

static struct v4l2_subdev_core_ops imx219_subdev_core_ops = {
	.queryctrl = v4l2_subdev_queryctrl,
	.querymenu = v4l2_subdev_querymenu,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.g_chip_ident = imx219_g_chip_ident,
	.s_power = imx219_s_power,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.g_ctrl = v4l2_subdev_g_ctrl,
};

static struct v4l2_subdev_ops imx219_subdev_ops = {
	.core = &imx219_subdev_core_ops,
	.video = &imx219_subdev_video_ops,
};


static int imx219_check_id(struct v4l2_subdev *sd)
{
	struct imx219 *priv = container_of(sd, struct imx219, subdev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned int ident_hi, ident_lo, ident;
	int err;

	err = imx219_power_up(priv, client);
	if (err)
		return err;

	/* Read id */
	err = regmap_read(priv->regmap, 0x0, &ident_hi);
	if (err)
		goto power_off;

	err = regmap_read(priv->regmap, 0x1, &ident_lo);
	if (err)
		goto power_off;

	ident = (ident_hi << 8) | ident_lo;

	if (ident != 0x219) {
		dev_err(&client->dev, "Wrong id 0x%x\n", ident);
		err = -ENODEV;
	}

power_off:
	imx219_power_off(priv, client);
	return err;
}

static int imx219_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct imx219 *priv;
	int err;

	priv = devm_kzalloc(&client->dev, sizeof(struct imx219), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	memcpy(priv->regulators, &imx219_regulators, sizeof(priv->regulators));
	err = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(priv->regulators),
				priv->regulators);
	if (err < 0) {
		if (err != -EPROBE_DEFER)
			dev_err(&client->dev, "failed to get regulators\n");
		return err;
	}

	priv->reset_gpio = devm_gpiod_get_optional(
		&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio)) {
		if (PTR_ERR(priv->reset_gpio) != -EPROBE_DEFER)
			dev_err(&client->dev, "failed to get reset GPIO\n");
		return PTR_ERR(priv->reset_gpio);
	}

	priv->mode = &imx219_modes[0];

	priv->regmap = devm_regmap_init_i2c(client, &imx219_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap ctl init failed: %ld\n",
			PTR_ERR(priv->regmap));
		return PTR_ERR(priv->regmap);
	}

	v4l2_i2c_subdev_init(&priv->subdev, client, &imx219_subdev_ops);

	err = imx219_check_id(&priv->subdev);
	if (err) {
		dev_err(&client->dev, "failed to check ID\n");
		return err;
	}

	v4l2_ctrl_handler_init(&priv->ctrl_handler, 2);

	priv->subdev.ctrl_handler = &priv->ctrl_handler;

	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			V4L2_CID_EXPOSURE, EXPOSURE_MIN, EXPOSURE_MAX,
			1, EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			V4L2_CID_GAIN, GAIN_MIN, GAIN_MAX, 1, GAIN_DEFAULT);

	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "failed to get controller\n");
		v4l2_ctrl_handler_free(&priv->ctrl_handler);
		return err;
	}

	err = v4l2_async_register_subdev(&priv->subdev);
	if (err) {
		dev_err(&client->dev, "Failed to register async subdev\n");
		return err;
	}

	return 0;
}

static int imx219_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx219 *priv = container_of(sd, struct imx219, subdev);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return 0;
}

static const struct i2c_device_id imx219_id[] = {
	{ "imx219", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, imx219_id);

#ifdef CONFIG_OF
static const struct of_device_id imx219_of_table[] = {
	{ .compatible = "sony,imx219" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx219_of_table);
#endif

static struct i2c_driver imx219_driver = {
	.driver = {
		.of_match_table = of_match_ptr(imx219_of_table),
		.name = "imx219",
		.owner = THIS_MODULE,
	},
	.probe = imx219_probe,
	.remove = imx219_remove,
	.id_table = imx219_id,
};
module_i2c_driver(imx219_driver);

MODULE_DESCRIPTION("Driver for Sony IM219 sensor");
MODULE_AUTHOR("Alban Bedel <alban.bedel@avionic-design.de>");
MODULE_LICENSE("GPL");
