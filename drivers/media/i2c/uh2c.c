/*
 * uh2c.c - Toshiba UH2C/D HDMI-CSI bridge driver
 *
 * Copyright (c) 2014, Armin Weiss <weii@zhaw.ch>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/hdmi.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-event.h>
#include <media/v4l2-of.h>
#include <media/soc_camera.h>

#include "uh2c-regmap.h"

static const struct regulator_bulk_data uh2c_regulators[] = {
	{ "vddc11" },
	{ "vdd11-hdmi" },
	{ "vdd12-mipi0" },
	{ "vdd12-mipi1" },
	{ "vddio18" },
	{ "vddio33" },
	{ "vdd33-hdmi" },
};

#define CSI_REG(n, r) ((r) + (0x200 * (n)))

struct uh2c {
	struct v4l2_subdev subdev;
	struct i2c_client *i2c_client;

	struct regmap *ctl_regmap;
	struct regmap *csi_regmap;
	struct regmap *hdmi_regmap;
	struct regmap *edid_regmap;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *int_gpio;

	struct clk *refclk;
	unsigned long refrate;

	struct regulator_bulk_data regulators[ARRAY_SIZE(uh2c_regulators)];

	struct v4l2_of_bus_mipi_csi2 ep[2];

	struct v4l2_mbus_framefmt framefmt;
	struct v4l2_fract pixelaspect;
	bool vsync;

	struct mutex lock;
};

static const enum v4l2_mbus_pixelcode uh2c_pixelcode[] = {
	V4L2_MBUS_FMT_RGB888_1X24,
	V4L2_MBUS_FMT_UYVY8_2X8,
};

static const struct regmap_range ctl_regmap_rw_ranges[] = {
	regmap_reg_range(0x0000, 0x008f),
	/* csi-registers in between */
	regmap_reg_range(0x0510, 0x0514),
	regmap_reg_range(0x0600, 0x06cc),
	regmap_reg_range(0x5000, 0x5094),
	regmap_reg_range(0x7000, 0x7016),
	regmap_reg_range(0x7082, 0x7082),
};

static const struct regmap_access_table ctl_regmap_access = {
	.yes_ranges = ctl_regmap_rw_ranges,
	.n_yes_ranges = ARRAY_SIZE(ctl_regmap_rw_ranges),
};

static const struct regmap_config ctl_regmap_config = {
	.reg_bits = 16,
	.reg_stride = 2,
	.val_bits = 16,
	.cache_type = REGCACHE_NONE,
	.max_register = 0x7fff,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.rd_table = &ctl_regmap_access,
	.wr_table = &ctl_regmap_access,
	.name = "ctl",
};

static const struct regmap_range csi_regmap_rw_ranges[] = {
	regmap_reg_range(0x0100, 0x04ff),
};

static const struct regmap_access_table csi_regmap_access = {
	.yes_ranges = csi_regmap_rw_ranges,
	.n_yes_ranges = ARRAY_SIZE(csi_regmap_rw_ranges),
};

static const struct regmap_config csi_regmap_config = {
	.reg_bits = 16,
	.reg_stride = 4,
	.val_bits = 32,
	.cache_type = REGCACHE_NONE,
	.max_register = 0x04ff,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.rd_table = &csi_regmap_access,
	.wr_table = &csi_regmap_access,
	.name = "csi",
};

static const struct regmap_range hdmi_regmap_rw_ranges[] = {
	regmap_reg_range(0x8410, 0x8ab0),
};

static const struct regmap_access_table hdmi_regmap_access = {
	.yes_ranges = hdmi_regmap_rw_ranges,
	.n_yes_ranges = ARRAY_SIZE(hdmi_regmap_rw_ranges),
};

static const struct regmap_config hdmi_regmap_config = {
	.reg_bits = 16,
	.reg_stride = 1,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
	.max_register = 0x8fff,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.rd_table = &hdmi_regmap_access,
	.wr_table = &hdmi_regmap_access,
	.name = "hdmi",
};

static const struct regmap_range edid_regmap_rw_ranges[] = {
	regmap_reg_range(0x8c00, 0x8cff),
};

static const struct regmap_access_table edid_regmap_access = {
	.yes_ranges = edid_regmap_rw_ranges,
	.n_yes_ranges = ARRAY_SIZE(edid_regmap_rw_ranges),
};

static const struct regmap_config edid_regmap_config = {
	.reg_bits = 16,
	.reg_stride = 1,
	.val_bits = 32,
	.cache_type = REGCACHE_NONE,
	.max_register = 0x8fff,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.rd_table = &edid_regmap_access,
	.wr_table = &edid_regmap_access,
	.name = "edid",
};

static int uh2c_hdmi_read_u16(struct uh2c *priv, unsigned reg,
			unsigned int *val)
{
	unsigned low, high;
	int err;

	err = regmap_read(priv->hdmi_regmap, reg, &low);
	if (!err)
		err = regmap_read(priv->hdmi_regmap, reg + 1, &high);
	if (!err)
		*val = (high << 8) | low;

	return err;
}

static int uh2c_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(uh2c_pixelcode))
	    return -EINVAL;

	*code = uh2c_pixelcode[index];
	return 0;
}

static int uh2c_priv_try_fmt(struct uh2c *priv,
		struct v4l2_mbus_framefmt *fmt)
{
	if (!priv->vsync)
		return -ENODATA;

	switch(fmt->code) {
	case V4L2_MBUS_FMT_RGB888_1X24:
		fmt->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	case V4L2_MBUS_FMT_UYVY8_2X8:
		switch (fmt->colorspace) {
		case V4L2_COLORSPACE_SMPTE170M:
		case V4L2_COLORSPACE_REC709:
			break;
		default:
			fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
		}
		break;
	default: /* Default to progressive RGB */
		fmt->code = V4L2_MBUS_FMT_RGB888_1X24;
		fmt->field = V4L2_FIELD_NONE;
		fmt->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}

	fmt->width = priv->framefmt.width;
	fmt->height = priv->framefmt.height;
	fmt->field = priv->framefmt.field;

	dev_info(&priv->i2c_client->dev,
		"Try format return: width=%d, height=%d, code=%x, field=%x\n",
		fmt->width, fmt->height, fmt->code, fmt->field);

	return 0;
}


static int uh2c_try_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct uh2c *priv = i2c_get_clientdata(client);
	int err;

	mutex_lock(&priv->lock);
	err = uh2c_priv_try_fmt(priv, fmt);
	mutex_unlock(&priv->lock);

	return err;
}

static int uh2c_g_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct uh2c *priv = i2c_get_clientdata(client);
	int err = 0;

	mutex_lock(&priv->lock);
	if (priv->vsync)
		*fmt = priv->framefmt;
	else
		err = -ENODATA;
	mutex_unlock(&priv->lock);

	return err;
}

static int uh2c_s_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct uh2c *priv = i2c_get_clientdata(client);
	unsigned int mode, colorspace;
	unsigned csi_fmt;
	int err;

	mutex_lock(&priv->lock);
	err = uh2c_priv_try_fmt(priv, fmt);
	if (err)
		goto finish;

	switch(fmt->code) {
	case V4L2_MBUS_FMT_RGB888_1X24:
		mode = 0;
		csi_fmt = 0x24;
		break;
	case V4L2_MBUS_FMT_UYVY8_2X8:
		mode = 1;
		csi_fmt = 0x1E;
		break;
	default:
		err = -EINVAL;
		goto finish;
	}

	switch(fmt->colorspace) {
	case V4L2_COLORSPACE_SRGB:
		colorspace = 0; /* RGB Full */
		break;
	case V4L2_COLORSPACE_SMPTE170M:
		colorspace = 3; /* 601 YCbCr Limited */
		break;
	case V4L2_COLORSPACE_REC709:
		colorspace = 5; /* 709 YCbCr Limited */
		break;
	default:
		err = -EINVAL;
		goto finish;
	}

	err = regmap_write(priv->hdmi_regmap, 0x8A00, mode);
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x8A08,
				(colorspace << 4) | 1);
	if (!err)
		err = regmap_update_bits(priv->ctl_regmap, 0x0004,
					3 << 6, (mode ? 3 : 0) << 6);
	/* Set the packet type for interlaced formats */
	if (!err)
		err = regmap_write(priv->ctl_regmap, 0x5002,
				(csi_fmt << 8) | csi_fmt);
	if (!err)
		err = regmap_write(priv->ctl_regmap, 0x5082,
				(csi_fmt << 8) | csi_fmt);

	/* Setup the splitter */
	if (priv->ep[0].flags) {
		if (!err)
			err = regmap_write(priv->ctl_regmap,
					0x5000, BIT(8) | BIT(0));
		if (!err)
			err = regmap_write(priv->ctl_regmap,
					0x500C, 0);
		if (!err)
			err = regmap_write(priv->ctl_regmap,
					0x500E, fmt->width);
		if (!err)
			err = regmap_write(priv->ctl_regmap,
					0x5080, 0);
	} else {
		if (!err)
			err = regmap_write(priv->ctl_regmap,
					0x5000, 0);
		if (!err)
			err = regmap_write(priv->ctl_regmap,
					0x500C, BIT(14));
		if (!err)
			err = regmap_write(priv->ctl_regmap,
					0x5080, BIT(8) | BIT(0));
		if (!err)
			err = regmap_write(priv->ctl_regmap,
					0x508C, 0);
		if (!err)
			err = regmap_write(priv->ctl_regmap,
					0x508E, fmt->width);
	}

	if (err)
		dev_err(&client->dev, "Failed to set format\n");

	dev_info(&priv->i2c_client->dev,
		"Set format return: width=%d, height=%d, code=%x\n",
		fmt->width, fmt->height, fmt->code);

finish:
	mutex_unlock(&priv->lock);
	return err;
}

static int uh2c_g_mbus_config(struct v4l2_subdev *sd,
		struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_4_LANE |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK |
		V4L2_MBUS_CSI2_CHANNEL_0;

	return 0;
}

static int uh2c_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *cc)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct uh2c *priv = i2c_get_clientdata(client);

	cc->bounds.left = 0;
	cc->bounds.top = 0;
	cc->bounds.width = priv->framefmt.width;
	cc->bounds.height = priv->framefmt.height;
	cc->defrect = cc->bounds;
	cc->pixelaspect = priv->pixelaspect;

	return 0;
}

static int uh2c_s_stream(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct uh2c *priv = i2c_get_clientdata(client);
	int i, err = 0;

	mutex_lock(&priv->lock);

	if (!priv->vsync) {
		mutex_unlock(&priv->lock);
		return -ENODATA;
	}

	if (on) {
		unsigned long enable = BIT(15);

		for (i = 0; !err && i < ARRAY_SIZE(priv->ep); i++) {
			if (!priv->ep[i].flags)
				continue;
			enable |= BIT(i);
			err = regmap_write(priv->csi_regmap,
					CSI_REG(i, 0x011C), 1);
		}

		if (!err)
			err = regmap_update_bits(priv->ctl_regmap, 0x0004,
						BIT(0) | BIT(1) | BIT(15),
						enable);
		/* Disable the video mute */
		if (!err)
			err = regmap_write(priv->hdmi_regmap, 0x857D, 0);
	} else {
		err = regmap_update_bits(priv->ctl_regmap, 0x0004,
					BIT(0) | BIT(1), 0);
	}

	mutex_unlock(&priv->lock);

	return err;
}


static int uh2c_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct uh2c *priv = i2c_get_clientdata(client);
	unsigned int val;
	int err;

	err = regmap_read(priv->ctl_regmap, 0x0000, &val);
	if (err)
		return err;

	id->ident = (val >> 8) & 0xFF;
	id->revision = val & 0xFF;

	return 0;
}

static int uh2c_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static struct v4l2_subdev_video_ops uh2c_subdev_video_ops = {
	.s_mbus_fmt = uh2c_s_fmt,
	.g_mbus_fmt = uh2c_g_fmt,
	.try_mbus_fmt = uh2c_try_fmt,
	.enum_mbus_fmt = uh2c_enum_fmt,
	.cropcap = uh2c_cropcap,
	.g_mbus_config = uh2c_g_mbus_config,
	.s_stream = uh2c_s_stream,
};

static struct v4l2_subdev_core_ops uh2c_subdev_core_ops = {
	.g_chip_ident = uh2c_g_chip_ident,
	.s_power = uh2c_s_power,
	.subscribe_event = v4l2_src_change_event_subdev_subscribe,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static struct v4l2_subdev_ops uh2c_subdev_ops = {
	.core = &uh2c_subdev_core_ops,
	.video = &uh2c_subdev_video_ops,
};

static int uh2c_hdmi_sys_irq_handler(struct uh2c *priv)
{
	struct i2c_client *client = priv->i2c_client;
	unsigned int status, mask;
	int err;

	/* Get the current status and mask */
	err = regmap_read(priv->hdmi_regmap, 0x8502, &status);
	if (!err)
		err = regmap_read(priv->hdmi_regmap, 0x8512, &mask);
	if (err)
		return 0;

	status &= ~mask;
	if (!status)
		return 0;

	/* And clear it */
	regmap_write(priv->hdmi_regmap, 0x8502, status);

	dev_info(&client->dev, "Got IRQ, HDMI SYS = %x\n", status);
	return 1;
}

static int uh2c_hdmi_clock_irq_handler(struct uh2c *priv)
{
	struct i2c_client *client = priv->i2c_client;
	unsigned int status, mask;
	int err;

	/* Get the current status and mask */
	err = regmap_read(priv->hdmi_regmap, 0x8503, &status);
	if (!err)
		err = regmap_read(priv->hdmi_regmap, 0x8513, &mask);
	if (err)
		return 0;

	status &= ~mask;
	if (!status)
		return 0;

	if (status & BIT(3))
		dev_info(&client->dev, "Deep Color mode changed\n");
	if (status & BIT(4)) {
		unsigned int high, height, width;
		regmap_read(priv->hdmi_regmap, 0x858E, &width);
		regmap_read(priv->hdmi_regmap, 0x858F, &high);
		width |= high << 8;
		regmap_read(priv->hdmi_regmap, 0x8590, &height);
		regmap_read(priv->hdmi_regmap, 0x8591, &high);
		height |= high << 8;
		dev_info(&client->dev, "HV counter changed: %ux%u\n",
			 width, height);
	}
	if (status & BIT(5)) {
		unsigned int high, hsize, vsize;
		regmap_read(priv->hdmi_regmap, 0x8582, &hsize);
		regmap_read(priv->hdmi_regmap, 0x8583, &high);
		hsize |= high << 8;
		regmap_read(priv->hdmi_regmap, 0x858C, &vsize);
		regmap_read(priv->hdmi_regmap, 0x858D, &high);
		vsize |= high << 8;
		dev_info(&client->dev, "DE size changed: %ux%u\n",
			 hsize, vsize);
	}

	/* And clear it */
	regmap_write(priv->hdmi_regmap, 0x8503, status);

	dev_info(&client->dev, "Got IRQ, HDMI CLOCK = %x\n", status);
	return 1;
}

static int uh2c_hdmi_packet_irq_handler(struct uh2c *priv)
{
	struct i2c_client *client = priv->i2c_client;
	union hdmi_infoframe frame;
	unsigned int status, mask;
	u8 buffer[32];
	int err;

	/* Get the current status and mask */
	err = regmap_read(priv->hdmi_regmap, 0x8504, &status);
	if (!err)
		err = regmap_read(priv->hdmi_regmap, 0x8514, &mask);
	if (err)
		return 0;

	status &= ~mask;
	if (!status)
		return 0;

	if (status & BIT(0)) {
		dev_info(&client->dev, "Got AVI info packet\n");
		err = regmap_bulk_read(priv->hdmi_regmap, 0x8710, buffer, 20);
		if (!err)
			err = hdmi_infoframe_unpack(&frame, buffer);
		if (!err) {
			print_hex_dump(KERN_INFO, "AVI: ", DUMP_PREFIX_NONE, 16, 1,
				buffer, sizeof(buffer), false);
			dev_info(&client->dev, "vic=%u, content=%u, repeat=%u\n",
				frame.avi.video_code, frame.avi.content_type,
				frame.avi.pixel_repeat);
			dev_info(&client->dev, "top=%u, bottom=%u, left=%u, right=%u\n",
				frame.avi.top_bar, frame.avi.bottom_bar,
				frame.avi.left_bar, frame.avi.right_bar);
		} else {
			dev_err(&client->dev, "Failed to parse AVI frame\n");
		}
	}
	if (status & BIT(1)) {
		dev_info(&client->dev, "Got Audio info packet\n");
	}

	/* And clear it */
	regmap_write(priv->hdmi_regmap, 0x8504, status);

	dev_info(&client->dev, "Got IRQ, HDMI PACKET = %x\n", status);
	return 1;
}

static int uh2c_hdmi_vsync_changed_irq_handler(struct uh2c *priv)
{
	unsigned int status, width, height, vi_status, vi_status1, clk_status;
	struct v4l2_event ev = {};
	unsigned int repeat = 0;
	int err;

	err = regmap_read(priv->hdmi_regmap, 0x8520, &status);
	if (err)
		return 0;

	dev_info(&priv->i2c_client->dev, "VSync changed: %s (0x%02x)\n",
		(status & BIT(7)) ? "found" : "lost", status);

	if (!err)
		err = regmap_read(priv->hdmi_regmap, 0x8521, &vi_status);
	if (!err)
		err = regmap_read(priv->hdmi_regmap, 0x8522, &vi_status1);
	if (!err)
		err = regmap_read(priv->hdmi_regmap, 0x8526, &clk_status);
	if (!err)
		dev_info(&priv->i2c_client->dev,
			"VI Status = 0x%02x, status1 = 0x%02x, clk_status = 0x%02x\n",
			vi_status, vi_status1, clk_status);

	mutex_lock(&priv->lock);
	priv->vsync = false;
	if (status & BIT(7)) {
		err = uh2c_hdmi_read_u16(priv, 0x8582, &width);
		if (!err)
			err = uh2c_hdmi_read_u16(priv, 0x858C, &height);
		if (!err) {
			repeat = ((vi_status >> 4) & 0xF) + 1;
			priv->framefmt.width = width / repeat;
			priv->framefmt.height = height;
			if (vi_status1 & BIT(0)) {
				priv->framefmt.height *= 2;
				priv->framefmt.field = V4L2_FIELD_INTERLACED_TB;
			} else {
				priv->framefmt.field = V4L2_FIELD_NONE;
			}
			/* 480i/p and 576i/p have special aspect ratio */
			switch(priv->framefmt.height) {
			case 576:
				priv->pixelaspect.numerator = 16;
				priv->pixelaspect.denominator = 15;
				break;
			case 480:
				priv->pixelaspect.numerator = 8;
				priv->pixelaspect.denominator = 9;
				break;
			default:
				priv->pixelaspect.numerator = 1;
				priv->pixelaspect.denominator = 1;
				break;
			}
			/* Correct the aspect to account for pixel repeating */
			priv->pixelaspect.numerator *= repeat;
			priv->vsync = true;
			ev.type = V4L2_EVENT_SOURCE_CHANGE;
			ev.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION;

			dev_info(&priv->i2c_client->dev,
				"Got new resolution: %ux%u%c (repeat %d)\n",
				width, height, (vi_status1 & BIT(0)) ? 'i' : 'p',
				repeat);
		}
	}

	/* If any of the above failed, report an End Of Stream */
	if (!priv->vsync) {
		priv->framefmt.width = 0;
		priv->framefmt.height = 0;
		priv->framefmt.field = V4L2_FIELD_ANY;
		priv->pixelaspect.numerator = 1;
		priv->pixelaspect.denominator = 1;
		ev.type = V4L2_EVENT_EOS;
	}

	mutex_unlock(&priv->lock);

	v4l2_subdev_notify(&priv->subdev, V4L2_DEVICE_NOTIFY_EVENT, &ev);

	return 1;
}

static int uh2c_hdmi_misc_irq_handler(struct uh2c *priv)
{
	struct i2c_client *client = priv->i2c_client;
	unsigned int status, mask;
	int err;

	/* Get the current status and mask */
	err = regmap_read(priv->hdmi_regmap, 0x850b, &status);
	if (!err)
		err = regmap_read(priv->hdmi_regmap, 0x851b, &mask);
	if (err)
		return 0;

	status &= ~mask;
	if (!status)
		return 0;

	if (status & BIT(1))
		uh2c_hdmi_vsync_changed_irq_handler(priv);
	if (status & BIT(5)) {
		dev_info(&client->dev, "Got video color changed\n");
	}

	/* And clear it */
	regmap_write(priv->hdmi_regmap, 0x850b, status);

	dev_info(&client->dev, "Got IRQ, HDMI MISC = %x\n", status);
	return 1;
}

static int uh2c_hdmi_irq_handler(struct uh2c *priv)
{
	struct i2c_client *client = priv->i2c_client;
	unsigned int int0, int1;
	int err, ret = 0;

	err = regmap_read(priv->hdmi_regmap, 0x8500, &int0);
	if (!err)
		err = regmap_read(priv->hdmi_regmap, 0x8501, &int1);
	if (err)
		return 0;

	dev_info(&client->dev, "Got IRQ, HDMI INT0 = %x, INT1 = %x\n", int0, int1);

	if (int0 & BIT(1))
		ret += uh2c_hdmi_misc_irq_handler(priv);
	if (int1 & BIT(0))
		ret += uh2c_hdmi_sys_irq_handler(priv);
	if (int1 & BIT(1))
		ret += uh2c_hdmi_clock_irq_handler(priv);
	if (int1 & BIT(2))
		ret += uh2c_hdmi_packet_irq_handler(priv);

	return ret;
}

static irqreturn_t uh2c_irq_handler(int irq, void *ctx)
{
	struct uh2c *priv = ctx;
	struct i2c_client *client = priv->i2c_client;
	unsigned int status;
	int err, ret = 0;

	err = regmap_read(priv->ctl_regmap, 0x0014, &status);
	if (err) {
		dev_err(&client->dev, "Failed to read IRQ status\n");
		return IRQ_NONE;
	}

	dev_info(&client->dev, "Got IRQ status = %x\n", status);

	if (status & BIT(9))
		ret += uh2c_hdmi_irq_handler(priv);

	/* Clear the status */
	regmap_write(priv->ctl_regmap, 0x0014, status);

	return ret > 0 ? IRQ_HANDLED : IRQ_NONE;
}

static int uh2c_load_edid(struct uh2c *priv, const void *data, unsigned size)
{
	int err;

	/* Max 1k */
	if (size > 0x400)
		return -EFBIG;
	/* Size must be multiple of 4 */
	if (size & 3)
		return -EINVAL;

	err = regmap_bulk_write(priv->edid_regmap, 0x8C00, data, size / 4);

	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x85E3, size & 0xFF);
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x85E3,
				   (size >> 8) & 0xFF);

	return err;
}

static const u32 uh2c_default_edid[] = {
	0xFFFFFF00, 0x00FFFFFF, 0x02096252, 0x01010101,
	0x030114FF, 0x785AA080, 0xA0C90D0A, 0x27984757,
	0x2F4C4812, 0x808100CF, 0x01010101, 0x01010101,
	0x01010101, 0x3A020101, 0x38711880, 0x2C58402D,
	0x84400045, 0x1E000063, 0xB0502166, 0x301B0051,
	0x00367040, 0x0063843A, 0x00001E00, 0x5400FC00,
	0x4948534F, 0x542D4142, 0x20200A56, 0xFD000000,
	0x0F4C1700, 0x0A000F51, 0x20202020, 0xA9012020,
	0x70250302, 0x04051049, 0x06020703, 0x09262001,
	0x07150707, 0x0C036CC0, 0x38003000, 0x2B2BCF2D,
	0x00E23333, 0x801D017F, 0x161C7118, 0x252C5820,
	0x63844000, 0x8C9E0000, 0x208AD00A, 0x10102DE0,
	0xB000963E, 0x00004384, 0x001F0E18, 0x1E005180,
	0x37804030, 0x5384DC00, 0xF11C0000, 0x51A00027,
	0x50302500, 0xDC003780, 0x00005384, 0x001AA91C,
	0x160050A0, 0x37203030, 0x5384DC00, 0xA21A0000,
};
#if 0
static int uh2c_calibrate(struct i2c_client *client)
{
	int tries = 10;
	int err;
	u32 val;

	/* Reset the calibration */
	err = uh2c_write_reg(client, 0x024C, UH2C_CSI, BIT(2));
	if (err)
		return err;
	usleep_range(1, 10);

	/* Start the calibration */
	err = uh2c_write_reg(client, 0x024C, UH2C_CSI, BIT(0));
	if (err)
		return err;

	do {
		usleep_range(100, 1000);
		err = uh2c_read_reg(client, 0x024C, UH2C_CSI, &val);
		if (err)
			return err;
		if (!(val & BIT(0))) {
			dev_err(&client->dev, "Calibration done!\n");
			return 0;
		}
		tries--;
	} while (tries > 0);

	dev_err(&client->dev, "Calibration timed out!\n");
	return -EINVAL;
}
#endif
static unsigned int clk_count(u64 rate, unsigned int ns)
{
	rate *= ns;
	if(do_div(rate, 1000000000))
		rate++; /* Round up the count */
	return rate;
}

static int uh2c_init_csi_tx(struct uh2c *priv, unsigned id,
			    unsigned long csi_rate)
{
	unsigned int tclk_pre, tclk_prepare, tclk_zero, tclk_exit, tclk_trail;
	unsigned int ths_prepare, ths_zero, ths_exit, ths_trail;
	unsigned int lptxcnt, t_wakeup, tclk_post;
	unsigned long hsck_rate = csi_rate * 2;
	u32 frs = -1, prd, diff, best_diff = -1;
	u32 best_prd = -1, best_fbd = -1;
	int err, csi_shift;
	u64 fbd;

	for (csi_shift = 3; csi_shift > 0; csi_shift -= 1) {
		hsck_rate = csi_rate << csi_shift;

		/* Too fast, try the next rate */
		if (hsck_rate > 1000000000)
			continue;
		/* Too slow, abort the next rate will be lower */
		if (hsck_rate < 62500000)
			return -EINVAL;

		if (hsck_rate > 500000000)
			frs = 0;
		else if (hsck_rate > 250000000)
			frs = 1;
		else if (hsck_rate > 125000000)
			frs = 2;
		else
			frs = 3;
		break;
	}

	/* Make sure we have valid settings */
	if (frs == -1)
		return -EINVAL;

	/* Try all pre divider values and find the best one */
	for (prd = 1; prd <= 16 && best_diff; prd++) {
		fbd = ((u64)hsck_rate * prd) << frs;
		diff = do_div(fbd, priv->refrate);
		if (fbd > 0x100)
			continue;
		if (diff < best_diff) {
			best_prd = prd;
			best_fbd = fbd;
			best_diff = diff;
		}
	}

	/* Make sure have valid settings */
	if (best_prd == -1 || best_fbd == -1) {
		dev_err(&priv->i2c_client->dev,
			"Failed to configure CSI PLL\n");
		return -EINVAL;
	}

	/* Compute the effective CSI rate and various timings */
	csi_rate = (priv->refrate / best_prd * best_fbd) >> (frs + csi_shift);

	/* LPTX clk must be less than 20MHz */
	lptxcnt = (csi_rate + 20000000 - 1) / 20000000;
	if (lptxcnt < 3)
		lptxcnt = 3;
	/* TWAKEUP >= 1ms (in LPTX clock count) */
	t_wakeup = clk_count(csi_rate / lptxcnt, 1000000);

	/* TCLK_PRE >= 8UI */
	tclk_pre = 4;
	/* 38ns <= TCLK_PREPARE <= 95ns */
	tclk_prepare = clk_count(csi_rate, 60);
	/* TCLK_ZERO + TCLK_PREPARE >= 300ns */
	tclk_zero = clk_count(csi_rate, 300) - tclk_prepare;
	/* TCLK_EXIT >= 100 ns */
	tclk_exit = clk_count(csi_rate, 100);
	/* TCLK_TRAIL >= 60ns */
	tclk_trail = clk_count(csi_rate, 60);
	/* TCLK_POST >= 60ns + 52*UI */
	tclk_post = clk_count(csi_rate, 60) + 26;

	/* 40ns + 4*UI <= THS_PREPARE <= 85ns + 6*UI, use 60ns + 4 UI */
	ths_prepare = clk_count(csi_rate, 60) + 2;
	/* THS_ZERO + THS_PREPARE >= 145ns + 10*UI */
	ths_zero = clk_count(csi_rate, 145) + 5 - ths_prepare;
	/* THS_EXIT >= 100ns */
	ths_exit = clk_count(csi_rate, 100);
	/* THS_TRAIL >= max(8*UI, 60ns + 4*UI) */
	ths_trail = clk_count(csi_rate, 60) + 2;
	if (ths_trail < 4)
		ths_trail = 4;

	/* Power the block */
	err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x0108), 1);
	if (err)
		return err;

	/* Configure the PLL */
	if (!err)
		err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x02AC),
				((best_prd - 1) << 16) | (frs << 10) |
				(best_fbd - 1));

	/* Select the clocks for CSI clock and data */
	if (!err)
		err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x010C),
				(3 - csi_shift) << 10 | (3 - csi_shift) << 8 |
				BIT(0));

	/* LINEINITCOUNT >= 100us */
	if (!err)
		err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x0120),
				clk_count(csi_rate, 100000));
	/* HSTX_TO_COUNT = 0 */
	if (!err)
		err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x0124), 0);
	/* Write the MIPI timings */
	if (!err)
		err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x0254),
				lptxcnt - 1);
	if (!err)
		err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x0258),
				(tclk_prepare << 16) | (tclk_pre << 8) |
				tclk_prepare);
	if (!err)
		err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x025C),
				(tclk_exit << 16) | tclk_trail);
	if (!err)
		err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x0260),
				(ths_zero << 16) | ths_prepare);
	if (!err)
		err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x0264),
				t_wakeup);
	if (!err)
		err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x0268),
				tclk_post);
	if (!err)
		err = regmap_write(priv->csi_regmap, CSI_REG(id, 0x026C),
				(ths_exit << 16) | ths_trail);
	/* TODO: Find out how to compute this value. Which clock drives
	 * the counter, and how long should we wait?  */
	if (!err)
		err = regmap_write(priv->csi_regmap,
				CSI_REG(id, 0x0270), 0x20);

	/* Enable all the voltage regulators */
	if (!err)
		err = regmap_write(priv->csi_regmap,
				CSI_REG(id, 0x0274), 0x1F);

	/* Enable the PLL */
	if (!err)
		err = regmap_write(priv->csi_regmap,
				CSI_REG(id, 0x02A0), 3);
	/* Enable the HSTX clock */
	if (!err)
		err = regmap_update_bits(priv->csi_regmap,
					CSI_REG(id, 0x010C), 1, 1);

	/* Set the wrappers for interlaced */
	if (!err)
		err = regmap_write(priv->ctl_regmap, 0x0510 + id * 4, 2);

	/* Set the number of lanes */
	if (!err)
		err = regmap_write(priv->csi_regmap, 0x0118,
				BIT(4) | priv->ep[id].num_data_lanes);

	return err;
}

static int uh2c_priv_init(struct uh2c *priv)
{
	u64 nco;
	int err;

	/* Disable everything, but enable I2C address increments */
	err = regmap_write(priv->ctl_regmap, 0x0004, BIT(2));

	/* Take out of sleep */
	if (!err)
		err = regmap_write(priv->ctl_regmap, 0x0002, BIT(7));

	/* TODO: Soft reset when no reset gpio is available */

	/* HDMI system clock */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x8540,
				   (priv->refrate / 10000) & 0xFF);
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x8541,
				   ((priv->refrate / 10000) >> 8) & 0xFF);
	/* Audio system clock */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x8630,
				   (priv->refrate / 100) & 0xFF);
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x8631,
				   ((priv->refrate / 100) >> 8) & 0xFF);
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x8632,
				   ((priv->refrate / 100) >> 16) & 0xFF);
	/* Audio PLL */
	if (priv->refrate == 42000000) {
		if (!err)
			err = regmap_write(priv->hdmi_regmap, 0x8670, 0);
	} else {
		if (!err)
			err = regmap_write(priv->hdmi_regmap, 0x8670, 2);

		nco = (u64)6144000 * (1 << 28);
		do_div(nco, priv->refrate);
		if (!err)
			err = regmap_write(priv->hdmi_regmap, 0x8671,
					   nco & 0xFF);
		if (!err)
			err = regmap_write(priv->hdmi_regmap, 0x8672,
					   (nco >> 8) & 0xFF);
		if (!err)
			err = regmap_write(priv->hdmi_regmap, 0x8673,
					   (nco >> 16) & 0xFF);
		if (!err)
			err = regmap_write(priv->hdmi_regmap, 0x8674,
					   (nco >> 24) & 0xFF);

		nco = (u64)5644800 * (1 << 28);
		do_div(nco, priv->refrate);
		if (!err)
			err = regmap_write(priv->hdmi_regmap, 0x8675,
					   nco & 0xFF);
		if (!err)
			err = regmap_write(priv->hdmi_regmap, 0x8676,
					   (nco >> 8) & 0xFF);
		if (!err)
			err = regmap_write(priv->hdmi_regmap, 0x8677,
					   (nco >> 16) & 0xFF);
		if (!err)
			err = regmap_write(priv->hdmi_regmap, 0x8678,
					   (nco >> 24) & 0xFF);
	}

	/* CSC controller */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x8A0C,
				   (priv->refrate / 10000) & 0xFF);
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x8A0D,
				   ((priv->refrate / 10000) >> 8) & 0xFF);

	/* Enable the audio PLL */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x84F0, 0x31);

	/* Load the EDID data */
	if (!err)
		err = uh2c_load_edid(priv, uh2c_default_edid,
				     sizeof(uh2c_default_edid));
	/* Set the EDID mode to RAM */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x85E0, 1);

	/* Link the PHY to DDC */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x8410, 3);
	/* Enable the DCC */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x84F4, 1);

	/* Enable auto video mute */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x857A, 3);

	/* Enable the HDMI system IRQ we need */
	if (0 && !err)
		err = regmap_write(priv->hdmi_regmap, 0x8512,
				   ~(BIT(0) | BIT(4) | BIT(5)));
	/* Enable the HDMI clock IRQ we need */
	if (0 && !err)
		err = regmap_write(priv->hdmi_regmap, 0x8513,
				   ~(BIT(3) | BIT(4) | BIT(5)));
	/* Enable the HDMI packet IRQ we need */
	if (0 && !err)
		err = regmap_write(priv->hdmi_regmap, 0x8514,
				   ~(BIT(0) | BIT(1)));
	/* Enable the HDMI misc IRQ we need */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x851b,
				   ~(BIT(1) | BIT(5)));

	/* Enable the HDMI IRQ */
	if (!err)
		err = regmap_write(priv->ctl_regmap, 0x0016, ~(BIT(9)));

	/* Finish the HDMI init */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x854A, 1);

	/* Setup the CSI transmiters */
	if (!err && priv->ep[0].flags)
		err = uh2c_init_csi_tx(priv, 0, 480000000);
	if (!err && priv->ep[1].flags)
		err = uh2c_init_csi_tx(priv, 1, 480000000);

	/* Set the default format to RGB */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x8A00, 0);
	priv->framefmt.code = V4L2_MBUS_FMT_RGB888_1X24;

	/* Enable the CSC */
	if (!err)
		err = regmap_write(priv->hdmi_regmap, 0x8A08, 1);

	return err;
}

static int uh2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = NULL;
	struct uh2c *priv;
	int err;

	priv = devm_kzalloc(&client->dev, sizeof(struct uh2c), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	/* Read the ports config from OF */
	while ((np = v4l2_of_get_next_endpoint(client->dev.of_node, np))) {
		struct v4l2_of_endpoint ep;

		if (!of_device_is_available(np))
			continue;

		v4l2_of_parse_endpoint(np, &ep);
		of_node_put(np);

		if (ep.bus_type != V4L2_MBUS_CSI2 || ep.port > 0 || ep.id > 1) {
			dev_err(&client->dev, "Endpoint is invalid\n");
			return -EINVAL;
		}

		priv->ep[ep.id] = ep.bus.mipi_csi2;
	}

	if (priv->ep[0].flags == 0 && priv->ep[1].flags == 0) {
		dev_err(&client->dev, "No port configured\n");
		return -EINVAL;
	}

	if (priv->ep[0].flags && priv->ep[1].flags) {
		dev_err(&client->dev,
			"Dual port configuration not yet supported\n");
		return -EINVAL;
	}

	memcpy(priv->regulators, &uh2c_regulators, sizeof(priv->regulators));
	err = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(priv->regulators),
				priv->regulators);
	if (err < 0) {
		if (err != -EPROBE_DEFER)
			dev_err(&client->dev, "failed to get regulators\n");
		return err;
	}
	/* TODO: Add a fixed-clock implementation so we can use this code */
#if 0
	priv->refclk = devm_clk_get(&client->dev, NULL);
	if (IS_ERR(priv->refclk)) {
		if (PTR_ERR(priv->refclk) != -EPROBE_DEFER)
			dev_err(&client->dev, "failed to get reference clock\n");
		return PTR_ERR(priv->refclk);
	}

	priv->refrate = clk_get_rate(priv->refclk);
#else
	priv->refrate = 48000000;
#endif

	if (priv->refrate < 40000000 || priv->refrate > 50000000) {
		dev_err(&client->dev, "reference is out of range: %lu\n",
			priv->refrate);
		return -EINVAL;
	}

	priv->reset_gpio = devm_gpiod_get_optional(
		&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio)) {
		if (PTR_ERR(priv->reset_gpio) != -EPROBE_DEFER)
			dev_err(&client->dev, "failed to get reset GPIO\n");
		return PTR_ERR(priv->reset_gpio);
	}

	mutex_init(&priv->lock);

	err = regulator_bulk_enable(
		ARRAY_SIZE(priv->regulators), priv->regulators);
	if (err) {
		dev_err(&client->dev, "failed to enable regulators\n");
		return err;
	}

	if (priv->refclk) {
		err = clk_prepare_enable(priv->refclk);
		if (err) {
			dev_err(&client->dev, "failed to enable reference clock\n");
			goto regulators_disable;
		}
	}
	usleep_range(10000, 20000);

	if (priv->reset_gpio) {
		/* Release Reset */
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		usleep_range(10, 20);
	}

	priv->i2c_client = client;
	i2c_set_clientdata(client, priv);

	priv->ctl_regmap = devm_regmap_init_i2c(client, &ctl_regmap_config);
	if (IS_ERR(priv->ctl_regmap)) {
		dev_err(&client->dev,
			"regmap ctl init failed: %ld\n", PTR_ERR(priv->ctl_regmap));
		err = PTR_ERR(priv->ctl_regmap);
		goto reset;
	}

	priv->csi_regmap = devm_regmap_init_i2c(client, &csi_regmap_config);
	if (IS_ERR(priv->csi_regmap)) {
		dev_err(&client->dev,
			"regmap csi init failed: %ld\n", PTR_ERR(priv->csi_regmap));
		err = PTR_ERR(priv->csi_regmap);
		goto reset;
	}

	priv->hdmi_regmap = devm_regmap_init_i2c(client, &hdmi_regmap_config);
	if (IS_ERR(priv->hdmi_regmap)) {
		dev_err(&client->dev,
			"regmap hdmi init failed: %ld\n", PTR_ERR(priv->hdmi_regmap));
		err = PTR_ERR(priv->hdmi_regmap);
		goto reset;
	}

	priv->edid_regmap = devm_regmap_init_i2c(client, &edid_regmap_config);
	if (IS_ERR(priv->edid_regmap)) {
		dev_err(&client->dev,
			"regmap edid init failed: %ld\n", PTR_ERR(priv->edid_regmap));
		err = PTR_ERR(priv->edid_regmap);
		goto reset;
	}

	err = uh2c_priv_init(priv);
	if (err) {
		dev_err(&client->dev, "failed to init chip: %d\n", err);
		goto reset;
	}

	err = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, uh2c_irq_handler, IRQF_ONESHOT,
					dev_name(&client->dev), priv);
	if (err) {
		dev_err(&client->dev, "failed to request IRQ %d: %d\n", client->irq, err);
		goto reset;
	}

	v4l2_i2c_subdev_init(&priv->subdev, client, &uh2c_subdev_ops);

	err = v4l2_async_register_subdev(&priv->subdev);
	if (err) {
		dev_err(&client->dev, "Failed to register async subdev\n");
		v4l2_device_unregister_subdev(&priv->subdev);
		goto reset;
	}

	return 0;

reset:
	if (priv->reset_gpio)
		gpiod_set_value_cansleep(priv->reset_gpio, 1);
	if (priv->refclk)
		clk_disable_unprepare(priv->refclk);

regulators_disable:
	regulator_bulk_disable(ARRAY_SIZE(priv->regulators), priv->regulators);
	return err;
}

static int uh2c_remove(struct i2c_client *client)
{
	struct uh2c *priv = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_device_unregister_subdev(&priv->subdev);

	if (priv->reset_gpio)
		gpiod_set_value_cansleep(priv->reset_gpio, 1);
	if (priv->refclk)
		clk_disable_unprepare(priv->refclk);
	regulator_bulk_disable(ARRAY_SIZE(priv->regulators), priv->regulators);

	return 0;
}

static const struct i2c_device_id uh2c_id[] = {
	{ "uh2c", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, uh2c_id);

#ifdef CONFIG_OF
static const struct of_device_id uh2c_of_table[] = {
	{ .compatible = "toshiba,tc358840xbg" },
	{ }
};
MODULE_DEVICE_TABLE(of, uh2c_of_table);
#endif

static struct i2c_driver uh2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(uh2c_of_table),
		.name = "uh2c",
		.owner = THIS_MODULE,
	},
	.probe = uh2c_probe,
	.remove = uh2c_remove,
	.id_table = uh2c_id,
};
module_i2c_driver(uh2c_driver);

MODULE_DESCRIPTION("Driver for Toshiba TC358840 HDMI-CSI bridge");
MODULE_AUTHOR("Armin Weiss (weii@zhaw.ch)");
MODULE_LICENSE("GPL v2");
