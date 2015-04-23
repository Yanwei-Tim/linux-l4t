/*
 * uh2c-regmap.h - Toshiba UH2C/D HDMI-CSI bridge register map
 *
 * Copyright (c) 2014, Julian Scheel <julian@jusst.de>
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

#define SYSCTL 0x0002
#define CONFCTL 0x0004

#define CSITX0_CLKEN 0x0108
#define CSITX0_LANE_ENABLE 0x0118
#define CSITX0_START 0x011c
