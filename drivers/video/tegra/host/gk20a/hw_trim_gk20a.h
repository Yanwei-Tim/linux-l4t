/*
 * drivers/video/tegra/host/gk20a/hw_trim_gk20a.h
 *
 * Copyright (c) 2012, NVIDIA Corporation.
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
 *
 */

 /*
  * Function naming determines intended use:
  *
  *     <x>_r(void) : Returns the offset for register <x>.
  *
  *     <x>_w(void) : Returns the word offset for word (4 byte) element <x>.
  *
  *     <x>_<y>_s(void) : Returns size of field <y> of register <x> in bits.
  *
  *     <x>_<y>_f(u32 v) : Returns a value based on 'v' which has been shifted
  *         and masked to place it at field <y> of register <x>.  This value
  *         can be |'d with others to produce a full register value for
  *         register <x>.
  *
  *     <x>_<y>_m(void) : Returns a mask for field <y> of register <x>.  This
  *         value can be ~'d and then &'d to clear the value of field <y> for
  *         register <x>.
  *
  *     <x>_<y>_<z>_f(void) : Returns the constant value <z> after being shifted
  *         to place it at field <y> of register <x>.  This value can be |'d
  *         with others to produce a full register value for <x>.
  *
  *     <x>_<y>_v(u32 r) : Returns the value of field <y> from a full register
  *         <x> value 'r' after being shifted to place its LSB at bit 0.
  *         This value is suitable for direct comparison with other unshifted
  *         values appropriate for use in field <y> of register <x>.
  *
  *     <x>_<y>_<z>_v(void) : Returns the constant value for <z> defined for
  *         field <y> of register <x>.  This value is suitable for direct
  *         comparison with unshifted values appropriate for use in field <y>
  *         of register <x>.
  */

#ifndef __hw_trim_gk20a_h__
#define __hw_trim_gk20a_h__
/*This file is autogenerated.  Do not edit. */

static inline u32 trim_sys_gpcpll_cfg_r(void)
{
	return 0x00137000;
}
static inline u32 trim_sys_gpcpll_cfg_enable_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_enable_f(u32 v)
{
	return (v & 0x1) << 0;
}
static inline u32 trim_sys_gpcpll_cfg_enable_m(void)
{
	return 0x1 << 0;
}
static inline u32 trim_sys_gpcpll_cfg_enable_v(u32 r)
{
	return (r >> 0) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_enable_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_enable_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_enable_no_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_enable_no_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_enable_yes_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_enable_yes_f(void)
{
	return 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_f(u32 v)
{
	return (v & 0x1) << 1;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_m(void)
{
	return 0x1 << 1;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_v(u32 r)
{
	return (r >> 1) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_init_f(void)
{
	return 0x2;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_power_on_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_power_on_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_power_off_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_power_off_f(void)
{
	return 0x2;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_f(u32 v)
{
	return (v & 0x1) << 4;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_m(void)
{
	return 0x1 << 4;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_v(u32 r)
{
	return (r >> 4) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_power_on_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_power_on_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_power_off_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_power_off_f(void)
{
	return 0x10;
}
static inline u32 trim_sys_gpcpll_cfg_lock_override_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_lock_override_f(u32 v)
{
	return (v & 0x1) << 5;
}
static inline u32 trim_sys_gpcpll_cfg_lock_override_m(void)
{
	return 0x1 << 5;
}
static inline u32 trim_sys_gpcpll_cfg_lock_override_v(u32 r)
{
	return (r >> 5) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_lock_override_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_lock_override_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_lock_override_disable_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_lock_override_disable_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_lock_override_enable_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_lock_override_enable_f(void)
{
	return 0x20;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_f(u32 v)
{
	return (v & 0x1) << 17;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_m(void)
{
	return 0x1 << 17;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_v(u32 r)
{
	return (r >> 17) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_false_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_false_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_true_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_true_f(void)
{
	return 0x20000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssa_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssa_f(u32 v)
{
	return (v & 0x1) << 24;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssa_m(void)
{
	return 0x1 << 24;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssa_v(u32 r)
{
	return (r >> 24) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssa_false_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssa_false_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssa_true_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssa_true_f(void)
{
	return 0x1000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssd_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssd_f(u32 v)
{
	return (v & 0x1) << 25;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssd_m(void)
{
	return 0x1 << 25;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssd_v(u32 r)
{
	return (r >> 25) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssd_false_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssd_false_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssd_true_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_pll_ssd_true_f(void)
{
	return 0x2000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_cml_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_cml_f(u32 v)
{
	return (v & 0x1) << 26;
}
static inline u32 trim_sys_gpcpll_cfg_pll_cml_m(void)
{
	return 0x1 << 26;
}
static inline u32 trim_sys_gpcpll_cfg_pll_cml_v(u32 r)
{
	return (r >> 26) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_cml_false_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_cml_false_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_pll_cml_true_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_pll_cml_true_f(void)
{
	return 0x4000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_4phcml_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_4phcml_f(u32 v)
{
	return (v & 0x1) << 27;
}
static inline u32 trim_sys_gpcpll_cfg_pll_4phcml_m(void)
{
	return 0x1 << 27;
}
static inline u32 trim_sys_gpcpll_cfg_pll_4phcml_v(u32 r)
{
	return (r >> 27) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_4phcml_false_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_4phcml_false_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_pll_4phcml_true_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_pll_4phcml_true_f(void)
{
	return 0x8000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_dll_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_dll_f(u32 v)
{
	return (v & 0x1) << 28;
}
static inline u32 trim_sys_gpcpll_cfg_pll_dll_m(void)
{
	return 0x1 << 28;
}
static inline u32 trim_sys_gpcpll_cfg_pll_dll_v(u32 r)
{
	return (r >> 28) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_dll_false_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_dll_false_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_pll_dll_true_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_pll_dll_true_f(void)
{
	return 0x10000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_600_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_600_f(u32 v)
{
	return (v & 0x1) << 29;
}
static inline u32 trim_sys_gpcpll_cfg_pll_600_m(void)
{
	return 0x1 << 29;
}
static inline u32 trim_sys_gpcpll_cfg_pll_600_v(u32 r)
{
	return (r >> 29) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_600_false_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_pll_600_false_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_pll_600_true_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_pll_600_true_f(void)
{
	return 0x20000000;
}
static inline u32 trim_sys_gpcpll_cfg_en_fstlck_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg_en_fstlck_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 trim_sys_gpcpll_cfg_en_fstlck_m(void)
{
	return 0x1 << 31;
}
static inline u32 trim_sys_gpcpll_cfg_en_fstlck_v(u32 r)
{
	return (r >> 31) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_en_fstlck_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_en_fstlck_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_en_fstlck_disable_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_en_fstlck_disable_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_en_fstlck_enable_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg_en_fstlck_enable_f(void)
{
	return 0x80000000;
}
static inline u32 trim_sys_gpcpll_coeff_r(void)
{
	return 0x00137004;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_s(void)
{
	return 8;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_f(u32 v)
{
	return (v & 0xff) << 0;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_m(void)
{
	return 0xff << 0;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_v(u32 r)
{
	return (r >> 0) & 0xff;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_init_f(void)
{
	return 0x1;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_min_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_min_f(void)
{
	return 0x1;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_max_v(void)
{
	return 0x000000FF;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_max_f(void)
{
	return 0xff;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_s(void)
{
	return 8;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_f(u32 v)
{
	return (v & 0xff) << 8;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_m(void)
{
	return 0xff << 8;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_v(u32 r)
{
	return (r >> 8) & 0xff;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_init_v(void)
{
	return 0x00000037;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_init_f(void)
{
	return 0x3700;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_min_v(void)
{
	return 0x00000008;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_min_f(void)
{
	return 0x800;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_max_v(void)
{
	return 0x000000FF;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_max_f(void)
{
	return 0xff00;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_s(void)
{
	return 6;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_f(u32 v)
{
	return (v & 0x3f) << 16;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_m(void)
{
	return 0x3f << 16;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_v(u32 r)
{
	return (r >> 16) & 0x3f;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_init_f(void)
{
	return 0x10000;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_min_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_min_f(void)
{
	return 0x10000;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_max_v(void)
{
	return 0x0000003F;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_max_f(void)
{
	return 0x3f0000;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_f(u32 v)
{
	return (v & 0x1) << 30;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_m(void)
{
	return 0x1 << 30;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_v(u32 r)
{
	return (r >> 30) & 0x1;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_m(void)
{
	return 0x1 << 31;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_v(u32 r)
{
	return (r >> 31) & 0x1;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_sel_vco_r(void)
{
	return 0x00137100;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_s(void)
{
	return 1;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_f(u32 v)
{
	return (v & 0x1) << 0;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_m(void)
{
	return 0x1 << 0;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_v(u32 r)
{
	return (r >> 0) & 0x1;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_bypass_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_bypass_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_vco_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_vco_f(void)
{
	return 0x1;
}
static inline u32 trim_sys_gpc2clk_out_r(void)
{
	return 0x00137250;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_s(void)
{
	return 6;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_f(u32 v)
{
	return (v & 0x3f) << 0;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_m(void)
{
	return 0x3f << 0;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_v(u32 r)
{
	return (r >> 0) & 0x3f;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_init_v(void)
{
	return 0x0000003C;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_init_f(void)
{
	return 0x3c;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by1_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by1_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by1p5_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by1p5_f(void)
{
	return 0x1;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by2_v(void)
{
	return 0x00000002;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by2_f(void)
{
	return 0x2;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by2p5_v(void)
{
	return 0x00000003;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by2p5_f(void)
{
	return 0x3;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by3_v(void)
{
	return 0x00000004;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by3_f(void)
{
	return 0x4;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by3p5_v(void)
{
	return 0x00000005;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by3p5_f(void)
{
	return 0x5;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by4_v(void)
{
	return 0x00000006;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by4_f(void)
{
	return 0x6;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by4p5_v(void)
{
	return 0x00000007;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by4p5_f(void)
{
	return 0x7;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by5_v(void)
{
	return 0x00000008;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by5_f(void)
{
	return 0x8;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by5p5_v(void)
{
	return 0x00000009;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by5p5_f(void)
{
	return 0x9;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by6_v(void)
{
	return 0x0000000A;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by6_f(void)
{
	return 0xa;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by6p5_v(void)
{
	return 0x0000000B;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by6p5_f(void)
{
	return 0xb;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by7_v(void)
{
	return 0x0000000C;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by7_f(void)
{
	return 0xc;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by7p5_v(void)
{
	return 0x0000000D;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by7p5_f(void)
{
	return 0xd;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by8_v(void)
{
	return 0x0000000E;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by8_f(void)
{
	return 0xe;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by8p5_v(void)
{
	return 0x0000000F;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by8p5_f(void)
{
	return 0xf;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by9_v(void)
{
	return 0x00000010;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by9_f(void)
{
	return 0x10;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by9p5_v(void)
{
	return 0x00000011;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by9p5_f(void)
{
	return 0x11;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by10_v(void)
{
	return 0x00000012;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by10_f(void)
{
	return 0x12;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by10p5_v(void)
{
	return 0x00000013;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by10p5_f(void)
{
	return 0x13;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by11_v(void)
{
	return 0x00000014;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by11_f(void)
{
	return 0x14;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by11p5_v(void)
{
	return 0x00000015;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by11p5_f(void)
{
	return 0x15;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by12_v(void)
{
	return 0x00000016;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by12_f(void)
{
	return 0x16;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by12p5_v(void)
{
	return 0x00000017;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by12p5_f(void)
{
	return 0x17;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by13_v(void)
{
	return 0x00000018;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by13_f(void)
{
	return 0x18;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by13p5_v(void)
{
	return 0x00000019;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by13p5_f(void)
{
	return 0x19;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by14_v(void)
{
	return 0x0000001A;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by14_f(void)
{
	return 0x1a;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by14p5_v(void)
{
	return 0x0000001B;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by14p5_f(void)
{
	return 0x1b;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by15_v(void)
{
	return 0x0000001C;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by15_f(void)
{
	return 0x1c;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by15p5_v(void)
{
	return 0x0000001D;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by15p5_f(void)
{
	return 0x1d;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by16_v(void)
{
	return 0x0000001E;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by16_f(void)
{
	return 0x1e;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by16p5_v(void)
{
	return 0x0000001F;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by16p5_f(void)
{
	return 0x1f;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by17_v(void)
{
	return 0x00000020;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by17_f(void)
{
	return 0x20;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by18_v(void)
{
	return 0x00000022;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by18_f(void)
{
	return 0x22;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by19_v(void)
{
	return 0x00000024;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by19_f(void)
{
	return 0x24;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by20_v(void)
{
	return 0x00000026;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by20_f(void)
{
	return 0x26;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by21_v(void)
{
	return 0x00000028;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by21_f(void)
{
	return 0x28;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by22_v(void)
{
	return 0x0000002A;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by22_f(void)
{
	return 0x2a;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by23_v(void)
{
	return 0x0000002C;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by23_f(void)
{
	return 0x2c;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by24_v(void)
{
	return 0x0000002E;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by24_f(void)
{
	return 0x2e;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by25_v(void)
{
	return 0x00000030;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by25_f(void)
{
	return 0x30;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by26_v(void)
{
	return 0x00000032;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by26_f(void)
{
	return 0x32;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by27_v(void)
{
	return 0x00000034;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by27_f(void)
{
	return 0x34;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by28_v(void)
{
	return 0x00000036;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by28_f(void)
{
	return 0x36;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by29_v(void)
{
	return 0x00000038;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by29_f(void)
{
	return 0x38;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by30_v(void)
{
	return 0x0000003A;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by30_f(void)
{
	return 0x3a;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by31_v(void)
{
	return 0x0000003C;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by31_f(void)
{
	return 0x3c;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_s(void)
{
	return 6;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_f(u32 v)
{
	return (v & 0x3f) << 8;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_m(void)
{
	return 0x3f << 8;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_v(u32 r)
{
	return (r >> 8) & 0x3f;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by1_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by1_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by1p5_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by1p5_f(void)
{
	return 0x100;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by2_v(void)
{
	return 0x00000002;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by2_f(void)
{
	return 0x200;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by2p5_v(void)
{
	return 0x00000003;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by2p5_f(void)
{
	return 0x300;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by3_v(void)
{
	return 0x00000004;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by3_f(void)
{
	return 0x400;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by3p5_v(void)
{
	return 0x00000005;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by3p5_f(void)
{
	return 0x500;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by4_v(void)
{
	return 0x00000006;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by4_f(void)
{
	return 0x600;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by4p5_v(void)
{
	return 0x00000007;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by4p5_f(void)
{
	return 0x700;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by5_v(void)
{
	return 0x00000008;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by5_f(void)
{
	return 0x800;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by5p5_v(void)
{
	return 0x00000009;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by5p5_f(void)
{
	return 0x900;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by6_v(void)
{
	return 0x0000000A;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by6_f(void)
{
	return 0xa00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by6p5_v(void)
{
	return 0x0000000B;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by6p5_f(void)
{
	return 0xb00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by7_v(void)
{
	return 0x0000000C;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by7_f(void)
{
	return 0xc00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by7p5_v(void)
{
	return 0x0000000D;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by7p5_f(void)
{
	return 0xd00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by8_v(void)
{
	return 0x0000000E;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by8_f(void)
{
	return 0xe00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by8p5_v(void)
{
	return 0x0000000F;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by8p5_f(void)
{
	return 0xf00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by9_v(void)
{
	return 0x00000010;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by9_f(void)
{
	return 0x1000;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by9p5_v(void)
{
	return 0x00000011;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by9p5_f(void)
{
	return 0x1100;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by10_v(void)
{
	return 0x00000012;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by10_f(void)
{
	return 0x1200;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by10p5_v(void)
{
	return 0x00000013;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by10p5_f(void)
{
	return 0x1300;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by11_v(void)
{
	return 0x00000014;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by11_f(void)
{
	return 0x1400;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by11p5_v(void)
{
	return 0x00000015;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by11p5_f(void)
{
	return 0x1500;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by12_v(void)
{
	return 0x00000016;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by12_f(void)
{
	return 0x1600;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by12p5_v(void)
{
	return 0x00000017;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by12p5_f(void)
{
	return 0x1700;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by13_v(void)
{
	return 0x00000018;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by13_f(void)
{
	return 0x1800;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by13p5_v(void)
{
	return 0x00000019;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by13p5_f(void)
{
	return 0x1900;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by14_v(void)
{
	return 0x0000001A;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by14_f(void)
{
	return 0x1a00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by14p5_v(void)
{
	return 0x0000001B;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by14p5_f(void)
{
	return 0x1b00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by15_v(void)
{
	return 0x0000001C;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by15_f(void)
{
	return 0x1c00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by15p5_v(void)
{
	return 0x0000001D;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by15p5_f(void)
{
	return 0x1d00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by16_v(void)
{
	return 0x0000001E;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by16_f(void)
{
	return 0x1e00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by16p5_v(void)
{
	return 0x0000001F;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by16p5_f(void)
{
	return 0x1f00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by17_v(void)
{
	return 0x00000020;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by17_f(void)
{
	return 0x2000;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by18_v(void)
{
	return 0x00000022;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by18_f(void)
{
	return 0x2200;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by19_v(void)
{
	return 0x00000024;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by19_f(void)
{
	return 0x2400;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by20_v(void)
{
	return 0x00000026;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by20_f(void)
{
	return 0x2600;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by21_v(void)
{
	return 0x00000028;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by21_f(void)
{
	return 0x2800;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by22_v(void)
{
	return 0x0000002A;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by22_f(void)
{
	return 0x2a00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by23_v(void)
{
	return 0x0000002C;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by23_f(void)
{
	return 0x2c00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by24_v(void)
{
	return 0x0000002E;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by24_f(void)
{
	return 0x2e00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by25_v(void)
{
	return 0x00000030;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by25_f(void)
{
	return 0x3000;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by26_v(void)
{
	return 0x00000032;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by26_f(void)
{
	return 0x3200;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by27_v(void)
{
	return 0x00000034;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by27_f(void)
{
	return 0x3400;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by28_v(void)
{
	return 0x00000036;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by28_f(void)
{
	return 0x3600;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by29_v(void)
{
	return 0x00000038;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by29_f(void)
{
	return 0x3800;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by30_v(void)
{
	return 0x0000003A;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by30_f(void)
{
	return 0x3a00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by31_v(void)
{
	return 0x0000003C;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by31_f(void)
{
	return 0x3c00;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_s(void)
{
	return 2;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_f(u32 v)
{
	return (v & 0x3) << 20;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_m(void)
{
	return 0x3 << 20;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_v(u32 r)
{
	return (r >> 20) & 0x3;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_init_f(void)
{
	return 0x100000;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_f(u32 v)
{
	return (v & 0x1) << 24;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_m(void)
{
	return 0x1 << 24;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_v(u32 r)
{
	return (r >> 24) & 0x1;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_init_f(void)
{
	return 0x1000000;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_f(u32 v)
{
	return (v & 0x1) << 25;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_m(void)
{
	return 0x1 << 25;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_v(u32 r)
{
	return (r >> 25) & 0x1;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_no_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_no_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_yes_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_yes_f(void)
{
	return 0x2000000;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating__prod_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating__prod_f(void)
{
	return 0x2000000;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_f(u32 v)
{
	return (v & 0x1) << 26;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_m(void)
{
	return 0x1 << 26;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_v(u32 r)
{
	return (r >> 26) & 0x1;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_no_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_no_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_yes_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_yes_f(void)
{
	return 0x4000000;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating__prod_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating__prod_f(void)
{
	return 0x4000000;
}
static inline u32 trim_sys_gpc2clk_out_gclks_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpc2clk_out_gclks_f(u32 v)
{
	return (v & 0x1) << 27;
}
static inline u32 trim_sys_gpc2clk_out_gclks_m(void)
{
	return 0x1 << 27;
}
static inline u32 trim_sys_gpc2clk_out_gclks_v(u32 r)
{
	return (r >> 27) & 0x1;
}
static inline u32 trim_sys_gpc2clk_out_gclks_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_gclks_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_gclks_no_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_gclks_no_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_gclks_yes_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_gclks_yes_f(void)
{
	return 0x8000000;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_m(void)
{
	return 0x1 << 31;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_v(u32 r)
{
	return (r >> 31) & 0x1;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_init_f(void)
{
	return 0x80000000;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_indiv1_mode_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_indiv1_mode_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_indiv4_mode_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_indiv4_mode_f(void)
{
	return 0x80000000;
}

#endif /* __hw_trim_gk20a_h__ */