// SPDX-License-Identifier: GPL-2.0-only
/*
 * Board-config ("INI") parsing and download for the external Marlin3-Lite
 * (SC2355) WCN combo.
 *
 * Ported from the Unisoc 5.4 vendor sc2355 driver (hw_param.c, cmdevt.c). The
 * config is a text INI (key = comma-separated values) that is marshalled by a
 * name table into the wifi_conf_t binary struct and pushed to the chip in four
 * fixed sections via CMD_DOWNLOAD_INI (each with a trailing CRC-16, handled by
 * sc23xx_download_ini_section()).
 *
 * Copyright 2021-2022 Unisoc (Shanghai) Technologies Co., Ltd
 */

#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

#include "sc23xx.h"
#include "cmd.h"
#include "hw_param.h"

/* Section numbers in the CMD_DOWNLOAD_INI frame. */
#define SEC1	1
#define SEC2	2
#define SEC3	3
#define SEC4	4

#define CF_TAB(NAME, MEM_OFFSET, TYPE) \
	{ NAME, (size_t)(&(((struct wifi_conf_t *)(0))->MEM_OFFSET)), TYPE }

/*
 * name -> (offset in wifi_conf_t, element type). type: 1=u8, 2=u16, 4=u32.
 * Kept verbatim from the vendor driver, including the Chain1_153/157 entries
 * that (in the vendor source) write into chain0 -- preserved so the emitted
 * bytes match the vendor exactly.
 */
static const struct nvm_name_table sc2355_nvm_table[] = {
	/* [Section 1: Version] */
	CF_TAB("Major", version.major, 2),
	CF_TAB("Minor", version.minor, 2),

	/* [Section 2] Board Config */
	CF_TAB("Calib_Bypass", board_config.calib_bypass, 2),
	CF_TAB("TxChain_Mask", board_config.txchain_mask, 1),
	CF_TAB("RxChain_Mask", board_config.rxchain_mask, 1),

	/* [Section 3] Board Config TPC */
	CF_TAB("DPD_LUT_idx", board_config_tpc.dpd_lut_idx[0], 1),
	CF_TAB("TPC_Goal_Chain0", board_config_tpc.tpc_goal_chain0[0], 2),
	CF_TAB("TPC_Goal_Chain1", board_config_tpc.tpc_goal_chain1[0], 2),

	/* [Section 4] TPC-LUT */
	CF_TAB("Chain0_LUT_0", tpc_lut.chain0_lut[0], 1),
	CF_TAB("Chain0_LUT_1", tpc_lut.chain0_lut[1], 1),
	CF_TAB("Chain0_LUT_2", tpc_lut.chain0_lut[2], 1),
	CF_TAB("Chain0_LUT_3", tpc_lut.chain0_lut[3], 1),
	CF_TAB("Chain0_LUT_4", tpc_lut.chain0_lut[4], 1),
	CF_TAB("Chain0_LUT_5", tpc_lut.chain0_lut[5], 1),
	CF_TAB("Chain0_LUT_6", tpc_lut.chain0_lut[6], 1),
	CF_TAB("Chain0_LUT_7", tpc_lut.chain0_lut[7], 1),
	CF_TAB("Chain1_LUT_0", tpc_lut.chain1_lut[0], 1),
	CF_TAB("Chain1_LUT_1", tpc_lut.chain1_lut[1], 1),
	CF_TAB("Chain1_LUT_2", tpc_lut.chain1_lut[2], 1),
	CF_TAB("Chain1_LUT_3", tpc_lut.chain1_lut[3], 1),
	CF_TAB("Chain1_LUT_4", tpc_lut.chain1_lut[4], 1),
	CF_TAB("Chain1_LUT_5", tpc_lut.chain1_lut[5], 1),
	CF_TAB("Chain1_LUT_6", tpc_lut.chain1_lut[6], 1),
	CF_TAB("Chain1_LUT_7", tpc_lut.chain1_lut[7], 1),

	/* [Section 5] Board Config Frequency Compensation */
	CF_TAB("2G_Channel_Chain0", board_conf_freq_comp.channel_2g_chain0[0], 1),
	CF_TAB("2G_Channel_Chain1", board_conf_freq_comp.channel_2g_chain1[0], 1),
	CF_TAB("5G_Channel_Chain0", board_conf_freq_comp.channel_5g_chain0[0], 1),
	CF_TAB("5G_Channel_Chain1", board_conf_freq_comp.channel_5g_chain1[0], 1),

	/* [Section 6] Rate To Power with BW 20M */
	CF_TAB("11b_Power", power_20m.power_11b[0], 1),
	CF_TAB("11ag_Power", power_20m.power_11ag[0], 1),
	CF_TAB("11n_Power", power_20m.power_11n[0], 1),
	CF_TAB("11ac_Power", power_20m.power_11ac[0], 1),

	/* [Section 7] Power Backoff */
	CF_TAB("Green_WIFI_offset", power_backoff.green_wifi_offset, 1),
	CF_TAB("HT40_Power_offset", power_backoff.ht40_power_offset, 1),
	CF_TAB("VHT40_Power_offset", power_backoff.vht40_power_offset, 1),
	CF_TAB("VHT80_Power_offset", power_backoff.vht80_power_offset, 1),
	CF_TAB("SAR_Power_offset", power_backoff.sar_power_offset, 1),
	CF_TAB("Mean_Power_offset", power_backoff.mean_power_offset, 1),

	/* [Section 8] Reg Domain */
	CF_TAB("reg_domain1", reg_domain.reg_domain1, 4),
	CF_TAB("reg_domain2", reg_domain.reg_domain2, 4),

	/* [Section 9] Band Edge Power offset (MKK, FCC, ETSI) */
	CF_TAB("BW20M", band_edge_power_offset.bw20m[0], 1),
	CF_TAB("BW40M", band_edge_power_offset.bw40m[0], 1),
	CF_TAB("BW80M", band_edge_power_offset.bw80m[0], 1),

	/* [Section 10] TX Scale */
	CF_TAB("Chain0_1", tx_scale.chain0[0][0], 1),
	CF_TAB("Chain1_1", tx_scale.chain1[0][0], 1),
	CF_TAB("Chain0_2", tx_scale.chain0[1][0], 1),
	CF_TAB("Chain1_2", tx_scale.chain1[1][0], 1),
	CF_TAB("Chain0_3", tx_scale.chain0[2][0], 1),
	CF_TAB("Chain1_3", tx_scale.chain1[2][0], 1),
	CF_TAB("Chain0_4", tx_scale.chain0[3][0], 1),
	CF_TAB("Chain1_4", tx_scale.chain1[3][0], 1),
	CF_TAB("Chain0_5", tx_scale.chain0[4][0], 1),
	CF_TAB("Chain1_5", tx_scale.chain1[4][0], 1),
	CF_TAB("Chain0_6", tx_scale.chain0[5][0], 1),
	CF_TAB("Chain1_6", tx_scale.chain1[5][0], 1),
	CF_TAB("Chain0_7", tx_scale.chain0[6][0], 1),
	CF_TAB("Chain1_7", tx_scale.chain1[6][0], 1),
	CF_TAB("Chain0_8", tx_scale.chain0[7][0], 1),
	CF_TAB("Chain1_8", tx_scale.chain1[7][0], 1),
	CF_TAB("Chain0_9", tx_scale.chain0[8][0], 1),
	CF_TAB("Chain1_9", tx_scale.chain1[8][0], 1),
	CF_TAB("Chain0_10", tx_scale.chain0[9][0], 1),
	CF_TAB("Chain1_10", tx_scale.chain1[9][0], 1),
	CF_TAB("Chain0_11", tx_scale.chain0[10][0], 1),
	CF_TAB("Chain1_11", tx_scale.chain1[10][0], 1),
	CF_TAB("Chain0_12", tx_scale.chain0[11][0], 1),
	CF_TAB("Chain1_12", tx_scale.chain1[11][0], 1),
	CF_TAB("Chain0_13", tx_scale.chain0[12][0], 1),
	CF_TAB("Chain1_13", tx_scale.chain1[12][0], 1),
	CF_TAB("Chain0_14", tx_scale.chain0[13][0], 1),
	CF_TAB("Chain1_14", tx_scale.chain1[13][0], 1),
	CF_TAB("Chain0_36", tx_scale.chain0[14][0], 1),
	CF_TAB("Chain1_36", tx_scale.chain1[14][0], 1),
	CF_TAB("Chain0_40", tx_scale.chain0[15][0], 1),
	CF_TAB("Chain1_40", tx_scale.chain1[15][0], 1),
	CF_TAB("Chain0_44", tx_scale.chain0[16][0], 1),
	CF_TAB("Chain1_44", tx_scale.chain1[16][0], 1),
	CF_TAB("Chain0_48", tx_scale.chain0[17][0], 1),
	CF_TAB("Chain1_48", tx_scale.chain1[17][0], 1),
	CF_TAB("Chain0_52", tx_scale.chain0[18][0], 1),
	CF_TAB("Chain1_52", tx_scale.chain1[18][0], 1),
	CF_TAB("Chain0_56", tx_scale.chain0[19][0], 1),
	CF_TAB("Chain1_56", tx_scale.chain1[19][0], 1),
	CF_TAB("Chain0_60", tx_scale.chain0[20][0], 1),
	CF_TAB("Chain1_60", tx_scale.chain1[20][0], 1),
	CF_TAB("Chain0_64", tx_scale.chain0[21][0], 1),
	CF_TAB("Chain1_64", tx_scale.chain1[21][0], 1),
	CF_TAB("Chain0_100", tx_scale.chain0[22][0], 1),
	CF_TAB("Chain1_100", tx_scale.chain1[22][0], 1),
	CF_TAB("Chain0_104", tx_scale.chain0[23][0], 1),
	CF_TAB("Chain1_104", tx_scale.chain1[23][0], 1),
	CF_TAB("Chain0_108", tx_scale.chain0[24][0], 1),
	CF_TAB("Chain1_108", tx_scale.chain1[24][0], 1),
	CF_TAB("Chain0_112", tx_scale.chain0[25][0], 1),
	CF_TAB("Chain1_112", tx_scale.chain1[25][0], 1),
	CF_TAB("Chain0_116", tx_scale.chain0[26][0], 1),
	CF_TAB("Chain1_116", tx_scale.chain1[26][0], 1),
	CF_TAB("Chain0_120", tx_scale.chain0[27][0], 1),
	CF_TAB("Chain1_120", tx_scale.chain1[27][0], 1),
	CF_TAB("Chain0_124", tx_scale.chain0[28][0], 1),
	CF_TAB("Chain1_124", tx_scale.chain1[28][0], 1),
	CF_TAB("Chain0_128", tx_scale.chain0[29][0], 1),
	CF_TAB("Chain1_128", tx_scale.chain1[29][0], 1),
	CF_TAB("Chain0_132", tx_scale.chain0[30][0], 1),
	CF_TAB("Chain1_132", tx_scale.chain1[30][0], 1),
	CF_TAB("Chain0_136", tx_scale.chain0[31][0], 1),
	CF_TAB("Chain1_136", tx_scale.chain1[31][0], 1),
	CF_TAB("Chain0_140", tx_scale.chain0[32][0], 1),
	CF_TAB("Chain1_140", tx_scale.chain1[32][0], 1),
	CF_TAB("Chain0_144", tx_scale.chain0[33][0], 1),
	CF_TAB("Chain1_144", tx_scale.chain1[33][0], 1),
	CF_TAB("Chain0_149", tx_scale.chain0[34][0], 1),
	CF_TAB("Chain1_149", tx_scale.chain1[34][0], 1),
	CF_TAB("Chain0_153", tx_scale.chain0[35][0], 1),
	CF_TAB("Chain1_153", tx_scale.chain0[35][0], 1),
	CF_TAB("Chain0_157", tx_scale.chain0[36][0], 1),
	CF_TAB("Chain1_157", tx_scale.chain0[36][0], 1),
	CF_TAB("Chain0_161", tx_scale.chain0[37][0], 1),
	CF_TAB("Chain1_161", tx_scale.chain1[37][0], 1),
	CF_TAB("Chain0_165", tx_scale.chain0[38][0], 1),
	CF_TAB("Chain1_165", tx_scale.chain1[38][0], 1),

	/* [Section 11] misc */
	CF_TAB("DFS_switch", misc.dfs_switch, 1),
	CF_TAB("power_save_switch", misc.power_save_switch, 1),
	CF_TAB("ex-Fem_and_ex-LNA_param_setup", misc.fem_lan_param_setup, 1),
	CF_TAB("rssi_report_diff", misc.rssi_report_diff, 1),

	/* [Section 12] debug reg */
	CF_TAB("address", debug_reg.address[0], 4),
	CF_TAB("value", debug_reg.value[0], 4),

	/* [Section 13] coex_config */
	CF_TAB("bt_performance_cfg0", coex_config.bt_performance_cfg0, 4),
	CF_TAB("bt_performance_cfg1", coex_config.bt_performance_cfg1, 4),
	CF_TAB("wifi_performance_cfg0", coex_config.wifi_performance_cfg0, 4),
	CF_TAB("wifi_performance_cfg2", coex_config.wifi_performance_cfg2, 4),
	CF_TAB("strategy_cfg0", coex_config.strategy_cfg0, 4),
	CF_TAB("strategy_cfg1", coex_config.strategy_cfg1, 4),
	CF_TAB("strategy_cfg2", coex_config.strategy_cfg2, 4),
	CF_TAB("compatibility_cfg0", coex_config.compatibility_cfg0, 4),
	CF_TAB("compatibility_cfg1", coex_config.compatibility_cfg1, 4),
	CF_TAB("ant_cfg0", coex_config.ant_cfg0, 4),
	CF_TAB("ant_cfg1", coex_config.ant_cfg1, 4),
	CF_TAB("isolation_cfg0", coex_config.isolation_cfg0, 4),
	CF_TAB("isolation_cfg1", coex_config.isolation_cfg1, 4),
	CF_TAB("reserved_cfg0", coex_config.reserved_cfg0, 4),
	CF_TAB("reserved_cfg1", coex_config.reserved_cfg1, 4),
	CF_TAB("reserved_cfg2", coex_config.reserved_cfg2, 4),
	CF_TAB("reserved_cfg3", coex_config.reserved_cfg3, 4),
	CF_TAB("reserved_cfg4", coex_config.reserved_cfg4, 4),
	CF_TAB("reserved_cfg5", coex_config.reserved_cfg5, 4),
	CF_TAB("reserved_cfg6", coex_config.reserved_cfg6, 4),
	CF_TAB("reserved_cfg7", coex_config.reserved_cfg7, 4),

	/* [Section 14] rf_config */
	CF_TAB("rf_config", rf_config.rf_data, 1),

	/* [Section 15] wifi_param */
	CF_TAB("roaming_trigger", wifi_param.roaming_param.trigger, 1),
	CF_TAB("roaming_delta", wifi_param.roaming_param.delta, 1),
	CF_TAB("roaming_5g_prefer", wifi_param.roaming_param.band_5g_prefer, 1),
};

/*
 * Classify a character while tokenising an INI line:
 *   1 = identifier char, 2 = decimal digit / '-', 3 = hex marker ('x'/'X'/'.'),
 *   4 = line/comment terminator, 0 = separator.
 */
static int hw_param_nvm_find_type(char key)
{
	if ((key >= 'a' && key <= 'w') ||
	    (key >= 'y' && key <= 'z') ||
	    (key >= 'A' && key <= 'W') ||
	    (key >= 'Y' && key <= 'Z') ||
	    key == '_')
		return 1;
	if ((key >= '0' && key <= '9') || key == '-')
		return 2;
	if (key == 'x' || key == 'X' || key == '.')
		return 3;
	if (key == '\0' || key == '\r' || key == '\n' || key == '#')
		return 4;
	return 0;
}

/* Store the parsed values for one key into the wifi_conf_t at the table offset. */
static int hw_param_nvm_set_cmd(const struct nvm_name_table *ptable,
				struct nvm_cali_cmd *cmd, void *p_data)
{
	int i;
	unsigned char *p;

	if (ptable->type != 1 && ptable->type != 2 && ptable->type != 4)
		return -1;

	p = (unsigned char *)(p_data) + ptable->mem_offset;

	for (i = 0; i < cmd->num; i++) {
		if (ptable->type == 1)
			*((unsigned char *)p + i) = (unsigned char)(cmd->par[i]);
		else if (ptable->type == 2)
			*((unsigned short *)p + i) = (unsigned short)(cmd->par[i]);
		else if (ptable->type == 4)
			*((unsigned int *)p + i) = (unsigned int)(cmd->par[i]);
	}
	return 0;
}

/* Split one INI line into a key (cmd->itm) and its integer values (cmd->par). */
static void hw_param_nvm_get_cmd_par(char *str, struct nvm_cali_cmd *cmd)
{
	int i, j, buftype, ctype, flag;
	unsigned int m_cmd_num = ARRAY_SIZE(cmd->par);
	char tmp[64];
	char c;
	long val;

	buftype = -1;
	ctype = 0;
	flag = 0;
	memset(cmd, 0, sizeof(struct nvm_cali_cmd));

	for (i = 0, j = 0; j < sizeof(tmp); i++) {
		c = str[i];
		ctype = hw_param_nvm_find_type(c);
		if (ctype == 1 || ctype == 2 || ctype == 3) {
			tmp[j] = c;
			j++;
			if (buftype == -1) {
				if (ctype == 2)
					buftype = 2;
				else
					buftype = 1;
			} else if (buftype == 2) {
				if (ctype == 1)
					buftype = 1;
			}
			continue;
		}
		if (buftype != -1) {
			tmp[j] = '\0';

			if (buftype == 1 && !flag) {
				strcpy(cmd->itm, tmp);
				flag = 1;
			} else {
				if (kstrtol(tmp, 0, &val))
					val = 0;
				if (cmd->num >= m_cmd_num)
					return;
				cmd->par[cmd->num] = val & 0xFFFFFFFF;
				cmd->num++;
			}
			buftype = -1;
			j = 0;
		}
		if (!ctype)
			continue;
		if (ctype == 4)
			return;
	}
}

static const struct nvm_name_table *
hw_param_nvm_cf_table_match(struct nvm_cali_cmd *cmd)
{
	int i;
	int len = ARRAY_SIZE(sc2355_nvm_table);

	if (!cmd)
		return NULL;
	for (i = 0; i < len; i++) {
		if (!sc2355_nvm_table[i].itm)
			continue;
		if (strcmp(sc2355_nvm_table[i].itm, cmd->itm))
			continue;
		return &sc2355_nvm_table[i];
	}
	return NULL;
}

/* Walk the INI buffer line by line, marshalling each recognised key. */
static int hw_param_nvm_buf_operate(char *pbuf, int file_len, void *p_data)
{
	int i, p;
	struct nvm_cali_cmd *cmd;
	struct wifi_conf_t *conf;
	const struct nvm_name_table *ptable;

	if (!pbuf || !file_len)
		return -1;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	for (i = 0, p = 0; i < file_len; i++) {
		if (pbuf[i] != '\n' && pbuf[i] != '\r' && pbuf[i] != '\0')
			continue;

		if ((i - p) >= 5) {
			hw_param_nvm_get_cmd_par(pbuf + p, cmd);
			ptable = hw_param_nvm_cf_table_match(cmd);
			if (ptable) {
				hw_param_nvm_set_cmd(ptable, cmd, p_data);
				if (strcmp(ptable->itm, "rf_config") == 0) {
					conf = (struct wifi_conf_t *)p_data;
					conf->rf_config.rf_data_len = cmd->num;
				}
			}
		}
		p = i + 1;
	}

	kfree(cmd);
	return 0;
}

static int sc23xx_parse_hw_param(struct sc23xx_dev *sdev, const char *fw_name,
				 struct wifi_conf_t *conf)
{
	const struct firmware *fw;
	char *buffer;
	int ret;

	ret = request_firmware(&fw, fw_name, wiphy_dev(sdev->wiphy));
	if (ret) {
		wiphy_err(sdev->wiphy, "failed to load board config %s (%d)\n",
			  fw_name, ret);
		return ret;
	}

	if (!fw->data || !fw->size) {
		release_firmware(fw);
		return -EINVAL;
	}

	/* Parser walks a NUL/newline-delimited copy; keep the fw buffer const. */
	buffer = vmalloc(fw->size);
	if (!buffer) {
		release_firmware(fw);
		return -ENOMEM;
	}
	memcpy(buffer, fw->data, fw->size);
	ret = hw_param_nvm_buf_operate(buffer, fw->size, conf);
	vfree(buffer);
	release_firmware(fw);

	return ret;
}

int sc23xx_load_hw_param(struct sc23xx_dev *sdev, const char *fw_name)
{
	struct wifi_conf_t *conf;
	struct wifi_conf_sec1_t *sec1;
	struct wifi_conf_sec2_t *sec2;
	struct wifi_config_param_t *wifi_param;
	int ret;

	conf = kzalloc(sizeof(*conf), GFP_KERNEL);
	if (!conf)
		return -ENOMEM;

	ret = sc23xx_parse_hw_param(sdev, fw_name, conf);
	if (ret)
		goto out;

	/* sec1/sec2 alias the head of wifi_conf_t; rf/param are separate. */
	sec1 = (struct wifi_conf_sec1_t *)conf;
	sec2 = (struct wifi_conf_sec2_t *)((char *)conf +
					   sizeof(struct wifi_conf_sec1_t));
	wifi_param = &conf->wifi_param;

	ret = sc23xx_download_ini_section(sdev, SEC1, sec1, sizeof(*sec1));
	if (ret)
		goto out;

	ret = sc23xx_download_ini_section(sdev, SEC2, sec2, sizeof(*sec2));
	if (ret)
		goto out;

	if (conf->rf_config.rf_data_len) {
		ret = sc23xx_download_ini_section(sdev, SEC3,
						  conf->rf_config.rf_data,
						  conf->rf_config.rf_data_len);
		if (ret)
			goto out;
	}

	ret = sc23xx_download_ini_section(sdev, SEC4, wifi_param,
					  sizeof(*wifi_param));

out:
	kfree(conf);
	return ret;
}
EXPORT_SYMBOL_GPL(sc23xx_load_hw_param);
