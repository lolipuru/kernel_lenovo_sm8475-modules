/*
 * linux/sound/cs35l45.h -- Platform data for CS35L45
 *
 * Copyright (c) 2022 Cirrus Logic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CS35L45_H
#define __CS35L45_H

#define CS35L45_NUM_SUPPLIES 2

struct of_entry {
	const char *name;
	unsigned int reg;
	unsigned int mask;
	unsigned int shift;
};

enum cs35l45_level {
	L0 = 0,
	L1,
	L2,
	L3,
	L4,
};

#define BPE_INST_LEVELS 4

enum bpe_inst_params {
	BPE_INST_THLD = 0,
	BPE_INST_ATTN,
	BPE_INST_ATK_RATE,
	BPE_INST_HOLD_TIME,
	BPE_INST_RLS_RATE,
	BPE_INST_PARAMS,
};

enum bpe_misc_params {
	BPE_INST_BPE_BYP = 0,
	BPE_INST_INF_HOLD_RLS,
	BPE_INST_L3_BYP,
	BPE_INST_L2_BYP,
	BPE_INST_L1_BYP,
	BPE_MODE_SEL,
	BPE_FILT_SEL,
	BPE_MISC_PARAMS,
};

#define BST_BPE_INST_LEVELS 5

enum bst_bpe_inst_params {
	BST_BPE_INST_THLD = 0,
	BST_BPE_INST_ILIM,
	BST_BPE_INST_SS_ILIM,
	BST_BPE_INST_ATK_RATE,
	BST_BPE_INST_HOLD_TIME,
	BST_BPE_INST_RLS_RATE,
	BST_BPE_INST_PARAMS,
};

enum bst_bpe_misc_params {
	BST_BPE_INST_INF_HOLD_RLS = 0,
	BST_BPE_IL_LIM_MODE,
	BST_BPE_OUT_OPMODE_SEL,
	BST_BPE_INST_L3_BYP,
	BST_BPE_INST_L2_BYP,
	BST_BPE_INST_L1_BYP,
	BST_BPE_FILT_SEL,
	BST_BPE_MISC_PARAMS,
};

enum bst_bpe_il_lim_params {
	BST_BPE_IL_LIM_THLD_DEL1 = 0,
	BST_BPE_IL_LIM_THLD_DEL2,
	BST_BPE_IL_LIM1_THLD,
	BST_BPE_IL_LIM1_DLY,
	BST_BPE_IL_LIM2_DLY,
	BST_BPE_IL_LIM_DLY_HYST,
	BST_BPE_IL_LIM_THLD_HYST,
	BST_BPE_IL_LIM1_ATK_RATE,
	BST_BPE_IL_LIM2_ATK_RATE,
	BST_BPE_IL_LIM1_RLS_RATE,
	BST_BPE_IL_LIM2_RLS_RATE,
	BST_BPE_IL_LIM_PARAMS,
};

enum ldpm_params {
	LDPM_GP1_BOOST_SEL = 0,
	LDPM_GP1_AMP_SEL,
	LDPM_GP1_DELAY,
	LDPM_GP1_PCM_THLD,
	LDPM_GP2_IMON_SEL,
	LDPM_GP2_VMON_SEL,
	LDPM_GP2_DELAY,
	LDPM_GP2_PCM_THLD,
	LDPM_PARAMS,
};

enum classh_params {
	CH_HDRM = 0,
	CH_RATIO,
	CH_REL_RATE,
	CH_OVB_THLD1,
	CH_OVB_THLDDELTA,
	CH_VDD_BST_MAX,
	CH_OVB_RATIO,
	CH_THLD1_OFFSET,
	AUD_MEM_DEPTH,
	CLASSH_PARAMS,
};

struct gpio_ctrl {
	bool is_present;
	u32 dir;
	u32 lvl;
	u32 op_cfg;
	u32 pol;
	u32 ctrl;
	u32 invert;
};

struct cs35l45_bpe_inst_cfg {
	bool is_present;
	u32 params[BPE_INST_PARAMS][BPE_INST_LEVELS];
};

struct cs35l45_bpe_misc_cfg {
	bool is_present;
	u32 params[BPE_MISC_PARAMS];
};

struct cs35l45_bst_bpe_inst_cfg {
	bool is_present;
	u32 params[BST_BPE_INST_PARAMS][BST_BPE_INST_LEVELS];
};

struct cs35l45_bst_bpe_misc_cfg {
	bool is_present;
	u32 params[BST_BPE_MISC_PARAMS];
};

struct cs35l45_bst_bpe_il_lim_cfg {
	bool is_present;
	u32 params[BST_BPE_IL_LIM_PARAMS];
};

struct cs35l45_hvlv_cfg {
	bool is_present;
	u32 hvlv_thld_hys;
	u32 hvlv_thld;
	u32 hvlv_dly;
};

struct cs35l45_ldpm_cfg {
	bool is_present;
	u32 params[LDPM_PARAMS];
};

struct cs35l45_classh_cfg {
	bool is_present;
	u32 params[CLASSH_PARAMS];
};

struct cs35l45_platform_data {
	u32 asp_sdout_hiz_ctrl;
	bool use_tdm_slots;
	const char *dsp_part_name;

	u32 ngate_ch1_hold;
	u32 ngate_ch1_thr;
	u32 ngate_ch2_hold;
	u32 ngate_ch2_thr;

	bool allow_hibernate;
	u32 global_en_gpio;

	struct cs35l45_bpe_inst_cfg bpe_inst_cfg;
	struct cs35l45_bpe_misc_cfg bpe_misc_cfg;
	struct cs35l45_bst_bpe_inst_cfg bst_bpe_inst_cfg;
	struct cs35l45_bst_bpe_misc_cfg bst_bpe_misc_cfg;
	struct cs35l45_bst_bpe_il_lim_cfg bst_bpe_il_lim_cfg;
	struct cs35l45_hvlv_cfg hvlv_cfg;
	struct cs35l45_ldpm_cfg ldpm_cfg;
	struct cs35l45_classh_cfg classh_cfg;

	struct gpio_ctrl gpio_ctrl1;
	struct gpio_ctrl gpio_ctrl2;
	struct gpio_ctrl gpio_ctrl3;
};

struct cs35l45_compr {
    struct wm_adsp *dsp;
    struct snd_compr_stream *stream;

    u32 *raw_buf;
    u32 buffer_size;
    u32 buffer_count;

    struct snd_compressed_buffer size; 

    u32 read_index;
    u32 last_read_index;
    int avail;
    u32 sample_rate;
    u64 copied_total;

    struct work_struct start_work;
    struct work_struct stop_work;
};

struct cs35l45_private {
	struct wm_adsp dsp;
	struct device *dev;
	struct regmap *regmap;
	struct regmap_irq_chip_data *irq_data;

	int irq;
	int i2c_addr;
	int max_quirks_read_nwords;

	enum control_bus_type bus_type;

	struct regulator_bulk_data supplies[CS35L45_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct pinctrl *pinctrl;
	struct pinctrl_state *reset_pinctrl_active;
	struct pinctrl_state *reset_pinctrl_sleep;

	struct snd_soc_component *component;
	struct snd_kcontrol_new fast_ctl;
	struct soc_enum fast_switch_enum;
	const char **fast_switch_names;
	bool fast_switch_en;
	unsigned int fast_switch_file_idx;

	unsigned int slot_width;
	int amplifier_mode;
	int speaker_status;
	bool initialized;

	struct cs35l45_compr *compr;

	struct work_struct dsp_pmu_work;
	struct work_struct dsp_pmd_work;
	struct delayed_work global_err_rls_work;
	struct mutex dsp_power_lock;
	struct completion virt2_mbox_comp;
	struct workqueue_struct *wq;

	struct cs35l45_platform_data pdata;
};

extern const struct of_entry bpe_inst_thld_map[BPE_INST_LEVELS];
extern const struct of_entry bpe_inst_attn_map[BPE_INST_LEVELS];
extern const struct of_entry bpe_inst_atk_rate_map[BPE_INST_LEVELS];
extern const struct of_entry bpe_inst_hold_time_map[BPE_INST_LEVELS];
extern const struct of_entry bpe_inst_rls_rate_map[BPE_INST_LEVELS];
extern const struct of_entry bpe_misc_map[BPE_MISC_PARAMS];

extern const struct of_entry bst_bpe_inst_thld_map[BST_BPE_INST_LEVELS];
extern const struct of_entry bst_bpe_inst_ilim_map[BST_BPE_INST_LEVELS];
extern const struct of_entry bst_bpe_inst_ss_ilim_map[BST_BPE_INST_LEVELS];
extern const struct of_entry bst_bpe_inst_atk_rate_map[BST_BPE_INST_LEVELS];
extern const struct of_entry bst_bpe_inst_hold_time_map[BST_BPE_INST_LEVELS];
extern const struct of_entry bst_bpe_inst_rls_rate_map[BST_BPE_INST_LEVELS];
extern const struct of_entry bst_bpe_misc_map[BST_BPE_MISC_PARAMS];
extern const struct of_entry bst_bpe_il_lim_map[BST_BPE_IL_LIM_PARAMS];
extern const struct of_entry ldpm_map[LDPM_PARAMS];
extern const struct of_entry classh_map[CLASSH_PARAMS];

extern const struct dev_pm_ops cs35l45_pm_ops;

int cs35l45_probe(struct cs35l45_private *cs35l45);
int cs35l45_remove(struct cs35l45_private *cs35l45);
int cs35l45_initialize(struct cs35l45_private *cs35l45);

int cs35l45_suspend_runtime(struct device *dev);
int cs35l45_resume_runtime(struct device *dev);

static inline const struct of_entry *
cs35l45_get_bpe_inst_entry(int level, int param)
{
	switch (param) {
	case BPE_INST_THLD:      return &bpe_inst_thld_map[level];
	case BPE_INST_ATTN:      return &bpe_inst_attn_map[level];
	case BPE_INST_ATK_RATE:  return &bpe_inst_atk_rate_map[level];
	case BPE_INST_HOLD_TIME: return &bpe_inst_hold_time_map[level];
	case BPE_INST_RLS_RATE:  return &bpe_inst_rls_rate_map[level];
	default:                 return NULL;
	}
}

static inline u32 *
cs35l45_get_bpe_inst_param(struct cs35l45_private *cs35l45, int level, int param)
{
	return &cs35l45->pdata.bpe_inst_cfg.params[param][level];
}

static inline u32 *
cs35l45_get_bpe_misc_param(struct cs35l45_private *cs35l45, int param)
{
	return &cs35l45->pdata.bpe_misc_cfg.params[param];
}

static inline const struct of_entry *
cs35l45_get_bst_bpe_inst_entry(int level, int param)
{
	switch (param) {
	case BST_BPE_INST_THLD:      return &bst_bpe_inst_thld_map[level];
	case BST_BPE_INST_ILIM:      return &bst_bpe_inst_ilim_map[level];
	case BST_BPE_INST_SS_ILIM:   return &bst_bpe_inst_ss_ilim_map[level];
	case BST_BPE_INST_ATK_RATE:  return &bst_bpe_inst_atk_rate_map[level];
	case BST_BPE_INST_HOLD_TIME: return &bst_bpe_inst_hold_time_map[level];
	case BST_BPE_INST_RLS_RATE:  return &bst_bpe_inst_rls_rate_map[level];
	default:                     return NULL;
	}
}

static inline u32 *
cs35l45_get_bst_bpe_inst_param(struct cs35l45_private *cs35l45, int level, int param)
{
	return &cs35l45->pdata.bst_bpe_inst_cfg.params[param][level];
}

static inline u32 *
cs35l45_get_bst_bpe_misc_param(struct cs35l45_private *cs35l45, int param)
{
	return &cs35l45->pdata.bst_bpe_misc_cfg.params[param];
}

static inline u32 *
cs35l45_get_bst_bpe_il_lim_param(struct cs35l45_private *cs35l45, int param)
{
	return &cs35l45->pdata.bst_bpe_il_lim_cfg.params[param];
}

static inline u32 *
cs35l45_get_ldpm_param(struct cs35l45_private *cs35l45, int param)
{
	return &cs35l45->pdata.ldpm_cfg.params[param];
}

static inline u32 *
cs35l45_get_classh_param(struct cs35l45_private *cs35l45, int param)
{
	return &cs35l45->pdata.classh_cfg.params[param];
}

#endif /* __CS35L41_H */
