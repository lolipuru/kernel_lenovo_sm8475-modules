// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2022, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, Lenovo. All rights reserved.
 */

#ifndef _DW9781_H_
#define _DW9781_H_

#include <linux/types.h>
#include <linux/firmware.h>
#include "cam_ois_dev.h"

int dw9781_init(void);
void dw9781_exit(void);

int dw9781_ois_check_and_fw_download(struct cam_ois_ctrl_t *o_ctrl, const char *fw_name);
int dw9781_ois_ready_check(struct cam_ois_ctrl_t *o_ctrl);
int dw9781_ois_fw_init(struct cam_ois_ctrl_t *o_ctrl);
int dw9781_ois_check_chip_id(struct cam_ois_ctrl_t *o_ctrl);
int dw9781_ois_check_fw_version(struct cam_ois_ctrl_t *o_ctrl);
int dw9781_ois_erase_mtp_and_shut_download_mode(struct cam_ois_ctrl_t *o_ctrl);
int dw9781_ois_fw_load(struct cam_ois_ctrl_t *o_ctrl, const char *fw_name);
int dw9781_ois_checksum_and_fw_version_verify(struct cam_ois_ctrl_t *o_ctrl, const char *fw_name);
int dw9781_ois_reset(struct cam_ois_ctrl_t *o_ctrl);
int dw9781_ois_reset_and_check(struct cam_ois_ctrl_t *o_ctrl, const char *fw_name);
int dw9781_ois_gyro_offset_calibration(struct cam_ois_ctrl_t *o_ctrl);
int dw9781_ois_apply_QTime(struct cam_ois_ctrl_t *o_ctrl);
void dw9781_set_result(int result);

#endif /* _DW9781_H_ */
