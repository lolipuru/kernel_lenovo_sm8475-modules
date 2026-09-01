// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2022, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, Lenovo. All rights reserved.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include "cam_ois_dev.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "dw9781.h"

static dev_t dw9781_dev;
static struct cdev dw9781_cdev;
static struct class *dw9781_class;
static wait_queue_head_t dw9781_wait;
static int g_result = 0;
static uint8_t g_flag = 0;

static struct cam_sensor_i2c_reg_array dw9781c_ready_check_array[] = {
	{0xd001, 0x0000, 0, 0},
	{0xfafa, 0x98ac, 0, 0},
	{0xf053, 0x70bd, 0, 0},
};

static struct cam_sensor_i2c_reg_array dw9781c_qTime_array[] = {
	{0x70db, 0x0000, 1, 0},
};

static struct cam_sensor_i2c_reg_array dw9781c_gyro_offset_cal_array[] = {
	{0x7011, 0x4015, 0, 0},
	{0x7010, 0x8000, 0, 0},
};

static struct cam_sensor_i2c_reg_array dw9781c_gyro_offset_cal_save_array[] = {
	{0x7011, 0x00aa, 10, 0},
	{0x7010, 0x8000, 0, 0},
};

static struct cam_sensor_i2c_reg_array dw9781c_fw_init_array[] = {
	{0xd001, 0x0000, 0, 0},
	{0xfafa, 0x98ac, 1, 0},
	{0xf053, 0x70bd, 1, 0},
	{0xd005, 0x0001, 0, 0},
	{0xdd03, 0x0002, 0, 0},
	{0xdd04, 0x0002, 0, 0},
	{0xde03, 0x0000, 0, 0},
	{0xde04, 0x0002, 10, 0},
	{0xde03, 0x0008, 0, 0},
	{0xde04, 0x0002, 10, 0},
	{0xde03, 0x0010, 0, 0},
	{0xde04, 0x0002, 10, 0},
	{0xde03, 0x0018, 0, 0},
	{0xde04, 0x0002, 10, 0},
	{0xde03, 0x0020, 0, 0},
	{0xde04, 0x0002, 10, 0},
};

static struct cam_sensor_i2c_reg_array dw9781c_fw_reset_array[] = {
	{0xd002, 0x0001, 4, 0},
	{0xd001, 0x0001, 25, 0},
	{0xebf1, 0x56fa, 0, 0},
};

static struct cam_sensor_i2c_reg_array dw9781c_erase_mtp_array[] = {
	{0xde03, 0x0027, 0, 0},
	{0xde04, 0x0008, 10, 0},
	{0xd000, 0x0000, 0, 0},
};

void dw9781_set_result(int result)
{
	g_result = result;
	CAM_DBG(CAM_OIS, "dw9781_set_result");
	wake_up(&dw9781_wait);
	g_flag = 1;
	CAM_DBG(CAM_OIS, "dw9781_set_result end");
}

static int dw9781_open(struct inode *inode, struct file *filp)
{
	CAM_DBG(CAM_OIS, "dw9781_open  major: %d  minor: %d\n",
		MAJOR(inode->i_rdev), MINOR(inode->i_rdev));
	return 0;
}

static int dw9781_release(struct inode *inode, struct file *filp)
{
	CAM_DBG(CAM_OIS, "dw9781_release  major: %d  minor: %d\n",
		MAJOR(inode->i_rdev), MINOR(inode->i_rdev));
	return 0;
}

static ssize_t dw9781_read(struct file *filp, char __user *buf,
	size_t count, loff_t *f_pos)
{
	char str[64];
	int len;

	len = sprintf(str, "%d", g_result);
	if (len > count)
		len = count;

	CAM_DBG(CAM_OIS, "OIS read result_value:%d ary:%s len:%d",
		g_result, str, len);

	if (copy_to_user(buf, str, len)) {
		CAM_ERR(CAM_OIS, "copy to user failed");
		return -EFAULT;
	}

	return len;
}

static ssize_t dw9781_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
	char kbuf[100];
	size_t len = (count < 100) ? count : 100;

	CAM_DBG(CAM_OIS, "OIS write");
	memset(kbuf, 0, sizeof(kbuf));
	if (copy_from_user(kbuf, buf, len))
		return -EFAULT;

	CAM_DBG(CAM_OIS, "writing context: %s\n", kbuf);
	return len;
}

static __poll_t dw9781_poll(struct file *filp, struct poll_table_struct *wait)
{
	__poll_t mask = 0;

	CAM_DBG(CAM_OIS, "dw9781_poll wait");
	poll_wait(filp, &dw9781_wait, wait);
	CAM_DBG(CAM_OIS, "dw9781_poll g_flag %d end", g_flag);

	if (g_flag == 1) {
		mask = (EPOLLIN | EPOLLRDNORM);
		g_flag = 0;
	}

	return mask;
}

static const struct file_operations dw9781_fops = {
	.owner   = THIS_MODULE,
	.open    = dw9781_open,
	.release = dw9781_release,
	.read    = dw9781_read,
	.write   = dw9781_write,
	.poll    = dw9781_poll,
};

int dw9781_init(void)
{
	int rc;

	CAM_DBG(CAM_OIS, "dw9781_init");
	init_waitqueue_head(&dw9781_wait);

	rc = alloc_chrdev_region(&dw9781_dev, 0, 1, "dw9781c_chrdev");
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "alloc_chrdev_region failed rc: %d", rc);
		return rc;
	}

	cdev_init(&dw9781_cdev, &dw9781_fops);
	dw9781_cdev.owner = THIS_MODULE;

	rc = cdev_add(&dw9781_cdev, dw9781_dev, 1);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "cdev_add failed rc: %d", rc);
		goto unregister_chrdev;
	}

	dw9781_class = class_create(THIS_MODULE, "dw9781_class");
	if (IS_ERR(dw9781_class)) {
		rc = PTR_ERR(dw9781_class);
		CAM_ERR(CAM_OIS, "class_create failed rc: %d", rc);
		goto del_cdev;
	}

	device_create(dw9781_class, NULL, dw9781_dev, NULL, "dw9781c_halo");
	return 0;

del_cdev:
	cdev_del(&dw9781_cdev);
unregister_chrdev:
	unregister_chrdev_region(dw9781_dev, 1);
	return rc;
}

void dw9781_exit(void)
{
	CAM_DBG(CAM_OIS, "dw9781_exit");
	device_destroy(dw9781_class, dw9781_dev);
	class_destroy(dw9781_class);
	cdev_del(&dw9781_cdev);
	unregister_chrdev_region(dw9781_dev, 1);
}

int dw9781_ois_ready_check(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	int i;
	struct cam_sensor_i2c_reg_setting i2c_settings;
	uint32_t val = 0;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.size = 1;
	i2c_settings.delay = 0;

	for (i = 0; i < ARRAY_SIZE(dw9781c_ready_check_array); i++) {
		i2c_settings.reg_setting = &dw9781c_ready_check_array[i];
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS ready check write failed. %d", rc);
			return -EINVAL;
		}
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0xf053, &val,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0 || (val != 0x70bd && val != 0x0)) {
		CAM_ERR(CAM_OIS, "[dw9781c_ois_ready_check] previous firmware download fail val: 0x%x", val);
		return -EINVAL;
	}

	return 0;
}

int dw9781_ois_fw_init(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	int i;
	struct cam_sensor_i2c_reg_setting i2c_settings;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	CAM_DBG(CAM_OIS, "ois fwinit count %d.", ARRAY_SIZE(dw9781c_fw_init_array));

	i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.size = 1;

	for (i = 0; i < ARRAY_SIZE(dw9781c_fw_init_array); i++) {
		CAM_DBG(CAM_OIS, "ois fwinit addr: 0x%x data: 0x%x delay:%d ms.",
			dw9781c_fw_init_array[i].reg_addr,
			dw9781c_fw_init_array[i].reg_data,
			dw9781c_fw_init_array[i].delay);

		i2c_settings.reg_setting = &dw9781c_fw_init_array[i];
		i2c_settings.delay = dw9781c_fw_init_array[i].delay;
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS fwinit addr:0x%x data:0x%x download failed. %d",
				dw9781c_fw_init_array[i].reg_addr,
				dw9781c_fw_init_array[i].reg_data, rc);
			return -EINVAL;
		}
		if (dw9781c_fw_init_array[i].delay)
			msleep(dw9781c_fw_init_array[i].delay);
	}

	CAM_DBG(CAM_OIS, "OIS fwinit settings success");
	return 0;
}

int dw9781_ois_check_chip_id(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	uint32_t chip_id = 0;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0xd000, &chip_id,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS read chip ID failed, rc %d", rc);
		return 0;
	}

	CAM_DBG(CAM_OIS, "dw9781_ois_check_chip_id: 0x%x", chip_id);
	if (chip_id == 0x9781)
		return 1;

	return 0;
}

int dw9781_ois_check_fw_version(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	uint32_t fw_ver = 0;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x70db, &fw_ver,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS get fw version failed, rc %d", rc);
		return 0;
	}

	CAM_INFO(CAM_OIS, "OIS get fw version, cur_fw_ver 0x%x", fw_ver);
	if (fw_ver != 0 && fw_ver != 0xffff)
		return 1;

	return 0;
}

int dw9781_ois_erase_mtp_and_shut_download_mode(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	int i;
	struct cam_sensor_i2c_reg_setting i2c_settings;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	CAM_DBG(CAM_OIS, "OIS erase_mtp reg count %d", ARRAY_SIZE(dw9781c_erase_mtp_array));

	i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.size = 1;

	for (i = 0; i < ARRAY_SIZE(dw9781c_erase_mtp_array); i++) {
		CAM_DBG(CAM_OIS, "OIS erase_mtp addr: 0x%x data: 0x%x delay:%d ms.",
			dw9781c_erase_mtp_array[i].reg_addr,
			dw9781c_erase_mtp_array[i].reg_data,
			dw9781c_erase_mtp_array[i].delay);

		i2c_settings.reg_setting = &dw9781c_erase_mtp_array[i];
		i2c_settings.delay = dw9781c_erase_mtp_array[i].delay;
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS erase_mtp failed. %d", rc);
			return -EINVAL;
		}
		if (dw9781c_erase_mtp_array[i].delay)
			msleep(dw9781c_erase_mtp_array[i].delay);
	}

	return 0;
}

int dw9781_ois_fw_load(struct cam_ois_ctrl_t *o_ctrl, const char *fw_name)
{
	int rc = 0;
	const struct firmware *fw = NULL;
	uint16_t *src_buf;
	uint8_t *read_buf;
	struct cam_sensor_i2c_reg_array *i2c_array = NULL;
	struct cam_sensor_i2c_reg_setting i2c_settings;
	uint32_t num_words, i;

	if (!o_ctrl || !fw_name) {
		CAM_ERR(CAM_OIS, "ois device or fw_name is NULL");
		return -EINVAL;
	}

	CAM_DBG(CAM_OIS, "ois firmware load");

	rc = request_firmware(&fw, fw_name, &o_ctrl->pdev->dev);
	if (rc < 0 || !fw) {
		CAM_ERR(CAM_OIS, "Failed to read fw %s size:%d", fw_name, (int)(fw ? fw->size : 0));
		return -EINVAL;
	}

	CAM_DBG(CAM_OIS, "ois firmware size %d", (int)fw->size);
	num_words = fw->size / 2;

	i2c_array = vmalloc(num_words * sizeof(struct cam_sensor_i2c_reg_array));
	if (!i2c_array) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array: fw_size: %u", (uint32_t)fw->size);
		release_firmware(fw);
		return -ENOMEM;
	}

	src_buf = (uint16_t *)fw->data;
	for (i = 0; i < num_words; i++) {
		i2c_array[i].reg_addr = 0x8000 + i;
		i2c_array[i].reg_data = swab16(src_buf[i]);
		i2c_array[i].delay = 0;
		i2c_array[i].data_mask = 0;
	}

	i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.size = num_words;
	i2c_settings.reg_setting = i2c_array;
	i2c_settings.delay = 0;

	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_settings);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW size(%d) download failed. %d", num_words * 2, rc);
		vfree(i2c_array);
		release_firmware(fw);
		return rc;
	}

	read_buf = vmalloc(fw->size);
	if (!read_buf) {
		CAM_ERR(CAM_OIS, "Failed in allocating read firmware.");
		vfree(i2c_array);
		release_firmware(fw);
		return -ENOMEM;
	}

	rc = camera_io_dev_read_seq(&(o_ctrl->io_master_info), 0x8000, read_buf,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD, num_words * 2);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW read failed, rc %d", rc);
		vfree(read_buf);
		vfree(i2c_array);
		release_firmware(fw);
		return rc;
	}

	for (i = 0; i < num_words; i++) {
		uint16_t r_val = swab16(((uint16_t *)read_buf)[i]);
		uint16_t f_val = swab16(src_buf[i]);
		if (r_val != f_val) {
			CAM_ERR(CAM_OIS, "OIS FW read failed fead_fw: 0x%x fw val:0x%x", r_val, f_val);
			rc = -EINVAL;
			break;
		}
	}

	vfree(read_buf);
	vfree(i2c_array);
	release_firmware(fw);
	return rc;
}

int dw9781_ois_checksum_and_fw_version_verify(struct cam_ois_ctrl_t *o_ctrl, const char *fw_name)
{
	int rc = 0;
	const struct firmware *fw = NULL;
	uint32_t reg_checksum = 0;
	uint32_t fw_checksum = 0;
	uint32_t cur_fw_ver = 0;

	if (!o_ctrl || !fw_name) {
		CAM_ERR(CAM_OIS, "ois device or fw_name is NULL");
		return -EINVAL;
	}

	rc = request_firmware(&fw, fw_name, &o_ctrl->pdev->dev);
	if (rc < 0 || !fw) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name);
		return -EINVAL;
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x70dc, &reg_checksum,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS read checksum failed");
		release_firmware(fw);
		return -EINVAL;
	}

	if (fw->size >= 4) {
		fw_checksum = ((uint16_t *)fw->data)[(fw->size / 2) - 1];
	}

	if (reg_checksum != fw_checksum) {
		CAM_ERR(CAM_OIS, "OIS checksum failed, reg_checksum 0x%x fw_checksum 0x%x",
			reg_checksum, fw_checksum);
		release_firmware(fw);
		return -EINVAL;
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x70db, &cur_fw_ver,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0 || cur_fw_ver == 0) {
		CAM_ERR(CAM_OIS, "OIS fw version failed, cur_fw_ver 0x%x", cur_fw_ver);
		release_firmware(fw);
		return -EINVAL;
	}

	release_firmware(fw);
	return 0;
}

int dw9781_ois_reset(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	int i;
	struct cam_sensor_i2c_reg_setting i2c_settings;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	CAM_DBG(CAM_OIS, "ois reset");
	CAM_DBG(CAM_OIS, "ois reset reg count %d", ARRAY_SIZE(dw9781c_fw_reset_array));

	i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.size = 1;

	for (i = 0; i < ARRAY_SIZE(dw9781c_fw_reset_array); i++) {
		CAM_DBG(CAM_OIS, "ois reset addr: 0x%x data: 0x%x delay:%d ms.",
			dw9781c_fw_reset_array[i].reg_addr,
			dw9781c_fw_reset_array[i].reg_data,
			dw9781c_fw_reset_array[i].delay);

		i2c_settings.reg_setting = &dw9781c_fw_reset_array[i];
		i2c_settings.delay = dw9781c_fw_reset_array[i].delay;
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS reset failed. %d", rc);
			return -EINVAL;
		}
		if (dw9781c_fw_reset_array[i].delay)
			msleep(dw9781c_fw_reset_array[i].delay);
	}

	return 0;
}

int dw9781_ois_check_and_fw_download(struct cam_ois_ctrl_t *o_ctrl, const char *fw_name)
{
	int rc = 0;

	if (!o_ctrl || !fw_name) {
		CAM_ERR(CAM_OIS, "ois device or fw_name is NULL");
		return -EINVAL;
	}

	rc = dw9781_ois_fw_init(o_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS fw init failed. %d", rc);
		return rc;
	}

	rc = dw9781_ois_ready_check(o_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS ready check failed. %d", rc);
		return rc;
	}

	rc = dw9781_ois_erase_mtp_and_shut_download_mode(o_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS erase mtp failed. %d", rc);
		return rc;
	}

	rc = dw9781_ois_fw_load(o_ctrl, fw_name);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS fw load failed. %d", rc);
		return rc;
	}

	rc = dw9781_ois_checksum_and_fw_version_verify(o_ctrl, fw_name);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS checksum and fw version verify failed");
		return rc;
	}

	rc = dw9781_ois_reset(o_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS reset failed");
		return rc;
	}

	return 0;
}

int dw9781_ois_reset_and_check(struct cam_ois_ctrl_t *o_ctrl, const char *fw_name)
{
	int rc = 0;

	if (!o_ctrl || !fw_name) {
		CAM_ERR(CAM_OIS, "ois device or fw_name is NULL");
		return -EINVAL;
	}

	rc = dw9781_ois_reset(o_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS reset failed. %d", rc);
		return rc;
	}

	if (!dw9781_ois_check_chip_id(o_ctrl) || !dw9781_ois_check_fw_version(o_ctrl)) {
		rc = dw9781_ois_check_and_fw_download(o_ctrl, fw_name);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS download failed. %d", rc);
			return rc;
		}
	}

	return 0;
}

int dw9781_ois_gyro_offset_calibration(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	int i;
	struct cam_sensor_i2c_reg_setting i2c_settings;
	uint32_t cal_status = 0;
	uint32_t x_offset = 0, y_offset = 0;
	int retry = 20;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	CAM_DBG(CAM_OIS, "OIS calibration");
	CAM_DBG(CAM_OIS, "OIS calibration reg count %d", ARRAY_SIZE(dw9781c_gyro_offset_cal_array));

	i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.size = 1;

	for (i = 0; i < ARRAY_SIZE(dw9781c_gyro_offset_cal_array); i++) {
		CAM_DBG(CAM_OIS, "OIS calibration addr: 0x%x data: 0x%x delay:%d ms.",
			dw9781c_gyro_offset_cal_array[i].reg_addr,
			dw9781c_gyro_offset_cal_array[i].reg_data,
			dw9781c_gyro_offset_cal_array[i].delay);

		i2c_settings.reg_setting = &dw9781c_gyro_offset_cal_array[i];
		i2c_settings.delay = dw9781c_gyro_offset_cal_array[i].delay;
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS calibration failed. %d", rc);
			return -EINVAL;
		}
		if (dw9781c_gyro_offset_cal_array[i].delay)
			msleep(dw9781c_gyro_offset_cal_array[i].delay);
	}

	while (retry-- > 0) {
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x7011, &cal_status,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
		if (rc == 0 && (cal_status & 0x8000))
			break;
		msleep(10);
	}

	if (!(cal_status & 0x8000)) {
		CAM_ERR(CAM_OIS, "OIS cal is not completed, cal_status:0x%x", cal_status);
		dw9781_set_result(0xff);
		return -EINVAL;
	}

	CAM_DBG(CAM_OIS, "OIS cal read %d status 0x%x", retry, cal_status);

	if (cal_status & 0x1) {
		CAM_DBG(CAM_OIS, "OIS cal X_OVER_NG %d", 1);
	}
	if (cal_status & 0x2) {
		CAM_DBG(CAM_OIS, "OIS cal Y_OVER_NG %d", 1);
	}
	if (cal_status & 0x10) {
		CAM_DBG(CAM_OIS, "OIS cal X_GYRO_RAW_DATA_CHECK %d", 1);
	}
	if (cal_status & 0x20) {
		CAM_DBG(CAM_OIS, "OIS cal Y_GYRO_RAW_DATA_CHECK %d", 1);
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x7012, &x_offset,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc == 0)
		CAM_DBG(CAM_OIS, "OIS x offset: 0x%x", x_offset);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x7013, &y_offset,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc == 0)
		CAM_DBG(CAM_OIS, "OIS y offset: 0x%x", y_offset);

	CAM_DBG(CAM_OIS, "OIS calibration save");
	CAM_DBG(CAM_OIS, "OIS calibration save reg count %d", ARRAY_SIZE(dw9781c_gyro_offset_cal_save_array));

	for (i = 0; i < ARRAY_SIZE(dw9781c_gyro_offset_cal_save_array); i++) {
		CAM_DBG(CAM_OIS, "OIS calibration save addr: 0x%x data: 0x%x delay:%d ms.",
			dw9781c_gyro_offset_cal_save_array[i].reg_addr,
			dw9781c_gyro_offset_cal_save_array[i].reg_data,
			dw9781c_gyro_offset_cal_save_array[i].delay);

		i2c_settings.reg_setting = &dw9781c_gyro_offset_cal_save_array[i];
		i2c_settings.delay = dw9781c_gyro_offset_cal_save_array[i].delay;
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS calibration save failed. %d", rc);
			return -EINVAL;
		}
		if (dw9781c_gyro_offset_cal_save_array[i].delay)
			msleep(dw9781c_gyro_offset_cal_save_array[i].delay);
	}

	if ((cal_status & 0x33) == 0) {
		CAM_DBG(CAM_OIS, "OIS cal success status 0x%x msg 0x%x", cal_status, 0);
		dw9781_set_result(0x2);
	} else {
		dw9781_set_result(0xff);
	}

	CAM_DBG(CAM_OIS, "OIS calibration retun value:%d end", g_result);
	return 0;
}

int dw9781_ois_apply_QTime(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	uint64_t qtime_ns = 0;
	uint64_t scaled;
	uint32_t val;
	struct cam_sensor_i2c_reg_setting qtime_settings;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	rc = cam_sensor_util_get_current_qtimer_ns(&qtime_ns);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "Failed to get current qtimer value: %d", rc);
		return rc;
	}

	/* Scale QTimer timestamp for DW9781 register */
	scaled = (qtime_ns >> 5) * 0xa7c5ac471b47843ULL;
	val = (uint32_t)((scaled >> 39) & 0xffff);

	dw9781c_qTime_array[0].reg_data = val;

	qtime_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	qtime_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	qtime_settings.size = 1;
	qtime_settings.reg_setting = dw9781c_qTime_array;
	qtime_settings.delay = 1;

	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &qtime_settings);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS qtime addr:0x%x data:0x%x update failed. %d",
			dw9781c_qTime_array[0].reg_addr, val, rc);
		return -EINVAL;
	}

	CAM_DBG(CAM_OIS, "OIS qtime addr:0x%x, data:0x%x, qtime_ns=%llu",
		dw9781c_qTime_array[0].reg_addr, val, qtime_ns);

	if (dw9781c_qTime_array[0].delay)
		msleep(dw9781c_qTime_array[0].delay);

	return 0;
}
