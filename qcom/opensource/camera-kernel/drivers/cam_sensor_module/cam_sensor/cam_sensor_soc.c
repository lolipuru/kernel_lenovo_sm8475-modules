// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2019, 2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <cam_sensor_cmn_header.h>
#include <cam_sensor_util.h>
#include <cam_sensor_io.h>
#include <cam_req_mgr_util.h>
#include "cam_sensor_soc.h"
#include "cam_soc_util.h"

uint32_t BoardType = 0;

int32_t cam_sensor_get_sub_module_index(struct device_node *of_node,
	struct cam_sensor_board_info *s_info)
{
	int rc = 0, i = 0;
	uint32_t val = 0;
	struct device_node *src_node = NULL;
	struct cam_sensor_board_info *sensor_info;

	sensor_info = s_info;

	for (i = 0; i < SUB_MODULE_MAX; i++)
		sensor_info->subdev_id[i] = -1;

	src_node = of_parse_phandle(of_node, "actuator-src", 0);
	if (!src_node) {
		CAM_DBG(CAM_SENSOR, "src_node NULL");
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		CAM_DBG(CAM_SENSOR, "actuator cell index %d, rc %d", val, rc);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "failed %d", rc);
			of_node_put(src_node);
			return rc;
		}
		sensor_info->subdev_id[SUB_MODULE_ACTUATOR] = val;
		of_node_put(src_node);
	}

	src_node = of_parse_phandle(of_node, "ois-src", 0);
	if (!src_node) {
		CAM_DBG(CAM_SENSOR, "src_node NULL");
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		CAM_DBG(CAM_SENSOR, " ois cell index %d, rc %d", val, rc);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "failed %d",  rc);
			of_node_put(src_node);
			return rc;
		}
		sensor_info->subdev_id[SUB_MODULE_OIS] = val;
		of_node_put(src_node);
	}

	src_node = of_parse_phandle(of_node, "eeprom-src", 0);
	if (!src_node) {
		CAM_DBG(CAM_SENSOR, "eeprom src_node NULL");
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		CAM_DBG(CAM_SENSOR, "eeprom cell index %d, rc %d", val, rc);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "failed %d", rc);
			of_node_put(src_node);
			return rc;
		}
		sensor_info->subdev_id[SUB_MODULE_EEPROM] = val;
		of_node_put(src_node);
	}

	src_node = of_parse_phandle(of_node, "led-flash-src", 0);
	if (!src_node) {
		CAM_DBG(CAM_SENSOR, " src_node NULL");
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		CAM_DBG(CAM_SENSOR, "led flash cell index %d, rc %d", val, rc);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "failed %d", rc);
			of_node_put(src_node);
			return rc;
		}
		sensor_info->subdev_id[SUB_MODULE_LED_FLASH] = val;
		of_node_put(src_node);
	}

	rc = of_property_read_u32(of_node, "csiphy-sd-index", &val);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "paring the dt node for csiphy rc %d", rc);
	else
		sensor_info->subdev_id[SUB_MODULE_CSIPHY] = val;

	return rc;
}

static int32_t cam_sensor_init_bus_params(struct cam_sensor_ctrl_t *s_ctrl)
{
	/* Validate input parameters */
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "failed: invalid params s_ctrl %pK",
			s_ctrl);
		return -EINVAL;
	}

	CAM_DBG(CAM_SENSOR,
		"master_type: %d", s_ctrl->io_master_info.master_type);
	/* Initialize cci_client */
	if (s_ctrl->io_master_info.master_type == CCI_MASTER) {
		s_ctrl->io_master_info.cci_client = kzalloc(sizeof(
			struct cam_sensor_cci_client), GFP_KERNEL);
		if (!(s_ctrl->io_master_info.cci_client))
			return -ENOMEM;
	} else if (s_ctrl->io_master_info.master_type == I2C_MASTER) {
		if (!(s_ctrl->io_master_info.client))
			return -EINVAL;
	} else {
		CAM_ERR(CAM_SENSOR,
			"Invalid master / Master type Not supported");
		return -EINVAL;
	}

	return 0;
}

static int32_t cam_sensor_driver_get_dt_data(struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int i = 0;
	struct cam_sensor_board_info *sensordata = NULL;
	struct device_node *of_node = s_ctrl->of_node;
	struct device_node *of_parent = NULL;
	struct cam_hw_soc_info *soc_info = &s_ctrl->soc_info;

	s_ctrl->sensordata = kzalloc(sizeof(*sensordata), GFP_KERNEL);
	if (!s_ctrl->sensordata)
		return -ENOMEM;

	sensordata = s_ctrl->sensordata;

	rc = cam_soc_util_get_dt_properties(soc_info);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed to read DT properties rc %d", rc);
		goto FREE_SENSOR_DATA;
	}

	rc =  cam_sensor_util_init_gpio_pin_tbl(soc_info,
			&sensordata->power_info.gpio_num_info);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed to read gpios %d", rc);
		goto FREE_SENSOR_DATA;
	}

	s_ctrl->id = soc_info->index;

	/* Validate cell_id */
	if (s_ctrl->id >= MAX_CAMERAS) {
		CAM_ERR(CAM_SENSOR, "Failed invalid cell_id %d", s_ctrl->id);
		rc = -EINVAL;
		goto FREE_SENSOR_DATA;
	}

	/* Store the index of BoB regulator if it is available */
	for (i = 0; i < soc_info->num_rgltr; i++) {
		if (!strcmp(soc_info->rgltr_name[i],
			"cam_bob")) {
			CAM_DBG(CAM_SENSOR,
				"i: %d cam_bob", i);
			s_ctrl->bob_reg_index = i;
			soc_info->rgltr[i] = devm_regulator_get(soc_info->dev,
				soc_info->rgltr_name[i]);
			if (IS_ERR_OR_NULL(soc_info->rgltr[i])) {
				CAM_WARN(CAM_SENSOR,
					"Regulator: %s get failed",
					soc_info->rgltr_name[i]);
				soc_info->rgltr[i] = NULL;
			} else {
				if (!of_property_read_bool(of_node,
					"pwm-switch")) {
					CAM_DBG(CAM_SENSOR,
					"No BoB PWM switch param defined");
					s_ctrl->bob_pwm_switch = false;
				} else {
					s_ctrl->bob_pwm_switch = true;
				}
			}
		}
	}

	/* Read subdev info */
	rc = cam_sensor_get_sub_module_index(of_node, sensordata);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed to get sub module index, rc=%d",
			 rc);
		goto FREE_SENSOR_DATA;
	}

	rc = cam_sensor_init_bus_params(s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR,
			"Failed in Initialize Bus params, rc %d", rc);
		goto FREE_SENSOR_DATA;
	}

	if (s_ctrl->io_master_info.master_type == CCI_MASTER) {
		/* Get CCI master */
		rc = of_property_read_u32(of_node, "cci-master",
			&s_ctrl->cci_i2c_master);
		CAM_DBG(CAM_SENSOR, "cci-master %d, rc %d",
			s_ctrl->cci_i2c_master, rc);
		if (rc < 0) {
			/* Set default master 0 */
			s_ctrl->cci_i2c_master = MASTER_0;
			rc = 0;
		}

		of_parent = of_get_parent(of_node);
		if (of_property_read_u32(of_parent, "cell-index",
				&s_ctrl->cci_num) < 0)
			/* Set default master 0 */
			s_ctrl->cci_num = CCI_DEVICE_0;

		s_ctrl->io_master_info.cci_client->cci_device
			= s_ctrl->cci_num;

		CAM_DBG(CAM_SENSOR, "cci-index %d", s_ctrl->cci_num);
	}

	if (of_property_read_u32(of_node, "sensor-position-pitch",
		&sensordata->pos_pitch) < 0) {
		CAM_DBG(CAM_SENSOR, "Invalid sensor position");
		sensordata->pos_pitch = 360;
	}
	if (of_property_read_u32(of_node, "sensor-position-roll",
		&sensordata->pos_roll) < 0) {
		CAM_DBG(CAM_SENSOR, "Invalid sensor position");
		sensordata->pos_roll = 360;
	}
	if (of_property_read_u32(of_node, "sensor-position-yaw",
		&sensordata->pos_yaw) < 0) {
		CAM_DBG(CAM_SENSOR, "Invalid sensor position");
		sensordata->pos_yaw = 360;
	}

	if (!of_property_read_bool(of_node, "aon-user")) {
		CAM_DBG(CAM_SENSOR,
			"SENSOR cell_idx: %d not use for AON usecase",
			s_ctrl->soc_info.index);
		s_ctrl->is_aon_user = false;
	} else {
		CAM_DBG(CAM_SENSOR,
			"SENSOR cell_idx: %d is user for AON usecase",
			s_ctrl->soc_info.index);
		s_ctrl->is_aon_user = true;
	}

	rc = cam_sensor_util_aon_registration(
		s_ctrl->sensordata->subdev_id[SUB_MODULE_CSIPHY],
		s_ctrl->is_aon_user);
	if (rc) {
		CAM_ERR(CAM_SENSOR, "Aon registration failed, rc: %d", rc);
		goto FREE_SENSOR_DATA;
	}

	if (!of_property_read_bool(of_node, "hw-no-ops"))
		s_ctrl->hw_no_ops = false;
	else
		s_ctrl->hw_no_ops = true;

	return rc;

FREE_SENSOR_DATA:
	kfree(sensordata);
	s_ctrl->sensordata = NULL;

	return rc;
}

static int32_t cam_get_boardtype_from_kernel(uint32_t boardid)
{
	uint32_t BoardidNumber[2][15] = {
		//PRC
		{
		(uint32_t)0,//EVB
		(uint32_t)1,//EVT1-1
		(uint32_t)16,//EVT1-2
		(uint32_t)17,//EVT1-3
		(uint32_t)2,//DVT1-1
		(uint32_t)3,//DVT1-2,preDVT for halo
		(uint32_t)18,//DVT1-3,DVT1B for halo
		(uint32_t)19,//DVT1-4,DVT1C for halo
		(uint32_t)4,//DVT2-1
		(uint32_t)5,//DVT2-2
		(uint32_t)20,//DVT2-3
		(uint32_t)21,//DVT2-4
		(uint32_t)6,//PVT1
		(uint32_t)22,//PVT2
		(uint32_t)7 //MP
		},
		//ROW
		{
		(uint32_t)8,//EVB
		(uint32_t)9,//EVT1-1
		(uint32_t)24,//EVT1-2
		(uint32_t)25,//EVT1-3
		(uint32_t)10,//DVT1-1
		(uint32_t)11,//DVT1-2
		(uint32_t)26,//DVT1-3
		(uint32_t)27,//DVT1-4
		(uint32_t)12,//DVT2-1
		(uint32_t)13,//DVT2-2
		(uint32_t)28,//DVT2-3
		(uint32_t)29,//DVT2-4
		(uint32_t)14,//PVT1
		(uint32_t)30,//PVT2
		(uint32_t)15//MP
		}
	};
	int32_t Xcount,Ycount,Flag = 0;
	for(Xcount = 0; Xcount < 2; Xcount++) {
		for(Ycount = 0; Ycount < 15; Ycount++) {
			if(boardid == BoardidNumber[Xcount][Ycount]) {
				Flag = Ycount;
				break;
			}
		}
	}
	return (Flag <= 7)? 1 : 2; 
}

static int32_t cam_get_boardid(void)
{
	struct device_node *node = NULL;
	int32_t count, gpioNumber;
	uint32_t Boardid = 0;
	char *gpioCompatibleStr[5] = {"regulator-107","regulator-108","regulator-109","regulator-180","regulator-181"};
	char *gpioNameStr[5] = {"107","108","109","180","181"};
	int32_t  gpioValue[5] = {0};

	for(count = 0; count < 5; count++) {
		gpioNumber = 0;
		node = of_find_compatible_node(NULL,NULL,gpioCompatibleStr[count]);
		if(node != NULL){
			CAM_DBG(CAM_SENSOR, "get %s success!\n", gpioCompatibleStr[count]);
		} else {
			CAM_DBG(CAM_SENSOR, "get %s fail!\n", gpioCompatibleStr[count]);
			return 0;
		}
		gpioNumber = of_get_named_gpio(node,"gpio",0);
		if(!gpio_request(gpioNumber,gpioNameStr[count]))
		{
			gpioValue[count] = __gpio_get_value(gpioNumber);
			CAM_DBG(CAM_SENSOR, "gpioValue[%s] : %d\n", gpioNameStr[count], gpioValue[count]);
			gpio_free(gpioNumber);
		}
		Boardid = Boardid | (uint32_t)(gpioValue[count] << count);
	}
	CAM_DBG(CAM_SENSOR, "Boradid : %u\n",Boardid);
	BoardType = cam_get_boardtype_from_kernel(Boardid);
	CAM_DBG(CAM_SENSOR, "BoardType : %d\n",BoardType);
	return 1;
}

int32_t cam_sensor_parse_dt(struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t i, rc = 0;
	struct cam_hw_soc_info *soc_info = &s_ctrl->soc_info;

	/* Parse dt information and store in sensor control structure */
	rc = cam_sensor_driver_get_dt_data(s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed to get dt data rc %d", rc);
		return rc;
	}
	cam_get_boardid();
	/* Initialize mutex */
	mutex_init(&(s_ctrl->cam_sensor_mutex));

	/* Initialize default parameters */
	for (i = 0; i < soc_info->num_clk; i++) {
		soc_info->clk[i] = devm_clk_get(soc_info->dev,
					soc_info->clk_name[i]);
		if (!soc_info->clk[i]) {
			CAM_ERR(CAM_SENSOR, "get failed for %s",
				 soc_info->clk_name[i]);
			rc = -ENOENT;
			return rc;
		}
	}
	/* Initialize regulators to default parameters */
	for (i = 0; i < soc_info->num_rgltr; i++) {
		soc_info->rgltr[i] = devm_regulator_get(soc_info->dev,
					soc_info->rgltr_name[i]);
		if (IS_ERR_OR_NULL(soc_info->rgltr[i])) {
			rc = PTR_ERR(soc_info->rgltr[i]);
			rc = rc ? rc : -EINVAL;
			CAM_ERR(CAM_SENSOR, "get failed for regulator %s",
				 soc_info->rgltr_name[i]);
			return rc;
		}
		CAM_DBG(CAM_SENSOR, "get for regulator %s",
			soc_info->rgltr_name[i]);
	}

	return rc;
}
