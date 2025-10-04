#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include <cam_sensor_util.h>
#include <cam_sensor_io.h>
#include "cam_soc_util.h"
#include <cam_req_mgr_util.h>
#include "cam_req_mgr_dev.h"
#include "cam_debug_util.h"
#include "cam_power_dev.h"
#include "cam_common_util.h"
#include "camera_main.h"

#define WL_SLAVE_ADDRESS              0x52

static bool cam0_val_status = FALSE;
static bool cam1_val_status = FALSE;
static bool cam2_val_status = FALSE;

static cam_power_ctrl_t *power_ctrl = NULL;
/*
 * cam 0 rear main camera
 * cam 1 front camera
 * cam 2 rear aux camera
 * bit6 to bit0 -> ldo7 to ldo1
*/
#define CAM0_LDO_ON_REG_VALUE         0x19
#define CAM1_LDO_ON_REG_VALUE         0x06
#define CAM2_LDO_ON_REG_VALUE         0x20
#define CAM_LDO_OFF_REG_VALUE         0x00

/*
 * cam_sensor_i2c_reg_array
 * reg_addr
 * reg_data
 * delay
 * data_mask
*/

static struct cam_sensor_i2c_reg_array init_reg_setting[] = {
	{0x01, 0x40,                  0x1, 0x0},
	{0x02, 0x00,                  0x1, 0x0},
	{0x03, 0x30,                  0x1, 0x0},
	{0x04, 0x30,                  0x1, 0x0},
	{0x05, 0x80,                  0x1, 0x0},
	{0x06, 0x80,                  0x1, 0x0},
	{0x07, 0x80,                  0x1, 0x0},
	{0x08, 0x80,                  0x1, 0x0},
	{0x09, 0x80,                  0x1, 0x0},
	{0x0A, 0x00,                  0x1, 0x0},
	{0x0B, 0x00,                  0x1, 0x0},
	{0x0C, 0x00,                  0x1, 0x0},
	{0x0D, 0x00,                  0x1, 0x0},
	{0x0E, 0x00,                  0x1, 0x0},
	{0x0F, 0x00,                  0x1, 0x0},
};

static struct cam_sensor_i2c_reg_array cam0_reg_on_setting[] = {
	{0x03, 0x28,                  0x1, 0x0}, //cam0 ldo1 dvdd  1.1V
	{0x06, 0x80,                  0x1, 0x0}, //cam0 ldo4 avdd  2.8V
	{0x07, 0x80,                  0x1, 0x0}, //cam0 ldo5 afvdd 2.8V
	{0x01, 0x2F,                  0x1, 0x0}, //Higher current limit
	{0x0E, CAM0_LDO_ON_REG_VALUE, 0x1, 0x0}, //enable ldo1, ldo4, ldo5
};

static struct cam_sensor_i2c_reg_array cam1_reg_on_setting[] = {
	{0x04, 0x30,                  0x1, 0x0}, //cam1 ldo2 dvdd 1.2V
	{0x05, 0x80,                  0x1, 0x0}, //cam1 ldo3 avdd 2.8V
	{0x01, 0x2F,                  0x1, 0x0}, //Higher current limit
	{0x0E, CAM1_LDO_ON_REG_VALUE, 0x1, 0x0}, //enable ldo2,ldo3
};

static struct cam_sensor_i2c_reg_array cam2_reg_on_setting[] = {
	{0x08, 0x80,                  0x1, 0x0}, //cam2 ldo6 avdd 2.8V
	{0x01, 0x2F,                  0x1, 0x0}, //Higher current limit
	{0x0E, CAM2_LDO_ON_REG_VALUE, 0x1, 0x0}, //enable ldo6
};

static struct cam_sensor_i2c_reg_array cam3_reg_on_setting[] = {
	{0x03, 0x30,                  0x1, 0x0}, //cam0 ldo1 dvdd  1.2V
	{0x06, 0x80,                  0x1, 0x0}, //cam0 ldo4 avdd  2.8V
	{0x07, 0x80,                  0x1, 0x0}, //cam0 ldo5 afvdd 2.8V
	{0x01, 0x2F,                  0x1, 0x0}, //Higher current limit
	{0x0E, CAM0_LDO_ON_REG_VALUE, 0x1, 0x0}, //enable ldo1, ldo4, ldo5
};

static struct cam_sensor_i2c_reg_array cam_reg_off_setting[] = {
	{0x0E, CAM_LDO_OFF_REG_VALUE, 0x2, 0x0}, //disable all
};

static struct cam_sensor_i2c_reg_setting wl_init_reg_setting = {
	.reg_setting = init_reg_setting,
	.size = 15,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting wl_cam0_reg_on_setting = {
	.reg_setting = cam0_reg_on_setting,
	.size = 5,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting wl_cam3_reg_on_setting = {
	.reg_setting = cam3_reg_on_setting,
	.size = 5,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting wl_cam1_reg_on_setting = {
	.reg_setting = cam1_reg_on_setting,
	.size = 4,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting wl_cam2_reg_on_setting = {
	.reg_setting = cam2_reg_on_setting,
	.size = 3,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting wl_reg_off_setting = {
	.reg_setting = cam_reg_off_setting,
	.size = 1,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

// Rear main camera
static int set_cam0_vol_on(bool on)
{
	int rc = 0;
	struct camera_io_master io_master_info =
		power_ctrl->io_master_info;

	rc = camera_io_init(&io_master_info);
	if (rc < 0) {
		return -EINVAL;
	}

	if (on)
	{
	    cam0_val_status = TRUE;

	    if ((cam1_val_status == TRUE) && (cam2_val_status == FALSE)) {
	        wl_cam0_reg_on_setting.reg_setting[4].reg_data = CAM0_LDO_ON_REG_VALUE | CAM1_LDO_ON_REG_VALUE;
	    } else if ((cam1_val_status == FALSE) && (cam2_val_status == TRUE)) {
	        wl_cam0_reg_on_setting.reg_setting[4].reg_data = CAM0_LDO_ON_REG_VALUE | CAM2_LDO_ON_REG_VALUE;
	    } else if ((cam1_val_status == TRUE) && (cam2_val_status == TRUE)) {
	        wl_cam0_reg_on_setting.reg_setting[4].reg_data = CAM0_LDO_ON_REG_VALUE | CAM1_LDO_ON_REG_VALUE | CAM2_LDO_ON_REG_VALUE;
	    } else {
	        wl_cam0_reg_on_setting.reg_setting[4].reg_data = CAM0_LDO_ON_REG_VALUE;
	    }

		rc = camera_io_dev_write(&(io_master_info), &wl_cam0_reg_on_setting);
		CAM_INFO(CAM_SENSOR, "set_cam0_vol_on reg_val: %x", wl_cam0_reg_on_setting.reg_setting[4].reg_data);
	}
	else
	{
	    cam0_val_status = FALSE;

	    if ((cam1_val_status == TRUE) && (cam2_val_status == FALSE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM1_LDO_ON_REG_VALUE;
	    } else if ((cam1_val_status == FALSE) && (cam2_val_status == TRUE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM2_LDO_ON_REG_VALUE;
	    } else if ((cam1_val_status == TRUE) && (cam2_val_status == TRUE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM1_LDO_ON_REG_VALUE | CAM2_LDO_ON_REG_VALUE;
	    } else {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM_LDO_OFF_REG_VALUE;
	    }

		rc = camera_io_dev_write(&(io_master_info), &wl_reg_off_setting);
	}
	camera_io_release(&io_master_info);

	return rc;
}

// Front camera
static int set_cam1_vol_on(bool on)
{
	int rc = 0;
	struct camera_io_master io_master_info =
		power_ctrl->io_master_info;

	rc = camera_io_init(&io_master_info);
	if (rc < 0) {
		return -EINVAL;
	}

	if (on)
	{
	    cam1_val_status = TRUE;

	    if ((cam0_val_status == TRUE) && (cam2_val_status == FALSE)) {
	        wl_cam1_reg_on_setting.reg_setting[3].reg_data = CAM1_LDO_ON_REG_VALUE | CAM0_LDO_ON_REG_VALUE;
	    } else if ((cam0_val_status == FALSE) && (cam2_val_status == TRUE)) {
	        wl_cam1_reg_on_setting.reg_setting[3].reg_data = CAM1_LDO_ON_REG_VALUE | CAM2_LDO_ON_REG_VALUE;
	    } else if ((cam0_val_status == TRUE) && (cam2_val_status == TRUE)) {
	        wl_cam1_reg_on_setting.reg_setting[3].reg_data = CAM1_LDO_ON_REG_VALUE | CAM0_LDO_ON_REG_VALUE | CAM2_LDO_ON_REG_VALUE;
	    } else {
	        wl_cam1_reg_on_setting.reg_setting[3].reg_data = CAM1_LDO_ON_REG_VALUE;
	    }

		rc = camera_io_dev_write(&(io_master_info), &wl_cam1_reg_on_setting);
		CAM_INFO(CAM_SENSOR, "set_cam1_vol_on reg_val: %x", wl_cam1_reg_on_setting.reg_setting[3].reg_data );
	}
	else
	{
	    cam1_val_status = FALSE;

	    if ((cam0_val_status == TRUE) && (cam2_val_status == FALSE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM0_LDO_ON_REG_VALUE;
	    } else if ((cam0_val_status == FALSE) && (cam2_val_status == TRUE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM2_LDO_ON_REG_VALUE;
	    } else if ((cam0_val_status == TRUE) && (cam2_val_status == TRUE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM0_LDO_ON_REG_VALUE | CAM2_LDO_ON_REG_VALUE;
	    } else {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM_LDO_OFF_REG_VALUE;
	    }

		rc = camera_io_dev_write(&(io_master_info), &wl_reg_off_setting);
	}

	camera_io_release(&io_master_info);

	return rc;
}

// Macro camera
static int set_cam2_vol_on(bool on)
{
	int rc = 0;
	struct camera_io_master io_master_info =
		power_ctrl->io_master_info;

	rc = camera_io_init(&io_master_info);
	if (rc < 0) {
		return -EINVAL;
	}

	if (on)
	{
	    cam2_val_status = TRUE;

	    if ((cam0_val_status == TRUE) && (cam1_val_status == FALSE)) {
	        wl_cam2_reg_on_setting.reg_setting[2].reg_data = CAM2_LDO_ON_REG_VALUE | CAM0_LDO_ON_REG_VALUE;
	    } else if ((cam0_val_status == FALSE) && (cam1_val_status == TRUE)) {
	        wl_cam2_reg_on_setting.reg_setting[2].reg_data = CAM2_LDO_ON_REG_VALUE | CAM1_LDO_ON_REG_VALUE;
	    } else if ((cam0_val_status == TRUE) && (cam1_val_status == TRUE)) {
	        wl_cam2_reg_on_setting.reg_setting[2].reg_data = CAM2_LDO_ON_REG_VALUE | CAM0_LDO_ON_REG_VALUE | CAM1_LDO_ON_REG_VALUE;
	    } else {
	        wl_cam2_reg_on_setting.reg_setting[2].reg_data = CAM2_LDO_ON_REG_VALUE;
	    }

		rc = camera_io_dev_write(&(io_master_info), &wl_cam2_reg_on_setting);
		CAM_INFO(CAM_SENSOR, "set_cam2_vol_on reg_val: %x", wl_cam2_reg_on_setting.reg_setting[2].reg_data);;
	}
	else
	{
	    cam2_val_status = FALSE;

	    if ((cam0_val_status == TRUE) && (cam1_val_status == FALSE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM0_LDO_ON_REG_VALUE;
	    } else if ((cam0_val_status == FALSE) && (cam1_val_status == TRUE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM1_LDO_ON_REG_VALUE;
	    } else if ((cam0_val_status == TRUE) && (cam1_val_status == TRUE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM0_LDO_ON_REG_VALUE | CAM1_LDO_ON_REG_VALUE;
	    } else {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM_LDO_OFF_REG_VALUE;
	    }

		rc = camera_io_dev_write(&(io_master_info), &wl_reg_off_setting);
	}

	camera_io_release(&io_master_info);
	return rc;
}

// 2st Rear main camera
static int set_cam3_vol_on(bool on)
{
	int rc = 0;
	struct camera_io_master io_master_info =
		power_ctrl->io_master_info;

	rc = camera_io_init(&io_master_info);
	if (rc < 0) {
		return -EINVAL;
	}

	if (on)
	{
	    cam0_val_status = TRUE;

	    if ((cam1_val_status == TRUE) && (cam2_val_status == FALSE)) {
	        wl_cam3_reg_on_setting.reg_setting[4].reg_data = CAM0_LDO_ON_REG_VALUE | CAM1_LDO_ON_REG_VALUE;
	    } else if ((cam1_val_status == FALSE) && (cam2_val_status == TRUE)) {
	        wl_cam3_reg_on_setting.reg_setting[4].reg_data = CAM0_LDO_ON_REG_VALUE | CAM2_LDO_ON_REG_VALUE;
	    } else if ((cam1_val_status == TRUE) && (cam2_val_status == TRUE)) {
	        wl_cam3_reg_on_setting.reg_setting[4].reg_data = CAM0_LDO_ON_REG_VALUE | CAM1_LDO_ON_REG_VALUE | CAM2_LDO_ON_REG_VALUE;
	    } else {
	        wl_cam3_reg_on_setting.reg_setting[4].reg_data = CAM0_LDO_ON_REG_VALUE;
	    }

		rc = camera_io_dev_write(&(io_master_info), &wl_cam3_reg_on_setting);
		CAM_INFO(CAM_SENSOR, "set_cam3_vol_on reg_val: %x", wl_cam3_reg_on_setting.reg_setting[4].reg_data);
	}
	else
	{
	    cam0_val_status = FALSE;

	    if ((cam1_val_status == TRUE) && (cam2_val_status == FALSE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM1_LDO_ON_REG_VALUE;
	    } else if ((cam1_val_status == FALSE) && (cam2_val_status == TRUE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM2_LDO_ON_REG_VALUE;
	    } else if ((cam1_val_status == TRUE) && (cam2_val_status == TRUE)) {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM1_LDO_ON_REG_VALUE | CAM2_LDO_ON_REG_VALUE;
	    } else {
	        wl_reg_off_setting.reg_setting[0].reg_data = CAM_LDO_OFF_REG_VALUE;
	    }

		rc = camera_io_dev_write(&(io_master_info), &wl_reg_off_setting);
	}

	camera_io_release(&io_master_info);
	return rc;
}

int cam_power_ldo_control(uint16_t cam_cell_id, bool enable)
{
	int rc = 0;
	CAM_INFO(CAM_SENSOR, "cam_power_ldo_control enable: %d,power_ctrl:%p", (int)enable,power_ctrl);
	if(power_ctrl == NULL)
		return rc;
	switch(cam_cell_id) {
		case CAM_CELL_ID_0:
			rc = set_cam0_vol_on(enable);
			break;
		case CAM_CELL_ID_1:
			rc = set_cam2_vol_on(enable);
			break;
		case CAM_CELL_ID_2:
			rc = set_cam1_vol_on(enable);
			break;
		case CAM_CELL_ID_3:
			rc = set_cam3_vol_on(enable);
		default:
			break;
	}
	return rc;
}

cam_power_ctrl_t* get_cam_power_ctrl(void)
{
	return power_ctrl;
}

static int cam_power_parse_dt(cam_power_ctrl_t *p_ctrl)
{
	int rc = 0;
	struct cam_sensor_cci_client *cci_client = NULL;

	if (p_ctrl->io_master_info.master_type == CCI_MASTER) {
		cci_client = p_ctrl->io_master_info.cci_client;
		if (!cci_client) {
			return -EINVAL;
		}
		cci_client->cci_i2c_master = MASTER_0;
		cci_client->cci_device     = CCI_DEVICE_1;
		cci_client->sid            = (WL_SLAVE_ADDRESS >> 1);
		cci_client->retries        = 3;
		cci_client->id_map         = 0;
		cci_client->i2c_freq_mode  = I2C_FAST_MODE;
	}

	return rc;
}

static const struct v4l2_subdev_internal_ops cam_power_internal_ops = {
	.close = NULL,
};

static struct v4l2_subdev_core_ops cam_power_subdev_core_ops = {
	.ioctl = NULL,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = NULL,
#endif
};

static struct v4l2_subdev_ops cam_power_subdev_ops = {
	.core = &cam_power_subdev_core_ops,
};

static int cam_power_init_subdev(cam_power_ctrl_t *p_ctrl)
{
	int rc = 0;

	p_ctrl->v4l2_dev_str.internal_ops = &cam_power_internal_ops;
	p_ctrl->v4l2_dev_str.ops          = &cam_power_subdev_ops;
	strlcpy(p_ctrl->device_name, CAM_POWER_NAME,
		sizeof(p_ctrl->device_name));
	p_ctrl->v4l2_dev_str.name         = p_ctrl->device_name;
	p_ctrl->v4l2_dev_str.sd_flags     =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	p_ctrl->v4l2_dev_str.ent_function = CAM_POWER_DEVICE_TYPE;
	p_ctrl->v4l2_dev_str.token        = p_ctrl;

	rc = cam_register_subdev(&(p_ctrl->v4l2_dev_str));

	return rc;
}

static int init_wl2864c(struct camera_io_master io_master_info)
{
	int rc = 0;

	rc = camera_io_init(&io_master_info);
	if (rc < 0) {
		return rc;
	}

	rc = camera_io_dev_write(&(io_master_info), &wl_init_reg_setting);

	camera_io_release(&io_master_info);
	CAM_ERR(CAM_SENSOR, "rc = %d in init_wl2864c", rc);
	return rc;
}

static int cam_power_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int32_t rc = 0;

	struct platform_device *pdev = to_platform_device(dev);
	struct regulator* rgltr_power1 = NULL;
	struct regulator* rgltr_power2 = NULL;

	cam_power_ctrl_t *p_ctrl     = NULL;
	CAM_INFO(CAM_SENSOR, "cam_power_component_bind");
	p_ctrl = kzalloc(sizeof(cam_power_ctrl_t), GFP_KERNEL);
	if (!p_ctrl)
		return -ENOMEM;

	p_ctrl->soc_info.pdev        = pdev;
	p_ctrl->soc_info.dev         = &pdev->dev;
	p_ctrl->soc_info.dev_name    = pdev->name;
	p_ctrl->power_device_type    = MSM_CAMERA_PLATFORM_DEVICE;
	p_ctrl->userspace_probe      = false;

	p_ctrl->io_master_info.master_type = CCI_MASTER;

	p_ctrl->io_master_info.cci_client  = kzalloc(
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!p_ctrl->io_master_info.cci_client) {
		rc = -ENOMEM;
		goto free_p_ctrl;
	}

	rc = cam_power_parse_dt(p_ctrl);
	if (rc) {
		goto free_cci_client;
	}

	rc = cam_power_init_subdev(p_ctrl);
	if (rc)
		goto free_cci_client;

	p_ctrl->bridge_intf.device_hdl       = -1;
	p_ctrl->bridge_intf.ops.get_dev_info = NULL;
	p_ctrl->bridge_intf.ops.link_setup   = NULL;
	p_ctrl->bridge_intf.ops.apply_req    = NULL;

	platform_set_drvdata(pdev, p_ctrl);
	v4l2_set_subdevdata(&p_ctrl->v4l2_dev_str.sd, p_ctrl);

	power_ctrl = p_ctrl;

	rgltr_power1 = devm_regulator_get(p_ctrl->soc_info.dev, "wl2864c_power1");
	rgltr_power2 = devm_regulator_get(p_ctrl->soc_info.dev, "wl2864c_power2");
	cam_soc_util_regulator_enable(rgltr_power1, "wl2864c_power1", 1800000, 1800000, 680000, 2);
	cam_soc_util_regulator_enable(rgltr_power2, "wl2864c_power2", 1800000, 1800000, 680000, 2);
	rc = init_wl2864c(p_ctrl->io_master_info);
	cam_soc_util_regulator_disable(rgltr_power1, "wl2864c_power1", 1800000, 1800000, 680000, 2);
	cam_soc_util_regulator_disable(rgltr_power2, "wl2864c_power2", 1800000, 1800000, 680000, 2);

	if (rc)
		goto free_cci_client;

	return rc;
free_cci_client:
	kfree(p_ctrl->io_master_info.cci_client);
free_p_ctrl:
	kfree(p_ctrl);
	return rc;
}

static void cam_power_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	cam_power_ctrl_t  *p_ctrl;
	p_ctrl = platform_get_drvdata(pdev);
	if (!p_ctrl) {
		return;
	}

	kfree(p_ctrl->io_master_info.cci_client);
	platform_set_drvdata(pdev, NULL);
	kfree(p_ctrl);
	return;
}

const static struct component_ops cam_power_component_ops = {
	.bind = cam_power_component_bind,
	.unbind = cam_power_component_unbind,
};

static int cam_power_platform_driver_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_power_component_ops);
	return 0;
}

static int32_t cam_power_platform_driver_probe(struct platform_device *pdev)
{
	int rc = 0;
	CAM_ERR(CAM_SENSOR, "cam_power_platform_driver_probe");
	rc = component_add(&pdev->dev, &cam_power_component_ops);

	return rc;
}


static const struct of_device_id cam_power_dt_match[] = {
	{ .compatible = "qcom,ic_power" },
	{ }
};

MODULE_DEVICE_TABLE(of, cam_power_dt_match);

struct platform_driver cam_power_platform_driver = {
	.driver = {
		.name           = "qcom,ic_power",
		.owner          = THIS_MODULE,
		.of_match_table = cam_power_dt_match,
	},
	.probe = cam_power_platform_driver_probe,
	.remove = cam_power_platform_driver_remove,
};

int32_t cam_power_driver_init(void)
{
	int rc = 0;
	CAM_ERR(CAM_SENSOR, "cam_power_driver_init");
	rc = platform_driver_register(&cam_power_platform_driver);
	//LENOVO_CUSTOM_BEGIN, xuegb2, 20220609, ZUIS-21049, Coverity: Fix two return issue
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "cam_power_driver_init failed");
	}
	//LENOVO_CUSTOM_END
	return rc;
}

void cam_power_driver_exit(void)
{
	platform_driver_unregister(&cam_power_platform_driver);
}


MODULE_DESCRIPTION("CAM POWER driver");
MODULE_LICENSE("GPL v2");
