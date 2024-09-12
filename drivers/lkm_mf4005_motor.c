/*
 * Copyright (c) 2024 Felipe Neves
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT lkm_mf4005
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/lkm_mf4005_motor.h>
#include "lkm_mf4005_motor_regs.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mf4005, CONFIG_LOG_DEFAULT_LEVEL);

struct mf4005_config {
	const struct device *can_bus;
	struct can_filter device_filter;
	int id;
	int max_current_ma;
	int encoder_resolution_bits;
	int max_speed_dps;
};

struct mf4005_data {
	struct k_sem can_signal;
	struct can_frame rx_frame;
};

static void can_rx_callback (const struct device *dev, struct can_frame *frame,
			     void *user_data)
{
	struct mf4005_data *dev_data = (struct mf4005_data *)user_data;
	switch(frame->data[0]) {
		case MF4005_PID_GET_REG:
		case MF4005_POSITION_GET_REG:
		case MF4005_STATE_GET_REG:
			memcpy(&dev_data->rx_frame, frame, sizeof(struct can_frame));
			k_sem_give(&dev_data->can_signal);				
		break;
		default:
			LOG_DBG("Unhandled message arrived reg: 0x%X", frame->data[0]);
		break;
	}
}

static int mf_motor_start(const struct device *dev)
{
	const struct mf4005_config *cfg = dev->config;

	struct can_frame tx_frame = {
		.flags = 0,
		.id = cfg->id,
		.dlc = 8,
		.data[0] = MF4005_TURN_ON_REG
	};

	return can_send(cfg->can_bus, &tx_frame, K_FOREVER, NULL, NULL);
}

static int mf_motor_stop(const struct device *dev)
{
	const struct mf4005_config *cfg = dev->config;

	struct can_frame tx_frame = {
		.flags = 0,
		.id = cfg->id,
		.dlc = 8,
		.data[0] = MF4005_TURN_OFF_REG
	};

	return can_send(cfg->can_bus, &tx_frame, K_FOREVER, NULL, NULL);
}

static int mf_motor_attr_set(const struct device *dev, enum mf4005_motor_attribute attr,
			     int attr_val)
{
	const struct mf4005_config *cfg = dev->config;
	struct mf4005_data *dev_data = dev->data;
	int ret;

	struct can_frame tx_frame = {
		.flags = 0,
		.id = cfg->id,
		.dlc = 8,
		.data[0] = MF4005_PID_GET_REG,
	};

	ret = can_send(cfg->can_bus, &tx_frame, K_FOREVER, NULL, NULL);
	if(ret < 0) {
		LOG_ERR("Failed to fetch the MF4005 attributes!");
		return ret;
	}

	ret = k_sem_take(&dev_data->can_signal, K_MSEC(500));
	if(ret < 0) {
		LOG_ERR("Fetch the MF4005 attributes timed out!");
		return ret;
	}

	switch(attr) {
		case MF4005_ATTR_POSITION_KP:
			dev_data->rx_frame.data[MF4005_POS_PID_KP_INDEX] = 
				(int8_t)(attr_val & 0xFF);
		break;
		case MF4005_ATTR_POSITION_KI:
			dev_data->rx_frame.data[MF4005_POS_PID_KI_INDEX] = 
				(int8_t)(attr_val & 0xFF);
		break;
		case MF4005_ATTR_SPEED_KP:
			dev_data->rx_frame.data[MF4005_SPD_PID_KP_INDEX] = 
				(int8_t)(attr_val & 0xFF);
		break;
		case MF4005_ATTR_SPEED_KI:
			dev_data->rx_frame.data[MF4005_SPD_PID_KI_INDEX] = 
				(int8_t)(attr_val & 0xFF);
		break;
		case MF4005_ATTR_TORQUE_KP:
			dev_data->rx_frame.data[MF4005_TOR_PID_KP_INDEX] = 
				(int8_t)(attr_val & 0xFF);
		break;
		case MF4005_ATTR_TORQUE_KI:
			dev_data->rx_frame.data[MF4005_TOR_PID_KI_INDEX] = 
				(int8_t)(attr_val & 0xFF);
		break;
		default:
			LOG_ERR("Attribute %d not supported", attr);
			return -ENOTSUP;
		break;
	}

	memcpy(&tx_frame.data, &dev_data->rx_frame.data, sizeof(tx_frame.data));
	tx_frame.data[0] = MF4005_PID_SET_REG;
	
	return can_send(cfg->can_bus, &tx_frame, K_FOREVER, NULL, NULL);
}

static int mf_motor_attr_get(const struct device *dev, enum mf4005_motor_attribute attr,
			     int *attr_val)
{
	const struct mf4005_config *cfg = dev->config;
	struct mf4005_data *dev_data = dev->data;
	int ret;

	struct can_frame tx_frame = {
		.flags = 0,
		.id = cfg->id,
		.dlc = 8,
		.data[0] = MF4005_PID_GET_REG,
	};

	if (!attr_val) {
		return -EINVAL;
	}

	ret = can_send(cfg->can_bus, &tx_frame, K_FOREVER, NULL, NULL);
	if(ret < 0) {
		LOG_ERR("Failed to fetch the MF4005 attributes!");
		return ret;
	}

	ret = k_sem_take(&dev_data->can_signal, K_MSEC(500));
	if(ret < 0) {
		LOG_ERR("Fetch the MF4005 attributes timed out!");
		return ret;
	}

	switch(attr) {
		case MF4005_ATTR_POSITION_KP:
			*attr_val = (int)dev_data->rx_frame.data[MF4005_POS_PID_KP_INDEX]; 
		break;
		case MF4005_ATTR_POSITION_KI:
			*attr_val = (int)dev_data->rx_frame.data[MF4005_POS_PID_KI_INDEX]; 
		break;
		case MF4005_ATTR_SPEED_KP:
			*attr_val = (int)dev_data->rx_frame.data[MF4005_SPD_PID_KP_INDEX]; 
		break;
		case MF4005_ATTR_SPEED_KI:
			*attr_val = (int)dev_data->rx_frame.data[MF4005_SPD_PID_KI_INDEX]; 
		break;
		case MF4005_ATTR_TORQUE_KP:
			*attr_val = (int)dev_data->rx_frame.data[MF4005_TOR_PID_KP_INDEX]; 
		break;
		case MF4005_ATTR_TORQUE_KI:
			*attr_val = (int)dev_data->rx_frame.data[MF4005_TOR_PID_KI_INDEX]; 
		break;
		default:
			LOG_ERR("Attribute %d not supported", attr);
			return -ENOTSUP;
		break;
	}
	
	return 0;
}

static int mf_motor_channel_set(const struct device *dev, enum mf4005_motor_channel chan,
			        int val)
{
	const struct mf4005_config *cfg = dev->config;
	struct can_frame tx_frame = {
		.flags = 0,
		.id = cfg->id,
		.dlc = 8,
	};

	switch(chan) {
		case MF4005_CHAN_POSITION_MILI_DEGREES:
			/* Scale position from degrees to 0.01 deg per LSB */
			val = val * MF4005_ANGLE_SCALE_MULT;

			tx_frame.data[0] = MF4005_POSITION_INC_REG;
			tx_frame.data[MF4005_ANGLE_INC_0_INDEX] = (uint8_t)(val & 0xFF);
			tx_frame.data[MF4005_ANGLE_INC_1_INDEX] = (uint8_t)((val >> 8) & 0xFF);
			tx_frame.data[MF4005_ANGLE_INC_2_INDEX] = (uint8_t)((val >> 16) & 0xFF);
			tx_frame.data[MF4005_ANGLE_INC_3_INDEX] = (uint8_t)((val >> 24) & 0xFF);
		break;

		case MF4005_CHAN_SPEED_DEG_SECONDS:
			if(val > cfg->max_speed_dps || val < -cfg->max_speed_dps) {
				LOG_ERR("Speed value %d dps is beyond the supported limit", val);
				return -EINVAL;
			}

			/* Scale speed from DPS to 0.01 dps per LSB */
			val = val * MF4005_SPEED_SCALE_MULT;

			tx_frame.data[0] = MF4005_SPEED_SET_REG;
			tx_frame.data[MF4005_SPPED_CTL_0_INDEX] = (uint8_t)(val & 0xFF);
			tx_frame.data[MF4005_SPPED_CTL_1_INDEX] = (uint8_t)((val >> 8) & 0xFF);
			tx_frame.data[MF4005_SPPED_CTL_2_INDEX] = (uint8_t)((val >> 16) & 0xFF);
			tx_frame.data[MF4005_SPPED_CTL_3_INDEX] = (uint8_t)((val >> 24) & 0xFF);
		break;

		case MF4005_CHAN_RAW_TORQUE_MILIAMPERES:			
			if(val > cfg->max_current_ma || val < -cfg->max_current_ma) {
				LOG_ERR("Current value %d mA is beyond the supported limit", val);
				return -EINVAL;
			}

			/*Scale current from miliamperes to the raw value */
			val = (val * MF4005_CURRENT_SCALE_MULT) / MF4005_CURRENT_HARD_LIMIT_MILIAMPERES;
			tx_frame.data[0] = MF4005_TORQUE_SET_REG;
			tx_frame.data[MF4005_TORQUE_CTL_0_INDEX] = (uint8_t)(val & 0xFF);
			tx_frame.data[MF4005_TORQUE_CTL_1_INDEX] = (uint8_t)((val >> 8) & 0xFF);
		break;

		default:
			LOG_ERR("Channel %d not supported", chan);
			return -ENOTSUP;
		break;
	}
	
	return can_send(cfg->can_bus, &tx_frame, K_FOREVER, NULL, NULL);
}

static int mf_motor_channel_get(const struct device *dev, enum mf4005_motor_channel chan,
			        int64_t *val)
{
	const struct mf4005_config *cfg = dev->config;
	struct mf4005_data *dev_data = dev->data;
	int ret;
	uint64_t raw;

	struct can_frame tx_frame = {
		.flags = 0,
		.id = cfg->id,
		.dlc = 8,
	};

	if (!val) {
		return -EINVAL;
	}

	switch(chan) {
		case MF4005_CHAN_POSITION_MILI_DEGREES:
			tx_frame.data[0] = MF4005_POSITION_GET_REG;
		break;

		case MF4005_CHAN_SPEED_DEG_SECONDS:
		case MF4005_CHAN_RAW_TORQUE_MILIAMPERES:			
			tx_frame.data[0] = MF4005_STATE_GET_REG;
		break;

		default:
			LOG_ERR("Channel %d not supported", chan);
			return -ENOTSUP;
		break;
	}
	
	ret = can_send(cfg->can_bus, &tx_frame, K_FOREVER, NULL, NULL);
	if(ret < 0) {
		LOG_ERR("Failed to fetch the MF4005 channel!");
		return ret;
	}

	ret = k_sem_take(&dev_data->can_signal, K_MSEC(500));
	if(ret < 0) {
		LOG_ERR("Fetch the MF4005 channel timed out!");
		return ret;
	}

	switch(chan) {
		case MF4005_CHAN_POSITION_MILI_DEGREES:
			raw = dev_data->rx_frame.data[MF4005_ANGLE_READ_6_INDEX];
			raw <<= 8;
			raw |= dev_data->rx_frame.data[MF4005_ANGLE_READ_5_INDEX];
			raw <<= 8;
			raw |= dev_data->rx_frame.data[MF4005_ANGLE_READ_4_INDEX];
			raw <<= 8;
			raw |= dev_data->rx_frame.data[MF4005_ANGLE_READ_3_INDEX];
			raw <<= 8;
			raw |= dev_data->rx_frame.data[MF4005_ANGLE_READ_2_INDEX];
			raw <<= 8;
			raw |= dev_data->rx_frame.data[MF4005_ANGLE_READ_1_INDEX];
			raw <<= 8;
			raw |= dev_data->rx_frame.data[MF4005_ANGLE_READ_0_INDEX];
			*val = (int64_t)(raw) / MF4005_ANGLE_SCALE_MULT;
		break;

		case MF4005_CHAN_SPEED_DEG_SECONDS:
			raw = dev_data->rx_frame.data[MF4005_SPEED_READ_1_INDEX];
			raw <<= 8;
			raw |= dev_data->rx_frame.data[MF4005_SPEED_READ_0_INDEX];
			*val = (int64_t)(raw) /MF4005_SPEED_SCALE_MULT;
		break;

		case MF4005_CHAN_RAW_TORQUE_MILIAMPERES:			
			raw = dev_data->rx_frame.data[MF4005_TORQUE_READ_1_INDEX];
			raw <<= 8;
			raw |= dev_data->rx_frame.data[MF4005_TORQUE_READ_0_INDEX];
			*val = ((int64_t)(raw) * MF4005_CURRENT_HARD_LIMIT_MILIAMPERES) /
				   MF4005_CURRENT_SCALE_MULT;
		break;

		default:
			LOG_ERR("Channel %d not supported", chan);
			return -ENOTSUP;
		break;
	}

	return 0;
}

static const struct mf4005_motor_driver_api api = {
	.motor_start = mf_motor_start,
	.motor_stop = mf_motor_stop,
	.motor_attr_set = mf_motor_attr_set,
	.motor_attr_get = mf_motor_attr_get,
	.motor_channel_set = mf_motor_channel_set,
	.motor_channel_get = mf_motor_channel_get,
};

static int mf4005_init(const struct device *dev)
{
	const struct mf4005_config *cfg = dev->config;
	struct mf4005_data *dev_data = (struct mf4005_data *)dev->data;
	int ret;

	k_sem_init(&dev_data->can_signal, 0, 1);

	if (!device_is_ready(cfg->can_bus)) {
		LOG_ERR("CAN bus device is not ready!");
		return -ENODEV;
	}

	can_stop(cfg->can_bus);

	ret = can_add_rx_filter(cfg->can_bus, can_rx_callback, dev_data, &cfg->device_filter);
	if (ret < 0) {
		LOG_ERR("Failed to set MF4005 filter in CAN");
		return ret;
	}

	ret = can_start(cfg->can_bus);
	if (ret < 0) {
		LOG_ERR("Failed to start the CAN bus!");
		return ret;
	}

	/* Check motor connection before proceed */
		struct can_frame tx_frame = {
		.flags = 0,
		.id = cfg->id,
		.dlc = 8,
		.data[0] = MF4005_PID_GET_REG,
	};
	ret = can_send(cfg->can_bus, &tx_frame, K_FOREVER, NULL, NULL);
	if(ret < 0) {
		LOG_ERR("Failed to access the MF4005 motor: (0x%x)", cfg->id);
		return ret;
	}

	ret = k_sem_take(&dev_data->can_signal, K_MSEC(500));
	if(ret < 0) {
		LOG_ERR("Motor: (0x%x) seems to be disconnected", cfg->id);
		return ret;
	}

	LOG_INF("Motor: (0x%x) connection checked, and its ready to use!", cfg->id);

#ifdef CONFIG_MF4005_ENABLE_AT_BOOT
	return mf4005_motor_start(dev);
#else
	return  mf4005_motor_stop(dev);
#endif

}

#define MF4005_INIT(n)                                                                      \
	static struct mf4005_data mf4005_data_##n;                                          \
	static const struct mf4005_config mf4005_cfg_##n = {                                \
		.can_bus = DEVICE_DT_GET(DT_INST_PARENT(n)),              					\
		.device_filter = {                                                          \
			.id = DT_REG_ADDR(DT_DRV_INST(n)),                                  \
			.mask = CAN_STD_ID_MASK                                             \
		},                                                                          \
		.id = DT_REG_ADDR(DT_DRV_INST(n)),											\
		.max_current_ma = DT_INST_PROP(n, max_current_ma),                          \
		.encoder_resolution_bits = DT_INST_PROP(n, encoder_resolution_bits),        \
		.max_speed_dps = DT_INST_PROP(n, max_speed_dps),                            \
	};                                                                                  \
	DEVICE_DT_INST_DEFINE(n, &mf4005_init, NULL, &mf4005_data_##n, &mf4005_cfg_##n,      \
			      POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(MF4005_INIT)
