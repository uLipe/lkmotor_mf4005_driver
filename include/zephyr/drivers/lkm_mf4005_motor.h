/*
 * Copyright (c) 2024 Felipe Neves
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef INCLUDE_LKM_MF4005_DRIVER_H_
#define INCLUDE_LKM_MF4005_DRIVER_H_

#include <errno.h>
#include <stdlib.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Motor channels.
 */
enum mf4005_motor_channel {
	MF4005_CHAN_POSITION_MILI_DEGREES = 0,
	MF4005_CHAN_SPEED_DEG_SECONDS,
	MF4005_CHAN_RAW_TORQUE_MILIAMPERES,
	MF4005_CHAN_COUNT,
	MF4005_CHAN_MAX = INT16_MAX,
};

/**
 * @brief Motor attribute types.
 */
enum mf4005_motor_attribute {
	MF4005_ATTR_POSITION_KP = 0,
	MF4005_ATTR_POSITION_KI,
	MF4005_ATTR_SPEED_KP,
	MF4005_ATTR_SPEED_KI,
	MF4005_ATTR_TORQUE_KP,
	MF4005_ATTR_TORQUE_KI,
	MF4005_ATTR_COUNT,
	MF4005_ATTR_MAX = INT16_MAX,
};

/**
 * @brief MF4005 motor driver generic API definition
 */

typedef int (*mf4005_motor_start_t)(const struct device *dev);

typedef int (*mf4005_motor_stop_t)(const struct device *dev);

typedef int (*mf4005_motor_attr_set_t)(const struct device *dev,
				       enum mf4005_motor_attribute attr,
				       int attr_val);

typedef int (*mf4005_motor_attr_get_t)(const struct device *dev,
				       enum mf4005_motor_attribute attr,
				       int *attr_val);

typedef int (*mf4005_motor_channel_set_t)(const struct device *dev,
				          enum mf4005_motor_channel chan,
					  int val);

typedef int (*mf4005_motor_channel_get_t)(const struct device *dev,
					  enum mf4005_motor_channel chan,
					  int64_t *val);

struct mf4005_motor_driver_api {
	mf4005_motor_start_t motor_start;
	mf4005_motor_stop_t motor_stop;
	mf4005_motor_attr_set_t motor_attr_set;
	mf4005_motor_attr_get_t motor_attr_get;
	mf4005_motor_channel_set_t motor_channel_set;
	mf4005_motor_channel_get_t motor_channel_get;
};

/**
 * @brief Enable power of the motor 
 *
 * @param dev Pointer to the MF40005 motor device
 *
 * @return 0 if successful, negative errno code if failure.
 */
static inline int mf4005_motor_start(const struct device *dev)
{
	const struct mf4005_motor_driver_api *api =
		(const struct mf4005_motor_driver_api *)dev->api;

	if (api->motor_start == NULL) {
		return -ENOSYS;
	}

	return api->motor_start(dev);
}

/**
 * @brief Disable the motor power 
 *
 * @param dev Pointer to the MF40005 motor device
 *
 * @return 0 if successful, negative errno code if failure.
 */
static inline int mf4005_motor_stop(const struct device *dev)
{
	const struct mf4005_motor_driver_api *api =
		(const struct mf4005_motor_driver_api *)dev->api;

	if (api->motor_stop == NULL) {
		return -ENOSYS;
	}

	return api->motor_stop(dev);
}

/**
 * @brief Set an attribute for a motor 
 *
 * @param dev Pointer to the MF40005 motor device
 * @param attr The attribute to set
 * @param val The value to set the attribute to
 *
 * @return 0 if successful, negative errno code if failure.
 */
static inline int mf4005_motor_attr_set(const struct device *dev, enum mf4005_motor_attribute attr,
			  		int attr_val)
{
	const struct mf4005_motor_driver_api *api =
		(const struct mf4005_motor_driver_api *)dev->api;

	if (api->motor_attr_set == NULL) {
		return -ENOSYS;
	}

	return api->motor_attr_set(dev, attr, attr_val);
}

/**
 * @brief Get an attribute for a motor 
 *
 * @param dev Pointer to the MF40005 motor device
 * @param attr The attribute to get
 * @param val pointer to the value to store the attribute get
 *
 * @return 0 if successful, negative errno code if failure.
 */
static inline int mf4005_motor_attr_get(const struct device *dev, enum mf4005_motor_attribute attr,
					int *attr_val)
{
	const struct mf4005_motor_driver_api *api =
		(const struct mf4005_motor_driver_api *)dev->api;

	if (api->motor_attr_get == NULL) {
		return -ENOSYS;
	}

	return api->motor_attr_get(dev, attr, attr_val);
}

/**
 * @brief Set an channel for a motor 
 *
 * @param dev Pointer to the MF40005 motor device
 * @param chan The channel to set
 * @param val The value to set the channel to
 *
 * @return 0 if successful, negative errno code if failure.
 */
static inline int mf4005_motor_channel_set(const struct device *dev,
					   enum mf4005_motor_channel chan, int val)
{
	const struct mf4005_motor_driver_api *api =
		(const struct mf4005_motor_driver_api *)dev->api;

	if (api->motor_channel_set == NULL) {
		return -ENOSYS;
	}

	return api->motor_channel_set(dev, chan, val);
}

/**
 * @brief Get a channel for a motor 
 *
 * @param dev Pointer to the MF40005 motor device
 * @param chan The channel to get
 * @param val The pointer where to store value from the channel
 *
 * @return 0 if successful, negative errno code if failure.
 */
static inline int mf4005_motor_channel_get(const struct device *dev,
					   enum mf4005_motor_channel chan, int64_t *val)
{
	const struct mf4005_motor_driver_api *api =
		(const struct mf4005_motor_driver_api *)dev->api;

	if (api->motor_channel_get == NULL) {
		return -ENOSYS;
	}

	return api->motor_channel_get(dev, chan, val);
}

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_LKM_MF4005_DRIVER_H_ */
