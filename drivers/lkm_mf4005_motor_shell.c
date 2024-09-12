#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/lkm_mf4005_motor.h>
#include <stdlib.h>

#define MOTOR_START_HELP                                               \
	"Turn on the power stage of the motor required to perform motion"  \
	"Syntax:\n"                                                        \
	"<device_name>"

#define MOTOR_STOP_HELP                                                \
	"Turns off the power stage of the motor and stops all motion "     \
	"Syntax:\n"                                                        \
	"<device_name>"

#define MOTOR_SET_ATTR_HELP                                            \
	"Set one of the motor attribute. Syntax:\n"                        \
	"<device_name> <attribute_name> <value> "                           \

#define MOTOR_SET_CHANNEL_HELP                                         \
	"Set the motors's channel value. Syntax:\n"                        \
	"<device_name> <channel_name> <value>"

#define MOTOR_GET_CHANNEL_HELP                                         \
	"Prints motor channel value. Syntax:\n"                            \
	"<device_name> <channel_name> "

static const char *motor_channel_name[MF4005_CHAN_COUNT] = {
	[MF4005_CHAN_POSITION_MILI_DEGREES] = "position_milideg",
	[MF4005_CHAN_SPEED_DEG_SECONDS] = "speed_deg_seconds",
	[MF4005_CHAN_RAW_TORQUE_MILIAMPERES] = "torque_miliamperes",
};

static const char *motor_attribute_name[MF4005_ATTR_COUNT] = {
	[MF4005_ATTR_POSITION_KP] = "pid_position_kp",
	[MF4005_ATTR_POSITION_KI] = "pid_position_ki",
	[MF4005_ATTR_SPEED_KP] = "pid_speed_kp",
	[MF4005_ATTR_SPEED_KI] = "pid_speed_ki",
	[MF4005_ATTR_TORQUE_KP] = "pid_torque_kp",
	[MF4005_ATTR_TORQUE_KI] = "pid_torque_ki",
};

/* Mutex to control access between 2 or more commands*/
K_MUTEX_DEFINE(cmd_mutex);

static int parse_named_int(const char *name, const char *heystack[], size_t count)
{
	char *endptr;
	int i;

	/* Attempt to parse channel name as a number first */
	i = strtoul(name, &endptr, 0);

	if (*endptr == '\0') {
		return i;
	}

	/* Channel name is not a number, look it up */
	for (i = 0; i < count; i++) {
		if (strcmp(name, heystack[i]) == 0) {
			return i;
		}
	}

	return -ENOTSUP;
}


static void channel_name_get(size_t idx, struct shell_static_entry *entry);
SHELL_DYNAMIC_CMD_CREATE(dsub_channel_name, channel_name_get);

static void attribute_name_get(size_t idx, struct shell_static_entry *entry);
SHELL_DYNAMIC_CMD_CREATE(dsub_attribute_name, attribute_name_get);

static void channel_name_get(size_t idx, struct shell_static_entry *entry)
{
	int cnt = 0;

	entry->syntax = NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;

	for (int i = 0; i < ARRAY_SIZE(motor_channel_name); i++) {
		if (motor_channel_name[i] != NULL) {
			if (cnt == idx) {
				entry->syntax = motor_channel_name[i];
				break;
			}
			cnt++;
		}
	}
}

static void attribute_name_get(size_t idx, struct shell_static_entry *entry)
{
	int cnt = 0;

	entry->syntax = NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;

	for (int i = 0; i < ARRAY_SIZE(motor_attribute_name); i++) {
		if (motor_attribute_name[i] != NULL) {
			if (cnt == idx) {
				entry->syntax = motor_attribute_name[i];
				break;
			}
			cnt++;
		}
	}
}

static void device_name_get(size_t idx, struct shell_static_entry *entry);

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}


static void device_name_get_for_chan(size_t idx, struct shell_static_entry *entry);

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name_for_chan, device_name_get_for_chan);

static void device_name_get_for_chan(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = &dsub_channel_name;
}

static void device_name_get_for_attr(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = &dsub_attribute_name;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name_for_attr, device_name_get_for_attr);


static int cmd_motor_start(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;

	if(argc != 2) {
		return -EINVAL;
	}

	ret = k_mutex_lock(&cmd_mutex, K_NO_WAIT);
	if (ret < 0) {
		shell_error(shell, "Another sensor reading in progress");
		return ret;
	}

	dev = device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(shell, "Device unknown (%s)", argv[1]);
		k_mutex_unlock(&cmd_mutex);
		return -ENODEV;
	}

	ret = mf4005_motor_start(dev);
	k_mutex_unlock(&cmd_mutex);

	return ret;
}

static int cmd_motor_stop(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;

	if(argc != 2) {
		return -EINVAL;
	}

	ret = k_mutex_lock(&cmd_mutex, K_NO_WAIT);
	if (ret < 0) {
		shell_error(shell, "Another sensor reading in progress");
		return ret;
	}

	dev = device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(shell, "Device unknown (%s)", argv[1]);
		k_mutex_unlock(&cmd_mutex);
		return -ENODEV;
	}

	ret = mf4005_motor_stop(dev);
	k_mutex_unlock(&cmd_mutex);

	return ret;
}

static int cmd_motor_attr_set(const struct shell *shell, size_t argc, char **argv)
{	const struct device *dev;
	int ret;

	if(argc != 4) {
		return -EINVAL;
	}

	ret = k_mutex_lock(&cmd_mutex, K_NO_WAIT);
	if (ret < 0) {
		shell_error(shell, "Another sensor reading in progress");
		return ret;
	}

	dev = device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(shell, "Device unknown (%s)", argv[1]);
		k_mutex_unlock(&cmd_mutex);
		return -ENODEV;
	}

	int attr = parse_named_int(argv[2], motor_attribute_name, MF4005_ATTR_COUNT);
	if(attr < 0) {
		shell_error(shell, "Attribute unknown (%s)", argv[2]);
		k_mutex_unlock(&cmd_mutex);
		return attr;
	}


	ret = mf4005_motor_attr_set(dev, attr, strtoul(argv[3], NULL, 0));
	k_mutex_unlock(&cmd_mutex);

	return ret;

}

static int cmd_motor_channel_set(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;

	if(argc != 4) {
		return -EINVAL;
	}

	ret = k_mutex_lock(&cmd_mutex, K_NO_WAIT);
	if (ret < 0) {
		shell_error(shell, "Another sensor reading in progress");
		return ret;
	}

	dev = device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(shell, "Device unknown (%s)", argv[1]);
		k_mutex_unlock(&cmd_mutex);
		return -ENODEV;
	}

	int chan = parse_named_int(argv[2], motor_channel_name, MF4005_CHAN_COUNT);
	if(chan < 0) {
		shell_error(shell, "Channel unknown (%s)", argv[2]);
		k_mutex_unlock(&cmd_mutex);
		return chan;
	}


	ret = mf4005_motor_channel_set(dev, chan, strtoul(argv[3], NULL, 0));
	k_mutex_unlock(&cmd_mutex);

	return ret;
}

static int cmd_motor_channel_get(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;
	int64_t chan_value;

	if(argc != 3) {
		return -EINVAL;
	}

	ret = k_mutex_lock(&cmd_mutex, K_NO_WAIT);
	if (ret < 0) {
		shell_error(shell, "Another sensor reading in progress");
		return ret;
	}

	dev = device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(shell, "Device unknown (%s)", argv[1]);
		k_mutex_unlock(&cmd_mutex);
		return -ENODEV;
	}

	int chan = parse_named_int(argv[2], motor_channel_name, MF4005_CHAN_COUNT);
	if(chan < 0) {
		shell_error(shell, "Channel unknown (%s)", argv[2]);
		k_mutex_unlock(&cmd_mutex);
		return chan;
	}

	ret = mf4005_motor_channel_get(dev, chan, &chan_value);
	if(ret < 0) {
		shell_error(shell, "Failed to read the channel (%s)", argv[2]);
		k_mutex_unlock(&cmd_mutex);
		return ret;
	}

	shell_print(shell, "Channel (%s) value: (%lld)", argv[2], chan_value);

	return ret;
}

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(
	lkm_mf4005,
	SHELL_CMD_ARG(motor_start, &dsub_device_name, MOTOR_START_HELP, cmd_motor_start, 2, 0),
	SHELL_CMD_ARG(motor_stop, &dsub_device_name, MOTOR_STOP_HELP, cmd_motor_stop, 2, 0),
	SHELL_CMD_ARG(motor_attr_set, &dsub_device_name_for_attr, MOTOR_SET_ATTR_HELP, cmd_motor_attr_set, 4, 0),
	SHELL_CMD_ARG(motor_channel_set, &dsub_device_name_for_chan, MOTOR_SET_CHANNEL_HELP, cmd_motor_channel_set, 4, 0),
	SHELL_CMD_ARG(motor_channel_get, &dsub_device_name_for_chan, MOTOR_GET_CHANNEL_HELP, cmd_motor_channel_get, 3, 0),
	SHELL_SUBCMD_SET_END
	);
/* clang-format on */

SHELL_CMD_REGISTER(lkm_mf4005, &lkm_mf4005, "LK MF4005 Brushless motor commands", NULL);
