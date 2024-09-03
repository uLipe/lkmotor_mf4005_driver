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

SHELL_STATIC_SUBCMD_SET_CREATE(
	lkm_mf4005,
	SHELL_CMD(motor_start, NULL, MOTOR_START_HELP, cmd_motor_start),
	SHELL_CMD(motor_stop, NULL, MOTOR_STOP_HELP, cmd_motor_stop),
	SHELL_CMD(motor_attr_set, NULL, MOTOR_SET_ATTR_HELP, cmd_motor_attr_set),
	SHELL_CMD(motor_channel_set, NULL, MOTOR_SET_CHANNEL_HELP, cmd_motor_channel_set),
	SHELL_CMD(motor_channel_get, NULL, MOTOR_GET_CHANNEL_HELP, cmd_motor_channel_get),
	SHELL_SUBCMD_SET_END
	);

SHELL_CMD_REGISTER(lkm_mf4005, &lkm_mf4005, "LK MF4005 Brushless motor commands", NULL);
