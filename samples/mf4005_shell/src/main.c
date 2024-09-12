/*
 * Copyright (c) 2024 Felipe Neves
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>

int main(void)
{
	printf("Running MF4005 Shell sample at %s\n", CONFIG_BOARD_TARGET);
	return 0;
}
