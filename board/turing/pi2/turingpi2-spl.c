// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Sam Edwards <CFSworks@gmail.com>
 *
 * Early init for the Turing Pi 2 clusterboard.
 */

#include <common.h>
#include <asm/gpio.h>
#include <init.h>
#include <sunxi_gpio.h>

static void turingpi2_ethsw_rst(void)
{
	/*
	 * RTL8370MB switch reset (active-low) on PG13 should be asserted very
	 * early in boot to prevent Ethernet from coming up until the switch
	 * can be configured. The user may be using the Turing Pi 2 as a router
	 * or some other kind of network isolation device.
	 */
	gpio_direction_output(SUNXI_GPG(13), 0);
}

int board_early_init_f(void)
{
	turingpi2_ethsw_rst();

	return 0;
}

#if CONFIG_IS_ENABLED(OF_CONTROL)
/// This method determines which device tree to load. The main
/// difference between 2.4 and 2.5 concerning the bootloader is the difference
/// in flash size. If we load the wrong dt, it can mean booting will fail as
/// the ubi mount cannot be mounted properly. ( due to not all pages being
/// visible of the flash)
int board_fit_config_name_match(const char *name) {
#if CONFIG_IS_ENABLED(BLOBLIST)
  int init_res = bloblist_maybe_init();
  if (init_res != 0) {
    debug("Error initializing bloblist: %d\n", init_res);
    return -ENODATA;
  }

  // Get the blob data
  tpi_board_info *info =
      (tpi_board_info *)bloblist_find(BLOBLISTT_U_BOOT_SPL_HANDOFF, 0);

  if (!info) {
    debug("Error: spl handoff blob not found\n");
    return -ENODATA;
  }

  int v2_4_match = info->hw_version == TP_VER(2, 4, 0) && strstr(name, "-v2.4");
  int v2_5_match = info->hw_version >= TP_VER(2, 5, 0) && strstr(name, "-v2.5");
  if (v2_4_match || v2_5_match) {
    return 0;
  }
  return -EINVAL;
#endif
}
#endif
