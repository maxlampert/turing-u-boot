// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Sam Edwards <CFSworks@gmail.com>
 * Copyright (C) 2024 Sven Rademakers <sven@turingpi.com>
 *
 * Early init for the Turing Pi 2 clusterboard.
 */

#include <asm/gpio.h>
#include <asm/io.h>
#include <bloblist.h>
#include <init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <stdio.h>
#include <sunxi_gpio.h>
#include <u-boot/crc.h>

#include "board_info.h"

#define TURING_PI2_LATCH_STATE_ADDR 0x0709010c
#define TURING_PI2_BOOT_COOKIE_ADDR 0x07090108
#define TURING_PI2_BOOT_COOKIE_WARM 0x32695054 /* "TPi2" ASCII */
#define TURING_PI2_BOOT_COOKIE_FEL 0x5aa5a55a
#define LOG_DEBUG
#define DEBUG
/*
 * RTL8370MB switch reset (active-low) on PG13 should be asserted very
 * early in boot to prevent Ethernet from coming up until the switch
 * can be configured. The user may be using the Turing Pi 2 as a router
 * or some other kind of network isolation device.
 */
static void turingpi2_ethsw_rst(u16 tpi_version) {
  if (tpi_version < TP_VER(2, 5, 0)) {
    gpio_direction_output(SUNXI_GPG(13), 0);
  } else {
    gpio_direction_output(SUNXI_GPG(3), 0);
  }
}

static void init_latches(u16 tpi_version) {
  if (tpi_version >= TP_VER(2, 5, 0)) {
    gpio_direction_output(SUNXI_GPD(7), 0);
    gpio_direction_output(SUNXI_GPD(6), 0);
    gpio_direction_output(SUNXI_GPD(5), 0);
    gpio_direction_output(SUNXI_GPD(3), 0);
    gpio_direction_output(SUNXI_GPD(4), 0);
    gpio_direction_output(SUNXI_GPD(8), 0);
    gpio_direction_output(SUNXI_GPD(9), 0);
    gpio_direction_output(SUNXI_GPD(10), 0);
    gpio_direction_output(SUNXI_GPD(11), 0);
    gpio_direction_output(SUNXI_GPD(20), 1);
    udelay(50);

    // reset state of latch_state_addr
    u16 latch_state = readl(TURING_PI2_LATCH_STATE_ADDR);
    writel(latch_state & 0xFFFFFE00, TURING_PI2_LATCH_STATE_ADDR);
  }
}

/*
 * Direct TWI2 register access for SPL EEPROM reading
 * Bypasses both DM and legacy I2C frameworks
 */
#define TWI2_BASE		0x02502800

/* TWI register offsets (Allwinner layout) */
#define TWI_ADDR		0x00
#define TWI_XADDR		0x04
#define TWI_DATA		0x08
#define TWI_CTRL		0x0C
#define TWI_STAT		0x10
#define TWI_CLK			0x14
#define TWI_SRST		0x18

/* Control register bits */
#define TWI_CTRL_INT_EN		(1 << 7)
#define TWI_CTRL_BUS_EN		(1 << 6)
#define TWI_CTRL_START		(1 << 5)
#define TWI_CTRL_STOP		(1 << 4)
#define TWI_CTRL_INT_FLAG	(1 << 3)
#define TWI_CTRL_ACK		(1 << 2)

/* Status values */
#define TWI_STAT_START		0x08
#define TWI_STAT_RSTART		0x10
#define TWI_STAT_ADDR_W_ACK	0x18
#define TWI_STAT_DATA_W_ACK	0x28
#define TWI_STAT_ADDR_R_ACK	0x40
#define TWI_STAT_DATA_R_ACK	0x50
#define TWI_STAT_DATA_R_NAK	0x58
#define TWI_STAT_IDLE		0xF8

static void twi2_init(void)
{
	/* Reset TWI2 */
	writel(1, TWI2_BASE + TWI_SRST);
	udelay(100);

	/* Set clock: ~100kHz assuming 24MHz APB clock
	 * CLK = (CLK_M << 3) | CLK_N, freq = APB / (10 * (2^CLK_N) * (CLK_M + 1))
	 * M=5, N=1 => 24MHz / (10 * 2 * 6) = 200kHz
	 */
	writel((5 << 3) | 1, TWI2_BASE + TWI_CLK);

	/* Enable bus */
	writel(TWI_CTRL_BUS_EN, TWI2_BASE + TWI_CTRL);
}

static int twi2_wait_flag(void)
{
	int timeout = 100000;
	while (!(readl(TWI2_BASE + TWI_CTRL) & TWI_CTRL_INT_FLAG)) {
		if (--timeout <= 0)
			return -ETIMEDOUT;
		udelay(1);
	}
	return 0;
}

static int twi2_start(void)
{
	writel(TWI_CTRL_BUS_EN | TWI_CTRL_START | TWI_CTRL_INT_FLAG,
	       TWI2_BASE + TWI_CTRL);
	return twi2_wait_flag();
}

static void twi2_stop(void)
{
	writel(TWI_CTRL_BUS_EN | TWI_CTRL_STOP | TWI_CTRL_INT_FLAG,
	       TWI2_BASE + TWI_CTRL);
	udelay(100);
}

static int twi2_send_byte(u8 byte, u8 expected_status)
{
	writel(byte, TWI2_BASE + TWI_DATA);
	writel(TWI_CTRL_BUS_EN | TWI_CTRL_INT_FLAG, TWI2_BASE + TWI_CTRL);

	if (twi2_wait_flag())
		return -ETIMEDOUT;

	if ((readl(TWI2_BASE + TWI_STAT) & 0xFF) != expected_status)
		return -EIO;

	return 0;
}

static int twi2_recv_byte(u8 *byte, int last)
{
	u32 ctrl = TWI_CTRL_BUS_EN | TWI_CTRL_INT_FLAG;
	if (!last)
		ctrl |= TWI_CTRL_ACK;
	writel(ctrl, TWI2_BASE + TWI_CTRL);

	if (twi2_wait_flag())
		return -ETIMEDOUT;

	*byte = readl(TWI2_BASE + TWI_DATA) & 0xFF;
	return 0;
}

static int board_info_from_eeprom(tpi_board_info *info) {
	u8 *buf = (u8 *)info;
	int i, ret;

	twi2_init();

	/* Send START */
	ret = twi2_start();
	if (ret) {
		printf("TWI2: start failed\n");
		goto out;
	}

	/* Send device address (write mode) to set EEPROM offset */
	ret = twi2_send_byte(0x50 << 1, TWI_STAT_ADDR_W_ACK);
	if (ret) {
		printf("TWI2: addr write failed\n");
		goto out;
	}

	/* Send EEPROM offset = 0 */
	ret = twi2_send_byte(0x00, TWI_STAT_DATA_W_ACK);
	if (ret) {
		printf("TWI2: offset failed\n");
		goto out;
	}

	/* Repeated START for read */
	ret = twi2_start();
	if (ret) {
		printf("TWI2: restart failed\n");
		goto out;
	}

	/* Send device address (read mode) */
	ret = twi2_send_byte((0x50 << 1) | 1, TWI_STAT_ADDR_R_ACK);
	if (ret) {
		printf("TWI2: addr read failed\n");
		goto out;
	}

	/* Read data bytes */
	for (i = 0; i < sizeof(tpi_board_info); i++) {
		int last = (i == sizeof(tpi_board_info) - 1);
		ret = twi2_recv_byte(&buf[i], last);
		if (ret) {
			printf("TWI2: read byte %d failed\n", i);
			goto out;
		}
	}

	ret = 0;

out:
	twi2_stop();
	return ret;
}

u32 compute_crc(tpi_board_info *info) {
  int info_offset = offsetof(tpi_board_info, hdr_version);
  u32 crc = crc32(0, (void *)info + info_offset,
                  sizeof(tpi_board_info) - info_offset);
  return ((crc & 0x000000FF) << 24) | ((crc & 0x0000FF00) << 8) |
         ((crc & 0x00FF0000) >> 8) | ((crc & 0xFF000000) >> 24);
}

#if CONFIG_IS_ENABLED(BLOBLIST)
tpi_board_info *setup_bloblist(void) {
  int init_res = bloblist_init();
  if (init_res) {
    printf("bloblist init err 0x%x\n", init_res);
    return NULL;
  }

  void *board_tag =
      bloblist_add(BLOBLISTT_U_BOOT_SPL_HANDOFF, sizeof(tpi_board_info), 0);
  if (!board_tag) {
    printf("no space for board_info in bloblist\n");
  }
  return board_tag;
}
#endif

int tp_board_init(void) {
  u32 cookie = readl(TURING_PI2_BOOT_COOKIE_ADDR);
  if (cookie == TURING_PI2_BOOT_COOKIE_FEL) {
    writel(TURING_PI2_BOOT_COOKIE_WARM, TURING_PI2_BOOT_COOKIE_ADDR);
    // signal FEL
    return 1;
  }

  tpi_board_info *info = NULL;

#if CONFIG_IS_ENABLED(BLOBLIST)
  info = setup_bloblist();
#else
  tpi_board_info stack;
  info = &stack;
#endif

  u16 version = TP_VER(2, 4, 0);
  int result = -ENOENT;
  if (info)
    result = board_info_from_eeprom(info);

  u32 crc = compute_crc(info);
  if (result || crc != info->crc32) {
    printf("Error(%x): invalid board info, defaulting to version 0x%x. crc=%x "
           "expected=%x ver=%x\n",
           result, version, info->crc32, crc, info->hw_version);
    info->hw_version = version;
  } else {
    version = info->hw_version;
  }

  if (cookie != TURING_PI2_BOOT_COOKIE_WARM) {
    turingpi2_ethsw_rst(version);
    init_latches(version);
    writel(TURING_PI2_BOOT_COOKIE_WARM, TURING_PI2_BOOT_COOKIE_ADDR);
  }

#if CONFIG_IS_ENABLED(BLOBLIST)
  bloblist_finish();
#endif
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
