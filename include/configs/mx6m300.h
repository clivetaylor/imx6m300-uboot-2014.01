/*
 * Copyright (C) 2015 Megger Instruments, Ltd
 *
 * Configuration settings for the Mithras M300 board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6M300_CONFIG_H
#define __MX6M300_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_CONSOLE_DEV	"ttymxc0"
#define CONFIG_MMCROOT		"/dev/mmcblk0p1"
/*
#if defined(CONFIG_MX6Q)
#define CONFIG_DEFAULT_FDT_FILE	"imx6q-sabresd-ldo.dtb"
#elif defined(CONFIG_MX6DL)
#define CONFIG_DEFAULT_FDT_FILE	"imx6dl-sabresd-ldo.dtb"
#endif
*/
//
//	M300 has 2GB DDR3
//
#define PHYS_SDRAM_SIZE		(2u * 1024 * 1024 * 1024)

#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

#include "mx6m300_common.h"
//
//	2 SDHC Devices (SDHC2 and SDHC4), default boot from SDHC2
//
#define CONFIG_SYS_FSL_USDHC_NUM	2
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC2 */
#endif

/* Framebuffer */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
//
//	Want serial as default console
//
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_IMX_HDMI

#endif                         /* __MX6M300_CONFIG_H */
