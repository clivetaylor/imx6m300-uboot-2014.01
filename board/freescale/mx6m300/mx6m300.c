/*
 * Copyright (C) 2015, Megger Instruments, Ltd
 *
 * Author: Clive Taylor <clive.taylor@megger.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

static void init_status_leds ( void );
static void show_status_leds ( int stat );
//int  board_video_skip ( int Default );
static void setup_display ( int DoLVDS );

int dram_init(void)
{
	//gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);
	gd->ram_size = (phys_size_t)CONFIG_DDR_MB * 1024 * 1024;

	return 0;
}

#define TEST_IO_PIN1	IMX_GPIO_NR(1,2)
#define TEST_IO_PIN2	IMX_GPIO_NR(1,3)
#define TEST_IO_PIN3	IMX_GPIO_NR(1,16)
#define TEST_IO_PIN4	IMX_GPIO_NR(1,17)
#define TEST_IO_PIN5	IMX_GPIO_NR(1,18)
#define TEST_IO_PIN6	IMX_GPIO_NR(1,19)
#define TEST_IO_PIN7	IMX_GPIO_NR(2,23)

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT6__UART1_RX_DATA   | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT7__UART1_TX_DATA   | MUX_PAD_CTRL(UART_PAD_CTRL),
	//MX6_PAD_SD3_DAT0__UART1_CTS_B     | MUX_PAD_CTRL(UART_PAD_CTRL),
	//MX6_PAD_SD3_DAT1__UART1_RTS_B	  | MUX_PAD_CTRL(UART_PAD_CTRL),
	//MX6_PAD_GPIO_2__GPIO1_IO02	  | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_CSI0_DAT12__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT13__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	/*
	MX6_PAD_CSIO_DAT16__UART4_CTS_B   | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSIO_DAT17__UART4_RTS_B	  | MUX_PAD_CTRL(UART_PAD_CTRL),
	*/
};

iomux_v3_cfg_t const uart5_pads[] = {
	MX6_PAD_CSI0_DAT14__UART5_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT15__UART5_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	/*
	MX6_PAD_CSIO_DAT18__UART5_CTS_B   | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSIO_DAT19__UART5_RTS_B	  | MUX_PAD_CTRL(UART_PAD_CTRL),
	*/
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_TXD0__ENET_TX_DATA0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD1__ENET_TX_DATA1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TX_EN__ENET_TX_EN   | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD0__ENET_RX_DATA0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD1__ENET_RX_DATA1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_CRS_DV__ENET_RX_EN  | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDIO__ENET_MDIO     | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RX_ER__ENET_RX_ER   | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO_16__ENET_REF_CLK    | MUX_PAD_CTRL(ENET_PAD_CTRL),
	
#if 0
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* AR8031 PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
#endif	// 0
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
#if 0
	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
#endif // 0
}

iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/*
	MX6_PAD_NANDF_D4__SD2_DATA4	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D5__SD2_DATA5	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D6__SD2_DATA6	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D7__SD2_DATA7	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	*/
	MX6_PAD_GPIO_4__GPIO1_IO04	| MUX_PAD_CTRL(NO_PAD_CTRL), /* Card Detect */
};

iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_KEY_COL0__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_COL1__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_ROW0__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_ROW1__GPIO4_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

//
//	init_status_leds
//
//	Set the GPIO direction for the status LEDs and set the multiplexor 
//	state for LED D7 (on GPIO2_IO23). The pins for GPIO1_IOxx are all
//	correctly multiplexed by default.
//
//	All GPIOs used here are initialised to 1 as the LEDs are active low
//
static void
init_status_leds ( )
{
	//
	//	Set IOMUX for GPIO1 Pins
	//
	imx_iomux_v3_setup_pad ( MX6_PAD_GPIO_2__GPIO1_IO02 );
	imx_iomux_v3_setup_pad ( MX6_PAD_GPIO_3__GPIO1_IO03 );
	imx_iomux_v3_setup_pad ( MX6_PAD_SD1_DAT0__GPIO1_IO16 );
	imx_iomux_v3_setup_pad ( MX6_PAD_SD1_DAT1__GPIO1_IO17 );
	imx_iomux_v3_setup_pad ( MX6_PAD_SD1_CMD__GPIO1_IO18 );
	imx_iomux_v3_setup_pad ( MX6_PAD_SD1_DAT2__GPIO1_IO19 );
	//
	//	Set IOMUX for EIM_CS0_B to GPIO2_IO23
	//
	imx_iomux_v3_setup_pad ( MX6_PAD_EIM_CS0__GPIO2_IO23 );
	//
	//	Set direction of GPIO2_IO23 to output, and set it high (LED off)
	//
	gpio_direction_output ( TEST_IO_PIN7, 1 );
	//
	//	Set direction of GPIO1_IO02, _IO03, _IO16, _IO17, _IO18, _IO19 to
	//	outputs, and set all high (LEDs off)
	//
	gpio_direction_output ( TEST_IO_PIN1, 1 );
	gpio_direction_output ( TEST_IO_PIN2, 1 );
	gpio_direction_output ( TEST_IO_PIN3, 1 );
	gpio_direction_output ( TEST_IO_PIN4, 1 );
	gpio_direction_output ( TEST_IO_PIN5, 1 );
	gpio_direction_output ( TEST_IO_PIN6, 1 );
}

//
//	show_status_leds
//
//	Display the current step as a 7 bit binary code on the status LEDs
//
//	Not very efficient code, but only there for debugging of u-boot
//
//	Parameter:
//		stat	Value to be represented (up to 127)
//
static void
show_status_leds ( int stat )
{
	//
	//	Set/clear LED (D13)
	//
	if ( stat && 1 )
		gpio_set_value ( TEST_IO_PIN1, 0 );
	else
		gpio_set_value ( TEST_IO_PIN1, 1 );
	//
	//	Set/clear LED (D12)
	//
	if ( stat && 2 )
		gpio_set_value ( TEST_IO_PIN2, 0 );
	else
		gpio_set_value ( TEST_IO_PIN2, 1 );
	//
	//	Set/clear LED (D11)
	//
	if ( stat && 4 )
		gpio_set_value ( TEST_IO_PIN3, 0 );
	else
		gpio_set_value ( TEST_IO_PIN3, 1 );
	//
	//	Set/clear LED (D10)
	//
	if ( stat && 8 )
		gpio_set_value ( TEST_IO_PIN4, 0 );
	else
		gpio_set_value ( TEST_IO_PIN4, 1 );
	//
	//	Set/clear LED (D9)
	//
	if ( stat && 0x10 )
		gpio_set_value ( TEST_IO_PIN5, 0 );
	else
		gpio_set_value ( TEST_IO_PIN5, 1 );
	//
	//	Set/clear LED (D8)
	//
	if ( stat && 0x20 )
		gpio_set_value ( TEST_IO_PIN6, 0 );
	else
		gpio_set_value ( TEST_IO_PIN6, 1 );
	//
	//	Set/clear LED (D7)
	//
	if ( stat && 0x40 )
		gpio_set_value ( TEST_IO_PIN7, 0 );
	else
		gpio_set_value ( TEST_IO_PIN7, 1 );

}

static void setup_spi(void)
{
	/*
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
	*/
}

iomux_v3_cfg_t const di0_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,	/* DISP0_CLK */
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,		/* DISP0_HSYNC */
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,		/* DISP0_VSYNC */
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	//__raw_writel( 2, 0x20E08FC );

	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
	imx_iomux_v3_setup_multiple_pads(uart5_pads, ARRAY_SIZE(uart5_pads));

}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = { {USDHC2_BASE_ADDR},
				      {USDHC4_BASE_ADDR} };

#define USDHC2_CD_GPIO	IMX_GPIO_NR(1, 4)
#define USDHC4_CD_GPIO	IMX_GPIO_NR(6, 8)

int 
board_mmc_getcd ( struct mmc *mmc )
{
	struct fsl_esdhc_cfg 	*cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int 			ret  = 0;

	switch ( cfg->esdhc_base ) {
		case USDHC2_BASE_ADDR :	ret = 1; // uSDHC2 is always present on Mithras M300 
					/*
					ret = !gpio_get_value(USDHC2_CD_GPIO);
					*/
					break;
		case USDHC4_BASE_ADDR :	ret = !gpio_get_value(USDHC4_CD_GPIO);
					/* uSDHC4 can be present/not present on Mithras M300 */
					break;
		}

	return ret;
}

int 
board_mmc_init ( bd_t *bis )
{
	s32 status = 0;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD2 (on module)
	 * mmc1                    SD4 (on carrier/instrument)
	 */
	for ( i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++ ) {
		switch (i) {
			case 0:	imx_iomux_v3_setup_multiple_pads ( usdhc2_pads, 
								   ARRAY_SIZE(usdhc2_pads) );
				gpio_direction_input(USDHC2_CD_GPIO);
				usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
				usdhc_cfg[0].max_bus_width = 4;
				break;
			case 1:	imx_iomux_v3_setup_multiple_pads ( usdhc4_pads, 
								   ARRAY_SIZE(usdhc4_pads) );
				gpio_direction_input(USDHC4_CD_GPIO);
				usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
				usdhc_cfg[1].max_bus_width = 8;
				break;
			default:printf ( "Warning: you configured more USDHC controllers"
			       		 "(%d) than supported by the board (%d)\n",
			       		 i + 1,
					 CONFIG_SYS_FSL_USDHC_NUM );
				return status;
			}
		status |= fsl_esdhc_initialize ( bis, &usdhc_cfg[i] );
		}

	return status;
}
#endif

int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

	printf ( "Initialising FEC and PHY. Phy is :%s/n", phydev->drv->name );
#if 0
	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);
#endif // 0
	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};

static int detect_hdmi(struct display_info_t const *dev)
{
	struct hdmi_regs *hdmi	= (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}


static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

static void
enable_rgb ( struct display_info_t const *dev )
{
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT |
	       IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT;
	writel(reg, &iomux->gpr[2]);
}

//
//	setup_iomux_lcd
//
//	Set the IO parallel RGB pin setup in the IOMUX controller
//
//	Note that production versions will need to control the backlight,
//	possibly through PWM for brightness control or GPIO as simple 
//	On/Off control.
//
void 
setup_iomux_lcd ( void )
{
	static const iomux_v3_cfg_t lcd_pads[] = {
		MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,	// Pixel clock
		MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15,		// Display enable
		MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,		// HSync
		MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,		// VSync
		MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00,		// B 0
		MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01,		// B 1
		MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02,		// B 2
		MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03,		// B 3
		MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04,		// B 4
		MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05,		// B 5
		MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06,		// B 6
		MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07,		// B 7
		MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08,		// G 0
		MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09,		// G 1
		MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10,		// G 2
		MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11,		// G 3
		MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12,		// G 4
		MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13,		// G 5
		MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14,		// G 6
		MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15,		// G 7
		MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16,		// R 0
		MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17,		// R 1
		MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18,		// R 2
		MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19,		// R 3
		MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20,		// R 4
		MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21,		// R 5
		MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22,		// R 6
		MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23,		// R 7
	};

	imx_iomux_v3_setup_multiple_pads ( lcd_pads, ARRAY_SIZE(lcd_pads) );
#if 0

	/* Turn on GPIO backlight */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_D24__GPIO3_24);
	gpio_direction_output(MX6LOCO_LCD_POWER, 1);

	/* Turn on display contrast */
	imx_iomux_v3_setup_pad(MX6_PAD_GPIO_1__GPIO1_1);
	gpio_direction_output(IMX_GPIO_NR(1, 1), 1);
#endif // 0
}


static struct display_info_t const displays[] = {
{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "Parallel RGB",
		.refresh        = 60,
		.xres           = 480,
		.yres           = 272,
		.pixclock       = 111111,	// Set to 9MHz
		.left_margin    = 40,		// Horizontal Back Porch
		.right_margin   = 5,		// Horizontal Front Porch
		.upper_margin   = 8,		// Vertical Back Porch
		.lower_margin   = 8,		// Vertical Front Porch
		.hsync_len      = 1,		// Pulse width in clocks
		.vsync_len      = 2,		// Pulse width in clocks
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} },
{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		/*
		 * For Kyocera TCG070WVLQEPNN-AN20:
		 *	Dot Clk  = 33.2MHz
		 *	H-active = 800 pixels
		 *	H-total  = 1056 (so 256 pixels of blanking + sync)
		 *	H-period = 31.8us
		 *	V-active = 480 lines
		 *	V-total  = 525 (so 45 lines of blanking + sync)
		 *	Refresh  = 60Hz
		 */
		.name           = "LVDS RGB Display",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 30120, //ps - 33.2MHz,
		.left_margin    = 84,    //Adjust to centre
		.right_margin   = 84,    // image horizontally
		.upper_margin   = 20,    //Adjust to centre
		.lower_margin   = 20,    //  image vertically
		.hsync_len      = 88,	 //using VESA's 8% rule
		.vsync_len      = 5,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

int 
board_video_skip ( void ) //int Default )
{
	int i;
	int ret;
	int Default = 1;
	char const *panel = getenv("panel");
	if (!panel) {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			struct display_info_t const *dev = displays+i;
			if (dev->detect && dev->detect(dev)) {
				panel = dev->mode.name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			i = Default;
			panel = displays[i].mode.name;
			printf("No panel detected: default to %s\n", panel);
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
		if (!ret) {
			displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		return -EINVAL;
	}

	return 0;
}

static void
setup_display ( int DoLVDS )
{

	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;
	
	printf ( "Entering setup_display, DoLVDS = %d\n", DoLVDS );
	if ( DoLVDS ) {	
		//
		//	HSync, VSync and Dot clock already set up
		//
		/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
		imx_iomux_v3_setup_multiple_pads(di0_pads, ARRAY_SIZE(di0_pads));
		}

	enable_ipu_clock();
#if 0
	//
	//	M300 has no HDMI output
	//
	imx_setup_hdmi();
#endif	// 0
	if ( !DoLVDS ) {
		reg = readl(&mxc_ccm->chsccdr);
		reg |= (CHSCCDR_CLK_SEL_LDB_DI0
			<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
		writel(reg, &mxc_ccm->chsccdr);
		}
	else {
		/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
		reg = readl(&mxc_ccm->CCGR3);
		reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
		writel(reg, &mxc_ccm->CCGR3);

		/* set LDB0, LDB1 clk select to 011/011 */
		reg = readl(&mxc_ccm->cs2cdr);
		reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
			 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
		reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
		      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
		writel(reg, &mxc_ccm->cs2cdr);

		reg = readl(&mxc_ccm->cscmr2);
		reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
		writel(reg, &mxc_ccm->cscmr2);

		reg = readl(&mxc_ccm->chsccdr);
		reg |= (CHSCCDR_CLK_SEL_LDB_DI0
			<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
		writel(reg, &mxc_ccm->chsccdr);

		reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
		     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
		     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_HIGH
		     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
		     | IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT
		     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
		     | IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT
		     | IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
		     | IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
		writel(reg, &iomux->gpr[2]);

		reg = readl(&iomux->gpr[3]);
		reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
				| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
				    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
			       << IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
		writel(reg, &iomux->gpr[3]);
		}
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();

	return cpu_eth_init(bis);
}

int board_early_init_f(void)
{
	int	LVDSVideo = 1;

	init_status_leds ( );
	show_status_leds ( 1 );

	setup_iomux_uart();
	if ( !LVDSVideo )
		setup_iomux_lcd ( );

	show_status_leds ( 2 );
#if 0
	gpio_direction_output ( TEST_IO_PIN, 1 );
	while ( 1 ) {
		gpio_set_value ( TEST_IO_PIN, 1 );
		udelay ( 1000000 );
		gpio_set_value ( TEST_IO_PIN, 0 );
		udelay ( 1000000 );
		}
#endif	// 0

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display ( 1 );
	show_status_leds ( 3 );
#endif

	return 0;
}



int board_init(void)
{
	//init_status_leds ( );
	show_status_leds ( 4 );

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	show_status_leds ( 5 );

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	/* No SD3 on Mithras M300
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	*/
	/* 8 bit bus width */
	{"mmc1", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{

#ifdef CONFIG_CMD_BMODE
	show_status_leds ( 6 );
	add_board_boot_modes(board_boot_modes);
	show_status_leds ( 7 );
#endif

	return 0;
}



int checkboard(void)
{
	puts("Board: Mithras MX6M300DL\n");
	return 0;
}
