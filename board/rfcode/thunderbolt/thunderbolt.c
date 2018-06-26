/*
 * (C) Copyright 2007-2008
 * Stelian Pop <stelian.pop@leadtechdesign.com>
 * Lead Tech Design <www.leadtechdesign.com>
 * 
 * (C) Copyright 2009 RF Code
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/arch/at91sam9260.h>
#include <asm/arch/at91sam9260_matrix.h>
#include <asm/arch/at91sam9_smc.h>
#include <asm/arch/at91_pmc.h>
#include <asm/arch/at91_rstc.h>
#include <asm/arch/gpio.h>
#include <asm/arch/io.h>
#ifdef CONFIG_SPI
#include <spi.h>
#endif
#ifdef CONFIG_USB_TTY
#include <usbdcore_at91.h>
#endif
#if defined(CONFIG_RESET_PHY_R) && defined(CONFIG_MACB)
#include <net.h>
#endif

/* Thunderbolt GPIO Outputs */
#define	POWER_LED		AT91_PIN_PB25
#define	ACTIVITY_LED		AT91_PIN_PB22
#define	SD_LED			AT91_PIN_PB30
#define	SPARE_LED		AT91_PIN_PB11
#define	STATUS_LED		AT91_PIN_PC8
#define	IN_USE_LED		AT91_PIN_PB23

#define	SD_NRST			AT91_PIN_PB10
#define	USBH_PWR_EN		AT91_PIN_PC11
#define	RADIO_PWR_EN		AT91_PIN_PB27
#define	FLASH_NWP		AT91_PIN_PC6
#define	BOOTFLASH_NWP		AT91_PIN_PA4

#define	SPI0_MISO		AT91_PIN_PA0
#define	SPI0_MOSI		AT91_PIN_PA1
#define	SPI0_SPCK		AT91_PIN_PA2
#define	SPI0_NPCS0		AT91_PIN_PA3

/* Thunderbolt GPIO Inputs */
#define	USBD_5V_SENSE		AT91_PIN_PC12
#define	USBH_PWR_FAULT		AT91_PIN_PC13

#define	SD_NCD			AT91_PIN_PB29
#define	SD_NWP			AT91_PIN_PB31
#define	SD_CMD			AT91_PIN_PA7

#define I2C_SDA_PIN		AT91_PIN_PA30
#define I2C_SCL_PIN		AT91_PIN_PA31

/* GPIO output function macros */
#define power_LED_on()		at91_set_gpio_value(POWER_LED, 0)
#define power_LED_off()		at91_set_gpio_value(POWER_LED, 1)
#define activity_LED_on()	at91_set_gpio_value(ACTIVITY_LED, 0)
#define activity_LED_off()	at91_set_gpio_value(ACTIVITY_LED, 1)
#define sd_LED_on()		at91_set_gpio_value(SD_LED, 0)
#define sd_LED_off()		at91_set_gpio_value(SD_LED, 1)
#define spare_LED_on()		at91_set_gpio_value(SPARE_LED, 0)
#define spare_LED_off()		at91_set_gpio_value(SPARE_LED, 1)
#define status_LED_on()		at91_set_gpio_value(STATUS_LED, 0)
#define status_LED_off()	at91_set_gpio_value(STATUS_LED, 1)
#define in_use_LED_on()		at91_set_gpio_value(IN_USE_LED, 0)
#define in_use_LED_off()	at91_set_gpio_value(IN_USE_LED, 1)

#define radio_pwr_enable()	at91_set_gpio_value(RADIO_PWR_EN, 1)
#define radio_pwr_disable()	at91_set_gpio_value(RADIO_PWR_EN, 0)
#define usbh_pwr_enable()	at91_set_gpio_value(USBH_PWR_EN, 0)
#define usbh_pwr_disable()	at91_set_gpio_value(USBH_PWR_EN, 1)
#define norflash_wp_enable()	at91_set_gpio_value(FLASH_NWP, 0)
#define norflash_wp_disable()	at91_set_gpio_value(FLASH_NWP, 1)
#define bootflash_wp_enable()	at91_set_gpio_value(BOOTFLASH_NWP, 0)
#define bootflash_wp_disable()	at91_set_gpio_value(BOOTFLASH_NWP, 1)


DECLARE_GLOBAL_DATA_PTR;

/* Global flag to indicate if USB TTY is in use or not */
int usbtty_console_in_use = 0;
/* Global flag to request user data reset in application */
int user_data_reset = 0;
/* Global flag to indicate if SMSC LAN8710 PHY was detected */
int rfc_new_phy = 1;	/* Assume true */

/* ------------------------------------------------------------------------- */
/* Simplistic Ascii-to-integer conversion                                    */
/* ------------------------------------------------------------------------- */
static u32 atoi(char *string)
{
	u32 result = 0;
	while (string && (*string >= '0') && (*string <= '9')) {
		result *= 10;
		result += *string - '0';
		string++;
	}
	return result;
}

/* ------------------------------------------------------------------------- */
/*
 * Miscelaneous platform dependent initializations
 */

static void thunderbolt_gpio_hw_init(void)
{
	/* Enable clocks to the PIO controllers that have inputs */
	at91_sys_write(AT91_PMC_PCER, (1 << AT91SAM9260_ID_PIOA) | (1 << AT91SAM9260_ID_PIOB) | 
					(1 << AT91SAM9260_ID_PIOC));
									
	at91_set_gpio_input(USBD_5V_SENSE, 0);	/* USB Device 5V sense, no pull-up */
	at91_set_gpio_input(USBH_PWR_FAULT, 0);	/* USB Host overcurrent, no pull-up */
	at91_set_gpio_input(SD_NCD, 0);		/* SD card detect, no pull-up */
	at91_set_gpio_input(SD_NWP, 0);		/* SD card write protection, no pull-up */
	
	/* Turn on all GPIO LEDs (some will be turned off in board_late_init) */
	at91_set_gpio_output(POWER_LED, 0);	/* Power LED on */
	at91_set_gpio_output(ACTIVITY_LED, 0);	/* Activity LED on */
	at91_set_gpio_output(SD_LED, 0);	/* SD LED on */
	at91_set_gpio_output(SPARE_LED, 0);	/* Spare LED on */
	at91_set_gpio_output(STATUS_LED, 0);	/* Status LED on */
	at91_set_gpio_output(IN_USE_LED, 0);	/* In Use LED on */

	at91_set_gpio_output(USBH_PWR_EN, 1);	/* USB Host power disabled */
	at91_set_gpio_output(RADIO_PWR_EN, 1);	/* Radio power enabled */
	at91_set_gpio_output(FLASH_NWP, 1);	/* NOR Flash write protect off */
	at91_set_gpio_output(BOOTFLASH_NWP, 0);	/* Boot Flash write protect on */
}

static void thunderbolt_serial_hw_init(void)
{
#ifdef CONFIG_USART0
	at91_set_A_periph(AT91_PIN_PB4, 0);	/* TXD0 */
	at91_set_A_periph(AT91_PIN_PB5, 1);	/* RXD0 */
	at91_sys_write(AT91_PMC_PCER, 1 << AT91_ID_US0);
#endif

#ifdef CONFIG_USART1
	at91_set_A_periph(AT91_PIN_PB6, 0);	/* TXD1 */
	at91_set_A_periph(AT91_PIN_PB7, 1);	/* RXD1 */
	at91_sys_write(AT91_PMC_PCER, 1 << AT91_ID_US1);
#endif

#ifdef CONFIG_USART2
	at91_set_A_periph(AT91_PIN_PB8, 0);	/* TXD2 */
	at91_set_A_periph(AT91_PIN_PB9, 1);	/* RXD2 */
	at91_sys_write(AT91_PMC_PCER, 1 << AT91_ID_US2);
#endif

#ifdef CONFIG_USART3	/* DBGU */
	at91_set_A_periph(AT91_PIN_PB14, 1);	/* DRXD */
	at91_set_A_periph(AT91_PIN_PB15, 0);	/* DTXD */
	at91_sys_write(AT91_PMC_PCER, 1 << AT91_ID_SYS);
#endif
}

static int thunderbolt_test_for_user_reset (void)
{
	unsigned int status = at91_sys_read(AT91_RSTC_SR);
	
	/* Always enable USBTTY console for a "user" reset (reset button pressed) */
	if ((status & AT91_RSTC_RSTTYP) == AT91_RSTC_RSTTYP_USER) {
		/* If we autoboot, we will pass the userdatareset flag */
		user_data_reset = 1;
		return 1;
	}
	return 0;
}

static int last_sda = 1;
//static int is_active = 0;
void tbolt_i2c_init()
{
	at91_set_GPIO_periph(I2C_SDA_PIN, 1);
	at91_set_multi_drive(I2C_SDA_PIN, 1);
	at91_set_GPIO_periph(I2C_SCL_PIN, 1);
	at91_set_multi_drive(I2C_SCL_PIN, 1);
	gpio_direction_output(I2C_SCL_PIN, 1);
	gpio_direction_output(I2C_SDA_PIN, 1);
}

void tbolt_i2c_active()
{
//	at91_set_gpio_value(I2C_SDA_PIN, 1);
}

void tbolt_i2c_tristate()
{
	at91_set_gpio_value(I2C_SDA_PIN, 1);
}

int tbolt_i2c_read()
{
	return at91_get_gpio_value(I2C_SDA_PIN);
}
	
void tbolt_i2c_sda(int v) 
{
	at91_set_gpio_value(I2C_SDA_PIN, v);
	last_sda = v;
}

void tbolt_i2c_scl(int v)
{
	at91_set_gpio_value(I2C_SCL_PIN, v);
}


#ifdef CONFIG_MACB
static void thunderbolt_macb_hw_init(void)
{
	/* Enable clock */
	at91_sys_write(AT91_PMC_PCER, 1 << AT91SAM9260_ID_EMAC);

	if (rfc_new_phy) {
		/* THUNDERBOLT REV 02 and later
		 * We can leave pull-ups enabled on:
		 * 	ERXD0 (PA14) => MODE0 = 1 (internal pullup)
		 * 	ERXD1 (PA15) => MODE1 = 1 (internal pullup)
		 *	ECOL  (PA29) => MODE2 = 1 (internal pullup)
		 * 	EMDIO (PA21)              (board pullup)
		 * But disable pull-ups on:
		 *	RXER  (PA18) => PHYAD0    (internal pulldown)
		 *	RXCLK (PA27) => PHYAD1    (internal pulldown)
		 * 	ERXD3 (PA26) => PHYAD2    (internal pulldown)
		 * 	ERXD2 (PA25) => MII mode  (internal pulldown)
		 * 	ECRS  (PA28)              (internal pulldown)
		 */
		writel(pin_to_mask(AT91_PIN_PA18) |
		       pin_to_mask(AT91_PIN_PA25) |
		       pin_to_mask(AT91_PIN_PA26) |
		       pin_to_mask(AT91_PIN_PA27) |
		       pin_to_mask(AT91_PIN_PA28),
		       pin_to_controller(AT91_PIN_PA0) + PIO_PUDR);
	}
	else {
		/* THUNDERBOLT REV 01 had SMSC LAN8700
		 * We can leave pull-ups enabled on:
		 * 	ERXD0 (PA14) => MODE0 = 1 (internal pullup & board pullup)
		 * 	ERXD1 (PA15) => MODE1 = 1 (internal pullup)
		 * 	EMDIO (PA21)              (board pullup)
		 * 	ERXD2 (PA25) => MODE2 = 1 (internal pullup)
		 * 	ECRS  (PA28) => ADDR4 = 1 (internal pullup)
		 * But disable pull-ups on:
		 *	RXDV  (PA17) => must be low on reset (internal pulldown & board pulldown)
		 *	RXER  (PA18) => must be low on reset (internal pulldown & board pulldown)
		 * 	ERXD3 (PA26) => nINTSEL = 0 (TX_ER/TXD4 mode) (board pulldown)
		 *	RXCLK (PA27) => REG enabled (internal pulldown & board pulldown)
		 *	COL   (PA29) => MII mode (internal pulldown)
		 */
		writel(pin_to_mask(AT91_PIN_PA17) |
		       pin_to_mask(AT91_PIN_PA18) |
		       pin_to_mask(AT91_PIN_PA26) |
		       pin_to_mask(AT91_PIN_PA27) |
		       pin_to_mask(AT91_PIN_PA29),
		       pin_to_controller(AT91_PIN_PA0) + PIO_PUDR);
	}
	
	/* Need to reset PHY -> 500ms ((2^14)/32768) reset */
	at91_sys_write(AT91_RSTC_MR, AT91_RSTC_KEY |
				     (AT91_RSTC_ERSTL & (0x0D << 8)) |
				     AT91_RSTC_URSTEN);

	at91_sys_write(AT91_RSTC_CR, AT91_RSTC_KEY | AT91_RSTC_EXTRST);

	/* Wait for end hardware reset */
	while (!(at91_sys_read(AT91_RSTC_SR) & AT91_RSTC_NRSTL)) {
		extern void usbtty_poll(void);
		/* If called after USB started, don't starve USB processing */
		if (usbtty_console_in_use)
			usbtty_poll();
	}

	/* Restore NRST value */
	at91_sys_write(AT91_RSTC_MR, AT91_RSTC_KEY |
				     (AT91_RSTC_ERSTL & (0x0 << 8)) |
				     AT91_RSTC_URSTEN);

	at91_set_A_periph(AT91_PIN_PA19, 0);	/* ETXCK_EREFCK */
	at91_set_A_periph(AT91_PIN_PA17, 0);	/* ERXDV */
	at91_set_A_periph(AT91_PIN_PA14, 0);	/* ERX0 */
	at91_set_A_periph(AT91_PIN_PA15, 0);	/* ERX1 */
	at91_set_A_periph(AT91_PIN_PA18, 0);	/* ERXER */
	at91_set_A_periph(AT91_PIN_PA16, 0);	/* ETXEN */
	at91_set_A_periph(AT91_PIN_PA12, 0);	/* ETX0 */
	at91_set_A_periph(AT91_PIN_PA13, 0);	/* ETX1 */
	at91_set_A_periph(AT91_PIN_PA21, 0);	/* EMDIO */
	at91_set_A_periph(AT91_PIN_PA20, 0);	/* EMDC */

#ifndef CONFIG_RMII
	at91_set_B_periph(AT91_PIN_PA28, 0);	/* ECRS */
	at91_set_B_periph(AT91_PIN_PA29, 0);	/* ECOL */
	at91_set_B_periph(AT91_PIN_PA25, 0);	/* ERX2 */
	at91_set_B_periph(AT91_PIN_PA26, 0);	/* ERX3 */
	at91_set_B_periph(AT91_PIN_PA27, 0);	/* ERXCK */

	at91_set_B_periph(AT91_PIN_PA23, 0);    /* ETX2 */
	at91_set_B_periph(AT91_PIN_PA24, 0);    /* ETX3 */
	at91_set_multi_drive(AT91_PIN_PA23, 0); /* SAM-BA sets these open-collector */
	at91_set_multi_drive(AT91_PIN_PA24, 0); /* SAM-BA sets these open-collector */
	at91_set_B_periph(AT91_PIN_PA22, 0);	/* ETXER */
#endif

}
#endif

#ifdef CONFIG_SPI
static void thunderbolt_spi_hw_init(void)
{
	at91_set_A_periph(SPI0_MISO, 0);	/* SPI0_MISO */
	at91_set_A_periph(SPI0_MOSI, 0);	/* SPI0_MOSI */
	at91_set_A_periph(SPI0_SPCK, 0);	/* SPI0_SPCK */
	at91_set_gpio_output(SPI0_NPCS0, 1);	/* SPI0_NPCS0 under manual control */

	/* Enable clock */
	at91_sys_write(AT91_PMC_PCER, 1 << AT91SAM9260_ID_SPI0);
}

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	return (bus == 0 && cs == 0);
}

void spi_cs_activate(struct spi_slave *slave)
{
	at91_set_gpio_value(SPI0_NPCS0, 0);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	at91_set_gpio_value(SPI0_NPCS0, 1);
}
#endif

static void thunderbolt_sdio_card_reset (void)
{
	at91_set_gpio_output(SD_NRST, 0);	/* Drive it low */
	udelay(10000);				/* Wait 10ms */
	at91_set_gpio_input(SD_NRST, 1);	/* Input w/ pull-up */
}

int board_init(void)
{
	/* Enable Ctrlc */
	console_init_f();

	/* This architecture number is defined in include/asm-arm/mach-types.h
	 * which is copied from the kernel headers for the kernel that u-boot
	 * will be loading. The kernel verifies that the arch number passed to
	 * it from u-boot (bootm command) matches the arch number compiled into
	 * the kernel
	 */
	/* arch number of Thunderbolt-Board */
	gd->bd->bi_arch_number = MACH_TYPE_RFCTHUNDERBOLT;
	/* adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	/* Initialize Serial */
	thunderbolt_serial_hw_init();
	
	/* Initialize GPIO */
	thunderbolt_gpio_hw_init();

	/* Reset SDIO card (proprietary) */
	thunderbolt_sdio_card_reset();
	
#ifdef CONFIG_SPI
	/* Initialize Bootflash SPI Pins */
	thunderbolt_spi_hw_init();
#endif
	
#ifdef CONFIG_MACB
	/* Initialize Ethernet MAC Pins */
	thunderbolt_macb_hw_init();
#endif

#ifdef CONFIG_USB_TTY
	/* Definitely enable USBTTY if reset button pressed */
	usbtty_console_in_use = thunderbolt_test_for_user_reset();
#endif

	return 0;
}

/* This function returns an integer representation of the environment
 * vairable "boardtype". This variable allows manufacturing to instruct the
 * software about the board type / hardware configuration
 */
static u32 get_board_type (void)
{
	/* Missing "boardtype" will be returned as zero			*/
	return atoi(getenv("boardtype"));
}

/* board_late_init is used to finish up initialization once the configuration
 * environment is available to us
 */
int board_late_init(void)
{
	u32 bdtype = get_board_type();
	
	//printf("BdTyp: %d\n", bdtype);
	/* Type 1 = "EconoBox", depopulated Thunderbolt board */
	if (bdtype == 1) {
		/* Enable pull-ups on otherwise floating input pins */
		at91_set_gpio_input(USBH_PWR_FAULT, 1);	/* n/c USB Host overcurrent, use pull-up */
		at91_set_gpio_input(SD_NCD, 1);		/* n/c SD card detect, use pull-up */
		at91_set_gpio_input(SD_NWP, 1);		/* n/c SD card write protection, use pull-up */
		at91_set_gpio_input(SD_CMD, 1);		/* n/c SD card CMD line, use pull-up */
		/* I2C_SDA_PIN, I2C_SCK_PIN already have pull-up enabled */
		
		at91_set_gpio_output(AT91_PIN_PC8, 1);	/* Activity LED off */
		setenv ("rootimg", "rootfs-m240.arm_nofpu.jffs2");
		///* If it's not already established, set slowclk to "1"	*/
		//if (getenv("slowclk") == NULL)
		//	setenv ("slowclk", "1");
	}
	/* Type 0 (default) = "Standard M250 Thunderbolt board */
	else {
		/* Turn off LEDs that should be off by default */
		at91_set_gpio_output(ACTIVITY_LED, 1);	/* Activity LED off */
		at91_set_gpio_output(SD_LED, 1);	/* SD LED off */
		at91_set_gpio_output(SPARE_LED, 1);	/* Spare LED off */
		at91_set_gpio_output(IN_USE_LED, 1);	/* In Use LED off */
		setenv ("rootimg", "rootfs.arm_nofpu.jffs2");
		//setenv ("slowclk", NULL);
	}
	return 0;
}

int dram_init(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_SIZE;
	return 0;
}

#ifdef CFG_USB_OHCI_BOARD_INIT
int usb_board_init(void)
{
	usbh_pwr_enable();
	return 0;
}

int usb_board_stop(void)
{
	usbh_pwr_disable();
	return 0;
}

int usb_board_init_fail(void)
{
	return usb_board_stop();
}
#endif

/* We will pass the "boardtype" value to the kernel as a board revision tag */
u32 get_board_rev(void)
{
	return get_board_type();
}

#ifdef CONFIG_RESET_PHY_R
void reset_phy(void)
{
	/* If we now know that this is an old version (REV01) */
	if (!rfc_new_phy) {
		//printf("Old PHY detected\n");
		/* Re-issue PHY reset with appropriate strapping for LAN8700 */
		thunderbolt_macb_hw_init();
	}
 #ifdef CONFIG_MACB
	/* Initialize ethernet HW addr prior to starting Linux */
extern void macb_set_ethaddr(void);
	macb_set_ethaddr();
	//eth_init(gd->bd); This method works, but takes more time because the interface is autonegotiated
 #endif
}
#endif

/* Just before we launch the OS, we might need to tweak some registers
 * If this is an M240, we [MIGHT] switch to a slower clock in Linux. U-Boot is
 * not as flexible with dynamic clocking (messes up UART baud, etc.)
 * so we don't change the clock until just before launching Linux.
 */
int board_prepare_for_os(void)
{
	u32 slowclk = atoi(getenv("slowclk"));
	
	/* M240 "EconoBox", default is slowclk=1			*/
	if (slowclk == 1)
	{
		unsigned int timeout = 65535;
		
		/* 18.432MHz x 128 / 5 = 471.859200 MHz */
		at91_sys_write(AT91_CKGR_PLLAR, 0x207FBF05);
		while (1)
		{
			/* Wait for PLLA to lock			*/
			if (at91_sys_read(AT91_PMC_SR) & AT91_PMC_LOCKA)
				break;
			if (--timeout == 0)
			{
				puts("timeout waiting for PLLA LOCK\n");
				return -1;
			}
		}
		/* Set ICPLLA for freq < 600MHz per datasheet */
		at91_sys_write(AT91_PMC_PLLICPR, AT91_PMC_ICPLLA);
	}
	return 0;
}	

/* Display the type of reset */
int do_reset_type (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	unsigned int status = at91_sys_read(AT91_RSTC_SR);
	
	switch (status & AT91_RSTC_RSTTYP)
	{
	case AT91_RSTC_RSTTYP_WAKEUP:
		puts("wakeup");
		break;
	case AT91_RSTC_RSTTYP_WATCHDOG:
		puts("watchdog");
		break;
	case AT91_RSTC_RSTTYP_SOFTWARE:
		puts("software");
		break;
	case AT91_RSTC_RSTTYP_USER:
		puts("user");
		break;
	default:
		puts("general");
		break;
	}
	puts(" reset\n");
	return 0;
}
U_BOOT_CMD(rsttype,	1,	1,	do_reset_type,
	"rsttype - Display reset type\n",
	NULL
);

#if 0
int debug_toggle_led(void)
{
	if (at91_get_gpio_value(ACTIVITY_LED))
		activity_LED_on();
	else activity_LED_off();
	return 0;
}
int debug_set_led(void)
{
	activity_LED_on();
	return 0;
}
int debug_clear_led(void)
{
	activity_LED_off();
	return 0;
}

int debug_toggle_sd_led(void)
{
	if (at91_get_gpio_value(SD_LED))
		sd_LED_on();
	else sd_LED_off();
	return 0;
}
int debug_set_sd_led(void)
{
	sd_LED_on();
	return 0;
}
int debug_clear_sd_led(void)
{
	sd_LED_off();
	return 0;
}

int debug_toggle_spare_led(void)
{
	if (at91_get_gpio_value(SPARE_LED))
		spare_LED_on();
	else spare_LED_off();
	return 0;
}
int debug_set_spare_led(void)
{
	spare_LED_on();
	return 0;
}
int debug_clear_spare_led(void)
{
	spare_LED_off();
	return 0;
}

#endif

//============================================================================
//============================================================================
#if 0	
int do_slowclock (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	unsigned int timeout = 65535;

	/* 18.432MHz x 128 / 5 = 471.859200 MHz */
	at91_sys_write(AT91_CKGR_PLLAR, 0x207FBF05);
	while (1)
	{
		if (at91_sys_read(AT91_PMC_SR) & AT91_PMC_LOCKA)
			break;
		if (--timeout == 0)
		{
			puts("timeout waiting for LOCKA\n");
			break;
		}
	}
	/* Set ICPLLA for freq < 600MHz per datasheet */
	at91_sys_write(AT91_PMC_PLLICPR, AT91_PMC_ICPLLA);
	
	puts("Clock change complete\n");
	return 0;
}
U_BOOT_CMD(slowclk,	1,	1,	do_slowclock,
	"slowclk - Switch to slower clock settings\n",
	NULL
);
#endif


#if 0

int do_ethernet_off (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	/* Reset PHY -> 500ms ((2^14)/32768) reset */
	at91_sys_write(AT91_RSTC_MR, AT91_RSTC_KEY |
				     (AT91_RSTC_ERSTL & (0x0D << 8)) |
				     AT91_RSTC_URSTEN);
	at91_sys_write(AT91_RSTC_CR, AT91_RSTC_KEY | AT91_RSTC_EXTRST);
	
	if (rfc_new_phy) {
		/* THUNDERBOLT REV02
		 * Set PHY mode = 110 (fully disabled)
		 * 	ERXD0 (PA14) => MODE0 = 0 (internal pullup)
		 * 	ERXD1 (PA15) => MODE1 = 1 (internal pullup)
		 *	ECOL  (PA29) => MODE2 = 1 (internal pullup) */
		at91_set_gpio_output(AT91_PIN_PA14, 0);
		at91_set_gpio_output(AT91_PIN_PA15, 1);
		at91_set_gpio_output(AT91_PIN_PA29, 1);
	}
	else {
		/* THUNDERBOLT REV01
		 * Set PHY mode = 110 (fully disabled)
		 * 	ERXD0 (PA14) => MODE0 = 0 (internal pullup & board pullup)
		 * 	ERXD1 (PA15) => MODE1 = 1 (internal pullup)
		 * 	ERXD2 (PA25) => MODE2 = 1 (internal pullup) */
		at91_set_gpio_output(AT91_PIN_PA14, 0);
		at91_set_gpio_output(AT91_PIN_PA15, 1);
		at91_set_gpio_output(AT91_PIN_PA25, 1);
	}
	
	/* Wait till reset pulse is done */
	while (!(at91_sys_read(AT91_RSTC_SR) & AT91_RSTC_NRSTL));

	/* Restore NRST value */
	at91_sys_write(AT91_RSTC_MR, AT91_RSTC_KEY |
				     (AT91_RSTC_ERSTL & (0x0 << 8)) |
				     AT91_RSTC_URSTEN);
				     
	if (rfc_new_phy) {
		/* THUNDERBOLT REV02
		 * Switch mode pins back to input w/pullup */
		at91_set_gpio_input(AT91_PIN_PA14, 1);
		at91_set_gpio_input(AT91_PIN_PA15, 1);
		at91_set_gpio_input(AT91_PIN_PA29, 1);
	}
	else {
		/* THUNDERBOLT REV01
		 * Switch mode pins back to input w/pullup */
		at91_set_gpio_input(AT91_PIN_PA14, 1);
		at91_set_gpio_input(AT91_PIN_PA15, 1);
		at91_set_gpio_input(AT91_PIN_PA25, 1);
	}
	
	/* Disable clock to EMAC */
	at91_sys_write(AT91_PMC_PCDR, 1 << AT91SAM9260_ID_EMAC);

	return 0;
}
U_BOOT_CMD(ethoff,	1,	1,	do_ethernet_off,
	"ethoff  - turn ethernet off\n",
	NULL
);

int do_ethernet_on (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	thunderbolt_macb_hw_init();
	return 0;
}
U_BOOT_CMD(ethon,	1,	1,	do_ethernet_on,
	"ethon   - turn ethernet on\n",
	NULL
);
#endif	// ethernet on/off

#if 0
#define AT91_SHDW_CR		(AT91_SHDWC + 0x00)	/* Shutdown Controller Control Register */
#define		AT91_SHDW_SHDWCMD	(1 << 0)		/* Shutdown Command */
#define		AT91_SHDW_KEY		(0xa5 << 24)		/* KEY Password */
#define AT91_SHDW_MR		(AT91_SHDWC + 0x00)	/* Shutdown Controller Mode Register */
#define		AT91_SHDW_WKMD_NONE	(0 << 0)		/* No wake on wake input */
#define		AT91_SHDW_WKMD_RISING	(1 << 0)		/* Wake on rising edge */
#define		AT91_SHDW_WKMD_FALLING	(2 << 0)		/* Wake on falling edge */
#define		AT91_SHDW_WKMD_BOTH	(3 << 0)		/* Wake on rising or falling edge */
#define		AT91_SHDW_CPTWK0	(0xF << 4)		/* 16 x slow clock period debounce */
#define		AT91_SHDW_RTTWKEN	(1 << 16)		/* RTT wake up enable */
#define AT91_SHDW_SR		(AT91_SHDWC + 0x00)	/* Shutdown Controller Status Register */
#define		AT91_SHDW_WAKEUP0	(1 << 0)		/* Wake up since last read */


int do_shutdown (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	at91_sys_write(AT91_SHDW_CR, AT91_SHDW_KEY | AT91_SHDW_SHDWCMD);
	return 0;
}
U_BOOT_CMD(shutdown,	1,	1,	do_shutdown,
	"shutdown- processor shutdown\n",
	NULL
);

int stay_idle = 0;

int enter_idle_state (void)
{
	debug_toggle_led();
	/* Turn off processor clock */
	at91_sys_write(AT91_PMC_SCDR, AT91_PMC_PCK);
	return 0;
}

int do_idle (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	stay_idle ^= 1;
	printf("idle=%d\n", stay_idle);
	return enter_idle_state();
}
U_BOOT_CMD(idle,	1,	1,	do_idle,
	"idle    - enter processor idle mode\n",
	NULL
);
#endif // shutdown & idle

#if 0
const char *textstring = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ00001111222233334444555566667777888899990000";
int do_tryagain (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	int rc = 0;
	ulong count;
	char buffer[512];
	
	if (argc < 0) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
#if 1
	printf("Reset SR = 0x%08X\n", at91_sys_read(AT91_RSTC_SR));
	return 0;
	
	if (argc == 2) {
		count = simple_strtoul(argv[1], NULL, 10);
	}
	else count = 64;
	
	if (count >= sizeof(buffer)) {
		puts("count too large\n");
		return 1;
	}
	
	memcpy(buffer, textstring, (int)count);
	buffer[count] = '\0';
	
	printf("Sending %d bytes on stderr\n", (int)count);
	eputs(buffer);
#elif 0	
	//at91udc_probe(&udc_pdata);

	//if ((rc = at91_udctty_initialize(gd->bd)) < 0)
		//printf("at91_udctty_initialize() returned %d\n", rc);
#elif 0	
	//thunderbolt_macb_hw_init();
	//eth_init(gd->bd);
	
	//printf("USRIO : 0x%04X\n", readl(AT91SAM9260_BASE_EMAC + 0xC0));
	
	/*printf("Putting PA23 and PA24 into output '1' mode\n");
	at91_set_multi_drive(AT91_PIN_PA23, 0);
	at91_set_multi_drive(AT91_PIN_PA24, 0);
	at91_set_gpio_output(AT91_PIN_PA23, 1);
	at91_set_gpio_output(AT91_PIN_PA24, 1);*/
	
	//puts("Waiting...");
	//udelay(2000000);
	//puts("done\n");
	
#endif	
	if (at91_get_gpio_value(ACTIVITY_LED))
	{
		activity_LED_on();
	} else activity_LED_off();

	return rc;
}

U_BOOT_CMD(try,	5,	1,	do_tryagain,
	"try     - Ryan utility commands\n",
	"try device                     - list available devices\n"
	"try write  <addr> <reg> <data> - write MII PHY <addr> register <reg>\n"
);
#endif
