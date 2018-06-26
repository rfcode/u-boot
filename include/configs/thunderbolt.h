/*
 * (C) Copyright 2007-2008
 * Stelian Pop <stelian.pop@leadtechdesign.com>
 * Lead Tech Design <www.leadtechdesign.com>
 *
 * Configuation settings for RF Code Thunderbolt board.
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

#ifndef __CONFIG_H
#define __CONFIG_H

#define XTAL18432
//#define XTALG20EK

/* ARM asynchronous clock */
#ifdef XTAL18432
 #define AT91_MAIN_CLOCK		393216000	/* from 18.432 MHz crystal x 128 / 3 / 2 */
 #define AT91_MASTER_CLOCK		131072000	/* peripheral = main / 3 */
#elif defined(XTALG20EK)
 #define AT91_MAIN_CLOCK		396288000	/* from 18.432 MHz crystal x 43 / 1 / 2 */
 #define AT91_MASTER_CLOCK		132096000	/* peripheral = main / 3 */
#else
 #define AT91_MAIN_CLOCK		400000000	/* from 20 MHz crystal x 40 / 1 / 2 */
 #define AT91_MASTER_CLOCK		133333333	/* peripheral = main / 3 */
#endif

#define CFG_HZ				1000000		/* 1us resolution */
#define AT91_SLOW_CLOCK			32768		/* slow clock */

#define CONFIG_ARM926EJS		1	/* This is an ARM926EJS Core	*/
#define CONFIG_AT91SAM9G20		1	/* It's an Atmel AT91SAM9G20 SoC*/
#define CONFIG_RFCTHUNDERBOLT		1	/* on an RF Code Thunderbolt Board	*/
#undef CONFIG_USE_IRQ				/* we don't need IRQ/FIQ stuff	*/

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs	*/
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1
#define CONFIG_REVISION_TAG		1

#define BOARD_LATE_INIT			1	/* Use board_late_init function	*/

/* U-Boot is loaded by bootstrap so base init is already done */
#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_SKIP_RELOCATE_UBOOT

/*
 * Hardware drivers
 */
#define CONFIG_ATMEL_USART		1
#undef CONFIG_USART0
#undef CONFIG_USART1
#undef CONFIG_USART2
#define CONFIG_USART3			1	/* USART 3 is DBGU */


#define SPI0_BASE			0xfffc8000
#define CONFIG_SPI			1
#define CONFIG_ATMEL_SPI		1
#define CONFIG_SPI_FLASH		1
#define CONFIG_SPI_FLASH_JEDEC		1
#define CONFIG_CMD_SF			1

#define CONFIG_BOOTDELAY		3
#define CONFIG_BOOT_RETRY_TIME		300
#define CONFIG_RESET_TO_RETRY		1
#define CONFIG_ABORT_SCRIPT_ON_ERROR	1

/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTFILESIZE	1
#define CONFIG_BOOTP_BOOTPATH		1
#define CONFIG_BOOTP_GATEWAY		1
#define CONFIG_BOOTP_HOSTNAME		1

/*
 * Command line configuration.
 */
#include <config_cmd_default.h>
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_AUTOSCRIPT
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_TERMINAL

#define CONFIG_CMD_PING			1
#define CONFIG_CMD_DHCP			1
#define CONFIG_CMD_USB			1
#define CONFIG_CMD_MII			1
#define CONFIG_CMD_BDINFO		1
#define CONFIG_CMD_I2C			1
#define CONFIG_CMD_DTT			1
#define CONFIG_CMD_DATE			1

/* SDRAM */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			0x20000000
#define PHYS_SDRAM_SIZE			0x02000000	/* 32 megs */

/* NOR flash */
#define CFG_FLASH_CFI			1
#define CFG_FLASH_CFI_DRIVER		1
#define CFG_FLASH_PROTECTION		1
#define CFG_FLASH_USE_BUFFER_WRITE	1
#define AT91_EBI_CS0_BASE		0x10000000
#define PHYS_FLASH_1			AT91_EBI_CS0_BASE
#define CFG_FLASH_BASE			AT91_EBI_CS0_BASE
#define CFG_MAX_FLASH_SECT		259
#define CFG_MAX_FLASH_BANKS		1

/* Ethernet */
#define CONFIG_MACB			1
#define CONFIG_MII			1
#define CONFIG_NET_MULTI		1
#define CONFIG_NET_RETRY_COUNT		20
#define CONFIG_RESET_PHY_R		1

/* USB Host */
// do we want USB host support?
#define CONFIG_USB_OHCI_NEW		1
#define LITTLEENDIAN			1
#define CONFIG_DOS_PARTITION		1
#define CFG_USB_OHCI_CPU_INIT		1
#define CFG_USB_OHCI_BOARD_INIT		1
#define CFG_USB_OHCI_REGS_BASE		0x00500000
#define CFG_USB_OHCI_SLOT_NAME		"at91sam9g20"
//#define CFG_USB_OHCI_MAX_ROOT_PORTS	2
#define CFG_USB_OHCI_MAX_ROOT_PORTS	1
#define CONFIG_USB_STORAGE		1

/* USB Device */
#define CFG_USBD_REGS_BASE		0xFFFA4000
#define CONFIG_USB_DEVICE		/* Include UDC driver */
#define CONFIG_USB_TTY			/* Bind the TTY driver to UDC */

#define CONFIG_USBD_MANUFACTURER	"RF Code"
#define CONFIG_USBD_PRODUCT_NAME	"M250 Reader"
#define CONFIG_USBD_VENDORID		0x1C40
#define CONFIG_USBD_PRODUCTID_GSERIAL	0x0000	// not used
#define CONFIG_USBD_PRODUCTID_CDCACM	0x05F1
#define CONFIG_USBD_BCD_DEVICE		0x0100
#define	CONFIG_USBD_VBUS_SENSE_PIN	AT91_PIN_PC12

#define CONFIG_USBD_ALT_MANUFACTURER		"RF Code"
#define CONFIG_USBD_ALT_PRODUCT_NAME		"Serial Device"
#define CONFIG_USBD_ALT_VENDORID		0x1C40
#define CONFIG_USBD_ALT_PRODUCTID_GSERIAL	0x0000	// not used
#define CONFIG_USBD_ALT_PRODUCTID_CDCACM	0x05F5
#define CONFIG_USBD_ALT_BCD_DEVICE		0x0100


#define CFG_LOAD_ADDR			0x21000000	/* load address for kernel */

#define CFG_MEMTEST_START		PHYS_SDRAM
#define CFG_MEMTEST_END			0x21e00000

/* env + u-boot + linux + rootfs in flash on CS0 */
#define CFG_ENV_IS_IN_FLASH		1
#define CFG_MONITOR_BASE		(CFG_FLASH_BASE + 0x20000)
#define CFG_ENV_OFFSET			0x10000
#define CFG_ENV_SIZE			0x08000
#define CFG_ENV_OFFSET_REDUND		0x18000

//#define CONFIG_ETHADDR      		00:50:C2:1A:00:00
#define CONFIG_BOOTCOMMAND		"bootm 10060000"
#define CONFIG_BOOTARGS			"mem=32M console=ttyS0,115200 "		\
					"root=/dev/mtdblock3 rootflags=noatime "\
					"mtdparts=physmap-flash.0:128k(env),"	\
					"256k(uboot),2M(linux),"		\
					"24M(root),-(data) rootfstype=jffs2"

#define CONFIG_BAUDRATE			115200
#define CFG_BAUDRATE_TABLE		{115200 , 19200, 38400, 57600, 9600 }

#define CFG_PROMPT			"RFCode> "
#define CFG_CBSIZE			256
#define CFG_MAXARGS			32
#define CFG_PBSIZE			(CFG_CBSIZE + sizeof(CFG_PROMPT) + 16)
#define CFG_LONGHELP			1
#define CONFIG_CMDLINE_EDITING		1

#define ROUND(A, B)			(((A) + (B)) & ~((B) - 1))
/*
 * Size of malloc() pool
 */
#define CFG_MALLOC_LEN			ROUND(3 * CFG_ENV_SIZE + 128*1024, 0x1000)
#define CFG_GBL_DATA_SIZE		128	/* 128 bytes for initial data */

#define CONFIG_STACKSIZE		(32*1024)	/* regular stack */

#ifdef CONFIG_USE_IRQ
#error CONFIG_USE_IRQ not supported
#endif

#define CONFIG_EXTRA_ENV_SETTINGS					\
	"postupg=tftp 21000000 u-boot.bin; protect off 10020000 1005ffff;" \
	" erase 10020000 +${filesize}; cp.b 21000000 10020000 ${filesize}\0" \
	"kernupg=tftp 21000000 thunderbolt_kernel.bin; protect off 10060000 1025FFFF;" \
	" erase 10060000 +${filesize}; cp.b 21000000 10060000 ${filesize}\0" \
	"rootupg=tftp 21000000 ${rootimg}; protect off 10260000 11A5FFFF;" \
	" erase 10260000 11A5FFFF; cp.b 21000000 10260000 ${filesize};" \
	" jclean 10260000 11A5FFFF\0" \
	"datareset=protect off 11A60000 11FFFFFF; erase 11A60000 11FFFFFF\0" \
	"forceusb=1\0"

#define CONFIG_SOFT_I2C		1	/* I2C bit-banged		*/
#define CFG_I2C_SPEED		100000	/* I2C speed and slave address	*/
#define CFG_I2C_SLAVE		0x7F
#define I2C_INIT		tbolt_i2c_init()
#define I2C_ACTIVE		tbolt_i2c_active()
#define I2C_TRISTATE		tbolt_i2c_tristate()
#define I2C_READ		tbolt_i2c_read()
#define I2C_SDA(bit)		tbolt_i2c_sda(bit)
#define I2C_SCL(bit)		tbolt_i2c_scl(bit)
#define I2C_DELAY		udelay(5)	/* 1/4 I2C clock duration */

#define CONFIG_DTT_LM75		1                /* ON Semi's LM75 */
#define CONFIG_DTT_SENSORS	{1}              /* Sensor address */
#define CFG_DTT_MAX_TEMP	70
#define CFG_DTT_LOW_TEMP	-30
#define CFG_DTT_HYSTERESIS	3
#define CONFIG_DTT_10X		1

#define CONFIG_RTC_PCF8563	1
#define CFG_I2C_RTC_ADDR	(0x51)

#endif

#ifndef __ASSEMBLY__
/* Thunderbolt-specific global variables */
extern int usbtty_console_in_use;
extern int user_data_reset;
extern int rfc_new_phy;
extern void tbolt_i2c_init(void);
extern void tbolt_i2c_active(void);
extern void tbolt_i2c_tristate(void);
extern int tbolt_i2c_read(void);
extern void tbolt_i2c_sda(int);
extern void tbolt_i2c_scl(int);
extern int board_prepare_for_os(void);

#endif
