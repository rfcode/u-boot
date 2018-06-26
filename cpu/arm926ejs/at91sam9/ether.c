/*
 * (C) Copyright 2007-2008
 * Stelian Pop <stelian.pop@leadtechdesign.com>
 * Lead Tech Design <www.leadtechdesign.com>
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
#include <asm/arch/hardware.h>

extern int macb_eth_initialize(int id, void *regs, unsigned int phy_addr);

#if defined(CONFIG_MACB) && defined(CONFIG_CMD_NET)
void at91sam9_eth_initialize(bd_t *bi)
{
 #ifdef CONFIG_RFCTHUNDERBOLT
	/* Try phy_addr = 0 (Thunderbolt REV02 and later) */
	if (macb_eth_initialize(0, (void *)AT91_BASE_EMAC, 0x00) == 0)
	{
		rfc_new_phy = 1;
	}
	/* If that failed, try phy_addr = 1F (Thunderbolt REV01) */
	else if (macb_eth_initialize(0, (void *)AT91_BASE_EMAC, 0x1F) == 0)
	{
		rfc_new_phy = 0;
	}
	else rfc_new_phy = -1;

 #else
	macb_eth_initialize(0, (void *)AT91_BASE_EMAC, 0x00);
 #endif
}
#endif
