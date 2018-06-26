/*
 * (C) Copyright 2009
 * RF Code, Inc.
 *
 * Based on
 * drivers/usb/usbdcore_omap1510.c
 * drivers/usb/gadget/at91_udc.c
 * 
 * AT91 USB device port driver
 * 	tested only with usbtty gadget driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 *
 */

#include <common.h>

#if defined(CONFIG_AT91SAM9G20) && defined(CONFIG_USB_DEVICE)

#include <asm/arch/at91sam9260.h>
#include <asm/arch/at91_pmc.h>
#include <asm/arch/gpio.h>
#include <asm/arch/io.h>

#include <asm/io.h>
#include <asm/errno.h>

#include "usbdcore.h"
#include "usbdcore_at91.h"
#include "usbdcore_ep0.h"

#define XUDCDBG(str)
#define XUDCDBGA(fmt,args...)

extern void serial_putu(unsigned val);

/* Some kind of debugging output... */
#if 1
#define UDCDBG(str)
#define UDCDBGA(fmt,args...)
#elif 1  /* Simple output to debug serial port */
#define UDCDBG(str) serial_puts(str "\n")
#define UDCDBGA(fmt,args...) serial_puts(fmt "\n")
#else  /* The bugs still exists... */
#define UDCDBG(str) serial_printf("[%s] %s:%d: " str "\n", __FILE__,__FUNCTION__,__LINE__)
#define UDCDBGA(fmt,args...) serial_printf("[%s] %s:%d: " fmt "\n", __FILE__,__FUNCTION__,__LINE__, ##args)
//#define UDCDBG(str) fprintf(stderr, "[%s] %s:%d: " str "\n", __FILE__,__FUNCTION__,__LINE__)
//#define UDCDBGA(fmt,args...) fprintf(stderr, "[%s] %s:%d: " fmt "\n", __FILE__,__FUNCTION__,__LINE__, ##args)
#endif

/* bits indicating OUT fifo has data ready */
#define	RX_DATA_READY	(AT91_UDP_RX_DATA_BK0 | AT91_UDP_RX_DATA_BK1)

/*
 * Endpoint FIFO CSR bits have a mix of bits, making it unsafe to just write
 * back most of the value you just read (because of side effects, including
 * bits that may change after reading and before writing).
 *
 * Except when changing a specific bit, always write values which:
 *  - clear SET_FX bits (setting them could change something)
 *  - set CLR_FX bits (clearing them could change something)
 *
 * There are also state bits like FORCESTALL, EPEDS, DIR, and EPTYPE
 * that shouldn't normally be changed.
 *
 * NOTE at91sam9260 docs mention synch between UDPCK and MCK clock domains,
 * implying a need to wait for one write to complete (test relevant bits)
 * before starting the next write.  This shouldn't be an issue given how
 * infrequently we write, except maybe for write-then-read idioms.
 */
#define	SET_FX	(AT91_UDP_TXPKTRDY)
#define	CLR_FX	(RX_DATA_READY | AT91_UDP_RXSETUP \
		| AT91_UDP_STALLSENT | AT91_UDP_TXCOMP)

#define at91_udp_read(reg) \
	__raw_readl((unsigned *)(CFG_USBD_REGS_BASE + reg))
#define at91_udp_write(reg, val) \
	__raw_writel(val, (unsigned *)(CFG_USBD_REGS_BASE + reg))

#define	NUM_ENDPOINTS	6

static struct usb_device_instance *udc_device;
static struct urb *ep0_urb = NULL;
static int wait_for_addr_ack = 0;
static int wait_for_config_ack = 0;

struct at91_ep {
	struct usb_endpoint_instance	*ep_instance;
	void				*creg;
	void				*dreg;

	unsigned			is_pingpong:1;
	//unsigned			stopped:1;
	unsigned			is_in:1;
	//unsigned			is_iso:1;
	unsigned			fifo_bank:1;
};

struct at91_ep	priv_ep[NUM_ENDPOINTS];

/****************************************************************************/

/* find_at91_ep
 * 
 * Given a pointer to a usb_endpoint_instance struct this function
 * returns our corresponding at91_ep struct pointer.
 */
static struct at91_ep *find_at91_ep (struct usb_endpoint_instance *epinst)
{
	int i;

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		if (priv_ep[i].ep_instance == epinst)
			return &priv_ep[i];
	}
	return NULL;
}

/* at91_find_epinst
 * 
 * Given an endpoint address this function returns the corresponding 
 * usb_endpoint_instance struct pointer.
 */
//static struct usb_endpoint_instance *at91_find_epinst (int epaddr)
//{
	//int i;

	//for (i = 0; i < udc_device->bus->max_endpoints; i++) {
		//if (udc_device->bus->endpoint_array[i].endpoint_address == epaddr)
			//return &udc_device->bus->endpoint_array[i];
	//}
	//return NULL;
//}

/* udc_state_transition_up
 * udc_state_transition_down
 * udc_state_transition
 *
 * Helper functions to implement device state changes.	The device states and
 * the events that transition between them are:
 *
 *				STATE_CONFIGURED
 *				/\	||
 *				||	\/
 *	DEVICE_CONFIGURED			DEVICE_DE_CONFIGURED
 *				/\	||
 *				||	\/
 *				STATE_ADDRESSED
 *				/\	||
 *				||	\/
 *	DEVICE_ADDRESS_ASSIGNED			DEVICE_RESET
 *				/\	||
 *				||	\/
 *				STATE_DEFAULT
 *				/\	||
 *				||	\/
 *	DEVICE_RESET				DEVICE_POWER_INTERRUPTION
 *				/\	||
 *				||	\/
 *				STATE_POWERED
 *				/\	||
 *				||	\/
 *	DEVICE_HUB_CONFIGURED			DEVICE_HUB_RESET
 *				/\	||
 *				||	\/
 *				STATE_ATTACHED
 *
 * udc_state_transition_up transitions up (in the direction from STATE_ATTACHED
 * to STATE_CONFIGURED) from the specified initial state to the specified final
 * state, passing through each intermediate state on the way.  If the initial
 * state is at or above (i.e. nearer to STATE_CONFIGURED) the final state, then
 * no state transitions will take place.
 *
 * udc_state_transition_down transitions down (in the direction from
 * STATE_CONFIGURED to STATE_ATTACHED) from the specified initial state to the
 * specified final state, passing through each intermediate state on the way.
 * If the initial state is at or below (i.e. nearer to STATE_ATTACHED) the final
 * state, then no state transitions will take place.
 */
static void udc_state_transition_up (usb_device_state_t initial,
				     usb_device_state_t final)
{
	if (initial < final) {
		switch (initial) {
		case STATE_ATTACHED:
			usbd_device_event_irq (udc_device,
					       DEVICE_HUB_CONFIGURED, 0);
			if (final == STATE_POWERED)
				break;
		case STATE_POWERED:
			usbd_device_event_irq (udc_device, DEVICE_RESET, 0);
			if (final == STATE_DEFAULT)
				break;
		case STATE_DEFAULT:
			usbd_device_event_irq (udc_device,
					       DEVICE_ADDRESS_ASSIGNED, 0);
			if (final == STATE_ADDRESSED)
				break;
		case STATE_ADDRESSED:
			usbd_device_event_irq (udc_device, DEVICE_CONFIGURED, 0);
		case STATE_CONFIGURED:
			break;
		default:
			break;
		}
	}
}

static void udc_state_transition_down (usb_device_state_t initial,
				       usb_device_state_t final)
{
	if (initial > final) {
		switch (initial) {
		case STATE_CONFIGURED:
			usbd_device_event_irq (udc_device, DEVICE_DE_CONFIGURED, 0);
			if (final == STATE_ADDRESSED)
				break;
		case STATE_ADDRESSED:
			usbd_device_event_irq (udc_device, DEVICE_RESET, 0);
			if (final == STATE_DEFAULT)
				break;
		case STATE_DEFAULT:
			usbd_device_event_irq (udc_device, DEVICE_POWER_INTERRUPTION, 0);
			if (final == STATE_POWERED)
				break;
		case STATE_POWERED:
			usbd_device_event_irq (udc_device, DEVICE_HUB_RESET, 0);
		case STATE_ATTACHED:
			break;
		default:
			break;
		}
	}
}

static void udc_state_transition (usb_device_state_t state)
{
	usb_device_state_t current = udc_device->device_state;
	
	/* Is this upward movement? */
	if (state > current)
		udc_state_transition_up(current, state);
	/* Is this downward movement? */
	else if (state < current)
		udc_state_transition_down(current, state);
}

/* at91_stall_ep
 *
 */
static void at91_stall_ep (struct at91_ep *ep)
{
	u32 csr = __raw_readl(ep->creg);

	UDCDBGA("endpoint stall ep_addr 0x%02x", ep->ep_instance->endpoint_address);
	
	csr &= ~SET_FX;
	csr |= CLR_FX | AT91_UDP_FORCESTALL;
	__raw_writel(csr, ep->creg);
}

/* at91_write_noniso_tx_fifo
 *
 * If the endpoint has an active tx_urb, then the next packet of data from the
 * URB is written to the tx FIFO. The total amount of data in the urb is given
 * by urb->actual_length. The maximum amount of data that can be sent in any
 * one packet is given by endpoint->tx_packetSize. The number of data bytes
 * from this URB that have already been transmitted is given by endpoint->sent.
 * endpoint->last is updated by this routine with the number of data bytes
 * transmitted in this packet.
 */
static int at91_write_noniso_tx_fifo (struct at91_ep *ep)
{
	struct usb_endpoint_instance *endpoint = ep->ep_instance;
	struct urb *urb = endpoint->tx_urb;
	unsigned int last;
	u32 *creg;
	u32 csr;
	
	UDCDBGA ("urb->buffer %p, buffer_length %d, actual_length %d",
		 urb->buffer, urb->buffer_length, urb->actual_length);
		 
	/* If ep_queue() calls us, the queue is empty and possibly in
	 * odd states like TXCOMP not yet cleared (we do it, saving at
	 * least one IRQ) or the fifo not yet being free.  Those aren't
	 * issues normally (IRQ handler fast path).
	 */
	creg = ep->creg;
	csr = __raw_readl(creg);
	if (csr & AT91_UDP_TXCOMP) {
		csr |= CLR_FX;
		csr &= ~(SET_FX | AT91_UDP_TXCOMP);
		__raw_writel(csr, creg);
		csr = __raw_readl(creg);
	}
	/* If the FIFO is still in use, we'll have to try later */
	if (csr & AT91_UDP_TXPKTRDY) {
		UDCDBG ("failed: TXPKTRDY still set");
		return -1;
	 }
	/* How many bytes can we send at this time? */	 
	last = MIN (urb->actual_length - endpoint->sent, endpoint->tx_packetSize);
	if (last > 0) {
		u8 *cp = urb->buffer + endpoint->sent;

		UDCDBGA ("endpoint->sent %d, tx_packetSize %d, last %d", endpoint->sent, endpoint->tx_packetSize, last);
		/* Push the bytes into the FIFO */
		outsb(ep->dreg, cp, last);
	}
	endpoint->last = last;
	
	// Set TXPKTRDY each time (allows zero-length-packets too)
	csr &= ~SET_FX;
	csr |= CLR_FX | AT91_UDP_TXPKTRDY;
	__raw_writel(csr, creg);
	return 0;
}

/* at91_read_noniso_rx_fifo
 *
 * If the endpoint has an active rcv_urb, then the next packet of data is read
 * from the rcv FIFO and written to rcv_urb->buffer at offset
 * rcv_urb->actual_length to append the packet data to the data from any
 * previous packets for this transfer.
 *
 * The return value is the number of bytes read from the FIFO for this packet.
 */
static int at91_read_noniso_rx_fifo (struct at91_ep *ep)
{
	struct usb_endpoint_instance *endpoint = ep->ep_instance;
	struct urb *urb = endpoint->rcv_urb;
	u32 *creg = ep->creg;
	u32 csr;
	int count, bufferspace;

	csr = __raw_readl(creg);
	if ((csr & RX_DATA_READY) == 0) return 0; /* nothing to read */

	count = (csr & AT91_UDP_RXBYTECNT) >> 16;
	bufferspace = urb->buffer_length - urb->actual_length;
	
	if (count > bufferspace) {
		usberr("buffer overflow ep 0x%02x", endpoint->endpoint_address);
		count = bufferspace;
	}
	
	if (count) {
		unsigned char *cp = urb->buffer + urb->actual_length;
		/* Read the bytes out of the FIFO and into the buffer */
		insb (ep->dreg, cp, count);
	}

	/* release and swap pingpong mem bank */
	csr |= CLR_FX;
	if (ep->is_pingpong) {
		if (ep->fifo_bank == 0) {
			csr &= ~(SET_FX | AT91_UDP_RX_DATA_BK0);
			ep->fifo_bank = 1;
		} else {
			csr &= ~(SET_FX | AT91_UDP_RX_DATA_BK1);
			ep->fifo_bank = 0;
		}
	} else
		csr &= ~(SET_FX | AT91_UDP_RX_DATA_BK0);
	__raw_writel(csr, creg);

	/* ??? avoid extra trips through IRQ logic for packets already in
	 * the fifo ... maybe preventing an extra (expensive) OUT-NAK
	if (is_done)
		done(ep, req, 0);
	else if (ep->is_pingpong) {
		bufferspace -= count;
		buf += count;
		goto rescan;
	}*/

	return count;
}

/* 
 * at91_udc_handle_ep
 * 
 * This function handles the RX/TX interrupt flag processing for
 * an endpoint
 */
static void at91_udc_handle_ep (struct at91_ep	*ep)
{
	struct usb_endpoint_instance *endpoint = ep->ep_instance;
	u32 *creg = ep->creg;
	u32 csr = __raw_readl(creg);

	UDCDBGA("%s Ep, csr=0x%08X", ep->is_in ? "IN" : "OUT", csr);

	if (ep->is_in) {
		int datasent = 0;
		if (csr & (AT91_UDP_STALLSENT | AT91_UDP_TXCOMP)) {
			/* We need to transmit a terminating zero-length packet now if
			 * we have sent all of the data in this URB and the transfer
			 * size was an exact multiple of the packet size.
			 */
			if (endpoint->tx_urb
			    && (endpoint->last == endpoint->tx_packetSize)
			    && (endpoint->tx_urb->actual_length - endpoint->sent -
				endpoint->last == 0)) {
				/* Prepare to transmit a zero-length packet. */
				endpoint->sent += endpoint->last;
				/* write 0 bytes of data to FIFO */
				//at91_write_noniso_tx_fifo (ep);
				csr |= CLR_FX | AT91_UDP_TXPKTRDY;
				csr &= ~(AT91_UDP_STALLSENT | AT91_UDP_TXCOMP);
				__raw_writel(csr, creg);
				datasent = 1;
				UDCDBG("Sending ZLP");
				endpoint->last = 0;
			} else if (endpoint->tx_urb
				   && endpoint->tx_urb->actual_length) {
				/* retire the data that was previously sent */
				usbd_tx_complete (endpoint);
				UDCDBG("usbd_tx_complete()");
				/* Check to see if we still have more data */
				if (endpoint->tx_urb
				    && endpoint->tx_urb->actual_length) {
					/* write data to FIFO */
					at91_write_noniso_tx_fifo (ep);
					datasent = 1;
					UDCDBG("more data to TX");
				}
			}
			
			/* at91_write_noniso_tx_fifo updates CSR but if we didn't
			 * call it, we will update CSR ourselves */
			if (!datasent) {
				csr |= CLR_FX;
				csr &= ~(SET_FX | AT91_UDP_STALLSENT | AT91_UDP_TXCOMP);
				__raw_writel(csr, creg);
			}
		}
	} else { /* Out EP */
		if (csr & AT91_UDP_STALLSENT) {
			/* STALLSENT bit == ISOERR */
			//if (ep->is_iso && req)
			//	req->req.status = -EILSEQ;
			csr |= CLR_FX;
			csr &= ~(SET_FX | AT91_UDP_STALLSENT);
			__raw_writel(csr, creg);
			// Read it back before proceeding
			csr = __raw_readl(creg);
		}
		if (csr & RX_DATA_READY) {
			unsigned nbytes;
			/* Read the data that is there */
			nbytes = at91_read_noniso_rx_fifo (ep);
			usbd_rcv_complete (endpoint, nbytes, 0);
		}
	}
}

/* at91_prepare_for_control_write_status
 *
 * This function just snoops on the setup packet for certain device
 * requests that we need to know about and act upon.
 */
static void at91_prepare_for_control_write_status (struct urb *urb)
{
	struct usb_device_request *request = &urb->device_request;
	unsigned tmp;

	/* Is this an out-directed, standard request targeted at our device? */
	if (request->bmRequestType == (USB_DIR_OUT|USB_TYPE_STANDARD|USB_RECIP_DEVICE))
	{
		/* check for a SET_ADDRESS request */
		if (request->bRequest == USB_REQ_SET_ADDRESS) {
			wait_for_addr_ack = 1;
			UDCDBG("wait for address ack");
			/* FADDR is set later, when we ack host STATUS */
		}
		/* check for a SET_CONFIGURATION request */
		else if (request->bRequest == USB_REQ_SET_CONFIGURATION) {
			tmp = at91_udp_read(AT91_UDP_GLB_STAT) & AT91_UDP_CONFG;
			
			/* If the config value is non-zero */
			if (request->wValue) {
				/* and we're currently not configured */
				wait_for_config_ack = (tmp == 0);
			} else { /* Or the config value is zero */
				/* and we currently are configured */
				wait_for_config_ack = (tmp != 0);
			}
			if (wait_for_config_ack)
				UDCDBG("wait for config ack");
			/* CONFG is toggled after status stage */
		}
	}
}

/* Handle address ACK received from host
 * 
 */
static void handle_addr_ack (void)
{
	u8 address = udc_device->address;
	u32 tmp = at91_udp_read(AT91_UDP_GLB_STAT);

	wait_for_addr_ack = 0;
	
	/* Write our address and the enable bit in the UDP_FEN register */
	at91_udp_write(AT91_UDP_FADDR, AT91_UDP_FEN | address);
	if (address)
		tmp |= AT91_UDP_FADDEN;
	else tmp &= ~AT91_UDP_FADDEN;
	at91_udp_write(AT91_UDP_GLB_STAT, tmp);

	/* Transition device state */
	if (address)
		udc_state_transition(STATE_ADDRESSED);
	else    udc_state_transition(STATE_DEFAULT);
	
	UDCDBGA("address=%d", address);
}

/* Handle config ACK received from host
 * 
 */
static void handle_config_ack (void)
{
	u32 tmp = at91_udp_read(AT91_UDP_GLB_STAT);
	tmp ^= AT91_UDP_CONFG;
	at91_udp_write(AT91_UDP_GLB_STAT, tmp);
	wait_for_config_ack = 0;

	/* Transition device state */
	if (tmp & AT91_UDP_CONFG)
		udc_state_transition(STATE_CONFIGURED);
	else	udc_state_transition(STATE_ADDRESSED);
	
	UDCDBGA("config=%d", (tmp & AT91_UDP_CONFG)?1:0);
}

/* Handle SETUP USB interrupt.
 * 
 */
static void at91_udc_setup (void)
{
	struct at91_ep *ep0 = &priv_ep[0];
	struct usb_endpoint_instance *endpoint = ep0->ep_instance;
	u32 *creg = ep0->creg;
	unsigned rxcount;
	u32 csr;
	int status = 0;

	/* read and ack SETUP; hard-fail for bogus packets */
	csr = __raw_readl(creg);
	rxcount = (csr & AT91_UDP_RXBYTECNT) >> 16;
	if (rxcount == 8) {
		unsigned char *datap =
			(unsigned char *) &ep0_urb->device_request;
			
		/* Copy setup packet from fifo to device_request field */
		insb (ep0->dreg, datap, rxcount);

		/* Check direction */
		if ((ep0_urb->device_request.bmRequestType & USB_REQ_DIRECTION_MASK)
		    == USB_REQ_HOST2DEVICE) {
			/* control write request */
			csr &= ~AT91_UDP_DIR;
			ep0->is_in = 0;
		} else {
			/* control read request */
			csr |= AT91_UDP_DIR;
			ep0->is_in = 1;
		}
		
		XUDCDBGA ("EP0 setup %s [%x %x %x %x %x %x %x %x]",
			 ep0->is_in ? "read" : "write",
			 *(datap + 0), *(datap + 1), *(datap + 2),
			 *(datap + 3), *(datap + 4), *(datap + 5),
			 *(datap + 6), *(datap + 7));
	} else {
		usberr("SETUP len %d, csr %08x", rxcount, csr);
		status = -EINVAL;
	}
	csr |= CLR_FX;
	csr &= ~(SET_FX | AT91_UDP_RXSETUP);
	__raw_writel(csr, creg);
	
	/* I've seen conditions (Win7, reset button used to enter uboot)
	 * where the EP0 TXCOMP interrupt doesn't seem to fire for the
	 * ZLP that we send for the Status IN stage on a SET_CONFIGURATION
	 * request. Therefore we detect a missed TXCOMP here incase we 
	 * were still waiting for it.					*/
	if (wait_for_addr_ack)
	{
		handle_addr_ack();
	}
	if (wait_for_config_ack)
	{
		handle_config_ack();
	}
	
	if (status != 0) {
		at91_stall_ep(ep0);
		return;
	}

	/* Try to process setup packet */
	if (ep0_recv_setup (ep0_urb)) {
		/* Not a valid setup packet, stall EP0 */
		at91_stall_ep(ep0);
		UDCDBG ("can't parse setup packet, still waiting for setup");
		return;
	}

	/* IN data stage (control read request) */
	if (ep0->is_in) {
		/* The ep0_recv_setup function has already placed our response
		 * packet data in ep0_urb->buffer and the packet length in
		 * ep0_urb->actual_length.
		 */
		endpoint->tx_urb = ep0_urb;
		endpoint->sent = 0;
		
		at91_write_noniso_tx_fifo (ep0);
	}
	/* OUT data stage (control write request) */
	else {
		/* Does this have data included in the data stage? */
		if (le16_to_cpu (ep0_urb->device_request.wLength)) {
			/* Prepare to receive it in ep0_urb */
			endpoint->rcv_urb = ep0_urb;
			ep0_urb->actual_length = 0;
			
			/* Currently there is no mechanism for the gadget
			 * driver to see the returned data */
		} else {
			/* Watch for certain device requests we care about */
			at91_prepare_for_control_write_status (ep0_urb);
		
			/* Send ZLP as ACK */
			XUDCDBG ("EP0 ZLP Tx (ACK of control write request)");
			csr &= ~SET_FX;
			csr |= CLR_FX | AT91_UDP_TXPKTRDY;
			__raw_writel(csr, creg);
		}
	}
	XUDCDBG ("...leaving");
}

/* Handle endpoint 0 TXCOMP flag indicating the host ACK'd an IN packet
 * we just sent
 */
static void at91_udc_ep0_tx (void)
{
	struct at91_ep *ep0 = &priv_ep[0];
	struct usb_endpoint_instance *endpoint = ep0->ep_instance;
	struct usb_device_request *request = &ep0_urb->device_request;
	u32 *creg = ep0->creg;
	u32 csr = __raw_readl(creg);	

	XUDCDBG ("TXCOMP on EP0");
	
	/* IN packet was ACK'd by the host, do we need to send another
	 * packet, a ZLP, finish SET_ADDRESS/SET_CONFIG or do nothing? */
	csr |= CLR_FX;
	csr &= ~(SET_FX | AT91_UDP_TXCOMP);
	
	/* Check direction */
	if (ep0->is_in) {
		/* This tx interrupt must be for a control read data
		 * stage packet.
		 */
		int wLength = le16_to_cpu (request->wLength);

		/* Update our count of bytes sent so far in this
		 * transfer.
		 */
		endpoint->sent += endpoint->last;

		/* We are finished with this transfer if we have sent
		 * all of the bytes in our tx urb (urb->actual_length)
		 * unless we need a zero-length terminating packet.  We
		 * need a zero-length terminating packet if we returned
		 * fewer bytes than were requested (wLength) by the host,
		 * and the number of bytes we returned is an exact
		 * multiple of the packet size endpoint->tx_packetSize.
		 */
		if ((endpoint->sent == ep0_urb->actual_length)
		    && ((ep0_urb->actual_length == wLength)
			|| (endpoint->last !=
			    endpoint->tx_packetSize))) {
			/* Done with control read data stage. */
			XUDCDBG ("control read data stage complete");
			/* Just clear flag */
			__raw_writel(csr, creg);
		} else {
			/* We still have another packet of data to send
			 * in this control read data stage or else we
			 * need a zero-length terminating packet.
			 */
			XUDCDBG ("control read data stage continue or ZLP");
			at91_write_noniso_tx_fifo (ep0);
		}
		
	} else {	// out direction
		u32 tmp;

		/* This tx interrupt must be for a control write status
		 * stage packet we just sent.
		 */
		XUDCDBG ("ACK on EP0 control write status stage packet");
		
		/* Just clear flag */
		__raw_writel(csr, creg);

		/* SET_ADDRESS takes effect now if we were waiting 
		 * for its status stage to complete */
		if (wait_for_addr_ack)
		{
			handle_addr_ack();
		}
		/* SET_CONFIG takes effect now if we were waiting 
		 * for its status stage to complete */
		else if (wait_for_config_ack)
		{
			handle_config_ack();
		}
	}
}

/* Handle endpoint 0 RX_DATA flag
 * 
 */
static void at91_udc_ep0_rx (void)
{
	struct at91_ep *ep0 = &priv_ep[0];
	struct usb_endpoint_instance *endpoint = ep0->ep_instance;
	u32 *creg = ep0->creg;
	u32 csr = __raw_readl(creg);
	unsigned nbytes;

	XUDCDBG ("RX on EP0");

	/* OUT packet arrived ... */
	csr |= CLR_FX;
	csr &= ~(SET_FX | AT91_UDP_RX_DATA_BK0);

	/* STATUS stage for control-IN (ack).  */
	if (ep0->is_in) {
		XUDCDBG("ep0 IN ACK'd");
	/* OUT DATA stage */
	} else {
		struct usb_device_request *request = &ep0_urb->device_request;
		
		/* Read the data that is there */
		nbytes = at91_read_noniso_rx_fifo (ep0);
		usbd_rcv_complete (endpoint, nbytes, 0);
		
		/* If we received all the data for the request */	
		if (ep0_urb->actual_length == request->wLength)
		{
			// Data stage is done
			csr |= CLR_FX | AT91_UDP_TXPKTRDY;
			XUDCDBG("sending ep0 ACK for out data stage");
		}
	}
	__raw_writel(csr, creg);
}

/* handle_ep0
 * 
 * This function handles the interrupt flag processing for EP0
 */
static void handle_ep0(void)
{
	struct at91_ep *ep0 = &priv_ep[0];
	u32 *creg = ep0->creg;
	u32 csr = __raw_readl(creg);

	/* Stall was sent? Clear the flag */
	if (csr & AT91_UDP_STALLSENT) {
		csr |= CLR_FX;
		csr &= ~(SET_FX | AT91_UDP_STALLSENT | AT91_UDP_FORCESTALL);
		__raw_writel(csr, creg);
		UDCDBG("ep0 stalled");
		csr = __raw_readl(creg);
	}
	/* Setup packet received? */
	if (csr & AT91_UDP_RXSETUP) {
		at91_udc_setup();
		return;
	}

	/* host ACKed an IN packet that we sent */
	if (csr & AT91_UDP_TXCOMP) {
		at91_udc_ep0_tx();
	}
	/* OUT packet arrived */
	if (csr & AT91_UDP_RX_DATA_BK0) {
		at91_udc_ep0_rx();
	}
}

/*----------------------------------------------------------------------------
 * Start of public functions.
 *---------------------------------------------------------------------------*/

/*
 * Return the status of the VBUS sense input
 */
int udc_vbus_active (void)
{
#ifdef CONFIG_USBD_VBUS_SENSE_PIN
	return at91_get_gpio_value(CONFIG_USBD_VBUS_SENSE_PIN);
#else
	/* No vbus sense, always assume vbus is present */
	return 1;
#endif
}

/*
 * udc_irq - polled interrupts
 */
void udc_irq(void)
{
	int rescans = 10;
	
	while (rescans--) {
		u32 status = at91_udp_read(AT91_UDP_ISR) & ~AT91_UDP_SOFINT;

		if (!status) break;

		XUDCDBGA("IRQ status = 0x%08X", status);
		
		/* USB reset irq:  not maskable */
		if (status & AT91_UDP_ENDBUSRES) {
			at91_udp_write(AT91_UDP_ICR, AT91_UDP_ENDBUSRES);
			udc_state_transition(STATE_DEFAULT);
			
			/* Despite spec sheet, we have to ensure EP0 is 
			 * enabled following a reset */
			__raw_writel(AT91_UDP_EPEDS | AT91_UDP_EPTYPE_CTRL,
				priv_ep[0].creg);
			wait_for_addr_ack = 0;
			wait_for_config_ack = 0;
			UDCDBG("end bus reset");

		/* host initiated suspend (3+ms bus idle) */
		} else if (status & AT91_UDP_RXSUSP) {
			at91_udp_write(AT91_UDP_ICR, AT91_UDP_RXSUSP);
			UDCDBG("bus suspend");

		/* host initiated resume */
		} else if (status & AT91_UDP_RXRSM) {
			at91_udp_write(AT91_UDP_ICR, AT91_UDP_RXRSM);
			UDCDBG("bus resume");

		/* endpoint IRQs are cleared by handling them */
		} else {
			int		i;
			unsigned	mask = 1;
			struct at91_ep	*ep = &priv_ep[1];

			if (status & mask)
				handle_ep0();
			for (i = 1; i < NUM_ENDPOINTS; i++) {
				mask <<= 1;
				if (status & mask)
					at91_udc_handle_ep(ep);
				ep++;
			}
		}
	}
}

/* Called to start packet transmission. */
int udc_endpoint_write (struct usb_endpoint_instance *endpoint)
{
	struct at91_ep *ep = find_at91_ep(endpoint);

	UDCDBGA ("Starting transmit on ep %d", endpoint->endpoint_address & USB_ENDPOINT_NUMBER_MASK);

	if (ep && endpoint->tx_urb) {
		/* write data to FIFO */
		return at91_write_noniso_tx_fifo (ep);
	}
	return -1;
}

/*
 * udc_setup_ep - setup endpoint
 *
 * Associate a physical endpoint with endpoint_instance
 */
void udc_setup_ep (struct usb_device_instance *device,
		   unsigned int epnum, struct usb_endpoint_instance *endpoint)
{
	struct at91_ep *ep;
	u32 *creg;
	int ep_addr;
	int packet_size;
	int attributes;
	
	// This gets called for ep0 then later it is called for
	// each ep > 0 and again when state -> DEVICE_ADDRESS_ASSIGNED
	
	UDCDBGA ("setting up endpoint addr 0x%02x", endpoint->endpoint_address);

	/* Allocate a urb for our own use with EP0 transactions */
	if (epnum == 0)
	{
		if (!ep0_urb) {
			ep0_urb = usbd_alloc_urb (device, endpoint);
		} else {
			usberr ("ep0_urb already allocated %p", ep0_urb);
		}
	}

	if (epnum >= NUM_ENDPOINTS) {
		usberr ("ep %d out of range", epnum);
		return;
	}
	ep = &priv_ep[epnum];
	creg = ep->creg;

	/* Remember this endpoint instance */
	ep->ep_instance = endpoint;
	
	ep_addr = endpoint->endpoint_address;
	if ((ep_addr & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
		/* IN endpoint */
		packet_size = endpoint->tx_packetSize;
		attributes = endpoint->tx_attributes;
		ep->is_in = 1;
	} else {
		/* OUT endpoint */
		packet_size = endpoint->rcv_packetSize;
		attributes = endpoint->rcv_attributes;
		ep->is_in = 0;
	}
	
	switch (attributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_CONTROL:
		if (epnum) {
			/* Only EP0 is allowed as control endpoint */
			UDCDBGA ("ep %d as control ep not allowed", epnum);
			return;
		}
		if ((packet_size != 8) && (packet_size != 16) && 
		    (packet_size != 32) && (packet_size != 64)) {
			UDCDBGA ("packet size %d not valid for control ep", packet_size);
			return;
		}
		/* enable ep0 */
		UDCDBG ("ep0 control endpoint enabled");
		__raw_writel(AT91_UDP_EPEDS | AT91_UDP_EPTYPE_CTRL, creg);
		break;
	case USB_ENDPOINT_XFER_BULK:
		if ((packet_size != 8) && (packet_size != 16) && 
		    (packet_size != 32) && (packet_size != 64)) {
			UDCDBGA ("packet size %d not valid for bulk ep %d", packet_size, epnum);
			return;
		}
		if ((ep_addr & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
			__raw_writel(AT91_UDP_EPEDS | AT91_UDP_EPTYPE_BULK_IN, creg);
			UDCDBGA ("ep%d bulk IN endpoint enabled", epnum);
		}
		else {
			__raw_writel(AT91_UDP_EPEDS | AT91_UDP_EPTYPE_BULK_OUT, creg);
			UDCDBGA ("ep%d bulk OUT endpoint enabled", epnum);
		}
		break;
	case USB_ENDPOINT_XFER_INT:
		if (packet_size > 64) {
			UDCDBGA ("packet size %d not valid for interrupt ep %d", packet_size, epnum);
			return;
		}
		if ((ep_addr & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
			__raw_writel(AT91_UDP_EPEDS | AT91_UDP_EPTYPE_INT_IN, creg);
			UDCDBGA ("ep%d interrupt IN endpoint enabled", epnum);
		}
		else {
			__raw_writel(AT91_UDP_EPEDS | AT91_UDP_EPTYPE_INT_OUT, creg);
			UDCDBGA ("ep%d interrupt OUT endpoint enabled", epnum);
		}
		break;
	default:
		UDCDBGA ("ep %d transfer type not supported", epnum);
		return;
	}
	/* reset/init endpoint fifo.  NOTE:  leaves fifo_bank alone,
	 * since endpoint resets don't reset hw pingpong state.
	 */
	at91_udp_write(AT91_UDP_RST_EP, 1 << epnum);
	at91_udp_write(AT91_UDP_RST_EP, 0);
}

/* Start to initialize h/w stuff */
int udc_init (void)
{
	int i;

	UDCDBG ("starting");
	
	/* Enable 48MHz clock to UDP system */
	at91_sys_write(AT91_PMC_SCER, AT91SAM926x_PMC_UDP);
	/* Enable Peripheral clock to UDP system */
	at91_sys_write(AT91_PMC_PCER, 1 << AT91SAM9260_ID_UDP);
	
	printf ("USB:   AT91 USB function module\n");

	// Disable and clear all interrupts
	at91_udp_write(AT91_UDP_IDR, 0xffffffff);
	at91_udp_write(AT91_UDP_ICR, 0xffffffff);
	
	/* Enable transceiver, pull-up off */
	at91_udp_write(AT91_UDP_TXVC, 0);
	
	/* Handle the static initialization of our private ep structs */
	for (i=0; i<NUM_ENDPOINTS; i++)
	{
		struct at91_ep *ep = &priv_ep[i];
		
		memset(ep, 0, sizeof(struct at91_ep));
		
		ep->creg = (void *)CFG_USBD_REGS_BASE + AT91_UDP_CSR(i);
		ep->dreg = (void *)CFG_USBD_REGS_BASE + AT91_UDP_FDR(i);
		/* The following endpoints are dual-buffered (pingpong) */
		if ((i==1) || (i==2) || (i==4) || (i==5))
			ep->is_pingpong = 1;
	}
	return 0;
}

/*
 * udc_startup - allow udc code to do any additional startup
 */
void udc_startup_events (struct usb_device_instance *device)
{
	UDCDBG ("starting");
	
	/* Save the device structure pointer */
	udc_device = device;
	
	/* The DEVICE_INIT event puts the USB device in STATE_INIT */
	usbd_device_event_irq (device, DEVICE_INIT, 0);

	/* The DEVICE_CREATE event puts the USB device in STATE_ATTACHED */
	usbd_device_event_irq (device, DEVICE_CREATE, 0);
}

/* Turn on the USB connection by enabling the pullup resistor */
void udc_connect (void)
{
	u32	txvc = at91_udp_read(AT91_UDP_TXVC);
	
	/* Delay before enabling USB to ensure that we have
	 * adequate usb-disconnect time in reboot scenario */
	udelay(2000000);
	UDCDBG ("connect, enable pull-up");
	txvc |= AT91_UDP_TXVC_PUON;
	at91_udp_write(AT91_UDP_TXVC, txvc);
}

/* Turn off the USB connection by disabling the pullup resistor */
void udc_disconnect (void)
{
	u32	txvc = at91_udp_read(AT91_UDP_TXVC);
	
	UDCDBG ("disconnect, disable pull-up");
	txvc &= ~AT91_UDP_TXVC_PUON;
	at91_udp_write(AT91_UDP_TXVC, txvc);
}

/* Flow control */
void udc_set_nak(int epid)
{
	
}

void udc_unset_nak (int epid)
{
	
}

#endif
