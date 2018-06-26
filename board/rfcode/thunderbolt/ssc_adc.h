/*
 * (C) Copyright 2009 RF Code
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

/* SSC Base Address */
#define SSC_BASE			0xFFFBC000

/* SSC register offsets */

/* SSC Control Register */
#define SSC_CR				0x00000000
#define SSC_CR_RXDIS_SIZE			 1
#define SSC_CR_RXDIS_OFFSET			 1
#define SSC_CR_RXEN_SIZE			 1
#define SSC_CR_RXEN_OFFSET			 0
#define SSC_CR_SWRST_SIZE			 1
#define SSC_CR_SWRST_OFFSET			15
#define SSC_CR_TXDIS_SIZE			 1
#define SSC_CR_TXDIS_OFFSET			 9
#define SSC_CR_TXEN_SIZE			 1
#define SSC_CR_TXEN_OFFSET			 8

/* SSC Clock Mode Register */
#define SSC_CMR				0x00000004
#define SSC_CMR_DIV_SIZE			12
#define SSC_CMR_DIV_OFFSET			 0

/* SSC Receive Clock Mode Register */
#define SSC_RCMR			0x00000010
#define SSC_RCMR_CKG_SIZE			 2
#define SSC_RCMR_CKG_OFFSET			 6
#define SSC_RCMR_CKI_SIZE			 1
#define SSC_RCMR_CKI_OFFSET			 5
#define SSC_RCMR_CKO_SIZE			 3
#define SSC_RCMR_CKO_OFFSET			 2
#define SSC_RCMR_CKS_SIZE			 2
#define SSC_RCMR_CKS_OFFSET			 0
#define SSC_RCMR_PERIOD_SIZE			 8
#define SSC_RCMR_PERIOD_OFFSET			24
#define SSC_RCMR_START_SIZE			 4
#define SSC_RCMR_START_OFFSET			 8
#define SSC_RCMR_STOP_SIZE			 1
#define SSC_RCMR_STOP_OFFSET			12
#define SSC_RCMR_STTDLY_SIZE			 8
#define SSC_RCMR_STTDLY_OFFSET			16

/* SSC Receive Frame Mode Register */
#define SSC_RFMR			0x00000014
#define SSC_RFMR_DATLEN_SIZE			 5
#define SSC_RFMR_DATLEN_OFFSET			 0
#define SSC_RFMR_DATNB_SIZE			 4
#define SSC_RFMR_DATNB_OFFSET			 8
#define SSC_RFMR_FSEDGE_SIZE			 1
#define SSC_RFMR_FSEDGE_OFFSET			24
#define SSC_RFMR_FSLEN_SIZE			 4
#define SSC_RFMR_FSLEN_OFFSET			16
#define SSC_RFMR_FSOS_SIZE			 4
#define SSC_RFMR_FSOS_OFFSET			20
#define SSC_RFMR_LOOP_SIZE			 1
#define SSC_RFMR_LOOP_OFFSET			 5
#define SSC_RFMR_MSBF_SIZE			 1
#define SSC_RFMR_MSBF_OFFSET			 7

/* SSC Transmit Clock Mode Register */
#define SSC_TCMR			0x00000018
#define SSC_TCMR_CKG_SIZE			 2
#define SSC_TCMR_CKG_OFFSET			 6
#define SSC_TCMR_CKI_SIZE			 1
#define SSC_TCMR_CKI_OFFSET			 5
#define SSC_TCMR_CKO_SIZE			 3
#define SSC_TCMR_CKO_OFFSET			 2
#define SSC_TCMR_CKS_SIZE			 2
#define SSC_TCMR_CKS_OFFSET			 0
#define SSC_TCMR_PERIOD_SIZE			 8
#define SSC_TCMR_PERIOD_OFFSET			24
#define SSC_TCMR_START_SIZE			 4
#define SSC_TCMR_START_OFFSET			 8
#define SSC_TCMR_STTDLY_SIZE			 8
#define SSC_TCMR_STTDLY_OFFSET			16

/* SSC Transmit Frame Mode Register */
#define SSC_TFMR			0x0000001c
#define SSC_TFMR_DATDEF_SIZE			 1
#define SSC_TFMR_DATDEF_OFFSET			 5
#define SSC_TFMR_DATLEN_SIZE			 5
#define SSC_TFMR_DATLEN_OFFSET			 0
#define SSC_TFMR_DATNB_SIZE			 4
#define SSC_TFMR_DATNB_OFFSET			 8
#define SSC_TFMR_FSDEN_SIZE			 1
#define SSC_TFMR_FSDEN_OFFSET			23
#define SSC_TFMR_FSEDGE_SIZE			 1
#define SSC_TFMR_FSEDGE_OFFSET			24
#define SSC_TFMR_FSLEN_SIZE			 4
#define SSC_TFMR_FSLEN_OFFSET			16
#define SSC_TFMR_FSOS_SIZE			 3
#define SSC_TFMR_FSOS_OFFSET			20
#define SSC_TFMR_MSBF_SIZE			 1
#define SSC_TFMR_MSBF_OFFSET			 7

/* SSC Receive Hold Register */
#define SSC_RHR				0x00000020
#define SSC_RHR_RDAT_SIZE			32
#define SSC_RHR_RDAT_OFFSET			 0

/* SSC Transmit Hold Register */
#define SSC_THR				0x00000024
#define SSC_THR_TDAT_SIZE			32
#define SSC_THR_TDAT_OFFSET			 0

/* SSC Receive Sync. Holding Register */
#define SSC_RSHR			0x00000030
#define SSC_RSHR_RSDAT_SIZE			16
#define SSC_RSHR_RSDAT_OFFSET			 0

/* SSC Transmit Sync. Holding Register */
#define SSC_TSHR			0x00000034
#define SSC_TSHR_TSDAT_SIZE			16
#define SSC_TSHR_RSDAT_OFFSET			 0

/* SSC Receive Compare 0 Register */
#define SSC_RC0R			0x00000038
#define SSC_RC0R_CP0_SIZE			16
#define SSC_RC0R_CP0_OFFSET			 0

/* SSC Receive Compare 1 Register */
#define SSC_RC1R			0x0000003c
#define SSC_RC1R_CP1_SIZE			16
#define SSC_RC1R_CP1_OFFSET			 0

/* SSC Status Register */
#define SSC_SR				0x00000040
#define SSC_SR_CP0_SIZE				 1
#define SSC_SR_CP0_OFFSET			 8
#define SSC_SR_CP1_SIZE				 1
#define SSC_SR_CP1_OFFSET			 9
#define SSC_SR_ENDRX_SIZE			 1
#define SSC_SR_ENDRX_OFFSET			 6
#define SSC_SR_ENDTX_SIZE			 1
#define SSC_SR_ENDTX_OFFSET			 2
#define SSC_SR_OVRUN_SIZE			 1
#define SSC_SR_OVRUN_OFFSET			 5
#define SSC_SR_RXBUFF_SIZE			 1
#define SSC_SR_RXBUFF_OFFSET			 7
#define SSC_SR_RXEN_SIZE			 1
#define SSC_SR_RXEN_OFFSET			17
#define SSC_SR_RXRDY_SIZE			 1
#define SSC_SR_RXRDY_OFFSET			 4
#define SSC_SR_RXSYN_SIZE			 1
#define SSC_SR_RXSYN_OFFSET			11
#define SSC_SR_TXBUFE_SIZE			 1
#define SSC_SR_TXBUFE_OFFSET			 3
#define SSC_SR_TXEMPTY_SIZE			 1
#define SSC_SR_TXEMPTY_OFFSET			 1
#define SSC_SR_TXEN_SIZE			 1
#define SSC_SR_TXEN_OFFSET			16
#define SSC_SR_TXRDY_SIZE			 1
#define SSC_SR_TXRDY_OFFSET			 0
#define SSC_SR_TXSYN_SIZE			 1
#define SSC_SR_TXSYN_OFFSET			10

/* SSC Interrupt Enable Register */
#define SSC_IER				0x00000044
#define SSC_IER_CP0_SIZE			 1
#define SSC_IER_CP0_OFFSET			 8
#define SSC_IER_CP1_SIZE			 1
#define SSC_IER_CP1_OFFSET			 9
#define SSC_IER_ENDRX_SIZE			 1
#define SSC_IER_ENDRX_OFFSET			 6
#define SSC_IER_ENDTX_SIZE			 1
#define SSC_IER_ENDTX_OFFSET			 2
#define SSC_IER_OVRUN_SIZE			 1
#define SSC_IER_OVRUN_OFFSET			 5
#define SSC_IER_RXBUFF_SIZE			 1
#define SSC_IER_RXBUFF_OFFSET			 7
#define SSC_IER_RXRDY_SIZE			 1
#define SSC_IER_RXRDY_OFFSET			 4
#define SSC_IER_RXSYN_SIZE			 1
#define SSC_IER_RXSYN_OFFSET			11
#define SSC_IER_TXBUFE_SIZE			 1
#define SSC_IER_TXBUFE_OFFSET			 3
#define SSC_IER_TXEMPTY_SIZE			 1
#define SSC_IER_TXEMPTY_OFFSET			 1
#define SSC_IER_TXRDY_SIZE			 1
#define SSC_IER_TXRDY_OFFSET			 0
#define SSC_IER_TXSYN_SIZE			 1
#define SSC_IER_TXSYN_OFFSET			10

/* SSC Interrupt Disable Register */
#define SSC_IDR				0x00000048
#define SSC_IDR_CP0_SIZE			 1
#define SSC_IDR_CP0_OFFSET			 8
#define SSC_IDR_CP1_SIZE			 1
#define SSC_IDR_CP1_OFFSET			 9
#define SSC_IDR_ENDRX_SIZE			 1
#define SSC_IDR_ENDRX_OFFSET			 6
#define SSC_IDR_ENDTX_SIZE			 1
#define SSC_IDR_ENDTX_OFFSET			 2
#define SSC_IDR_OVRUN_SIZE			 1
#define SSC_IDR_OVRUN_OFFSET			 5
#define SSC_IDR_RXBUFF_SIZE			 1
#define SSC_IDR_RXBUFF_OFFSET			 7
#define SSC_IDR_RXRDY_SIZE			 1
#define SSC_IDR_RXRDY_OFFSET			 4
#define SSC_IDR_RXSYN_SIZE			 1
#define SSC_IDR_RXSYN_OFFSET			11
#define SSC_IDR_TXBUFE_SIZE			 1
#define SSC_IDR_TXBUFE_OFFSET			 3
#define SSC_IDR_TXEMPTY_SIZE			 1
#define SSC_IDR_TXEMPTY_OFFSET			 1
#define SSC_IDR_TXRDY_SIZE			 1
#define SSC_IDR_TXRDY_OFFSET			 0
#define SSC_IDR_TXSYN_SIZE			 1
#define SSC_IDR_TXSYN_OFFSET			10

/* SSC Interrupt Mask Register */
#define SSC_IMR				0x0000004c
#define SSC_IMR_CP0_SIZE			 1
#define SSC_IMR_CP0_OFFSET			 8
#define SSC_IMR_CP1_SIZE			 1
#define SSC_IMR_CP1_OFFSET			 9
#define SSC_IMR_ENDRX_SIZE			 1
#define SSC_IMR_ENDRX_OFFSET			 6
#define SSC_IMR_ENDTX_SIZE			 1
#define SSC_IMR_ENDTX_OFFSET			 2
#define SSC_IMR_OVRUN_SIZE			 1
#define SSC_IMR_OVRUN_OFFSET			 5
#define SSC_IMR_RXBUFF_SIZE			 1
#define SSC_IMR_RXBUFF_OFFSET			 7
#define SSC_IMR_RXRDY_SIZE			 1
#define SSC_IMR_RXRDY_OFFSET			 4
#define SSC_IMR_RXSYN_SIZE			 1
#define SSC_IMR_RXSYN_OFFSET			11
#define SSC_IMR_TXBUFE_SIZE			 1
#define SSC_IMR_TXBUFE_OFFSET			 3
#define SSC_IMR_TXEMPTY_SIZE			 1
#define SSC_IMR_TXEMPTY_OFFSET			 1
#define SSC_IMR_TXRDY_SIZE			 1
#define SSC_IMR_TXRDY_OFFSET			 0
#define SSC_IMR_TXSYN_SIZE			 1
#define SSC_IMR_TXSYN_OFFSET			10

/* SSC PDC Receive Pointer Register */
#define SSC_PDC_RPR			0x00000100

/* SSC PDC Receive Counter Register */
#define SSC_PDC_RCR			0x00000104

/* SSC PDC Transmit Pointer Register */
#define SSC_PDC_TPR			0x00000108

/* SSC PDC Receive Next Pointer Register */
#define SSC_PDC_RNPR			0x00000110

/* SSC PDC Receive Next Counter Register */
#define SSC_PDC_RNCR			0x00000114

/* SSC PDC Transmit Counter Register */
#define SSC_PDC_TCR			0x0000010c

/* SSC PDC Transmit Next Pointer Register */
#define SSC_PDC_TNPR			0x00000118

/* SSC PDC Transmit Next Counter Register */
#define SSC_PDC_TNCR			0x0000011c

/* SSC PDC Transfer Control Register */
#define SSC_PDC_PTCR			0x00000120
#define SSC_PDC_PTCR_RXTDIS_SIZE		 1
#define SSC_PDC_PTCR_RXTDIS_OFFSET		 1
#define SSC_PDC_PTCR_RXTEN_SIZE			 1
#define SSC_PDC_PTCR_RXTEN_OFFSET		 0
#define SSC_PDC_PTCR_TXTDIS_SIZE		 1
#define SSC_PDC_PTCR_TXTDIS_OFFSET		 9
#define SSC_PDC_PTCR_TXTEN_SIZE			 1
#define SSC_PDC_PTCR_TXTEN_OFFSET		 8

/* SSC PDC Transfer Status Register */
#define SSC_PDC_PTSR			0x00000124
#define SSC_PDC_PTSR_RXTEN_SIZE			 1
#define SSC_PDC_PTSR_RXTEN_OFFSET		 0
#define SSC_PDC_PTSR_TXTEN_SIZE			 1
#define SSC_PDC_PTSR_TXTEN_OFFSET		 8

/* START bit field values */
#define SSC_START_CONTINUOUS	0
#define SSC_START_TX_RX		1
#define SSC_START_LOW_RF	2
#define SSC_START_HIGH_RF	3
#define SSC_START_FALLING_RF	4
#define SSC_START_RISING_RF	5
#define SSC_START_LEVEL_RF	6
#define SSC_START_EDGE_RF	7
#define SSS_START_COMPARE_0	8

/* CKI bit field values */
#define SSC_CKI_FALLING		0
#define SSC_CKI_RISING		1

/* CKO bit field values */
#define SSC_CKO_NONE		0
#define SSC_CKO_CONTINUOUS	1
#define SSC_CKO_TRANSFER	2

/* CKS bit field values */
#define SSC_CKS_DIV		0
#define SSC_CKS_CLOCK		1
#define SSC_CKS_PIN		2

/* FSEDGE bit field values */
#define SSC_FSEDGE_POSITIVE	0
#define SSC_FSEDGE_NEGATIVE	1

/* FSOS bit field values */
#define SSC_FSOS_NONE		0
#define SSC_FSOS_NEGATIVE	1
#define SSC_FSOS_POSITIVE	2
#define SSC_FSOS_LOW		3
#define SSC_FSOS_HIGH		4
#define SSC_FSOS_TOGGLE		5

/* Bit manipulation macros */
#define SSC_BIT(name)					\
	(1 << SSC_##name##_OFFSET)
#define SSC_BF(name, value)				\
	(((value) & ((1 << SSC_##name##_SIZE) - 1))	\
	 << SSC_##name##_OFFSET)
#define SSC_BFEXT(name, value)				\
	(((value) >> SSC_##name##_OFFSET)		\
	 & ((1 << SSC_##name##_SIZE) - 1))
#define SSC_BFINS(name, value, old)			\
	(((old) & ~(((1 << SSC_##name##_SIZE) - 1)	\
	<< SSC_##name##_OFFSET)) | SSC_BF(name, value))

/* Register access macros */
#define ssc_readl(reg)		__raw_readl(SSC_BASE + SSC_##reg)
#define ssc_writel(reg, value)	__raw_writel((value), SSC_BASE + SSC_##reg)
