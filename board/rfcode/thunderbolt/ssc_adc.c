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
#include <common.h>
#include <asm/arch/at91sam9260.h>
#include <asm/arch/at91_pmc.h>
#include <asm/arch/gpio.h>
#include <asm/arch/io.h>
#include <dtt.h>
#include "ssc_adc.h"

/* How many channels of data are in each SSC frame */
#define NUM_CHANNELS		2
/* How many bytes are consumed per sample (includes both channels) */
#define BYTES_PER_SAMPLE	(2 * NUM_CHANNELS)
/* How many samples do we want to store */
#define SAMPLES_PER_BUFFER	256
/* How many bytes of memory are consumed for each DMA buffer */
#define DMA_BUFFER_BYTES	(SAMPLES_PER_BUFFER * BYTES_PER_SAMPLE)
/* How many 32-bit words of memory are consumed for each DMA buffer */
#define DMA_BUFFER_WORDS	(DMA_BUFFER_BYTES / 4)

#define SAMPLERATE		262144
#define nSEC_PER_SAMPLE		((1000000000 + SAMPLERATE/2)/SAMPLERATE)

extern u32 get_board_rev(void);

enum outputmodes {
	RAW_SAMPLES,
	RAW_SAMPLES_ALL,
	VOLTAGE,
	VOLTAGE_ALL,
	DBM,
	DBM_ALL,
};

static u32 samplebuffer[DMA_BUFFER_WORDS];
static int ssc_initialized = 0;
static int econobox_board = 0;

/****************************************************************************/
/* Issue a wake up sequence to the ADC since it seems to 
 * power up in the sleep state by default */
static void thunderbolt_adc_wake(void)
{
	at91_set_gpio_output(AT91_PIN_PB16, 0);		// SCK output
	at91_set_gpio_output(AT91_PIN_PB17, 0);		// CONV output
	
	at91_set_gpio_value(AT91_PIN_PB16, 1);		// SCK high
	at91_set_gpio_value(AT91_PIN_PB16, 0);		// SCK low

	at91_set_gpio_value(AT91_PIN_PB17, 1);		// CONV high
	at91_set_gpio_value(AT91_PIN_PB17, 0);		// CONV low

	at91_set_gpio_value(AT91_PIN_PB17, 1);		// CONV high
	at91_set_gpio_value(AT91_PIN_PB17, 0);		// CONV low

	at91_set_gpio_value(AT91_PIN_PB16, 1);		// SCK high
	at91_set_gpio_value(AT91_PIN_PB16, 0);		// SCK low
}

/* Initialize the ADC, the SSC pins and the SSC function */
static void thunderbolt_ssc_init (void)
{
	/* Issue wake-up sequence to the ADC */
	thunderbolt_adc_wake();

	/* Now configure the SSC pins for designated functions */
	at91_set_A_periph(AT91_PIN_PB16, 1);	// TK pin
	at91_set_A_periph(AT91_PIN_PB17, 1);	// TF pin
	at91_set_A_periph(AT91_PIN_PB19, 1);	// RD pin
	
	/* Enable clock to the SSC */
	at91_sys_write(AT91_PMC_PCER, 1 << AT91SAM9260_ID_SSC);
	
	/* Set up the SSC registers */
	ssc_writel(CR, SSC_BIT(CR_SWRST));

	ssc_writel(PDC_RPR, 0);
	ssc_writel(PDC_RCR, 0);
	ssc_writel(PDC_RNPR, 0);
	ssc_writel(PDC_RNCR, 0);

	ssc_writel(PDC_TPR, 0);
	ssc_writel(PDC_TCR, 0);
	ssc_writel(PDC_TNPR, 0);
	ssc_writel(PDC_TNCR, 0);

	/* 131.072 MHz  MCK	    (based off of 18.432MHz XTAL)
	 *  13.1072 MHz Bitclk	    (MCK/10)
	 * 262.144 kHz  Sample rate (Bitclk / 50 clocks per sample) */

	/* set SSC clock mode register */
	ssc_writel(CMR, 5);	/* MCK / (2 x 5) = 13.1072 MHz */

	/* set transmit clock mode and format
	 * We don't use the transmitter but we need it to generate TF and TK */
	ssc_writel(TCMR, 
			SSC_BF(TCMR_PERIOD, 24) |		// 2*(24+1) = 50 bit period
			SSC_BF(TCMR_STTDLY, 0) |		// 0 bit start delay
			SSC_BF(TCMR_START, SSC_START_RISING_RF) | // start = rising TF
			SSC_BF(TCMR_CKO, SSC_CKO_CONTINUOUS) |	// continuous clock output
			SSC_BF(TCMR_CKS, SSC_CKS_DIV));		// use divided clock
			
	ssc_writel(TFMR, 
			SSC_BF(TFMR_FSOS, SSC_FSOS_POSITIVE) |	// positive TF pulse output
			SSC_BF(TFMR_FSLEN, 0));			// TF length is 1 bit
			
	/* set receive clock mode and format */
	ssc_writel(RCMR, 
			SSC_BF(RCMR_STTDLY, 0) |		// 0 bit start delay
			SSC_BF(RCMR_START, SSC_START_TX_RX) |	// start = tx start
			SSC_BF(RCMR_CKI, SSC_CKI_FALLING) |	// sample on falling edge of CLK
			SSC_BF(RCMR_CKO, SSC_CKO_NONE) |	// no RK output
			SSC_BF(RCMR_CKS, SSC_CKS_CLOCK));	// use TK for clock
			
	ssc_writel(RFMR,
			SSC_BF(RFMR_FSOS, SSC_FSOS_NONE) |	// no RF pulse output
			SSC_BF(RFMR_DATNB, 0) |			// 1 channel (treat as single 32bit word)
			SSC_BIT(RFMR_MSBF) |			// MSB first
			SSC_BF(RFMR_DATLEN, 31));		// 32 bits per transfer

	ssc_initialized = 1;
	
	/* If this is an EconoBox board, we have only 1 RF channel and no temp sensor */
	if (get_board_rev() == 1)
		econobox_board = 1;
}

/* Get a block of samples out of the ADC and put them into samplebuffer */
static void thunderbolt_ssc_capture (void)
{
	if (!ssc_initialized)
		thunderbolt_ssc_init();

	/* Read the SSC status register */
	ssc_readl(SR);

	/* Set up the PDC buffer pointer and count registers */
	ssc_writel(PDC_RPR, (unsigned int)samplebuffer);
	ssc_writel(PDC_RCR, SAMPLES_PER_BUFFER);
		
	/* Enable the SSC Tx and Rx systems */
	ssc_writel(CR, SSC_BIT(CR_TXEN) | SSC_BIT(CR_RXEN));
	ssc_writel(PDC_PTCR, SSC_BIT(PDC_PTCR_RXTEN));

	/* Wait for the transfer to complete */
	while (!(ssc_readl(SR) & SSC_BIT(SR_ENDRX)));
	
	/* If this hardware platform only supports one channel */
	if (econobox_board)
	{
		u16 *ptr = (u16 *)samplebuffer;
		int i;
		
		/* Loop through all the samples in the buffer */
		/* and make it look like only one channel     */
		for(i=0; i<DMA_BUFFER_WORDS; i++)
		{
			/* Move ch2 (populated) to ch1 */
			*(ptr+1) = *ptr;
			*ptr = 0;
			ptr+=2;
		}
	}
}

/* Convert a raw 16 bit value to tenths of a dBm for the given channel 
 * (requires calibration data stored in cal1/cal2 environment variables). */
static int to_dbm_tenths (unsigned channel, unsigned value)
{
	char *cal = getenv((channel == 1) ? "cal1" : "cal2");
	char *end;
	unsigned high = 0;
	unsigned low = 0;
	int err = 1;
	int dbm;
	
	/* Find the two data points that contain our value */ 
	for (dbm=-40; cal && cal[0] && (dbm >= -115); dbm -= 5) {
		ulong entry = simple_strtoul(cal, &end, 16);
		if (end) {
			/* If we already have a 'low' reading shift and update */
			if (low != 0) {
				high = low;
				low = (unsigned)entry;
				/* Did we find the range containing the value */
				if (value >= low) {
					err = 0;
					break;
				}
				else if (high <= low) {
					printf("err: cal%d @ %ddbm <= previous entry\n", channel, dbm);
					break;
				}
			}
			else {
				low = (unsigned)entry; /* First time through */
			}
			/* Move cal pointer to the next entry in the string */
			if (*end) {
				cal = end+1;
			}
			else {	/* reached the end of the line */
				if (dbm == -115) err = 0;
				break;
			}
		}
		else break; /* failed to convert value */
	}

	if (!err) {
		int range = high - low;
		int offset = value - low;

		dbm = dbm * 10;	/* convert to tenths of a dbm */
		if (offset >= 0) {
			/* Linear interpolation of dBm value with rounding
			 * (5 dbm per interval = 50 dbm-tenths) */
			dbm += ((offset * 50) + (range / 2)) / range;
			/* Cap the maximum at -35.0 dBm */
			return (dbm > -350) ? -350 : dbm;
		}
		else {
			/* Linear extrapolation of dBm value below -115 dBm
			 * with rounding (5 dbm per interval = 50 dbm-tenths) */
			dbm += ((offset * 50) - (range / 2)) / range;
			/* Cap the maximum at -120.0 dBm */
			return (dbm < -1200) ? -1200 : dbm;
		}
	}
	return -1990; /* Return unreal value (-199.0 dBm) */
}

#if 0
/* Convert a raw 16 bit value to dBm for the given channel (requires
 * calibration data stored in cal1/cal2 environment variables). */
static int to_dbm (unsigned channel, unsigned value)
{
	/* Return value rounded to nearest whole dbm */
	return (to_dbm_tenths(channel, value) - 5) / 10;
}
#endif

/* Convert 16-bit sample to mVolts with Vref=2.5V */
#define to_mvolts(x) ((x) * 2500 / 0xFFFF)

/* Find the min/max values in samplebufer and optionally print each value */
static void thunderbolt_adc_minmax (u16 *ch1_min, u16 *ch1_max, 
		u16 *ch2_min, u16 *ch2_max, int mode)
{
	int i;
	u16 *ptr = (u16 *)samplebuffer;
	
	*ch1_min = *ch2_min = 0xFFFF;
	*ch1_max = *ch2_max = 0;
	
	/* Loop through all the samples in the buffer */
	for(i=0; i<DMA_BUFFER_WORDS; i++)
	{
		/* Little-Endian, ch2 comes first */
		u16 ch2 = *(ptr++) << 2;
		u16 ch1 = *(ptr++) << 2;
		
		/* Update min/max if appropriate */
		if (ch1 < *ch1_min) *ch1_min = ch1;
		if (ch1 > *ch1_max) *ch1_max = ch1;
		if (ch2 < *ch2_min) *ch2_min = ch2;
		if (ch2 > *ch2_max) *ch2_max = ch2;
		
		/* Output raw pseudo 16-bit values? */
		if (mode == RAW_SAMPLES_ALL)
		{
			printf("%d,0x%04x,0x%04x\n", i, ch1, ch2);
		}
		/* Output voltage readings? */
		else if (mode == VOLTAGE_ALL)
		{
			u32 v1 = to_mvolts(ch1);
			u32 v2 = to_mvolts(ch2);
			
			printf("%d,%u.%03u,%u.%03u\n", i,
				v1 / 1000, v1 % 1000,
				v2 / 1000, v2 % 1000);
		}
		/* Output dbm values? */
		else if (mode == DBM_ALL)
		{
			int dbm1 = to_dbm_tenths(1, ch1);
			int dbm2 = to_dbm_tenths(2, ch2);
			printf("%d,%d.%d,%d.%d\n", i, 
				dbm1 / 10, -dbm1 % 10,
				dbm2 / 10, -dbm2 % 10);
		}
	}
}

//#define STD_4TO1_REDUX
#define SAMPLE_SMOOTH_REDUX

/* Capture an avg value for each channel from samplebuffer using the same 
 * sample reduction method that the reader algorithm uses */
static void thunderbolt_adc_avg (u16 *avg1, u16 *avg2)
{
	int i;
	u32 ch1_accum = 0, ch2_accum = 0;
	u16 *ptr = (u16 *)samplebuffer;
#ifdef SAMPLE_SMOOTH_REDUX
	u16 pval1, pval2;
#endif
	
#ifdef STD_4TO1_REDUX
	/* Average of 4-to-1 (max) reduction samples */
	for(i=0; i<DMA_BUFFER_WORDS; i+=4)
	{
		int j;
		u16 max1 = 0, max2 = 0;
		for (j=0; j<4; j++)
		{
			/* Little-Endian, ch2 comes first */
			u16 ch2 = *(ptr++) << 2;
			u16 ch1 = *(ptr++) << 2;
		
			if (ch1 > max1) max1 = ch1;
			if (ch2 > max2) max2 = ch2;
		}
		ch1_accum += max1;
		ch2_accum += max2;
	}
	*avg1 = ch1_accum >> 6;	/* yield pseudo 16-bit values */
	*avg2 = ch2_accum >> 6;	/* yield pseudo 16-bit values */
#elif defined (SAMPLE_SMOOTH_REDUX)
	/* Average of 4-to-1 (max) reduction samples with smoothing */
	pval1 = ptr[1] << 2;
	pval2 = ptr[0] << 2;
	for(i=0; i<DMA_BUFFER_WORDS; i+=4)
	{
		int j;
		u32 max1 = 0, max2 = 0;
		for (j=0; j<4; j++)
		{
			/* Little-Endian, ch2 comes first */
			u16 ch2 = *(ptr++) << 2;
			u16 ch1 = *(ptr++) << 2;
		
			if (((ch1+pval1)/2) > max1) max1 = ((ch1+pval1)/2);
			if (((ch2+pval2)/2) > max2) max2 = ((ch2+pval2)/2);
			pval1 = ch1;
			pval2 = ch2;
		}
		ch1_accum += max1;
		ch2_accum += max2;
	}
	*avg1 = ch1_accum >> 6;	/* yield pseudo 16-bit values */
	*avg2 = ch2_accum >> 6;	/* yield pseudo 16-bit values */
#else	
	/* Simple average of all the samples in the buffer */
	for(i=0; i<DMA_BUFFER_WORDS; i++)
	{
		/* Little-Endian, ch2 comes first */
		u16 ch2 = *(ptr++) << 2;
		u16 ch1 = *(ptr++) << 2;
		
		ch1_accum += ch1;
		ch2_accum += ch2;
	}
	*avg1 = ch1_accum >> 8;	/* yield pseudo 16-bit values */
	*avg2 = ch2_accum >> 8;	/* yield pseudo 16-bit values */
#endif
}

/* Display min/max/avg values from the ADC */
static void thunderbolt_adc_display_minmaxavg (int mode)
{
	u16 ch1_min, ch1_max, ch1_avg, ch2_min, ch2_max, ch2_avg;
	
	/* Get a new batch of samples */
	thunderbolt_ssc_capture();
	
	/* Find the min and max for each channel, possibly print each value */
	thunderbolt_adc_minmax(&ch1_min, &ch1_max, &ch2_min, &ch2_max, mode);
	
	/* Find the avg value */
	thunderbolt_adc_avg(&ch1_avg, &ch2_avg);
	
	/* Raw output mode (pseudo 16 bit values) */
	if ((mode == RAW_SAMPLES) || (mode == RAW_SAMPLES_ALL)) {
		printf("CH1 min=0x%04X max=0x%04X avg=0x%04X\n",
			ch1_min, ch1_max, ch1_avg);
		printf("CH2 min=0x%04X max=0x%04X avg=0x%04X\n",
			ch2_min, ch2_max, ch2_avg);
	}
	/* Voltage output mode */
	else if ((mode == VOLTAGE) || (mode == VOLTAGE_ALL))
	{
		u32 vmin, vmax, vavg;
		
		vmin = to_mvolts(ch1_min);
		vmax = to_mvolts(ch1_max);
		vavg = to_mvolts(ch1_avg);
		printf("CH1 min=%u.%03u max=%u.%03u avg=%u.%03u V\n",
			vmin / 1000, vmin % 1000,
			vmax / 1000, vmax % 1000,
			vavg / 1000, vavg % 1000);
			
		vmin = to_mvolts(ch2_min);
		vmax = to_mvolts(ch2_max);
		vavg = to_mvolts(ch2_avg);
		printf("CH2 min=%u.%03u max=%u.%03u avg=%u.%03u V\n",
			vmin / 1000, vmin % 1000,
			vmax / 1000, vmax % 1000,
			vavg / 1000, vavg % 1000);
	}
	/* dBm output mode (requires cal1 and cal2 env variables) */
	else if ((mode == DBM) || (mode == DBM_ALL)) {
		int dbm_min, dbm_max, dbm_avg;
		
		dbm_min = to_dbm_tenths(1, ch1_min),
		dbm_max = to_dbm_tenths(1, ch1_max),
		dbm_avg = to_dbm_tenths(1, ch1_avg);
		printf("CH1 min=%d.%d max=%d.%d avg=%d.%d dBm\n",
			dbm_min / 10, -dbm_min % 10,
			dbm_max / 10, -dbm_max % 10,
			dbm_avg / 10, -dbm_avg % 10);
		dbm_min = to_dbm_tenths(2, ch2_min),
		dbm_max = to_dbm_tenths(2, ch2_max),
		dbm_avg = to_dbm_tenths(2, ch2_avg);
		printf("CH2 min=%d.%d max=%d.%d avg=%d.%d dBm\n",
			dbm_min / 10, -dbm_min % 10,
			dbm_max / 10, -dbm_max % 10,
			dbm_avg / 10, -dbm_avg % 10);
	}
}

/* Try to find a single RF pulse and display information about it */
static void thunderbolt_adc_findpulse (int channel, unsigned thresh)
{
	int i, j;
	int keep_looking = 1;
	u16 ch1, ch2, val, min=0xFFFF;
	u32 accum = 0;
	u16 *ptr;

	while (keep_looking)
	{
		/* If the user gave us a threshold, go with it */
		if (thresh) {
			/* Match threshold to raw sample values */
			min = (u16)thresh;
		}
		else {
			/* Loop through a few buffers to establish a thresh.
			 * We assume that pulses are sporadic so an avg will
			 * get us pretty close to the noise floor */
			for (j=0, accum=0; j<4; j++)
			{
				/* Get a new batch of samples */
				thunderbolt_ssc_capture();
				
				/* Loop through all the samples in the buffer */
				ptr = (u16 *)samplebuffer;
				for(i=0; i<DMA_BUFFER_WORDS; i++)
				{
					/* Little-Endian, ch2 comes first */
					ch2 = *(ptr++) << 2;
					ch1 = *(ptr++) << 2;
					
					if (channel == 1) accum += ch1;
					else accum += ch2;
				}
			}
			/* Take the resulting average value */
			min = (accum >> 10);
			/* Add margin to create the threshold */
			if (min < 0xE000)
				min += 0x2000;
		}
		
		/* Try to find a pulse greater than the min threshold */
		for (j=0; j<5000; j++)
		{
			int start = -1, end = -1;
			u16 pulsemax = 0;
			
			/* Watch for ctrl-c */
			if (ctrlc())
			{
				puts("interrupt\n");
				keep_looking = 0;
				break;
			}
				
			/* Get a new batch of samples */
			thunderbolt_ssc_capture();
			
			/* Loop through all the samples in the buffer */
			ptr = (u16 *)samplebuffer;
			for(i=0; i<DMA_BUFFER_WORDS; i++)
			{
				/* Little-Endian, ch2 comes first */
				ch2 = *(ptr++) << 2;
				ch1 = *(ptr++) << 2;
				
				if (channel == 1) val = ch1;
				else val = ch2;
				
				/* Is signal strong enough? */
				if (val > min)
				{
					/* Remember the start index */
					if (start < 0)
						start = i;
					/* and the maximum ADC reading */
					if (val > pulsemax)
						pulsemax = val;
				}
				else if (start >= 0)
				{
					/* Remember the end of the pulse */
					end = i;
					break;
				}
			}
			/* If the whole buffer was above the threshold */
			if ((start == 0) && (end < 0)) {
				end = i;	/* end it */
			}
			/* If we found a complete pulse */
			if (end > 0)
			{
				int width = 0;
				int delta = pulsemax / 10; /* within 10% */
				
				/* Back up a fixed number of data points */
				j = start - 5;
				if (j < 0) j=0;
				
				while (j < end + 5 && (j < DMA_BUFFER_WORDS))
				{
					ptr = (u16 *)(&samplebuffer[j]);
					if (channel == 1)
						val = *(ptr + 1) << 2;
					else val = *ptr << 2;
					
					/* Is it part of the pulse? */
					if (j >= start && j < end)
					{
						/* Is it within 'delta' codes
						 * of the pulsemax value? */
						if ((val + delta) >= pulsemax)
						{
							printf("%04X **\n", val);
							width++;
						}
						else printf("%04X *\n", val);
					}
					else printf("%04X\n", val);
					j++;
				}
				printf("CH%d pulse max=0x%04X thresh=0x%04X width=%duS\n",
					channel, pulsemax, min,
					(width * nSEC_PER_SAMPLE + 500)/1000);
					
				keep_looking = 0;
				break;
			}
		}
	}
}

/* User command to take a group of samples from the ADC and report
 * min/max/avg values */
static int do_sample (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	int mode = RAW_SAMPLES;
	
	/* If the only arg is "all" use default format */
	if ((argc == 2) && !strcmp(argv[1], "all"))
	{
		mode = RAW_SAMPLES_ALL;
	}
	/* Else do they want voltage readings? */
	else if ((argc > 1) && !strcmp(argv[1], "v"))
	{
		mode = VOLTAGE;
	}
	/* Else do they want dBm readings? */
	else if ((argc > 1) && !strcmp(argv[1], "db"))
	{
		mode = DBM;
	}
	/* If they supplied an additional "all" arg, use the "all" version */
	if ((argc > 2) && !strcmp(argv[2], "all"))
	{
		mode++;
	}
	
	/* Take a group of samples and output min/max/avg values */
	thunderbolt_adc_display_minmaxavg(mode);
	
	return 0;
}

U_BOOT_CMD(sample,	3,	1,	do_sample,
	"sample  - take a group of ADC samples and report min/max/avg values\n",
	"\n"
	"    - take a group of ADC samples and report min/max/avg values\n"
	"sample <mode> {all}\n"
	"    - <mode> is either\n"
	"        raw  - output full resolution hexadecimal values\n"
	"        v    - output volts\n"
	"        db   - output dBm values\n"
	"    - if \"all\" is included, all samples will be output\n"
);

/* User command to find an RF pulse on a given channel and display
 * ADC value and pulse width information for the pulse */
static int do_findpulse (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	int channel = 1;
	ulong thresh = 0;
	
	if (argc < 2) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
	
	if (!strcmp(argv[1], "2"))
	{
		channel = 2;
	}
	if (argc > 2) {
		thresh = simple_strtoul(argv[2], NULL, 16);
	}
	
	/* Find an RF pulse and display its stats */
	thunderbolt_adc_findpulse(channel, (unsigned int)thresh);
	
	return 0;
}

U_BOOT_CMD(getpulse,	3,	1,	do_findpulse,
	"getpulse- watch for an RF pulse on the requested channel\n",
	"chan\n"
	"    - watch for an RF pulse on the requested channel and\n"
	"      report details about the strength and timing\n"
	"getpulse chan thresh\n"
	"    - watch for an RF pulse on the requested channel using\n"
	"      the given threshold\n"
);

/* Low pass filter on accumulated average values (higher = longer time constant) */
#define FILTER_SHIFT	6

/* User command to interactively determine the calibration data points */
static int do_calibrate (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	int channel = 0;
	int siglevel = -40;
	int index = 0;
	u32 accum1 = 0;
	u32 accum2 = 0;
	u16 cal1[16];
	u16 cal2[16];
	char buf[128];
	int start_caltmp;
	int end_caltmp;
	ulong usec_last = 0;
	
	/* Optional parameter is single channel to calibrate */
	if (argc == 2) {
		if (!strcmp(argv[1], "1"))
		{
			channel = 1;
		}
		else if (!strcmp(argv[1], "2"))
		{
			channel = 2;
		}
		else {
			printf("Usage:\n%s\n", cmdtp->usage);
			return 1;
		}
	}
	/* Make sure we know if this is an Econobox board before starting */
	if (!ssc_initialized)
		thunderbolt_ssc_init();
	
	if (!econobox_board)
		/* Grab board temperature at start of calibration */
		start_caltmp = dtt_get_temp_10x(1);
	else start_caltmp = 0;

	/* Column headers */
	puts("Input     CH1    CH2\n");
	while (index < 16)
	{
		/* Show the requested dBm level */
		printf("%4ddBm : 0x0000 0x0000", siglevel);
		while(1)
		{
			u16 calavg1, calavg2;
			
			/* Get avg value for each channel */
			thunderbolt_ssc_capture();
			thunderbolt_adc_avg(&calavg1, &calavg2);
			
			/* Low pass filter the values in accumulators */
			accum1 = accum1 - (accum1 >> FILTER_SHIFT) + calavg1;
			accum2 = accum2 - (accum2 >> FILTER_SHIFT) + calavg2;
			
			/* Every 250msec output the values */
			if (get_timer(usec_last) >= 250000) {
				usec_last = get_timer_masked();
				/* Back up the cursor and print the new values */
				puts("\b\b\b\b\b\b\b\b\b\b\b\b\b");
				printf("0x%04X 0x%04X", accum1 >> FILTER_SHIFT, 
							accum2 >> FILTER_SHIFT);
			}

			/* Check for user input */
			if (tstc()) {
				char ch = getc();
				
				if (ch == 0x03)	/* Control-C */
				{
					puts("\n");
					return 1;
				}
				else if (ch == '\r') /* Enter */
				{
					break;
				}
			}
		}
		/* Remember the filtered values and advance */
		cal1[index] = accum1 >> FILTER_SHIFT;
		cal2[index] = accum2 >> FILTER_SHIFT;
		/* Back up the cursor and print the captured values w/ NL */
		puts("\b\b\b\b\b\b\b\b\b\b\b\b\b");
		printf("0x%04X 0x%04X\n", cal1[index], cal2[index]);
		index++;
		siglevel -= 5;
	}
	
	if (!econobox_board)
		/* Grab board temperature at end of calibration */
		end_caltmp = dtt_get_temp_10x(1);
	else end_caltmp = 0;
	
	/* If 2C or more move during calibration, its crap */
	if(((start_caltmp-end_caltmp) >= 20) || (end_caltmp-start_caltmp >= 20)) {
		printf("ERROR: board temperature change by %d C during calibration!\n",
			(end_caltmp-start_caltmp)/10);
	}
	else {
		/* If we're not doing only channel 2, display and store channel 1 */
		if (channel != 2)
		{
			sprintf(buf, "%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X",
				cal1[0], cal1[1], cal1[2], cal1[3], cal1[4], cal1[5], cal1[6], cal1[7], 
				cal1[8], cal1[9], cal1[10],cal1[11],cal1[12],cal1[13],cal1[14],cal1[15]);
			printf("cal1=%s\n", buf);
			setenv("cal1", buf);
			if (!econobox_board)
			{
				sprintf(buf, "%d", end_caltmp*100);
				printf("cal1tmp=%s\n", buf);
				setenv("cal1tmp", buf);
			}
		}
		/* If we're not doing only channel 1, display and store channel 2 */
		if (channel != 1)
		{
			sprintf(buf, "%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X",
				cal2[0], cal2[1], cal2[2], cal2[3], cal2[4], cal2[5], cal2[6], cal2[7], 
				cal2[8], cal2[9], cal2[10],cal2[11],cal2[12],cal2[13],cal2[14],cal2[15]);
			printf("cal2=%s\n", buf);
			setenv("cal2", buf);
			if (!econobox_board)
			{
				sprintf(buf, "%d", end_caltmp*100);
				printf("cal2tmp=%s\n", buf);
				setenv("cal2tmp", buf);
			}
		}
	}
	return 0;
}

U_BOOT_CMD(calibrate,	2,	1,	do_calibrate,
	"cal     - interactive process to capture calibration settings\n",
	"chan\n"
	"    - capture calibration settings for only the requested channel\n"
);

/* User command to show real-time dBm values */
static int show_dbm (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	/* Column headers */
	puts("   CH1         CH2\n");
	puts("----------  ----------\n");
	puts("-115.0 dBm  -115.0 dBm");
	while(1)
	{
		u16 avg1, avg2;
		int dbm1, dbm2;
		
		/* Get avg value for each channel */
		thunderbolt_ssc_capture();
		thunderbolt_adc_avg(&avg1, &avg2);
		
		dbm1 = to_dbm_tenths(1, avg1);
		dbm2 = to_dbm_tenths(2, avg2);
		
		/* Back up the cursor and print the new values */
		puts("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
		printf("%4d.%d dBm  %4d.%d dBm", 
			dbm1/10, -dbm1 % 10, dbm2/10, -dbm2 % 10);

		/* Check for user input */
		if (tstc()) {
			char ch = getc();
			
			if (ch == 0x03)	/* Control-C */
			{
				puts("\n");
				return 1;
			}
			else if (ch == '\r') /* Enter */
			{
				puts("\n");
			}
		}
		else udelay(250000);	/* 250ms delay */
	}
	return 0;
}

U_BOOT_CMD(dbm,	1,	1,	show_dbm,
	"dbm     - display continuous dBm readings for both channels\n",
	NULL
);
