/***************************************************************************
*   Copyright (C) 2011 - 2022 by Analog Devices, Inc.                     *
*   Based on ice100.c of UrJTAG                                           *
*   Jie Zhang  <jie.zhang@analog.com>                                     *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <target/image.h>
#include <helper/log.h>
#include <helper/configuration.h>
#include "libusb_helper.h"

#ifdef _WIN32
#include "usbmux.h"
#endif

/*
 * Internal Structures
 */

/* JTAG TMS/TDI Data */
typedef struct
{
	uint8_t tms;			/* TMS data */
	uint8_t tdi;			/* TDI data */
} tap_pairs;

/* For collecting data */
typedef struct
{
	int32_t idx;			/* Index where data is to be collected */
	int32_t pos;			/* Bit position where data is to be collected */
	void *ptr;				/* This can point to a scan_command or swd_packet */
} dat_dat;

/* Master scan control structure */
typedef struct
{
	int32_t total;			/* Max number of tap pointers */
	int32_t cur_idx;		/* Where to add next, or total */
	int32_t bit_pos;		/* Position to place next bit */
	int32_t num_dat;		/* Total posible data collection points */
	int32_t cur_dat;		/* Index to dat array for data to be collected */
	int32_t rcv_dat;		/* Index to retreive collected data */
	dat_dat *dat;			/* Pointer to data collection points */
	unsigned char *cmd;		/* Pointer to command, which encompasses pairs */
	tap_pairs *pairs;		/* Pointer to tap pairs array */
} num_tap_pairs;

/* Cable params_t structure with our data */
typedef struct
{
	libusb_device_handle *usb_handle; /* USB handle */
	uint32_t cur_freq;				/* JTAG Frequency */
	uint32_t cur_voltage;			/* Voltage 1: 1.8V, 2: 2.5V, 3: 3.3/5V, ICE-2000 only */
	uint32_t cur_delay;				/* Delay, ICE-2000 only */
	uint16_t version;				/* Firmware Version */
	uint32_t default_scanlen;		/* #Scan pairs in scan */
	uint32_t trigger_scanlen;		/* High water mark */
	uint32_t tap_pair_start_idx;	/* depends on firmware version */
	uint32_t num_rcv_hdr_bytes;		/* Number of data bytes in received raw scan data header */
	uint32_t max_raw_data_tx_items;	/* depends on firmware version */
	int32_t wr_ep;					/* USB End Write Point */
	int32_t wr_timeout;				/* USB Write Timeout */
	int32_t wr_buf_sz;				/* USB Write Buffer Size */
	int32_t r_ep;					/* USB End Read Point */
	int32_t r_timeout;				/* USB Read Timeout */
	int32_t r_buf_sz;				/* USB Read Buffer Size */
	num_tap_pairs tap_info;			/* For collecting and sending tap scans */
	bool use_usbmux;				/* If true, use USB MUX for USB communication */
#ifdef _WIN32
	HANDLE mux_handle;				/* USB MUX handle */
#endif
} params_t;

/* Emulators's USB Data structure */
typedef struct
{
	uint32_t command;		/* What to do */
	uint32_t buffer;		/* used for Kit only, always initialized to 0 */
	uint32_t count;			/* Amount of data in bytes to send */
} usb_command_block;


/*
 * Internal Prototypes
 */

static int perform_scan(uint8_t **rdata);
static int do_rawscan(uint8_t firstpkt, uint8_t lastpkt,
					  int32_t collect_dof, int32_t dif_cnt, uint8_t *raw_buf,
					  uint8_t *out);
static int add_scan_data(int32_t, uint8_t *, bool, struct scan_command *);
static uint8_t *get_recv_data(int32_t, int32_t, uint8_t *);
static uint16_t do_host_cmd(uint8_t cmd, uint8_t param, int32_t r_data);
static uint32_t do_single_reg_value(uint8_t reg, int32_t r_data,
									int32_t wr_data, uint32_t data);

/*
 * Debug Macros
 */

#if 0    /* set to 1 to output debug info about scans */

//#define DSP_SCAN_DATA
#define DUMP_EACH_RCV_DATA
//#define DSP_SCAN_CAUSE
#define DEBUG(...)    printf(__VA_ARGS__)

#else

#define DEBUG(...)

#endif


/*
 * Internal Data, defines and Macros
 */
#define ICE_DEFAULT_SCAN_LEN			0x7FF0	/* Max DIF is 0x2AAA8, but DMA is only 16 bits. */
#define ICE_TRIGGER_SCAN_LEN			0x7FD8	/* Start checking for RTI/TLR for xmit */

#define SELECTIVE_RAW_SCAN_HDR_SZ		12

#define DAT_SZ							0x8000	/* size allocated for reading data */
#define DAT_SZ_INC						0x40	/* size to increase if data full */

/* USB Emulator Commands */
#define HOST_GET_FW_VERSION				0x01	/* get the firmware version */
#define HOST_REQUEST_RX_DATA			0x02	/* host request to transmit data */
#define HOST_REQUEST_TX_DATA			0x04	/* host request to transmit data */
#define HOST_GET_SINGLE_REG				0x08	/* set a JTAG register */
#define HOST_SET_SINGLE_REG				0x09	/* set a JTAG register */
#define HOST_PROGRAM_FLASH				0x0C	/* program flash */
#define HOST_HARD_RESET_JTAG_CTRLR		0x0E	/* do a hard reset on JTAG controller */
#define HOST_SET_TRST					0x1F	/* changes TRST Line state */
#define HOST_GET_TRST					0x20	/* gets TRST Line state */
#define HOST_DO_SELECTIVE_RAW_SCAN		0x21	/* Return only data needed */
#define HOST_SET_2000_VOLTAGE			0x24
#define HOST_SET_INTERFACE_MODE			0x25
#define HOST_DISCONNECT					0x27

/* Registers */
#define REG_AUX							0x00
#define REG_SCR							0x04
#define REG_FREQ						0x40

#define SCR_DEFAULT						0x30A0461
#define SCR_TRST_BIT					0x0000040

/* Ice USB controls */
#define ICE_1000_WRITE_ENDPOINT			0x06
#define ICE_1000_READ_ENDPOINT			0x05
#define ICE_1000_USB_CONNECTION_TIMEOUT	10000
#define ICE_1000_USB_WRITE_TIMEOUT		10000
#define ICE_1000_USB_READ_TIMEOUT		30000
#define ICE_1000_WRITE_BUFFER_SIZE		0x9800
#define ICE_1000_READ_BUFFER_SIZE		0x8000


/* frequency settings for ICE-1000 */
#define MAX_FREQ_1000	3
static const uint8_t freq_set_1000[MAX_FREQ_1000] = { 45, 22, 8  };
static const uint32_t avail_freqs_1000[MAX_FREQ_1000] = { 1000000, 2000000, 5000000 };
/* frequency settings for ICE-2000 */
#define MAX_FREQ_2000	7
static const uint8_t freq_set_2000[MAX_FREQ_2000] = { 45, 22, 8, 4, 2, 1, 0 };
static const uint32_t avail_freqs_2000[MAX_FREQ_2000] = { 1000000, 2000000, 5000000, 9000000, 15000000, 23000000, 46000000 };

/*
 * Internal Macros
 */

#ifdef _WIN32
#define adi_usb_read_or_ret(buf, len)									\
	do {																\
		if (cable_params.use_usbmux)									\
		{																\
			USB_MUX_ERROR mux_ret = usbmux_read(cable_params.mux_handle, \
				buf, len, cable_params.r_ep | LIBUSB_ENDPOINT_IN, cable_params.r_timeout);	\
			if (mux_ret != USB_MUX_OK) return ERROR_FAIL;				\
		}																\
		else															\
		{																\
			int __ret, __actual, __size = (len);						\
			__ret = libusb_bulk_transfer(cable_params.usb_handle,		\
									cable_params.r_ep | LIBUSB_ENDPOINT_IN, \
									(unsigned char *)(buf), __size,		\
									&__actual, cable_params.r_timeout);	\
			if (__ret || __actual != __size)							\
			{															\
				LOG_ERROR("unable to read from usb to " #buf ": "		\
						"wanted %i bytes but only received %i bytes",	\
						__size, __actual);								\
				return ERROR_FAIL;										\
			}															\
		}																\
	} while (0)

#define adi_usb_write_or_ret(buf, len)									\
	do {																\
		if (cable_params.use_usbmux)									\
		{																\
			USB_MUX_ERROR mux_ret = usbmux_write(cable_params.mux_handle,	\
				buf, len, cable_params.wr_ep | LIBUSB_ENDPOINT_OUT,			\
				cable_params.wr_timeout);									\
			if (mux_ret != USB_MUX_OK) return ERROR_FAIL;				\
		}																\
		else															\
		{																\
			int __ret, __actual, __size = (len);						\
			__ret = libusb_bulk_transfer(cable_params.usb_handle,		\
									cable_params.wr_ep | LIBUSB_ENDPOINT_OUT, \
									(unsigned char *)(buf), __size,		\
									&__actual, cable_params.wr_timeout);\
			if (__ret || __actual != __size)							\
			{															\
				LOG_ERROR("unable to write from " #buf " to usb: "		\
						"wanted %i bytes but only wrote %i bytes",		\
						__size, __actual);								\
				return ERROR_FAIL;										\
			}															\
		}																\
	} while (0)

#else
#define adi_usb_read_or_ret(buf, len)								\
	do {															\
		int __ret, __actual, __size = (len);						\
		__ret = libusb_bulk_transfer(cable_params.usb_handle,		\
								cable_params.r_ep | LIBUSB_ENDPOINT_IN, \
								(unsigned char *)(buf), __size,		\
								&__actual, cable_params.r_timeout);	\
		if (__ret || __actual != __size)							\
		{															\
			LOG_ERROR("unable to read from usb to " #buf ": "		\
					"wanted %i bytes but only received %i bytes",	\
					__size, __actual);								\
			return ERROR_FAIL;										\
		}															\
	} while (0)

#define adi_usb_write_or_ret(buf, len)								\
	do {															\
		int __ret, __actual, __size = (len);						\
		__ret = libusb_bulk_transfer(cable_params.usb_handle,		\
								cable_params.wr_ep | LIBUSB_ENDPOINT_OUT, \
								(unsigned char *)(buf), __size,		\
								&__actual, cable_params.wr_timeout);\
		if (__ret || __actual != __size)							\
		{															\
			LOG_ERROR("unable to write from " #buf " to usb: "		\
					"wanted %i bytes but only wrote %i bytes",		\
					__size, __actual);								\
			return ERROR_FAIL;										\
		}															\
	} while (0)
#endif


static params_t cable_params;


/*
 * System Interface Functions
 */

extern struct adapter_driver *adapter_driver;
static const char *adi_cable_name(void)
{
	if (adapter_driver == NULL)
		return "";

	if (strcmp(adapter_driver->name, "ice1000") == 0)
		return "ICE-1000";
	else if (strcmp(adapter_driver->name, "ice2000") == 0)
		return "ICE-2000";
	else
		return "unknown";
}

/*
 * Gets available Frequency index.
 */
static int adi_get_freq(uint32_t freq, int arr_sz, const uint32_t *freq_arr)
{
	int i;

	/* Verify Frequency is valid */
	for (i = 0; i < arr_sz; i++)
	{
		if (freq == freq_arr[i])
		{
			/* spot on */
			break;
		}
		else if (freq < freq_arr[i])
		{
			/* an in between frequency */
			if (i > 0)
				i--;
			break;
		}
	}

	if (i == arr_sz)
	{
		/* must of entered something above the max! */
		i--;
	}

	return i;
}

/*
 * Sets ICE-1000 Frequency
 */
static void ice1000_set_freq(uint32_t freq)
{
	params_t *params = &cable_params;

	/* Verify Frequency is valid */
	if (freq != params->cur_freq)
	{
		/* only change if different from current settings */
		int idx = adi_get_freq(freq, MAX_FREQ_1000, &avail_freqs_1000[0]);

		if (avail_freqs_1000[idx] != params->cur_freq)
		{
			/* only change if different from current settings
			 * this call's frequency may have been not one of
			 * the defined settings, but ends up there */
			params->cur_freq = freq;
			do_single_reg_value(REG_FREQ, 1, 1, freq_set_1000[idx]);
		}
	}
}

static int ice2000_validate_ircapture(int test_data_length, int total_ir_length,
									  const uint8_t *ir_test_in, uint8_t *ir_test_out)
{
	int retval, i;

	if (test_data_length % 64)
		return ERROR_FAIL;

	jtag_add_plain_ir_scan(test_data_length + total_ir_length,
						   ir_test_in, ir_test_out, TAP_IDLE);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < test_data_length / 64; i++)
	{
		uint64_t val_in, val_out;
		val_in = buf_get_u64(ir_test_in, i * 64, 64);
		val_out = buf_get_u64(ir_test_out, total_ir_length + i * 64, 64);
		if (val_in != val_out)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void ice2000_set_voltage_freq_delay(uint32_t voltage, uint32_t freq, uint32_t delay)
{
	uint32_t value = freq | (delay << 8) | (voltage << 16);

	do_single_reg_value(REG_FREQ, 1, 1, value);
}

/* TEST_DATA_LENGTH % 64 should be 0 */
#define TEST_DATA_LENGTH 0x8000

static int ice2000_find_delay(uint32_t voltage, uint32_t freq)
{
	struct jtag_tap *tap;
	int total_ir_length, test_data_length, ir_test_length;
	uint8_t *ir_test_in, *ir_test_out;
	params_t *params = &cable_params;
	int delay, delay_window_size;
	int first_good_delay = -1, last_good_delay = -1;
	int retval;
	int i;
	int idx;

	total_ir_length = 0;
	tap = jtag_tap_next_enabled(NULL);
	for (; tap != NULL; tap = jtag_tap_next_enabled(tap))
		total_ir_length += tap->ir_length;

	test_data_length = TEST_DATA_LENGTH;
	ir_test_length = DIV_ROUND_UP(test_data_length + total_ir_length, 8);

	ir_test_in = malloc(ir_test_length);
	if (ir_test_in == NULL)
		return ERROR_FAIL;
	ir_test_out = malloc(ir_test_length);
	if (ir_test_out == NULL)
	{
		free(ir_test_in);
		return ERROR_FAIL;
	}

	/* fill random test data */
	for (i = 0; i < test_data_length / 8; i++)
		ir_test_in[i] = rand();

	/* after this scan, all TAPs will capture BYPASS instructions */
	buf_set_ones(ir_test_in + test_data_length / 8, total_ir_length);

	/* the good delay window size is roughly one cycle. the delay chip
	   is 0.25ns. */
	idx = adi_get_freq(freq, MAX_FREQ_2000, &avail_freqs_2000[0]);
	delay_window_size = 1000000000 / avail_freqs_2000[idx] * 4;

	jtag_add_reset(0, 0);
	jtag_add_tlr();
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		goto done;

	for (delay = 0; delay <= 0xff; delay++)
	{
		ice2000_set_voltage_freq_delay(voltage, freq_set_2000[idx], delay);

		if (ice2000_validate_ircapture(test_data_length, total_ir_length,
									   ir_test_in, ir_test_out) == ERROR_OK)
		{
			if (first_good_delay < 0)
				first_good_delay = delay;

			last_good_delay = delay;
		}
		else
		{
			if (last_good_delay > 0)
				break;
		}
	}

	/* if we cannot find a good delay */
	if (first_good_delay < 0)
		retval = ERROR_FAIL;

	/* if we find a whole window of good delays */
	else if (first_good_delay > 0 && last_good_delay < 0xff)
		params->cur_delay = (first_good_delay + last_good_delay) / 2;

	/* all delays are valid, just pick the middle one */
	else if (first_good_delay == 0 && last_good_delay == 0xff)
		params->cur_delay = (first_good_delay + last_good_delay) / 2;

	/* we only find a partial window */
	else if (first_good_delay == 0 && last_good_delay < 0xff)
	{
		/* we still can get the best value */
		if (last_good_delay - delay_window_size / 2 >= 0)
			params->cur_delay = last_good_delay - delay_window_size / 2;
		/* or make sure the margin is big enough (10 is just a guess) */
		else if (last_good_delay >= 10)
			params->cur_delay = 0;
		/* otherwise we just fail */
		else
			retval = ERROR_FAIL;
	}

	/* we only find a partial window */
	else /* first_good_delay > 0 && last_good_delay == 0xff */
	{
		/* we still can get the best value */
		if (first_good_delay + delay_window_size / 2 <= 0xff)
			params->cur_delay = first_good_delay + delay_window_size / 2;
		/* or make sure the margin is big enough (10 is just a guess) */
		else if (0xff - first_good_delay >= 10)
			params->cur_delay = first_good_delay;
		/* otherwise we just fail */
		else
			retval = ERROR_FAIL;
	}

	/* restore the original settings */
	idx = adi_get_freq(params->cur_freq, MAX_FREQ_2000, &avail_freqs_2000[0]);
	ice2000_set_voltage_freq_delay(params->cur_voltage, freq_set_2000[idx], params->cur_delay);

done:
	free(ir_test_in);
	free(ir_test_out);

	if (retval == ERROR_OK)
		LOG_INFO("%s delay %d", adi_cable_name(), params->cur_delay);
	else
		LOG_ERROR("%s cannot find a good delay", adi_cable_name());

	return retval;
}

/*
 * Sets ICE-2000 Frequency
 */
static int ice2000_set_freq(uint32_t freq)
{
	params_t *params = &cable_params;
	int idx = adi_get_freq(freq, MAX_FREQ_2000, &avail_freqs_2000[0]);

	/* only change if different from current settings */
	if (avail_freqs_2000[idx] != params->cur_freq)
	{
		if (ice2000_find_delay(params->cur_voltage, avail_freqs_2000[idx]) != ERROR_OK)
			return ERROR_FAIL;
		params->cur_freq = freq;
		ice2000_set_voltage_freq_delay(params->cur_voltage, freq_set_2000[idx], params->cur_delay);
	}

	return ERROR_OK;
}

static int ice1000_firmware_crc(uint16_t *p)
{
	usb_command_block usb_cmd_blk;

	usb_cmd_blk.command = HOST_REQUEST_RX_DATA;
	usb_cmd_blk.count = 2;
	usb_cmd_blk.buffer = 0;

	adi_usb_write_or_ret((uint8_t*)&usb_cmd_blk, sizeof (usb_cmd_blk));

	adi_usb_read_or_ret(p, sizeof (*p));

	return ERROR_OK;
}

static uint16_t crc16_ccitt(const uint8_t *data, int length, uint16_t crc)
{
	int i;

	for (i = 0; i < length; i++)
	{
		uint8_t b = data[i];
		int j;

		for (j = 0; j < 8; j++)
		{
			bool add = ((crc >> 15) != (b >> 7));
			crc <<= 1;
			b <<= 1;
			if (add)
				crc ^= 0x1021;
		}
	}

	return crc;
}

static int ice1000_send_flash_data(struct image *firmware, uint16_t *crcp)
{
/* Flash programming is much slower than jtag operation. So we have
   to use a much smaller buffer size to avoid USB transfer timeout.  */
#define ICE_1000_FLASH_DATA_BUFFER_SIZE 0x400

	uint8_t buffer[ICE_1000_FLASH_DATA_BUFFER_SIZE];
	uint8_t first = 1, last = 0;
	int i;
	uint16_t crc = 0xffff;
	size_t total_size = 0, total_written = 0;

	for (i = 0; i < firmware->num_sections; i++)
		total_size += firmware->sections[i].size;

	LOG_OUTPUT("updating ... 0%%");

	for (i = 0; i < firmware->num_sections; i++)
	{
		size_t section_size;
		uint8_t *section_buffer;
		int remaining;
		uint32_t address;
		size_t size_read;
		int ret;

		section_size = firmware->sections[i].size;
		section_buffer = malloc(section_size);
		if (section_buffer == NULL)
		{
			LOG_ERROR("error allocating buffer for section (%d bytes)",
					  firmware->sections[i].size);
			return ERROR_FAIL;
		}

		ret = image_read_section(firmware, i, 0, section_size, section_buffer, &size_read);
		if (ret != ERROR_OK || size_read != section_size)
		{
			free(section_buffer);
			return ret;
		}

		crc = crc16_ccitt(section_buffer, section_size, crc);

		remaining = section_size;
		address = firmware->sections[i].base_address;

		while (remaining)
		{
			usb_command_block usb_cmd_blk;
			uint32_t count;
			int percentage;


			if (remaining < ICE_1000_FLASH_DATA_BUFFER_SIZE - 16)
				count = remaining;
			else
				count = ICE_1000_FLASH_DATA_BUFFER_SIZE - 16;
			remaining -= count;
			if (remaining == 0)
				last = 1;

			buffer[0] = first;
			buffer[1] = last;
			buffer[2] = HOST_PROGRAM_FLASH;
			buffer[3] = 0;
			memcpy(buffer + 4, &address, 4);
			memcpy(buffer + 8, &count, 4);
			memcpy(buffer + 12, &crc, 2);
			memcpy(buffer + 16, section_buffer + section_size - remaining - count, count);

			usb_cmd_blk.command = HOST_REQUEST_TX_DATA;
			usb_cmd_blk.count = count + 16;
			usb_cmd_blk.buffer = 0;
			adi_usb_write_or_ret(&usb_cmd_blk, sizeof (usb_cmd_blk));

			adi_usb_write_or_ret(buffer, usb_cmd_blk.count);

			first = 0;

			address += count;

			total_written += count;

			if (total_written == total_size)
				percentage = 100;
			else
				percentage = (int) (total_written * 100.0 / total_size);
			LOG_OUTPUT("\rupdating ... %d%%", percentage);
		}

		free(section_buffer);
	}

	*crcp = crc;

	LOG_OUTPUT("\r\n");

	return ERROR_OK;
}

static int ice1000_update_firmware(const char *filename)
{
	struct image ice1000_firmware_image;
	unsigned short crc1, crc2;
	int ret;

	LOG_INFO("Updating to firmware %s", filename);

	ice1000_firmware_image.base_address = 0;
	ice1000_firmware_image.base_address_set = 0;

	ret = image_open(&ice1000_firmware_image, filename, "ihex");
	if (ret != ERROR_OK)
		return ret;

	ret = ice1000_send_flash_data(&ice1000_firmware_image, &crc1);
	if (ret != ERROR_OK)
		return ret;

	if ((ret = ice1000_firmware_crc(&crc2)) != ERROR_OK)
		return ret;

	image_close(&ice1000_firmware_image);

	if (crc1 == crc2)
		return ERROR_OK;
	else
	{
		LOG_ERROR("CRCs do NOT match");
		return ERROR_FAIL;
	}
}

/*
 * This function sets us up the cable and data
 */
static int adi_connect(const uint16_t *vids, const uint16_t *pids)
{
	const char *cable_name = adi_cable_name();
	libusb_device_handle *dev;
	libusb_device *udev;
	struct libusb_config_descriptor *config;
	uint8_t configuration;
	char *firmware_filename	= get_firmware_filename();
	int i, ret;

	dev = NULL;
#ifdef _WIN32
	cable_params.mux_handle = NULL;
#endif

	if (cable_params.use_usbmux)
	{
#ifdef _WIN32
		ret = usbmux_open(&cable_params.mux_handle, ICE_1000_USB_CONNECTION_TIMEOUT);
		if (ret)
		{
			LOG_ERROR("failed to open USB MUX.");
			return ERROR_FAIL;
		}
#else
		LOG_ERROR("USB MUX is not supported on this host.");
		return ERROR_FAIL;
#endif
	}
	else
	{
		ret = jtag_libusb_open(vids, pids, NULL, &dev, NULL);
		if (ret != ERROR_OK)
			return ret;

		udev = libusb_get_device(dev);
		libusb_get_active_config_descriptor(udev, &config);
		configuration = config->bConfigurationValue;
		libusb_free_config_descriptor (config);
		libusb_set_configuration(dev, configuration);
		ret = libusb_claim_interface(dev, 0);
		if (ret)
		{
			LOG_ERROR("libusb_claim_interface failed: %d", ret);
			libusb_close(dev);
			return ERROR_FAIL;
		}

		LOG_DEBUG("usb interface claimed!");

		/* For an unknown reason, this is needed for using ICE-1000/2000
		with xHCI controller on Linux. */
		libusb_set_interface_alt_setting (dev, 0, 0);
	}

	cable_params.tap_info.dat = malloc(sizeof(dat_dat) * DAT_SZ);
	if (!cable_params.tap_info.dat)
	{
		LOG_ERROR("_malloc(%d) fails", (int)(sizeof(dat_dat) * DAT_SZ));
		if (dev)
		{
			libusb_release_interface(dev, 0);
			libusb_close(dev);
			dev = NULL;
		}

#ifdef _WIN32
		if (cable_params.mux_handle)
		{
			usbmux_close(cable_params.mux_handle);
			cable_params.mux_handle = NULL;
		}
#endif
		return ERROR_FAIL;
	}

	/* Initialize receive data array to unused */
	for (i = 0; i < DAT_SZ; ++i)
	{
		cable_params.tap_info.dat[i].idx = -1;
		cable_params.tap_info.dat[i].pos = -1;
	}

	cable_params.usb_handle			= dev;
	cable_params.tap_info.bit_pos	= 0x80;
	cable_params.tap_info.num_dat	= DAT_SZ;
	cable_params.tap_info.rcv_dat	= -1;
	cable_params.tap_info.cur_dat	= -1;

	cable_params.default_scanlen	= ICE_DEFAULT_SCAN_LEN;
	cable_params.trigger_scanlen	= ICE_TRIGGER_SCAN_LEN;
	cable_params.wr_ep				= ICE_1000_WRITE_ENDPOINT;
	cable_params.r_ep				= ICE_1000_READ_ENDPOINT;
	cable_params.wr_timeout			= ICE_1000_USB_WRITE_TIMEOUT;
	cable_params.r_timeout			= ICE_1000_USB_READ_TIMEOUT;
	cable_params.wr_buf_sz			= ICE_1000_WRITE_BUFFER_SIZE;
	cable_params.r_buf_sz			= ICE_1000_READ_BUFFER_SIZE;

	cable_params.version = do_host_cmd(HOST_GET_FW_VERSION, 0, 1);

	LOG_INFO("%s firmware version is %d.%d.%d",
			 cable_name,
			 ((cable_params.version >> 8) & 0xFF),
			 ((cable_params.version >> 4) & 0x0F),
			 ((cable_params.version)	  & 0x0F));

	if (cable_params.version <= 0x0101)
		LOG_WARNING("This firmware version is obsolete. Please update to the latest version.");

	if (firmware_filename)
	{
		ret = ice1000_update_firmware(firmware_filename);
		if (ret == ERROR_OK)
			LOG_INFO("The firmware has been updated successfully. "
					 "Please unplug the %s cable and reconnect it to finish the update process.", cable_name);
		else
			LOG_ERROR("The firmware failed to update.");
		return ERROR_JTAG_INIT_FAILED;
	}
	
	/* Set frequency to lowest value */
	if (strcmp (cable_name, "ICE-2000") == 0)
	{
		/* Turn on the voltage regulators */
		do_host_cmd(HOST_SET_2000_VOLTAGE, 1, 0);

		/* set interface mode to JTAG */
		do_host_cmd(HOST_SET_INTERFACE_MODE, 0, 0);

		/* If user has not set the voltage, default it to 3.3V. */
		if (cable_params.cur_voltage == 0)
			cable_params.cur_voltage = 3;

		if (cable_params.cur_voltage == 1)
			LOG_INFO("%s voltage 1.8V", cable_name);
		else if (cable_params.cur_voltage == 2)
			LOG_INFO("%s voltage 2.5V", cable_name);
		else /* cable_params.cur_voltage == 3 */
			LOG_INFO("%s voltage 3.3V", cable_name);

		/* Set the frequency to the lowest. */
		cable_params.cur_freq = avail_freqs_2000[0];

		/* Set the delay to 0. */
		ice2000_set_voltage_freq_delay(cable_params.cur_voltage, freq_set_2000[0], 0);
	}
	else if (strcmp (cable_name, "ICE-1000") == 0)
	{
		/* set interface mode to JTAG */
		do_host_cmd(HOST_SET_INTERFACE_MODE, 0, 0);

		ice1000_set_freq(avail_freqs_1000[0]);
	}

	/* HOST_HARD_RESET_JTAG_CTRLR will toggle TRST. This command has to be
	   sent after voltage regulators are turned on for ICE-2000. */
	do_host_cmd(HOST_HARD_RESET_JTAG_CTRLR, 0, 0);

	/* There is a bug in implementation of HOST_HARD_RESET_JTAG_CTRLR command
	   in 1.0.1 or earlier version firmware which does not hold TRST for enough
	   time. The processors, which ICE-1000/2000 usually work with, like legacy
	   Blackfin and ADSP-SC589 requires at least 4 JTAG CLKs. As a workaround,
	   we hold TRST low for 4 us, which should be enough even for the lowest
	   frequecy we can set, 1 MHz. */
	if (cable_params.version <= 0x0101)
	{
		do_host_cmd(HOST_SET_TRST, 1, 0);
		usleep(4);
		do_host_cmd(HOST_SET_TRST, 0, 0);
	}

	uint32_t value = 1;
	do_single_reg_value(REG_AUX, 1, 1, value);
	value = 0;
	do_single_reg_value(REG_AUX, 1, 1, value);

	cable_params.tap_pair_start_idx = SELECTIVE_RAW_SCAN_HDR_SZ;
	cable_params.max_raw_data_tx_items = cable_params.wr_buf_sz - cable_params.tap_pair_start_idx;
	cable_params.num_rcv_hdr_bytes = cable_params.tap_pair_start_idx;

	return ERROR_OK;
}

/*
 * takes tdi and tms and sends it out right away
 */
static int adi_clock(int32_t tms, int32_t tdi, int32_t cnt)
{
	num_tap_pairs *tap_info = &cable_params.tap_info;

	if (tap_info->pairs == NULL)
	{
		unsigned char *cmd;
		int32_t new_sz = cable_params.default_scanlen;
		uint8_t bit_set;
		int i, j;

		cmd = malloc((sizeof (tap_pairs) * new_sz) + 1 + cable_params.tap_pair_start_idx);
		if (cmd == NULL)
		{
			LOG_ERROR("malloc(%ld) fails",
					  (long int)(sizeof (tap_pairs) * new_sz) + 1 + cable_params.tap_pair_start_idx);
			return ERROR_FAIL;
		}

		tap_info->cmd = cmd;
		/* point our pairs to the space that was allocated */
		tap_info->pairs = (tap_pairs *)(cmd + cable_params.tap_pair_start_idx);	/* new pointer */

		/* initialize some of our structure */

		bit_set = 0x80;
		i = 0;
		tap_info->pairs[i].tms = 0;
		tap_info->pairs[i].tdi = 0;

		/* go through and set tms and tdi to the appropriate values */
		for (j = 0; j < cnt; j++)
		{
			tap_info->pairs[i].tms |= tms ? bit_set : 0;
			tap_info->pairs[i].tdi |= tdi ? bit_set : 0;
			bit_set >>= 1;
			if (!bit_set)
			{
				/* start over again */
				bit_set = 0x80;
				i++;
				tap_info->pairs[i].tms = 0;
				tap_info->pairs[i].tdi = 0;
			}
		}

		tap_info->total = new_sz;
		tap_info->cur_idx = cnt / 8;		/* we scan in multiples of 32 */
		tap_info->bit_pos = bit_set;

		return ERROR_OK;
	}
	else
	{
		int i, j;
		uint8_t bit_set;

		bit_set = tap_info->bit_pos;
		i = tap_info->cur_idx;

		for (j = 0; j < cnt; j++)
		{
			tap_info->pairs[i].tms |= tms ? bit_set : 0;
			tap_info->pairs[i].tdi |= tdi ? bit_set : 0;
			bit_set >>= 1;
			if (!bit_set)
			{
				/* start over again */
				bit_set = 0x80;
				i++;
				tap_info->pairs[i].tms = 0;
				tap_info->pairs[i].tdi = 0;
			}
		}

		tap_info->cur_idx = i;
		tap_info->bit_pos = bit_set;

		return ERROR_OK;
	}
}

static int ice1000_init(void)
{
	const uint16_t vids[] = { 0x064b, 0 };
	const uint16_t pids[] = { 0x0617, 0 };

	int retval;

	retval = adi_connect(vids, pids);
	if (retval != ERROR_OK)
	{
		if (retval == -ENODEV)
			LOG_ERROR("ICE-1000 emulator not found");

		LOG_ERROR("cannot connect to ICE-1000 emulator");
	}

	return retval;
}

static int ice2000_init(void)
{
	const uint16_t vids[] = { 0x064b, 0 };
	const uint16_t pids[] = { 0x0283, 0 };

	int retval;

	retval = adi_connect(vids, pids);
	if (retval != ERROR_OK)
	{
		if (retval == -ENODEV)
			LOG_ERROR("ICE-2000 emulator not found");

		LOG_ERROR("cannot connect to ICE-2000 emulator");
	}

	return retval;
}

static int ice1000_quit(void)
{
	do_host_cmd(HOST_DISCONNECT, 0, 0);

	if (cable_params.usb_handle)
	{
		libusb_release_interface(cable_params.usb_handle, 0);
		libusb_close(cable_params.usb_handle);
	}

#ifdef _WIN32
	if (cable_params.mux_handle)
	{
		usbmux_close(cable_params.mux_handle);
	}
#endif

	free(cable_params.tap_info.dat);

	return ERROR_OK;
}

static int ice2000_quit(void)
{
	/* Turn off the voltage regulators */
	do_host_cmd(HOST_SET_2000_VOLTAGE, 0, 0);

	do_host_cmd(HOST_DISCONNECT, 0, 0);

	if (cable_params.usb_handle)
	{
		libusb_release_interface(cable_params.usb_handle, 0);
		libusb_close(cable_params.usb_handle);
	}

#ifdef _WIN32
	if (cable_params.mux_handle)
	{
		usbmux_close(cable_params.mux_handle);
	}
#endif

	free(cable_params.tap_info.dat);

	return ERROR_OK;
}

static int ice1000_speed(int speed)
{
	if (speed >= MAX_FREQ_1000 && speed < 0)
	{
		LOG_ERROR("bad speed %d, should between %d and %d.",
				  speed, 0, MAX_FREQ_1000 - 1);
		return ERROR_FAIL;
	}

	ice1000_set_freq(avail_freqs_1000[speed]);

	return ERROR_OK;
}

static int ice1000_speed_div(int speed, int *khz)
{
	*khz = avail_freqs_1000[speed] / 1000;
	return ERROR_OK;
}

static int ice1000_khz(int khz, int *speed)
{
	*speed = adi_get_freq(khz * 1000, MAX_FREQ_1000, &avail_freqs_1000[0]);
	return ERROR_OK;
}

static int ice2000_speed(int speed)
{
	if (speed >= MAX_FREQ_2000 && speed < 0)
	{
		LOG_ERROR("bad speed %d, should between %d and %d.",
				  speed, 0, MAX_FREQ_2000 - 1);
		return ERROR_FAIL;
	}

	return ice2000_set_freq(avail_freqs_2000[speed]);
}

static int ice2000_speed_div(int speed, int *khz)
{
	*khz = avail_freqs_2000[speed] / 1000;
	return ERROR_OK;
}

static int ice2000_khz(int khz, int *speed)
{
	*speed = adi_get_freq(khz * 1000, MAX_FREQ_2000, &avail_freqs_2000[0]);
	return ERROR_OK;
}

/*
 * Takes Data received (rcv_dataptr) and puts it in
 * todo date out transfer
 */
static uint8_t *get_recv_data(int32_t len, int32_t idx_dat, uint8_t *rcv_data)
{
	uint8_t *buf;
	num_tap_pairs *tap_info = &cable_params.tap_info;
	int32_t dat_idx = tap_info->dat[idx_dat].idx;
	uint8_t *rcvBuf = rcv_data + cable_params.num_rcv_hdr_bytes + dat_idx;
	int32_t bit_set = tap_info->dat[idx_dat].pos;
	int32_t i;

#ifdef DUMP_EACH_RCV_DATA
	DEBUG("Idx = %d; Read len = %d\n", dat_idx, len);
#endif

	buf = (uint8_t *)calloc(DIV_ROUND_UP(len, 8), 1);

	if (buf == NULL)
		LOG_ERROR("malloc(%d) fails", DIV_ROUND_UP(len, 8));

	if (idx_dat < 0)
	{
		DEBUG("get_recv_data(): No Received Data\n");
		return NULL;
	}

	for (i = 0; i < len; i++)
	{
		buf[i/8] |= ((*rcvBuf & bit_set) ? 1 : 0) << (i % 8);

#ifdef DUMP_EACH_RCV_DATA
		if (i % 8 == 0 && i != 0)
			DEBUG("%d", buf[i/8 - 1]);
		if (((i + 1) % 64) == 0)
			putchar('\n');
		else if (((i + 1) % 8) == 0)
			putchar(' ');
#endif
		bit_set >>= 1;

		if (!bit_set)
		{
			bit_set = 0x80;
			rcvBuf++;
			dat_idx++;
		}
	}

	/* this is set for getting the extra TDO bits */
	tap_info->dat[idx_dat].idx = dat_idx;
	tap_info->dat[idx_dat].pos = bit_set;

	return buf;
}

static int ice1000_tap_execute(void)
{
	num_tap_pairs *tap_info = &cable_params.tap_info;
	uint8_t *buf;
	int i, retval;

	if (tap_info->cur_idx == 0 && tap_info->bit_pos == 0x80
		&& tap_info->cur_dat == -1)
		return ERROR_OK;

	buf = NULL;
	perform_scan(&buf);

	retval = ERROR_OK;

	for (i = 0; i <= tap_info->cur_dat; i++)
	{
		uint8_t *buffer;
		struct scan_command *command = tap_info->dat[i].ptr;

		buffer = get_recv_data(jtag_scan_size(command), tap_info->rcv_dat, buf);
		tap_info->rcv_dat++;
		if (jtag_read_buffer(buffer, command) != ERROR_OK)
			return ERROR_JTAG_QUEUE_FAILED;

		free(buffer);
	}

	free(buf);
	if (tap_info->pairs)
	{
		free(tap_info->cmd);
		tap_info->pairs = NULL;
		tap_info->cmd = NULL;
	}
	tap_info->total = 0;
	tap_info->cur_idx = 0;
	tap_info->bit_pos = 0x80;
	tap_info->cur_dat = -1;
	tap_info->rcv_dat = -1;

	return retval;
}

static int ice1000_execute_reset(struct jtag_command *cmd)
{
	int retval;

	retval = ice1000_tap_execute();
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG_IO("reset trst: %i srst %i",
			cmd->cmd.reset->trst, cmd->cmd.reset->srst);

	if ((cmd->cmd.reset->trst == 1)
		|| (cmd->cmd.reset->srst
			&& (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
	{
		tap_set_state(TAP_RESET);
	}

	do_host_cmd(HOST_SET_TRST, cmd->cmd.reset->trst ? 0 : 1, 0);
	return ERROR_OK;
}

static void ice1000_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
	{
		tap_set_end_state(state);
	}
	else
	{
		LOG_ERROR("BUG: %s is not a valid end state", tap_state_name(state));
		exit(-1);
	}
}

static int ice1000_tap_ensure_space(unsigned int bits)
{
	int retval = ERROR_OK;

	if (cable_params.tap_info.cur_idx + DIV_ROUND_UP(bits, 8) >= cable_params.trigger_scanlen)
		retval = ice1000_tap_execute();

	return retval;
}

static int ice1000_tap_append_step(int tms, int tdi)
{
	int retval;
	retval = adi_clock(tms, tdi, 1);
	return retval;
}

static void ice1000_state_move(void)
{
	int i;
	int tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	uint8_t tms_scan_bits = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = 0; i < tms_scan_bits; i++)
	{
		tms = (tms_scan >> i) & 1;
		ice1000_tap_append_step(tms, 0);
	}

	tap_set_state(tap_get_end_state());
}

static void ice1000_path_move(int num_states, tap_state_t *path)
{
	int i;

	for (i = 0; i < num_states; i++)
	{
		if (path[i] == tap_state_transition(tap_get_state(), false))
		{
			ice1000_tap_append_step(0, 0);
		}
		else if (path[i] == tap_state_transition(tap_get_state(), true))
		{
			ice1000_tap_append_step(1, 0);
		}
		else
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
					tap_state_name(tap_get_state()), tap_state_name(path[i]));
			exit(-1);
		}

		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());
}

static int ice1000_runtest(int num_cycles)
{
	int retval, i;
	tap_state_t saved_end_state = tap_get_end_state();

	retval = ice1000_tap_ensure_space(num_cycles + 16);
	if (retval != ERROR_OK)
		return retval;

	if (tap_get_state() != TAP_IDLE)
	{
		ice1000_end_state(TAP_IDLE);
		ice1000_state_move();
	}

	for (i = 0; i < num_cycles; i++)
		ice1000_tap_append_step(0, 0);

	/* finish in end_state */
	ice1000_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
	{
		ice1000_state_move();
	}

	return ERROR_OK;
}

static int ice1000_execute_runtest(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("runtest %i cycles, end in %i",
			cmd->cmd.runtest->num_cycles,
			cmd->cmd.runtest->end_state);

	ice1000_end_state(cmd->cmd.runtest->end_state);

	ice1000_runtest(cmd->cmd.runtest->num_cycles);

	return ERROR_OK;
}

static int ice1000_execute_tlr_reset(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

	ice1000_end_state(cmd->cmd.statemove->end_state);
	ice1000_state_move();

	/* Move to Run-Test/Idle */
	ice1000_tap_append_step(0, 0);
	tap_set_state(TAP_IDLE);

	return ERROR_OK;
}

static int ice1000_execute_pathmove(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("pathmove: %i states, end in %i",
		cmd->cmd.pathmove->num_states,
		cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	ice1000_path_move(cmd->cmd.pathmove->num_states,
			cmd->cmd.pathmove->path);

	return ERROR_OK;
}

/*
 * This function takes CABLE_TRANSFER todo data,
 * and adds it to the tms/tdi scan structure
 * If reading data, sets that up too
 */
static int add_scan_data(int32_t num_bits, uint8_t *in, bool out, struct scan_command *command)
{
	int32_t bit_cnt	 = num_bits % 8;
	int32_t byte_cnt = (num_bits >> 3) + (bit_cnt ? 1 : 0);
	int32_t i, bit_set;
	tap_pairs *tap_scan = NULL;
	int32_t idx;
	num_tap_pairs *tap_info = &cable_params.tap_info;

	if (in == NULL)
		LOG_WARNING("NO IN DATA!!!%s", out ? " BUT there is out data!" : "");

	if (tap_info->pairs == NULL)
	{	/* really should never get here, but must not crash system. Would be rude */
		int32_t new_sz = cable_params.default_scanlen + 4;
		unsigned char *cmd;

		cmd = malloc((sizeof (tap_pairs) * new_sz) + 1 + cable_params.tap_pair_start_idx);
		if (cmd == NULL)
		{
			LOG_ERROR("malloc(%ld) fails",
					  (long int)(sizeof (tap_pairs) * new_sz) + 1 + cable_params.tap_pair_start_idx);
			return ERROR_FAIL;
		}

		tap_info->cur_dat = -1;
		tap_info->rcv_dat = -1;
		tap_info->bit_pos = 0x80;
		tap_info->total = new_sz;
		tap_info->cmd = cmd;	/* new pointer */
		tap_info->pairs = (tap_pairs *)(cmd + cable_params.tap_pair_start_idx);	/* new pointer */

		tap_scan = tap_info->pairs;
		tap_scan->tms = 0;
		tap_scan->tdi = 0;
		idx = tap_info->cur_idx = 1;	/* first pair is 0 */
		tap_scan++;
		tap_scan->tdi = 0;
		tap_scan->tms = 0;
	}
	else if ((tap_info->total - tap_info->cur_idx) < byte_cnt)
	{	/* to small, increase size! */
		unsigned char *cmd;
		int32_t new_sz;

		DEBUG("Reallocating scan_data\n");

		new_sz = tap_info->total + byte_cnt + 8;
		cmd = realloc(tap_info->cmd, (sizeof (tap_pairs) * new_sz) + 4 + cable_params.tap_pair_start_idx);
		if (cmd == NULL)
		{
			LOG_ERROR("realloc(%ld) fails",
				(long int)(sizeof (tap_pairs) * new_sz) + 4 + cable_params.tap_pair_start_idx);
			return ERROR_FAIL;
		}

		tap_info->total = new_sz;		/* resize size */
		tap_info->cmd = cmd;			/* new pointer */
		tap_info->pairs = (tap_pairs *)(cmd + cable_params.tap_pair_start_idx);	/* new pointer */

		tap_scan = tap_info->pairs;
		idx = tap_info->cur_idx;		/* to add on */
		tap_scan = &tap_info->pairs[idx];
	}
	else
	{
		idx = tap_info->cur_idx;		/* to add on */
		tap_scan = &tap_info->pairs[idx];
	}

	bit_set = tap_info->bit_pos;

	if (out)
	{	/* Setup where we start to read, can be more than 1 */
		if (tap_info->rcv_dat == -1)
		{
			tap_info->rcv_dat = 0;
		}
		tap_info->cur_dat++;
		if (tap_info->cur_dat >= tap_info->num_dat)
		{
			int32_t new_sz;
			dat_dat *datPtr;

			new_sz = tap_info->num_dat + DAT_SZ_INC;
			datPtr = realloc(tap_info->dat, sizeof (dat_dat) * new_sz);
			if (datPtr == NULL)
			{
				LOG_ERROR("realloc(%ld) fails",
					(long int)(sizeof (dat_dat) * new_sz));
				return ERROR_FAIL;
			}
			tap_info->dat = datPtr;
			tap_info->num_dat = new_sz;

		}
		tap_info->dat[tap_info->cur_dat].idx = idx;
		tap_info->dat[tap_info->cur_dat].pos = bit_set;
		tap_info->dat[tap_info->cur_dat].ptr = command;
	}

	/* Build Scan.	If command is NULL, IN is TMS. Otherwise,
	   TMS will always be zero except the last bit! */
	for (i = 0; i < num_bits; i++)
	{
		if (command != NULL)
		{
			tap_scan->tdi |= (in[i / 8] >> (i % 8)) & 0x1 ? bit_set : 0;
			if (i == num_bits - 1)
				tap_scan->tms |= bit_set;
		}
		else
			tap_scan->tms |= (in[i / 8] >> (i % 8)) & 0x1 ? bit_set : 0;

		bit_set >>= 1;
		if (!bit_set)
		{
			bit_set = 0x80;
			idx++;
			tap_scan++;
			tap_scan->tdi = 0;
			tap_scan->tms = 0;
		}
	}

	tap_info->cur_idx = idx;
	tap_info->bit_pos = bit_set;

	return ERROR_OK;
}

static int ice1000_scan(bool ir_scan, enum scan_type type, uint8_t *buffer,
		int scan_size, struct scan_command *command)
{
	tap_state_t saved_end_state;
	int retval;

	retval = ice1000_tap_ensure_space(scan_size + 16);
	if (retval != ERROR_OK)
		return retval;

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	ice1000_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	/* Only move if we're not already there */
	if (tap_get_state() != tap_get_end_state())
		ice1000_state_move();

	ice1000_end_state(saved_end_state);

	/* Scan */
	add_scan_data(scan_size, buffer, type != SCAN_OUT, command);

	/* We are in Exit1, go to Pause */
	ice1000_tap_append_step(0, 0);

	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
	{
		ice1000_state_move();
	}

	return ERROR_OK;
}

static int ice1000_execute_scan(struct jtag_command *cmd)
{
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	LOG_DEBUG_IO("scan end in %s", tap_state_name(cmd->cmd.scan->end_state));

	ice1000_end_state(cmd->cmd.scan->end_state);

	scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
	LOG_DEBUG_IO("scan input, length = %d", scan_size);

	type = jtag_scan_type(cmd->cmd.scan);
	ice1000_scan(cmd->cmd.scan->ir_scan,
			type, buffer, scan_size, cmd->cmd.scan);

	free(buffer);

	return ERROR_OK;
}

static int ice1000_execute_sleep(struct jtag_command *cmd)
{
	int retval;

	retval = ice1000_tap_execute();
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG_IO("sleep %" PRIi32 "", cmd->cmd.sleep->us);

	jtag_sleep(cmd->cmd.sleep->us);
	return ERROR_OK;
}

static int ice1000_execute_stableclocks(struct jtag_command *cmd)
{
	int tms;

	switch (tap_get_state()) {
	case TAP_RESET:
		/* tms must be '1' to stay
		 * n TAP_RESET mode
		 */
		tms = 1;
		break;
	case TAP_DRSHIFT:
	case TAP_IDLE:
	case TAP_DRPAUSE:
	case TAP_IRSHIFT:
	case TAP_IRPAUSE:
		/* else, tms should be '0' */
		tms = 0;
		break;
	default:
		return ERROR_FAIL;
	}

	adi_clock(tms, 0, cmd->cmd.stableclocks->num_cycles);

	return ERROR_OK;
}

static int ice1000_execute_tms(struct jtag_command *cmd)
{
	int num_bits = cmd->cmd.tms->num_bits;
	uint8_t *bits = (uint8_t *)cmd->cmd.tms->bits;
	int count = DIV_ROUND_UP(num_bits, 8);
	int retval;

	retval = ice1000_tap_ensure_space(count);
	if (retval != ERROR_OK)
		return retval;

	retval = add_scan_data(num_bits, bits, false, NULL);

	return retval;
}

static int ice1000_execute_command(struct jtag_command *cmd)
{
	int retval;

	switch (cmd->type)
	{
	case JTAG_RESET:
		retval = ice1000_execute_reset(cmd);
		break;
	case JTAG_RUNTEST:
		retval = ice1000_execute_runtest(cmd);
		break;
	case JTAG_TLR_RESET:
		retval = ice1000_execute_tlr_reset(cmd);
		break;
	case JTAG_PATHMOVE:
		retval = ice1000_execute_pathmove(cmd);
		break;
	case JTAG_SCAN:
		retval = ice1000_execute_scan(cmd);
		break;
	case JTAG_SLEEP:
		retval = ice1000_execute_sleep(cmd);
		break;
	case JTAG_STABLECLOCKS:
		retval = ice1000_execute_stableclocks(cmd);
		break;
	case JTAG_TMS:
		retval = ice1000_execute_tms(cmd);
		break;
	default:
		LOG_ERROR("BUG: unknown JTAG command type encountered");
		retval = ERROR_JTAG_QUEUE_FAILED;
		break;
	}
	return retval;
}

static int ice1000_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int retval = ERROR_OK;

#ifdef _WIN32
#define USB_MUX_MAX_LOCK_ATTEMPTS 50
	if (cable_params.mux_handle)
	{
		unsigned int attempt = 0;
		do {
			attempt++;

			// attempt to acquire the USB lock
			USB_MUX_ERROR mux_ret = usbmux_lock(cable_params.mux_handle);

			if (mux_ret == USB_MUX_OK)
			{
				break;
			}
			else if (mux_ret == USB_MUX_BUSY)
			{
				/* Send out the message every 5 attempts */
				if ((attempt % 5) == 0)
				{
					LOG_DEBUG("MUX is busy after %u attempts, retrying.", attempt);
				}

				if (attempt == USB_MUX_MAX_LOCK_ATTEMPTS)
				{
					// Failed to acquire lock (TIMEOUT)
					LOG_ERROR("Timeout acquiring USB lock after %u attempts", attempt);
					return ERROR_TIMEOUT;
				}
			}
			else
			{
				LOG_ERROR("USB error: Failed to acquire USB lock (error %d).", mux_ret);
				return ERROR_FAIL;
			}
			usleep(100000);
			keep_alive();
		} while (1);
	}
#endif

	/* TODO add blink */
	while (cmd != NULL)
	{
		if (ice1000_execute_command(cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;
		cmd = cmd->next;
	}

	if (retval != ERROR_OK)
	{
#ifdef _WIN32
		if (cable_params.mux_handle)
		{
			// release USB lock
			usbmux_unlock(cable_params.mux_handle);
		}
#endif
		return retval;
	}

	retval = ice1000_tap_execute();

#ifdef _WIN32
	if (cable_params.mux_handle)
	{
		// release USB lock
		usbmux_unlock(cable_params.mux_handle);
	}
#endif

	return retval;
}

/*
 * Read & Write Registers
 *
 * XXX: error handling doesn't quite work with this return
 * XXX: probably needs converting from memory arrays to byte shifts
 *      so we work regardless of host endian
 */
static uint32_t do_single_reg_value(uint8_t reg, int32_t r_data, int32_t wr_data, uint32_t data)
{
	usb_command_block usb_cmd_blk;
	union {
		uint8_t b[24];
		uint32_t l[6];
	} cmd_buffer;
	uint32_t count = 0;
	int32_t i, size = wr_data ? 8 : 4;

	usb_cmd_blk.command = HOST_REQUEST_TX_DATA;
	usb_cmd_blk.count = size;
	usb_cmd_blk.buffer = 0;

	adi_usb_write_or_ret((uint8_t*)&usb_cmd_blk, sizeof(usb_cmd_blk));
	i = 0;

	/* send HOST_SET_SINGLE_REG command */
	cmd_buffer.b[i++] = 1;
	cmd_buffer.b[i++] = 0;
	cmd_buffer.b[i++] = wr_data ? HOST_SET_SINGLE_REG : HOST_GET_SINGLE_REG;
	cmd_buffer.b[i++] = reg;
	if (wr_data)
	{
		cmd_buffer.l[i / 4] = data;
	}

	adi_usb_write_or_ret(cmd_buffer.b, size);

	if (r_data)
		adi_usb_read_or_ret((uint8_t*)&count, sizeof (count));

	return count;
}

/*
 * Send Host Command.
 *
 * XXX: error handling doesn't quite work with this return
 * XXX: probably needs converting from memory arrays to byte shifts
 *      so we work regardless of host endian
 */
static uint16_t do_host_cmd(uint8_t cmd, uint8_t param, int32_t r_data)
{
	usb_command_block usb_cmd_blk;
	uint16_t results = 0;
	union {
		uint8_t b[20];
		uint32_t l[20/4];
	} cmd_buffer;
	int32_t size = 5;

	usb_cmd_blk.command = HOST_REQUEST_TX_DATA;
	usb_cmd_blk.count = size;
	usb_cmd_blk.buffer = 0;

	adi_usb_write_or_ret((uint8_t*)&usb_cmd_blk, sizeof(usb_cmd_blk));

	/* send command */
	cmd_buffer.b[0] = 0;
	cmd_buffer.b[1] = 0;
	cmd_buffer.b[2] = cmd;
	cmd_buffer.b[3] = 0;
	cmd_buffer.b[4] = 0;

	if (cmd == HOST_SET_TRST)
		cmd_buffer.b[0] = param;
	else
		cmd_buffer.b[4] = param;

	adi_usb_write_or_ret(cmd_buffer.b, size);

	if (r_data)
	{
		usb_cmd_blk.command = HOST_REQUEST_RX_DATA;
		usb_cmd_blk.count = 2;
		usb_cmd_blk.buffer = 0;

		adi_usb_write_or_ret((uint8_t*)&usb_cmd_blk, sizeof (usb_cmd_blk));

		adi_usb_read_or_ret ((uint8_t*)&results, sizeof (results));
	}

	return results;
}

/*
 *    Controlling function to do a scan.
 *    rdata is a pointer to storage for the pointer
 *    allocated here to return data if needed
 */
static int perform_scan(uint8_t **rdata)
{
	num_tap_pairs *tap_info = &cable_params.tap_info;
	uint8_t firstpkt = 1, lastpkt = 0, *in = NULL, *out = NULL;
	int32_t idx, collect_data = 0;
	uint32_t cur_len = tap_info->cur_idx;
	uint32_t rem_len;

	/* Data is scan as 32 bit words, so boundaries are adjusted here */
	if (tap_info->bit_pos != 0x80) /* meaning no dangling bits? */
	{	/* yes, so straighten out! */
		cur_len++;
		tap_info->pairs[cur_len].tms = 0;
		tap_info->pairs[cur_len].tdi = 0;
	}

	/* Pad with zeros */
	cur_len++;
	tap_info->pairs[cur_len].tms = 0;
	tap_info->pairs[cur_len].tdi = 0;

	while (cur_len & 0x03)
	{	/* expect to be in 32 bit words */
		cur_len++;
		tap_info->pairs[cur_len].tms = 0;
		tap_info->pairs[cur_len].tdi = 0;
	}

	tap_info->cur_idx = cur_len;
	rem_len = cur_len * sizeof (tap_pairs);

	if (cur_len > cable_params.default_scanlen)
	{
		LOG_ERROR("TAP Scan length %d is greater than DIF Memory",
			tap_info->cur_idx);
		return ERROR_FAIL;
	}

	if (tap_info->cur_dat != -1)
	{	/* yes we have data, so allocate for data plus header */
		size_t len;

		len = cur_len + cable_params.tap_pair_start_idx + 16;
		if (tap_info->dat[0].idx > 12)
			len -= tap_info->dat[0].idx;

		out = malloc(len);
		if (out == NULL)
		{
			LOG_ERROR("malloc(%ld) fails", (long int)len);
			return ERROR_FAIL;
		}
		*rdata = out;
		collect_data = 1;
	}
	else
	{	/* no data, so allocate for just header */
		out = malloc(cable_params.tap_pair_start_idx + 16);
		if (out == NULL)
		{
			LOG_ERROR("malloc(%d) fails", cable_params.tap_pair_start_idx + 16);
			return ERROR_FAIL;
		}
		collect_data = 0;
	}

	in = (uint8_t *)tap_info->pairs;
	idx = 0;

	/* Here if data is too large, we break it up into manageable chunks */
	do
	{
		cur_len = (rem_len >= cable_params.max_raw_data_tx_items) ? cable_params.max_raw_data_tx_items : rem_len;

		if (cur_len == rem_len)
			lastpkt = 1;

		do_rawscan(firstpkt, lastpkt, collect_data, cur_len, &in[idx] - cable_params.tap_pair_start_idx, out);

		rem_len -= cur_len;
		idx += cur_len;
		firstpkt = 0;

	} while (rem_len);

	if (tap_info->cur_dat == -1)
	{	/* no data to return, so free it */
		free(out);
	}

	return ERROR_OK;
}

/*
 *    description of raw scan packet structure:
 *
 *        [0]        : first packet flag (do setup work if needed)
 *        [1]        : last packet flag (start the scan and cleanup if needed)
 *        [2]        : command ID
 *        [3]        : collect DOF flag (need to read DOF)
 *        [4-5]      : DIF count
 *        [6-7]      : scan length count
 *        [8-9]      : first scan pair
 *        [10...]    : more scan pairs
 *
 *    Data input:
 *
 *        firstpkt   : Is this the first packet of the scan? 0 = NO
 *        lastpkt    : Is this the last packet of the scan? 0 = NO
 *        collect_dof: Are we collecting data?  0 = NO
 *        dif_cnt    : Number of bytes to send
 *        *raw_buf   : Pointer to Scan Data buffer * cmd to send
 *        *out       : Pointer to Scan Data buffer to receive
 *
 * XXX: probably needs converting from memory arrays to byte shifts
 *      so we work regardless of host endian
 */
static int do_rawscan(uint8_t firstpkt, uint8_t lastpkt,
					  int32_t collect_dof, int32_t dif_cnt,
					  uint8_t *raw_buf, uint8_t *out)
{
	usb_command_block usb_cmd_blk;
	num_tap_pairs *tap_info = &cable_params.tap_info;
	int32_t i, dof_start = 0;
	uint32_t data;
	uint32_t size = cable_params.tap_pair_start_idx + dif_cnt;

	usb_cmd_blk.command = HOST_REQUEST_TX_DATA;
	usb_cmd_blk.count = size;
	usb_cmd_blk.buffer = 0;

	/* first send Xmit request with the count of what will be sent */
	adi_usb_write_or_ret((uint8_t*)&usb_cmd_blk, sizeof (usb_cmd_blk));
	i = 0;

	/* send HOST_DO_SELECTIVE_RAW_SCAN command */
	raw_buf[i++] = firstpkt;
	raw_buf[i++] = lastpkt;
	raw_buf[i++] = HOST_DO_SELECTIVE_RAW_SCAN;
	if ((collect_dof && lastpkt) && (tap_info->dat[0].idx > 12))
	{
		int32_t j, offset;

		dof_start = tap_info->dat[0].idx;
		offset = dof_start & 7;
		dof_start -= offset & 7;
		tap_info->dat[0].idx = offset;

		for (j = 1; j <= tap_info->cur_dat; j++)
		{
			tap_info->dat[j].idx -= dof_start;
		}
	}

	raw_buf[i++] = collect_dof ? 1 : 0;
	data = dif_cnt / 4;			/* dif count in longs */
	memcpy(raw_buf + i, &data, 4);
	data = tap_info->cur_idx / 4;  /* count in longs */
	memcpy(raw_buf + i + 2, &data, 4);

	{	/* only Ice emulators use this */
		memcpy(raw_buf + i + 4, &dof_start, 4);
	}

	adi_usb_write_or_ret(raw_buf, size);

	if (lastpkt)
	{
		int32_t cur_rd_bytes = 0, tot_bytes_rd = 0, rd_bytes_left;

		rd_bytes_left = cable_params.num_rcv_hdr_bytes + ((collect_dof) ? (tap_info->cur_idx - dof_start) : 0);

		while (tot_bytes_rd < rd_bytes_left)
		{
			cur_rd_bytes = ((rd_bytes_left - tot_bytes_rd) > cable_params.r_buf_sz) ?
				cable_params.r_buf_sz : (rd_bytes_left - tot_bytes_rd);

			adi_usb_read_or_ret((uint8_t*)(out + tot_bytes_rd), cur_rd_bytes);
			tot_bytes_rd += cur_rd_bytes;
		}

		if (out[0] != 2)
		{
			LOG_ERROR("Scan Error!");
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(ice2000_handle_voltage_command)
{
	uint32_t voltage;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], voltage);

	if (voltage == 0 || voltage > 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* This command can only be used before adi_connect */
	if (cable_params.usb_handle)
		return ERROR_FAIL;

	cable_params.cur_voltage = voltage;

	return ERROR_OK;
}

COMMAND_HANDLER(ice1000_use_usbmux)
{
	bool use_usbmux;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_BOOL(CMD_ARGV[0], use_usbmux, "true", "false");

	/* This command can only be used before adi_connect */
	if (cable_params.usb_handle)
		return ERROR_FAIL;

	cable_params.use_usbmux = use_usbmux;

	return ERROR_OK;
}

static struct jtag_interface ice1000_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = ice1000_execute_queue,
};

static const struct command_registration ice1000_command_handlers[] = {
	{
		.name = "use_usbmux",
		.handler = &ice1000_use_usbmux,
		.mode = COMMAND_CONFIG,
		.usage = "use_usbmux ['true'|'false']",
	},

	COMMAND_REGISTRATION_DONE
};

struct adapter_driver ice1000_adapter_driver = {
	.name = "ice1000",
	.transports = jtag_only,
	.commands = ice1000_command_handlers,

	.init = ice1000_init,
	.quit = ice1000_quit,
	.speed = ice1000_speed,
	.khz = ice1000_khz,
	.speed_div = ice1000_speed_div,

	.jtag_ops = &ice1000_interface,	
};

static const struct command_registration ice2000_command_handlers[] = {
	{
		.name = "ice2000_voltage",
		.handler = &ice2000_handle_voltage_command,
		.mode = COMMAND_CONFIG,
		.usage = "voltage ['1'|'2'|'3']",
	},

	{
		.name = "use_usbmux",
		.handler = &ice1000_use_usbmux,
		.mode = COMMAND_CONFIG,
		.usage = "use_usbmux ['true'|'false']",
	},

	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface ice2000_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = ice1000_execute_queue,
};

struct adapter_driver ice2000_adapter_driver = {
	.name = "ice2000",
	.transports = jtag_only,
	.commands = ice2000_command_handlers,

	.init = ice2000_init,
	.quit = ice2000_quit,
	.speed = ice2000_speed,
	.khz = ice2000_khz,
	.speed_div = ice2000_speed_div,

	.jtag_ops = &ice2000_interface,	
};
