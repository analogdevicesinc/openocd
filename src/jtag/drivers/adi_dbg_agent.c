/***************************************************************************
*   Copyright (C) 2021-2023 Analog Devices, Inc.                          *
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

/*
 * Debug Macros
 */

#if 1    /* set to 1 to output debug info about scans */

//#define DSP_SCAN_DATA
//#define DUMP_EACH_RCV_DATA
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

#define RAW_SCAN_HDR_SZ					8

#define DAT_SZ							0x4000	/* size allocated for reading data */
#define DAT_SZ_INC						0x40	/* size to increase if data full */

/* USB Emulator Commands */
#define HOST_GET_FW_VERSION				0x01	/* get the firmware version */
#define HOST_REQUEST_RX_DATA			0x02	/* host request to transmit data */
#define HOST_RX_DATA                   	0x03	/* host has requested to receive data */
#define HOST_REQUEST_TX_DATA			0x04	/* host request to transmit data */
#define HOST_TX_DATA                   	0x05	/* host has requested to transmit data */
#define HOST_DO_RAW_SCAN               	0x06	/* do a raw scan */
#define HOST_DO_LOOPBACK				0x07	/* usb loopback testing */
#define HOST_HARD_RESET_KIT				0x08
#define HOST_SET_TRST                  	0x09	/* changes TRST Line state  */

#define HOST_PREP_FIRMWARE_UPDATE      	0x0A	/* prepare to update the firmware */
#define HOST_READ_EEPROM               	0x0B	/* read the target's EEPROM */
#define HOST_WRITE_EEPROM              	0x0C	/* write to the target's EEPROM */
#define HOST_DISCONNECT					0x0E	/* disconnect from debug mode */

/* Ice USB controls */
#define WRITE_ENDPOINT			0x02
#define READ_ENDPOINT			0x01
#define USB_WRITE_TIMEOUT		10000
#define USB_CONNECTION_TIMEOUT	10000
#define USB_READ_TIMEOUT		30000
#define WRITE_BUFFER_SIZE		0x4800
#define READ_BUFFER_SIZE		0x4000
#define MAX_DIF_SIZE			(27 * 1024)     /* 0x7008 is the max but leave some room */

/* Latest firmware version for Debug Agent */
#define CURRENT_USBDA_FW_VERSION	0x0009

#define MAX_USB_IDS 8
/* vid = pid = 0 marks the end of the list */
static uint16_t dbgagent_vid[MAX_USB_IDS + 1] = { 0 };
static uint16_t dbgagent_pid[MAX_USB_IDS + 1] = { 0 };

/*
 * Internal Macros
 */

#ifdef _WIN32
#define adi_usb_read_or_ret(buf, len)										\
	do {																	\
		int __ret, __actual, __size = (len);							\
		__ret = libusb_bulk_transfer(cable_params.usb_handle,			\
								cable_params.r_ep | LIBUSB_ENDPOINT_IN, \
								(unsigned char *)(buf), __size,			\
								&__actual, cable_params.r_timeout);		\
		if (__ret || __actual != __size)								\
		{																\
			LOG_ERROR("unable to read from usb to " #buf ": "			\
					"wanted %i bytes but only received %i bytes",		\
					__size, __actual);									\
			LOG_ERROR("return %d",__ret);								\
			return ERROR_FAIL;											\
		}																\
	} while (0)

#define adi_usb_write_or_ret(buf, len)										\
	do {																	\
		int __ret, __actual, __size = (len);							\
		__ret = libusb_bulk_transfer(cable_params.usb_handle,			\
								cable_params.wr_ep | LIBUSB_ENDPOINT_OUT, \
								(unsigned char *)(buf), __size,			\
								&__actual, cable_params.wr_timeout);	\
		if (__ret || __actual != __size)								\
		{																\
			LOG_ERROR("unable to write from " #buf " to usb: "			\
					"wanted %i bytes but only wrote %i bytes",			\
					__size, __actual);									\
			LOG_ERROR("return %d",__ret);								\
			return ERROR_FAIL;											\
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

	if (strcmp(adapter_driver->name, "dbgagent") == 0)
		return "ADI Debug Agent";
	else
		return "unknown";
}

/* TEST_DATA_LENGTH % 64 should be 0 */
#define TEST_DATA_LENGTH 0x8000

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
	int i, ret;

	dev = NULL;

	ret = jtag_libusb_open(vids, pids, &dev, NULL);
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
	cable_params.wr_ep				= WRITE_ENDPOINT;
	cable_params.r_ep				= READ_ENDPOINT;
	cable_params.wr_timeout			= USB_WRITE_TIMEOUT;
	cable_params.r_timeout			= USB_READ_TIMEOUT;
	cable_params.wr_buf_sz			= WRITE_BUFFER_SIZE;
	cable_params.r_buf_sz			= READ_BUFFER_SIZE;

	cable_params.version = do_host_cmd(HOST_GET_FW_VERSION, 0, 1);

	LOG_INFO("%s firmware version is %d.%d.%d",
			 cable_name,
			 ((cable_params.version >> 8) & 0xFF),
			 ((cable_params.version >> 4) & 0x0F),
			 ((cable_params.version)	  & 0x0F));

	if (cable_params.version < CURRENT_USBDA_FW_VERSION)
		LOG_WARNING("This firmware version is obsolete. Please update to the latest version.");

	do_host_cmd(HOST_SET_TRST, 1, 0);
	usleep(4);
	do_host_cmd(HOST_SET_TRST, 0, 0);

	cable_params.tap_pair_start_idx = RAW_SCAN_HDR_SZ;
	cable_params.max_raw_data_tx_items = cable_params.wr_buf_sz - cable_params.tap_pair_start_idx;
	cable_params.num_rcv_hdr_bytes = 3;  // this is where our TDO actually starts

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

static int dbgagent_init(void)
{
	int retval;

	retval = adi_connect(dbgagent_vid, dbgagent_pid);
	if (retval != ERROR_OK)
	{
		if (retval == -ENODEV)
			LOG_ERROR("Debug agent not found");

		LOG_ERROR("cannot connect to the debug agent");
	}

	return retval;
}

static int dbgagent_quit(void)
{
	do_host_cmd(HOST_SET_TRST, 0, 0);

	// indicate to the debug agent that we are shutting down
	do_host_cmd(HOST_DISCONNECT, 0, 0);

	if (cable_params.usb_handle != NULL)
	{
		libusb_release_interface(cable_params.usb_handle, 0);
		libusb_close(cable_params.usb_handle);
	}

	free(cable_params.tap_info.dat);

	return ERROR_OK;
}

static int dbgagent_speed(int speed)
{
	return ERROR_OK;
}

static int dbgagent_khz(int khz, int *speed)
{
	*speed = 0;
	return ERROR_OK;
}

static int dbgagent_speed_div(int speed, int *khz)
{
	*khz = 5000;
	return ERROR_OK;
}
/*
 * Takes Data received (rcv_dataptr) and puts it in
 * todo data out transfer
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
			DEBUG("%x", buf[i/8 - 1]);
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

static int dbgagent_tap_execute(void)
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

static int dbgagent_execute_reset(struct jtag_command *cmd)
{
	int retval;

	retval = dbgagent_tap_execute();
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

static void dbgagent_end_state(tap_state_t state)
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

static int dbgagent_tap_ensure_space(unsigned int bits)
{
	int retval = ERROR_OK;

	if (cable_params.tap_info.cur_idx + DIV_ROUND_UP(bits, 8) >= cable_params.trigger_scanlen)
		retval = dbgagent_tap_execute();

	return retval;
}

static int dbgagent_tap_append_step(int tms, int tdi)
{
	int retval;
	retval = adi_clock(tms, tdi, 1);
	return retval;
}

static void dbgagent_state_move(void)
{
	int i;
	int tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	uint8_t tms_scan_bits = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = 0; i < tms_scan_bits; i++)
	{
		tms = (tms_scan >> i) & 1;
		dbgagent_tap_append_step(tms, 0);
	}

	tap_set_state(tap_get_end_state());
}

static void dbgagent_path_move(int num_states, tap_state_t *path)
{
	int i;

	for (i = 0; i < num_states; i++)
	{
		if (path[i] == tap_state_transition(tap_get_state(), false))
		{
			dbgagent_tap_append_step(0, 0);
		}
		else if (path[i] == tap_state_transition(tap_get_state(), true))
		{
			dbgagent_tap_append_step(1, 0);
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

static int dbgagent_runtest(int num_cycles)
{
	int retval, i;
	tap_state_t saved_end_state = tap_get_end_state();

	retval = dbgagent_tap_ensure_space(num_cycles + 16);
	if (retval != ERROR_OK)
		return retval;

	if (tap_get_state() != TAP_IDLE)
	{
		dbgagent_end_state(TAP_IDLE);
		dbgagent_state_move();
	}

	for (i = 0; i < num_cycles; i++)
		dbgagent_tap_append_step(0, 0);

	/* finish in end_state */
	dbgagent_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
	{
		dbgagent_state_move();
	}

	return ERROR_OK;
}

static int dbgagent_execute_runtest(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("runtest %i cycles, end in %i",
			cmd->cmd.runtest->num_cycles,
			cmd->cmd.runtest->end_state);

	dbgagent_end_state(cmd->cmd.runtest->end_state);

	dbgagent_runtest(cmd->cmd.runtest->num_cycles);

	return ERROR_OK;
}

static int dbgagent_execute_tlr_reset(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

	dbgagent_end_state(cmd->cmd.statemove->end_state);
	dbgagent_state_move();

	/* Move to Run-Test/Idle */
	dbgagent_tap_append_step(0, 0);
	tap_set_state(TAP_IDLE);

	return ERROR_OK;
}

static int dbgagent_execute_pathmove(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("pathmove: %i states, end in %i",
		cmd->cmd.pathmove->num_states,
		cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	dbgagent_path_move(cmd->cmd.pathmove->num_states,
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

static int dbgagent_scan(bool ir_scan, enum scan_type type, uint8_t *buffer,
		int scan_size, struct scan_command *command)
{
	tap_state_t saved_end_state;
	int retval;

	retval = dbgagent_tap_ensure_space(scan_size + 16);
	if (retval != ERROR_OK)
		return retval;

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	dbgagent_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	/* Only move if we're not already there */
	if (tap_get_state() != tap_get_end_state())
		dbgagent_state_move();

	dbgagent_end_state(saved_end_state);

	/* Scan */
	add_scan_data(scan_size, buffer, type != SCAN_OUT, command);

	/* We are in Exit1, go to Pause */
	dbgagent_tap_append_step(0, 0);

	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
	{
		dbgagent_state_move();
	}

	return ERROR_OK;
}

static int dbgagent_execute_scan(struct jtag_command *cmd)
{
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	LOG_DEBUG_IO("scan end in %s", tap_state_name(cmd->cmd.scan->end_state));

	dbgagent_end_state(cmd->cmd.scan->end_state);

	scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
	LOG_DEBUG_IO("scan input, length = %d", scan_size);

	type = jtag_scan_type(cmd->cmd.scan);
	dbgagent_scan(cmd->cmd.scan->ir_scan,
			type, buffer, scan_size, cmd->cmd.scan);

	free(buffer);

	return ERROR_OK;
}

static int dbgagent_execute_sleep(struct jtag_command *cmd)
{
	int retval;

	retval = dbgagent_tap_execute();
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG_IO("sleep %" PRIi32 "", cmd->cmd.sleep->us);

	jtag_sleep(cmd->cmd.sleep->us);
	return ERROR_OK;
}

static int dbgagent_execute_stableclocks(struct jtag_command *cmd)
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

static int dbgagent_execute_tms(struct jtag_command *cmd)
{
	int num_bits = cmd->cmd.tms->num_bits;
	uint8_t *bits = (uint8_t *)cmd->cmd.tms->bits;
	int count = DIV_ROUND_UP(num_bits, 8);
	int retval;

	retval = dbgagent_tap_ensure_space(count);
	if (retval != ERROR_OK)
		return retval;

	retval = add_scan_data(num_bits, bits, false, NULL);

	return retval;
}

static int dbgagent_execute_command(struct jtag_command *cmd)
{
	int retval;

	switch (cmd->type)
	{
	case JTAG_RESET:
		retval = dbgagent_execute_reset(cmd);
		break;
	case JTAG_RUNTEST:
		retval = dbgagent_execute_runtest(cmd);
		break;
	case JTAG_TLR_RESET:
		retval = dbgagent_execute_tlr_reset(cmd);
		break;
	case JTAG_PATHMOVE:
		retval = dbgagent_execute_pathmove(cmd);
		break;
	case JTAG_SCAN:
		retval = dbgagent_execute_scan(cmd);
		break;
	case JTAG_SLEEP:
		retval = dbgagent_execute_sleep(cmd);
		break;
	case JTAG_STABLECLOCKS:
		retval = dbgagent_execute_stableclocks(cmd);
		break;
	case JTAG_TMS:
		retval = dbgagent_execute_tms(cmd);
		break;
	default:
		LOG_ERROR("BUG: unknown JTAG command type encountered");
		retval = ERROR_JTAG_QUEUE_FAILED;
		break;
	}
	return retval;
}

static int dbgagent_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int retval = ERROR_OK;

	/* TODO add blink */
	while (cmd != NULL)
	{
		if (dbgagent_execute_command(cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;
		cmd = cmd->next;
	}

	if (retval != ERROR_OK)
	{
		return retval;
	}

	retval = dbgagent_tap_execute();

	return retval;
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

		adi_usb_read_or_ret((uint8_t*)&results, sizeof (results));
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
	uint8_t firstpkt = 1, lastpkt = 1, *in = NULL, *out = NULL;
	int32_t idx_in, idx_out, collect_data = 0;
	uint32_t cur_len = tap_info->cur_idx;
	uint32_t rem_len;
	uint32_t scan_status_bytes = 3;		/* number of bytes letting us know if the scan was successful */
	uint32_t out_inc = 0;
		
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
	idx_in = 0;
	idx_out = 0;

	/* Here if data is too large, we break it up into manageable chunks */
	do
	{
		cur_len = (rem_len > MAX_DIF_SIZE) ? MAX_DIF_SIZE : rem_len;

		do_rawscan(firstpkt, lastpkt, collect_data, cur_len, &in[idx_in] - cable_params.tap_pair_start_idx, &out[idx_out]);
		if(idx_in != 0)
		{
			// each scan gives us scan status, remove it from our buffer
			// if it is not the first scan
			uint8_t *pData = &out[idx_out + scan_status_bytes];
			for(size_t i = 0; i < cur_len/2 + scan_status_bytes; i++)
			{
				out[idx_out+i] = pData[i];
			}
			out_inc = (cur_len/2);	
		}
		else
		{
			out_inc = (cur_len/2) + scan_status_bytes;	
		}

		rem_len -= cur_len;
		idx_in += cur_len;
		idx_out += out_inc;
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
	int32_t num_scan_pairs = (dif_cnt >= 2) ? dif_cnt / 2 : 2;
	int32_t scan_pairs_in_longs = (num_scan_pairs >= 4) ? num_scan_pairs / 4 : 1;

	usb_cmd_blk.command = HOST_REQUEST_TX_DATA;
	usb_cmd_blk.count = size;
	usb_cmd_blk.buffer = 0;

	/* first send Xmit request with the count of what will be sent */
	adi_usb_write_or_ret((uint8_t *)&usb_cmd_blk, sizeof (usb_command_block));
	i = 0;

	/* send HOST_DO_RAW_SCAN command */
	raw_buf[i++] = firstpkt;
	raw_buf[i++] = lastpkt;
	raw_buf[i++] = HOST_DO_RAW_SCAN;
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
	data = (dif_cnt >= 4) ? dif_cnt / 4 : 1;			/* dif count in longs */
	memcpy(raw_buf + i, &data, 4);
	memcpy(raw_buf + i + 2, &scan_pairs_in_longs, 2);

	adi_usb_write_or_ret(raw_buf, size);

	if (lastpkt)
	{
		int32_t cur_rd_bytes = 0, tot_bytes_rd = 0, rd_bytes_left;
		int32_t buf_index = 0;

		rd_bytes_left = RAW_SCAN_HDR_SZ + ((collect_dof) ? ((scan_pairs_in_longs * 4) - dof_start) : 0);

		while (tot_bytes_rd < rd_bytes_left)
		{
			cur_rd_bytes = ((rd_bytes_left - tot_bytes_rd) > cable_params.r_buf_sz) ?
				cable_params.r_buf_sz : (rd_bytes_left - tot_bytes_rd);

			adi_usb_read_or_ret(out + buf_index, cur_rd_bytes);
			if ((out + buf_index)[0] != 2)
			{
				LOG_ERROR("Scan Error!");
				return ERROR_FAIL;
			}
			tot_bytes_rd += cur_rd_bytes;
			buf_index += (cur_rd_bytes - 8);
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(dbgagent_handle_vid_pid_command)
{
	if (CMD_ARGC > MAX_USB_IDS * 2) {
		LOG_WARNING("ignoring extra IDs in dbgagent_vid_pid "
			"(maximum is %d pairs)", MAX_USB_IDS);
		CMD_ARGC = MAX_USB_IDS * 2;
	}
	if (CMD_ARGC < 2 || (CMD_ARGC & 1)) {
		LOG_WARNING("incomplete dbgagent_vid_pid configuration directive");
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		/* remove the incomplete trailing id */
		CMD_ARGC -= 1;
	}

	unsigned i;
	for (i = 0; i < CMD_ARGC; i += 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i], dbgagent_vid[i >> 1]);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], dbgagent_pid[i >> 1]);
	}

	/*
	 * Explicitly terminate, in case there are multiples instances of
	 * dbgagent_vid_pid.
	 */
	dbgagent_vid[i >> 1] = dbgagent_pid[i >> 1] = 0;

	return ERROR_OK;
}

static const struct command_registration dbgagent_command_handlers[] = {
	{
		.name = "dbgagent_vid_pid",
		.handler = &dbgagent_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of the debug agent",
		.usage = "(vid pid)* ",
	},

	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface dbgagent_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = dbgagent_execute_queue,
};

struct adapter_driver dbgagent_adapter_driver = {
	.name = "dbgagent",
	.transports = jtag_only,
	.commands = dbgagent_command_handlers,

	.init = dbgagent_init,
	.quit = dbgagent_quit,
	.speed = dbgagent_speed,
	.khz = dbgagent_khz,
	.speed_div = dbgagent_speed_div,

	.jtag_ops = &dbgagent_interface,	
};
