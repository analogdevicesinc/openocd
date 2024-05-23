/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2021-2024 Analog Devices, Inc.                          *
 ***************************************************************************/

#ifndef _USB_MUX_H
#define _USB_MUX_H

#include <stdint.h>
#include <stdbool.h>

#ifdef _WIN32
#include <windows.h>

#define DLLEXPORT __declspec(dllexport)

#else

#define DLLEXPORT
typedef unsigned long DWORD;
typedef void *HANDLE;
#endif

#define USB_MUX_BUFFER_SIZE 0x9800
#define USB_MUX_PIPE_NAME TEXT("\\\\.\\pipe\\usbmux")
#define USB_MUX_DEFAULT_PIPE_CONNECT_TIMEOUT 20000 // 20 seconds

/* TYPE + NUMBYTES + ENDPOINT + TIMEOUT */
#define USB_MUX_MSG_HEADER_SIZE 0x1 + 0x4 + 0x4 + 0x4
#define USB_MUX_MSG_HEADER_NUMBYTES_INDEX 1
#define USB_MUX_MSG_HEADER_ENDPOINT_INDEX USB_MUX_MSG_HEADER_NUMBYTES_INDEX + 4
#define USB_MUX_MSG_HEADER_TIMEOUT_INDEX USB_MUX_MSG_HEADER_ENDPOINT_INDEX + 4

#define USB_MUX_RESPONSE_SIZE 4

typedef enum {
	USB_MUX_OK = 0,
	USB_MUX_FAIL = -1,
	USB_MUX_BUFFER_OVERRUN = -2,
	USB_MUX_TIMEOUT = -3,
	USB_MUX_BUSY = -4,
} USB_MUX_ERROR;

typedef enum {
	READ,
	WRITE,
	LOCK,
	UNLOCK,
	GET_PID
} UsbMuxMessageType;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Open a handle to the USB MUX named pipe.
 * @param[out] handle Pointer to the USB MUX named pipe handle
 * @param timeout Named pipe connection timeout
 * @return return code to indicate success or failure
 */
DLLEXPORT DWORD usbmux_open(HANDLE *handle, DWORD timeout);

/**
 * Close the handle to the USB MUX named pipe.
 * @param handle USB MUX named pipe handle
 */
DLLEXPORT void usbmux_close(HANDLE handle);

/**
 * Read data from the given USB endpoint (blocking).
 * @param handle USB MUX named pipe handle
 * @param[out] buffer Pointer to the buffer to receive the data
 * @param num_bytes Number of bytes to read
 * @param endpoint USB endpoint to read from
 * @param timeout Read timeout (milliseconds)
 * @return USB_MUX_ERROR code to indicate success or failure
 */
DLLEXPORT USB_MUX_ERROR usbmux_read(HANDLE handle, uint8_t *buffer, uint32_t num_bytes, uint32_t endpoint, uint32_t timeout);

/**
 * Write data to the given USB endpoint (blocking).
 * @param handle USB MUX named pipe handle
 * @param[in] buffer Pointer to the buffer of data to write
 * @param num_bytes Number of bytes to write
 * @param endpoint USB endpoint to write to
 * @param timeout Write timeout (milliseconds)
 * @return USB_MUX_ERROR code to indicate success or failure
 */
DLLEXPORT USB_MUX_ERROR usbmux_write(HANDLE handle, uint8_t *buffer, uint32_t num_bytes, uint32_t endpoint, uint32_t timeout);

/**
 * Lock the USB MUX connection (blocking).
 * @param handle USB MUX named pipe handle
 * @return USB_MUX_ERROR code to indicate success or failure
 */
DLLEXPORT USB_MUX_ERROR usbmux_lock(HANDLE handle);

/**
 * Unlock the USB MUX connection.
 * @param handle USB MUX named pipe handle
 * @return USB_MUX_ERROR code to indicate success or failure
 */
DLLEXPORT USB_MUX_ERROR usbmux_unlock(HANDLE handle);

/**
 * Get the product ID for the connected USB device
 * @param handle USB MUX named pipe handle
 * @param[out] pProductId Pointer to the product ID
 * @return USB_MUX_ERROR code to indicate success or failure
 */
DLLEXPORT USB_MUX_ERROR usbmux_get_product_id(HANDLE handle, uint16_t *product_id);

#ifdef __cplusplus
}
#endif

#endif // _USB_MUX_H
