// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2021-2024 Analog Devices, Inc.                          *
 ***************************************************************************/

#include "usbmux.h"

#ifndef _WIN32
#warning USB MUX only supported on Windows
#endif

/**
	* Open a handle to the USB MUX named pipe.
	* @param[out] handle Pointer to the USB MUX named pipe handle
	* @param timeout Named pipe connection timeout
	* @return return code to indicate success or failure
	*/
DWORD usbmux_open(HANDLE * handle, DWORD timeout)
{
#ifdef _WIN32
	while (true) {
		*handle = CreateFile(USB_MUX_PIPE_NAME,
				GENERIC_READ | GENERIC_WRITE,
				0,
				NULL,
				OPEN_EXISTING,
				0,
				NULL);

		// Break if we have a valid pipe handle
		if (*handle != INVALID_HANDLE_VALUE)
			break;

		if (GetLastError() != ERROR_PIPE_BUSY)
			return (DWORD)USB_MUX_FAIL;

		if (!WaitNamedPipe(USB_MUX_PIPE_NAME, timeout))
			return (DWORD)USB_MUX_TIMEOUT;
	}

	// Connection successful, set pipe mode
	DWORD mode = PIPE_READMODE_MESSAGE;
	BOOL success = SetNamedPipeHandleState(*handle,
			&mode,
			NULL,
			NULL);

	if (!success)
		return (DWORD)USB_MUX_FAIL;

#endif
	return (DWORD)USB_MUX_OK;
}

/**
	* Close the handle to the USB MUX named pipe.
	* @param handle USB MUX named pipe handle
	*/
void usbmux_close(HANDLE handle)
{
#ifdef _WIN32
	CloseHandle(handle);
#endif
}

/**
	* Populate the given header with the appropriate values in the header format.
	* @param[out] header The header buffer to populate
	* @param message_type The message type
	* @param num_bytes The number of bytes to be sent in the message (after the header)
	* @param endpoint The USB endpoint to communicate with
	* @param timeout The USB read / write timeout
	*/
void usbmux_create_header(uint8_t *header, UsbMuxMessageType message_type, uint32_t num_bytes, uint32_t endpoint, uint32_t timeout)
{
#ifdef _WIN32
	if (!header)
		return;

	header[0] = (uint8_t)message_type;
	for (int i = 0; i < 4; i++) {
		header[i + 1] = (uint8_t)((num_bytes >> i * 8) & 0xFF);
		header[i + 5] = (uint8_t)((endpoint >> i * 8) & 0xFF);
		header[i + 9] = (uint8_t)((timeout >> i * 8) & 0xFF);
	}
#endif
}

/**
	* Read data from the given USB endpoint (blocking).
	* @param handle USB MUX named pipe handle
	* @param[out] buffer Pointer to the buffer to receive the data
	* @param num_bytes Number of bytes to read
	* @param endpoint USB endpoint to read from
	* @param timeout Read timeout (milliseconds)
	* @return USB_MUX_ERROR code to indicate success or failure
	*/
USB_MUX_ERROR usbmux_read(HANDLE handle, uint8_t *buffer, uint32_t num_bytes, uint32_t endpoint, uint32_t timeout)
{
#ifdef _WIN32
	if (num_bytes > USB_MUX_BUFFER_SIZE)
		return USB_MUX_BUFFER_OVERRUN;

	BOOL ret = FALSE;
	DWORD bytes_actual = 0;
	uint8_t header[USB_MUX_MSG_HEADER_SIZE];

	usbmux_create_header(header, READ, num_bytes, endpoint, timeout);

	// Send the read message header
	ret = WriteFile(handle,
			header,
			USB_MUX_MSG_HEADER_SIZE,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != USB_MUX_MSG_HEADER_SIZE)
		return USB_MUX_FAIL;

	// Read the response from the server
	DWORD response;
	ret = ReadFile(handle,
			&response,
			USB_MUX_RESPONSE_SIZE,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != USB_MUX_RESPONSE_SIZE || !response)
		return USB_MUX_FAIL;

	// Read back the data
	ret = ReadFile(handle,
			buffer,
			num_bytes,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != num_bytes)
		return USB_MUX_FAIL;

#endif
	return USB_MUX_OK;
}

/**
	* Write data to the given USB endpoint (blocking).
	* @param pData Pointer to the USB MUX interface data
	* @param buffer Pointer to the buffer of data to write
	* @param num_bytes Number of bytes to write
	* @param endpoint USB endpoint to write to
	* @param timeout Write timeout (milliseconds)
	* @return USB_MUX_ERROR code to indicate success or failure
	*/
USB_MUX_ERROR usbmux_write(HANDLE handle, uint8_t *buffer, uint32_t num_bytes, uint32_t endpoint, uint32_t timeout)
{
#ifdef _WIN32
	if (num_bytes > USB_MUX_BUFFER_SIZE)
		return USB_MUX_BUFFER_OVERRUN;

	BOOL ret = FALSE;
	DWORD bytes_actual = 0;
	uint8_t header[USB_MUX_MSG_HEADER_SIZE];

	usbmux_create_header(header, WRITE, num_bytes, endpoint, timeout);

	// Send the write message header
	ret = WriteFile(handle,
			header,
			USB_MUX_MSG_HEADER_SIZE,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != USB_MUX_MSG_HEADER_SIZE)
		return USB_MUX_FAIL;

	ret = WriteFile(handle,
			buffer,
			num_bytes,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != num_bytes)
		return USB_MUX_FAIL;

	// Read the response from the server
	DWORD response;
	ret = ReadFile(handle,
			&response,
			USB_MUX_RESPONSE_SIZE,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != USB_MUX_RESPONSE_SIZE || !response)
		return USB_MUX_FAIL;

#endif
	return USB_MUX_OK;
}

/**
	* Lock the USB MUX connection.
	*/
USB_MUX_ERROR usbmux_lock(HANDLE handle)
{
#ifdef _WIN32
	BOOL ret = FALSE;
	DWORD response = 0;
	DWORD bytes_actual = 0;
	uint8_t header[USB_MUX_MSG_HEADER_SIZE];

	usbmux_create_header(header, LOCK, 0, 0, 0);

	ret = WriteFile(handle,
			header,
			USB_MUX_MSG_HEADER_SIZE,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != USB_MUX_MSG_HEADER_SIZE)
		return USB_MUX_FAIL;

	// Read the response from the server
	ret = ReadFile(handle,
			&response,
			USB_MUX_RESPONSE_SIZE,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != USB_MUX_RESPONSE_SIZE)
		return USB_MUX_FAIL;

	if (!response) {
		// return busy if already locked
		return USB_MUX_BUSY;
	}
#endif
	return USB_MUX_OK;
}

/**
	* Unlock the USB MUX connection.
	*/
USB_MUX_ERROR usbmux_unlock(HANDLE handle)
{
#ifdef _WIN32
	BOOL ret = FALSE;
	DWORD bytes_actual = 0;
	uint8_t header[USB_MUX_MSG_HEADER_SIZE];

	usbmux_create_header(header, UNLOCK, 0, 0, 0);

	// Send the unlock message header
	ret = WriteFile(handle,
			header,
			USB_MUX_MSG_HEADER_SIZE,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != USB_MUX_MSG_HEADER_SIZE)
		return USB_MUX_FAIL;

	// Read the response from the server
	DWORD response;
	ret = ReadFile(handle,
			&response,
			USB_MUX_RESPONSE_SIZE,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != USB_MUX_RESPONSE_SIZE || !response)
		return USB_MUX_FAIL;

#endif
	return USB_MUX_OK;
}

/**
	* Get the product ID for the connected USB device
	* @param handle USB MUX named pipe handle
	* @param[out] product_id Pointer to the product ID
	* @return USB_MUX_ERROR code to indicate success or failure
	*/
USB_MUX_ERROR usbmux_get_product_id(HANDLE handle, uint16_t *product_id)
{
#ifdef _WIN32
	BOOL ret = FALSE;
	DWORD bytes_actual = 0;
	uint8_t header[USB_MUX_MSG_HEADER_SIZE];

	usbmux_create_header(header, GET_PID, 0, 0, 0);

	// Send the 'GET_PID' message header
	ret = WriteFile(handle,
			header,
			USB_MUX_MSG_HEADER_SIZE,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != USB_MUX_MSG_HEADER_SIZE)
		return USB_MUX_FAIL;

	// Read the response from the server
	DWORD response;
	ret = ReadFile(handle,
			&response,
			USB_MUX_RESPONSE_SIZE,
			&bytes_actual,
			NULL);

	if (!ret || bytes_actual != USB_MUX_RESPONSE_SIZE || !response)
		return USB_MUX_FAIL;

	*product_id = (uint16_t)response;
#endif
	return USB_MUX_OK;
}

