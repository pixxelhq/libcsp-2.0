/****************************************************************************
 * **File:** csp/drivers/usart.h
 *
 * **Description:** USART driver
 *
 * .. note:: This interface implementation only support ONE open UART connection.
 ****************************************************************************/

#pragma once

#include <csp/interfaces/csp_if_kiss.h>

#if (CSP_ZEPHYR)
#include <zephyr/device.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * OS file handle.
 */
#if (CSP_ZEPHYR)
typedef const struct device * csp_usart_fd_t;
#else
typedef int csp_usart_fd_t;
#endif

/**
 * Usart configuration.
 * @see csp_usart_open()
 */
typedef struct csp_usart_conf {
	const char * device;   /**< USART device.*/
	uint32_t baudrate;     /**< bits per second. */
	uint8_t databits;      /**< Number of data bits. */
	uint8_t stopbits;      /**< Number of stop bits. */
	uint8_t paritysetting; /**< Parity setting. */
} csp_usart_conf_t;

/**
 * Callback for returning data to application.
 *
 * @param[in] user_data User data
 * @param[in] buf data received.
 * @param[in] len data length (number of bytes in \a buf).
 * @param[out] pxTaskWoken Valid reference if called from ISR, otherwise NULL!
 */
typedef void (*csp_usart_callback_t)(void * user_data, uint8_t * buf, size_t len, void * pxTaskWoken);

/**
 * Opens an UART device.
 * Opens the UART device and creates a thread for reading/returning data to the application.
 *
 * .. note:: On read failure, exit() will be called - terminating the process.
 *
 * @param[in] conf UART configuration.
 * @param[in] rx_callback receive data callback.
 * @param[in] user_data reference forwarded to the rx_callback function.
 * @param[out] fd the opened file descriptor.
 * @return #CSP_ERR_NONE on success, otherwise an error code.
 */
int csp_usart_open(const csp_usart_conf_t * conf, csp_usart_callback_t rx_callback, void * user_data, csp_usart_fd_t * fd);

/**
 * Write data on open UART.
 *
 * @param[in] data Outbound data to send over the USART interface.
 * @param[in] data_length Outbound data length.
 * @return number of bytes written on success, a negative value on failure.
 */
int csp_usart_write(csp_usart_fd_t fd, const void * data, size_t data_length);

/**
 * Lock the device, so only a single user can write to the serial port at a time
 *
 * @param[in] driver_data Driver data
 */
void csp_usart_lock(void * driver_data);

/**
 * Unlock the USART again
 *
 * @param[in] driver_data Driver data
 */
void csp_usart_unlock(void * driver_data);

/**
 * Opens UART device and add KISS interface.
 *
 * This is a convenience function for opening an UART device and adding it as an interface with a given name.
 *
 * .. note:: On read failures, exit() will be called - terminating the process.
 *
 * @param[in] conf UART configuration.
 * @param[in] ifname interface name (will be copied), or use NULL for default name.
 * @param[in] addr CSP address of the interface.
 * @param[out] return_iface the added interface.
 * @return #CSP_ERR_NONE on success, otherwise an error code.
 */
int csp_usart_open_and_add_kiss_interface(const csp_usart_conf_t * conf, const char * ifname, uint16_t addr, csp_iface_t ** return_iface);

/** Packets a KISS interface's transmit queue holds. Each interface opened by
 *  csp_usart_open_and_add_kiss_interface() has one, drained by its own TX
 *  thread, so a sender never writes to the tty itself: it pushes the packet
 *  (a pointer, no copy) and returns at once. A full queue refuses the packet
 *  whole (csp_usart_kiss_tx_queue_full_count()). Same model and length as
 *  CSP_CAN_SOCKETCAN_TX_QUEUE_LEN. */
#define CSP_USART_KISS_TX_QUEUE_LEN 64

/**
 * Set the scheduling priority of KISS TX threads created from now on.
 *
 * > 0 asks for SCHED_FIFO at that priority on POSIX (clamped to the policy's
 * range; a refusal falls back to default scheduling); 0 (the default) creates
 * a plain thread. Call before csp_usart_open_and_add_kiss_interface().
 *
 * @param[in] priority SCHED_FIFO priority, or 0 for default scheduling.
 */
void csp_usart_kiss_set_tx_priority(int priority);

/** @return the value last given to csp_usart_kiss_set_tx_priority(), 0 by default */
int csp_usart_kiss_tx_priority(void);

/**
 * Packets refused because the interface's transmit queue was full. Nothing
 * of them reached the wire; each is also counted in iface->tx_error.
 * @return the count, or 0 when @p iface is not a KISS interface of this driver.
 */
uint32_t csp_usart_kiss_tx_queue_full_count(const csp_iface_t * iface);

/**
 * Deepest the transmit queue has been, in packets (headroom indicator;
 * near CSP_USART_KISS_TX_QUEUE_LEN the next burst drops).
 * @return the depth, or 0 when @p iface is not a KISS interface of this driver.
 */
uint32_t csp_usart_kiss_tx_queue_depth_max(const csp_iface_t * iface);

/**
 * Packets the TX thread dropped unsent because their csp_packet_t::timestamp_tx
 * deadline (a csp_get_ms() value, 0 = none) had passed when they were
 * dequeued. Always whole: once a frame is on the wire the packet is finished.
 * Each is also counted in iface->tx_error.
 * @return the count, or 0 when @p iface is not a KISS interface of this driver.
 */
uint32_t csp_usart_kiss_tx_deadline_giveup_count(const csp_iface_t * iface);

/**
 * Platform hook: start a detached thread running @p fn(@p arg) for a KISS
 * interface's transmit queue. POSIX implements it (SCHED_FIFO when
 * @p priority > 0); a platform without threads returns #CSP_ERR_NOTSUP and
 * the interface then transmits synchronously on the caller's thread.
 *
 * .. note:: There is no stop path: like the RX thread, it lives as long as
 *           the process.
 *
 * @param[in] fn thread body
 * @param[in] arg passed to @p fn
 * @param[in] name interface name, used for the thread name
 * @param[in] priority SCHED_FIFO priority, or 0 for default scheduling
 * @return #CSP_ERR_NONE on success, otherwise an error code.
 */
int csp_usart_tx_thread_create(void * (*fn)(void *), void * arg, const char * name, int priority);

#ifdef __cplusplus
}
#endif
