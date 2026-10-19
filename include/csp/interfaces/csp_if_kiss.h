/****************************************************************************
 * **File:** csp/interfaces/csp_if_kiss.h
 *
 * **Description:** KISS interface (serial).
 ****************************************************************************/
#pragma once

#include <csp/csp_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Default name of KISS interface.
 */
#define CSP_IF_KISS_DEFAULT_NAME "KISS"

/**
 * Send KISS frame (implemented by driver).
 *
 * @param[in] driver_data driver data from #csp_iface_t
 * @param[in] data data to send
 * @param[in] len length of \a data.
 * @return #CSP_ERR_NONE on success, otherwise an error code.
 */
typedef int (*csp_kiss_driver_tx_t)(void *driver_data, const uint8_t * data, size_t len);

/**
 * Hand a whole packet to the driver's transmit thread (implemented by driver, optional).
 *
 * Called on the sender's thread by csp_kiss_tx(); must not block. On
 * #CSP_ERR_NONE the driver owns the packet and will transmit it with
 * csp_kiss_tx_packet_sync() (which frees it) or free it. On any other
 * return the packet stays with the caller (csp_send_direct_iface(), which
 * frees it and counts tx_error).
 *
 * @param[in] driver_data driver data from #csp_iface_t
 * @param[in] packet packet to transmit
 * @return #CSP_ERR_NONE when queued, otherwise an error code.
 */
typedef int (*csp_kiss_driver_tx_enqueue_t)(void * driver_data, csp_packet_t * packet);

/**
 * KISS Rx mode/state.
 */
typedef enum {
	KISS_MODE_NOT_STARTED,  /**< No start detected */
	KISS_MODE_STARTED,      /**< Started on a KISS frame */
	KISS_MODE_ESCAPED,      /**< Rx escape character */
	KISS_MODE_SKIP_FRAME,   /**< Skip remaining frame, wait for end character */
} csp_kiss_mode_t;

/**
 * KISS interface data (state information).
 */
typedef struct {
	csp_kiss_driver_tx_t tx_func; /**< Tx function */
	csp_kiss_driver_tx_enqueue_t tx_enqueue; /**< Optional: hand whole packets to a driver TX thread; NULL = transmit synchronously on the caller's thread */
	csp_kiss_mode_t rx_mode; /**< Rx mode/state. */
	unsigned int rx_length; /**< Rx length */
	bool rx_first; /**< Rx first - if set, waiting for first character
						(== TNC_DATA) after start */
	csp_packet_t * rx_packet; /**< CSP packet for storing Rx data. */
} csp_kiss_interface_data_t;

/**
 * Add interface.
 *
 * @param[in] iface CSP interface, initialized with name and
 * 								inteface_data pointing to a valid #csp_kiss_interface_data_t.
 * @return #CSP_ERR_NONE on success, otherwise an error code.
 */
int csp_kiss_add_interface(csp_iface_t * iface);

/**
 * Send CSP packet over KISS (nexthop).
 *
 * @return #CSP_ERR_NONE on success, otherwise an error code.
 */
int csp_kiss_tx(csp_iface_t * iface, uint16_t via, csp_packet_t * packet, int from_me);

/**
 * Encode and transmit one packet on the calling thread.
 *
 * The whole KISS frame (FEND, TNC_DATA, escaped header + data, FEND) is
 * built in one buffer and handed to the driver's tx_func in a single call,
 * under csp_usart_lock(). csp_kiss_tx() calls this when the driver has no
 * tx_enqueue hook; a driver's TX thread calls it from the hook's queue.
 *
 * Ownership: the packet is freed on #CSP_ERR_NONE and left to the caller on
 * failure, the same rule as csp_can_tx_packet_sync().
 *
 * @param[in] iface KISS interface
 * @param[in] packet packet to transmit; its id must already be in packet->id
 * @return #CSP_ERR_NONE on success, otherwise the driver's error code.
 */
int csp_kiss_tx_packet_sync(csp_iface_t * iface, csp_packet_t * packet);

/**
 * Process received CAN frame.
 *
 * Called from driver when a chunk of data has been received. Once a complete
 * frame has been received, the CSP packet will be routed on.
 *
 * @param[in] iface incoming interface.
 * @param[in] buf received data.
 * @param[in] len length of \a buf.
 * @param[out] pxTaskWoken Valid reference if called from ISR, otherwise NULL!
 */
void csp_kiss_rx(csp_iface_t * iface, const uint8_t * buf, size_t len, void * pxTaskWoken);

#ifdef __cplusplus
}
#endif
