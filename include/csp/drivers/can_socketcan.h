/****************************************************************************
 * **File:** csp/drivers/csp_socketcan.h
 *
 * **Description:** Socket CAN driver (Linux).
 *
 * .. note:: This driver requires the libsocketcan library.
 ****************************************************************************/
#pragma once

#include <csp/interfaces/csp_if_can.h>
#include <linux/can.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Open CAN socket and add CSP interface.
 *
 * Parameters:
 * @param[in] device CAN device name (Linux device).
 * @param[in] ifname CSP interface name, use #CSP_IF_CAN_DEFAULT_NAME for default name.
 * @param[in] node_id CSP address of the interface.
 * @param[in] bitrate if different from 0, it will be attempted to change the
 *            bitrate on the CAN device - this may require increased OS privileges.
 * @param[in] promisc if true, receive all CAN frames. If false a filter
 *                    is set on the CAN device, using device->addr
 * @param[out] return_iface the added interface.
 * @return The added interface, or NULL in case of failure.
 */
int csp_can_socketcan_open_and_add_interface(const char * device, const char * ifname, unsigned int node_id, int bitrate, bool promisc, csp_iface_t ** return_iface);

/**
 * Initialize socketcan and add CSP interface.
 *
 * :bdg-warning-line:`deprecated` version 1.6, use csp_can_socketcan_open_and_add_interface()
 *
 * Parameters:
 * @param[in] device CAN device name (Linux device).
 * @param[in] node_id CSP address of the interface.
 * @param[in] bitrate if different from 0, it will be attempted to change the
 *            bitrate on the CAN device - this may require increased OS privileges.
 * @param[in] promisc if true, receive all CAN frames. If false a filter
 *                    is set on the CAN device, using device->addr
 * @return The added interface, or NULL in case of failure.
 */
csp_iface_t * csp_can_socketcan_init(const char * device, unsigned int node_id, int bitrate, bool promisc);

/**
 * Stop the Rx thread and free resources (testing).
 *
 * .. note:: This will invalidate CSP, because an interface can't be removed.
 *			 This is primarily for testing.
 *
 * Parameters:
 * @param[in] iface interface to stop.
 * @return #CSP_ERR_NONE on success, otherwise an error code.
 */
int csp_can_socketcan_stop(csp_iface_t * iface);

/**
 * Build the CAN_RAW_FILTER set for a socketcan interface.
 *
 * The kernel treats the array as a disjunction, so "addressed to me OR
 * broadcast" is expressed as separate entries. For CSP 1.x this yields the
 * interface address and the 5-bit broadcast address (one entry when they
 * coincide); for CSP 2.x the interface address, the subnet broadcast and the
 * global broadcast. In promiscuous mode a single accept-all entry is built.
 *
 * @param[in] version CSP protocol version (1 or 2)
 * @param[in] addr interface address
 * @param[in] netmask interface netmask in bits (CSP 2.x subnet broadcast)
 * @param[in] promisc true to accept every frame
 * @param[out] filters array to fill
 * @param[in] max_filters capacity of @p filters, at least 3
 * @return number of filters written, or #CSP_ERR_INVAL if @p filters is too small.
 */
int csp_can_socketcan_build_filters(uint8_t version, uint16_t addr, uint16_t netmask, bool promisc, struct can_filter * filters, unsigned int max_filters);

/**
 * Times a transmit had to back off because the socket buffer was full (ENOBUFS).
 * Measures qdisc/txqueuelen pressure; the retry loop itself only reports total failure.
 * @return count, 0 for NULL or a non-socketcan interface.
 */
uint32_t csp_can_socketcan_tx_enobufs_count(const csp_iface_t * iface);

/**
 * Controller RX FIFO overruns (CAN_ERR_CRTL_RX_OVERFLOW) seen on this interface.
 * The controller drops isolated frames in hardware and reports them only as an
 * error frame, so this is the one loss path invisible to every other counter.
 * Each overrun is also counted under csp_iface_t::rx_error.
 * @return count, 0 for NULL or a non-socketcan interface.
 */
uint32_t csp_can_socketcan_rx_overrun_count(const csp_iface_t * iface);

#ifdef __cplusplus
}
#endif
