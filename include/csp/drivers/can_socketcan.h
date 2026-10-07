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

/** Sleep between two attempts to write a frame the kernel would not take. */
#define CSP_CAN_SOCKETCAN_TX_BACKOFF_MS 5
/** Total back-off a single frame may spend before the packet is abandoned. */
#define CSP_CAN_SOCKETCAN_TX_BUDGET_MS 1000
/** SO_SNDBUF requested at open. The kernel clamps it to net.core.wmem_max. */
#define CSP_CAN_SOCKETCAN_SNDBUF_BYTES (4 * 1024 * 1024)

/** What csp_can_socketcan_tx_backoff() tells the transmit loop to do next. */
typedef enum {
	CSP_CAN_SOCKETCAN_TX_RETRY_NOW,   /**< EINTR: write again at once, budget untouched */
	CSP_CAN_SOCKETCAN_TX_RETRY_SLEEP, /**< buffers full: sleep CSP_CAN_SOCKETCAN_TX_BACKOFF_MS, then write again */
	CSP_CAN_SOCKETCAN_TX_GIVE_UP,     /**< budget exhausted: abandon the packet */
	CSP_CAN_SOCKETCAN_TX_FAIL,        /**< hard error (or bad argument): abandon the packet */
} csp_can_socketcan_tx_action_t;

/**
 * Decide how the transmit loop reacts to a failed write().
 *
 * The socket is non-blocking, so two different "no room" conditions reach
 * the driver: ENOBUFS when the netdev qdisc (txqueuelen) is full and EAGAIN
 * when the per-socket send buffer (SO_SNDBUF) is full. Both are the controller
 * not draining fast enough, and both back off for CSP_CAN_SOCKETCAN_TX_BACKOFF_MS
 * against one shared CSP_CAN_SOCKETCAN_TX_BUDGET_MS. Returning GIVE_UP only
 * once @p waiting_ms has reached the budget means a packet is abandoned after
 * the budget has genuinely been slept, never after a burst of instantaneous
 * retries. EINTR is retried without charging the budget.
 *
 * @param[in] errnum errno from the failed write()
 * @param[in,out] waiting_ms back-off already spent on this frame; advanced on RETRY_SLEEP
 * @return the action to take
 */
csp_can_socketcan_tx_action_t csp_can_socketcan_tx_backoff(int errnum, uint32_t * waiting_ms);

/**
 * Times a transmit had to back off because the netdev qdisc was full (ENOBUFS).
 * Measures txqueuelen pressure. Zero here with csp_can_socketcan_tx_eagain_count()
 * climbing means the socket send buffer, not the qdisc, is the choke point.
 * @return count, 0 for NULL or a non-socketcan interface.
 */
uint32_t csp_can_socketcan_tx_enobufs_count(const csp_iface_t * iface);

/**
 * Times a transmit had to back off because the socket send buffer was full
 * (EAGAIN): more frames in flight than SO_SNDBUF allows, i.e. the controller
 * is not completing frames as fast as they are offered. On a shared bus this
 * is what losing arbitration looks like from the socket.
 * @return count, 0 for NULL or a non-socketcan interface.
 */
uint32_t csp_can_socketcan_tx_eagain_count(const csp_iface_t * iface);

/**
 * CAN_ERR_BUSOFF error frames: the controller left the bus. With a restart
 * delay configured it comes back on its own, but every frame queued meanwhile
 * was lost and every peer's reassembly of them is stranded.
 * @return count, 0 for NULL or a non-socketcan interface.
 */
uint32_t csp_can_socketcan_bus_off_count(const csp_iface_t * iface);

/**
 * CAN_ERR_CRTL_TX_PASSIVE notifications: the controller's TX error counter
 * crossed 127, so it is failing to get ACKs on this bus. Transmits still go
 * out but slowly, which is one way the socket send buffer comes to fill.
 * @return count, 0 for NULL or a non-socketcan interface.
 */
uint32_t csp_can_socketcan_tx_passive_count(const csp_iface_t * iface);

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
