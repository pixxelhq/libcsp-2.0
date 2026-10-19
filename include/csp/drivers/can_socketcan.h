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
#include <time.h>

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
/** Total back-off a single frame may spend before the packet is abandoned.
 *  The dead-bus stop: the TX thread (see CSP_CAN_SOCKETCAN_TX_QUEUE_LEN) gives
 *  up on a packet after this much back-off on one frame, so a bus that never
 *  takes a frame again (bus-off, cable out) cannot wedge the thread. This is
 *  the only way a partial packet reaches the wire. A packet whose
 *  csp_packet_t::timestamp_tx holds a csp_get_ms() deadline is dropped
 *  earlier, and whole: at dequeue, before its first frame. Once a frame has
 *  gone out the packet is finished whatever the deadline, so a saturated bus
 *  costs stale packets, never partial ones. */
#define CSP_CAN_SOCKETCAN_TX_BUDGET_MS 1000
/** Packets a transmit queue holds. Each socketcan interface has one, drained
 *  by its own TX thread, so a sender never writes to the socket itself: it
 *  pushes the packet (a pointer, no copy) and returns at once. A full queue
 *  refuses the packet whole (csp_can_socketcan_tx_queue_full_count()), so
 *  back-pressure never leaves a partial packet on the wire. Latency in the
 *  queue is bounded by the packets' own deadlines, not by this length. */
#define CSP_CAN_SOCKETCAN_TX_QUEUE_LEN 64
/** SO_SNDBUF requested at open. The kernel clamps it to net.core.wmem_max. */
#define CSP_CAN_SOCKETCAN_SNDBUF_BYTES (4 * 1024 * 1024)
/** SO_RCVBUF requested at open. The kernel clamps it to net.core.rmem_max. */
#define CSP_CAN_SOCKETCAN_RCVBUF_BYTES (4 * 1024 * 1024)

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
 * Packets abandoned because the deadline in csp_packet_t::timestamp_tx passed
 * while the kernel was refusing frames (see CSP_CAN_SOCKETCAN_TX_BUDGET_MS).
 * Each one is also counted under csp_iface_t::tx_error.
 * @return count, 0 for NULL or a non-socketcan interface.
 */
uint32_t csp_can_socketcan_tx_deadline_giveup_count(const csp_iface_t * iface);

/**
 * Packets refused at enqueue because the interface's transmit queue was full
 * (CSP_CAN_SOCKETCAN_TX_QUEUE_LEN). Refused whole: nothing of them reached the
 * wire. Each one is also counted under csp_iface_t::tx_error by the caller.
 * @return count, 0 for NULL or a non-socketcan interface.
 */
uint32_t csp_can_socketcan_tx_queue_full_count(const csp_iface_t * iface);

/**
 * Deepest the transmit queue has been, in packets, sampled after each enqueue.
 * Near CSP_CAN_SOCKETCAN_TX_QUEUE_LEN means the bus cannot carry the offered
 * load and queue-full drops are close.
 * @return packets, 0 for NULL or a non-socketcan interface.
 */
uint32_t csp_can_socketcan_tx_queue_depth_max(const csp_iface_t * iface);

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
 * Scheduling for the receive thread of every interface opened AFTER this call.
 *
 * The RX thread was created with default attributes, i.e. SCHED_OTHER, the
 * lowest class in a process whose rate groups run SCHED_FIFO. Whenever those
 * are busy for longer than the reassembly timeout the thread does not run,
 * frames queue in the socket, and every packet in flight times out at the
 * receiver although nothing was lost on the wire. With @p priority > 0 the
 * thread is created SCHED_FIFO at that priority (clamped to the policy's
 * range); if the kernel refuses (no CAP_SYS_NICE) the driver logs once and
 * falls back to the default attributes. 0 restores the default.
 *
 * @param[in] priority SCHED_FIFO priority, 0 = default scheduling
 */
void csp_can_socketcan_set_rx_priority(int priority);

/** @return the value last given to csp_can_socketcan_set_rx_priority(), 0 by default */
int csp_can_socketcan_rx_priority(void);

/**
 * Scheduling for the transmit thread of every interface opened AFTER this call,
 * with the same rules as csp_can_socketcan_set_rx_priority(). The default (-1)
 * follows the RX priority, which is what a node wants: the drain must outrank
 * the rate groups that fill the queue, and blocking in it costs nobody else.
 *
 * @param[in] priority SCHED_FIFO priority, 0 = default scheduling, -1 = same as RX
 */
void csp_can_socketcan_set_tx_priority(int priority);

/** @return the effective TX thread priority for interfaces opened now (0 = default scheduling) */
int csp_can_socketcan_tx_priority(void);

/**
 * Socket-to-read latency: kernel receive timestamp (SO_TIMESTAMPNS) to the
 * moment the RX thread read the frame, in microseconds. This is the time a
 * frame spent queued because the RX thread was not running. Returns the
 * maximum observed since the previous call with @p reset, and clears it when
 * @p reset is true, so a periodic reader gets one maximum per period.
 * @return microseconds, 0 for NULL or a non-socketcan interface.
 */
uint32_t csp_can_socketcan_rx_latency_max_us(const csp_iface_t * iface, bool reset);

/**
 * Frames whose socket-to-read latency exceeded CSP_CAN_PBUF_TIMEOUT_MS: each
 * one is a frame that reached the controller in time and was then held in the
 * socket long enough to time out the reassembly it belonged to.
 * @return count, 0 for NULL or a non-socketcan interface.
 */
uint32_t csp_can_socketcan_rx_late_count(const csp_iface_t * iface);

/**
 * Pure helper behind the latency counters: @p now - @p rx in microseconds,
 * 0 if @p rx is not before @p now or the difference is implausible (> 1 h,
 * which only a clock step produces).
 */
uint32_t csp_can_socketcan_latency_us(const struct timespec * rx, const struct timespec * now);

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
