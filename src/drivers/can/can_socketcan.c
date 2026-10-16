

#ifndef _GNU_SOURCE
#define _GNU_SOURCE /* pthread_setname_np */
#endif
#include <csp/drivers/can_socketcan.h>

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include <csp/csp_debug.h>
#include <sys/socket.h>
#include <sys/select.h> /* fd_set/select(); not pulled in transitively on musl */
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <libsocketcan.h>

#include <csp/csp.h>
#include <csp/csp_id.h>
#include <csp/arch/csp_time.h>
#include <sched.h>
#include <time.h>
#include <poll.h>
#include <stdatomic.h>
#include <csp/arch/csp_queue.h>

/* A give-up is logged at most this often per interface. A starved sender
 * abandons packets in bursts, and one line per packet flooded the FSW logger. */
#define GIVEUP_LOG_INTERVAL_MS 1000

/* One queued transmission: the packet (owned by the queue until the TX thread
 * has sent or dropped it) and the next-hop the interface layer resolved. */
typedef struct {
	csp_packet_t * packet;
	uint16_t via;
} socketcan_tx_item_t;

// CAN interface data, state, etc.
typedef struct {
	char name[CSP_IFLIST_NAME_MAX + 1];
	csp_iface_t iface;
	csp_can_interface_data_t ifdata;
	pthread_t rx_thread;
	pthread_t tx_thread;
	bool tx_thread_started;
	atomic_bool tx_stop;         /**< asks the TX thread to return; it is joined, never cancelled, so no queue mutex is left locked */
	csp_queue_handle_t txq;      /**< transmit queue of socketcan_tx_item_t, see CSP_CAN_SOCKETCAN_TX_QUEUE_LEN */
	csp_static_queue_t txq_static;
	char txq_buf[CSP_CAN_SOCKETCAN_TX_QUEUE_LEN * sizeof(socketcan_tx_item_t)];
	int socket;
	uint32_t tx_enobufs_retries; /**< write() backoffs on ENOBUFS, see csp_can_socketcan_tx_enobufs_count() */
	uint32_t tx_eagain_retries;  /**< write() backoffs on EAGAIN, see csp_can_socketcan_tx_eagain_count() */
	uint32_t tx_deadline_giveups; /**< packets abandoned because their timestamp_tx deadline passed, see csp_can_socketcan_tx_deadline_giveup_count() */
	uint32_t tx_queue_full_drops; /**< packets refused because the transmit queue was full, see csp_can_socketcan_tx_queue_full_count() */
	uint32_t tx_queue_depth_max;  /**< deepest the transmit queue has been, see csp_can_socketcan_tx_queue_depth_max() */
	uint32_t rx_overrun_count;   /**< CAN_ERR_CRTL_RX_OVERFLOW error frames, see csp_can_socketcan_rx_overrun_count() */
	uint32_t bus_off_count;      /**< CAN_ERR_BUSOFF error frames, see csp_can_socketcan_bus_off_count() */
	uint32_t tx_passive_count;   /**< CAN_ERR_CRTL_TX_PASSIVE error frames, see csp_can_socketcan_tx_passive_count() */
	uint32_t giveup_last_ms;     /**< csp_get_ms() of the last give-up line printed, 0 = never */
	uint32_t giveup_suppressed;  /**< give-ups not printed since giveup_last_ms */
	uint32_t rx_latency_max_us;  /**< max socket-to-read latency since last reset, see csp_can_socketcan_rx_latency_max_us() */
	uint32_t rx_late_count;      /**< frames read more than CSP_CAN_PBUF_TIMEOUT_MS after the kernel stamped them */
	bool rx_timestamps;          /**< SO_TIMESTAMPNS accepted on this socket */
} can_context_t;

/* SCHED_FIFO priority for RX threads created from now on; 0 = default attributes */
static int rx_thread_priority = 0;

void csp_can_socketcan_set_rx_priority(int priority) {
	rx_thread_priority = (priority < 0) ? 0 : priority;
}

int csp_can_socketcan_rx_priority(void) {
	return rx_thread_priority;
}

/* SCHED_FIFO priority for TX threads created from now on; -1 = follow RX */
static int tx_thread_priority = -1;

void csp_can_socketcan_set_tx_priority(int priority) {
	tx_thread_priority = (priority < -1) ? -1 : priority;
}

int csp_can_socketcan_tx_priority(void) {
	return (tx_thread_priority < 0) ? rx_thread_priority : tx_thread_priority;
}

uint32_t csp_can_socketcan_latency_us(const struct timespec * rx, const struct timespec * now) {
	if ((rx == NULL) || (now == NULL)) {
		return 0;
	}
	int64_t us = ((int64_t)now->tv_sec - (int64_t)rx->tv_sec) * 1000000LL + ((int64_t)now->tv_nsec - (int64_t)rx->tv_nsec) / 1000LL;
	if ((us <= 0) || (us > 3600LL * 1000000LL)) {
		return 0;
	}
	return (uint32_t)us;
}

static int csp_can_tx_frame(void * driver_data, uint32_t id, const uint8_t * data, uint8_t dlc, const csp_packet_t * packet);

/* Resolve the driver context behind an interface without dereferencing a
 * foreign driver_data. Three checks, because other drivers (KISS) embed the
 * interface in a context with the same {name, iface} prefix: the interface
 * must be a CAN interface (nexthop), its CAN interface data must name this
 * driver's transmit function, and the context's iface member must be the
 * interface itself. */
static const can_context_t * socketcan_ctx(const csp_iface_t * iface) {
	if (!csp_can_is_can_interface(iface) || (iface->driver_data == NULL)) {
		return NULL;
	}
	const csp_can_interface_data_t * ifdata = iface->interface_data;
	if (ifdata->tx_func != csp_can_tx_frame) {
		return NULL;
	}
	const can_context_t * ctx = iface->driver_data;
	return (&ctx->iface == iface) ? ctx : NULL;
}

uint32_t csp_can_socketcan_tx_enobufs_count(const csp_iface_t * iface) {
	const can_context_t * ctx = socketcan_ctx(iface);
	return ctx ? ctx->tx_enobufs_retries : 0;
}

uint32_t csp_can_socketcan_rx_overrun_count(const csp_iface_t * iface) {
	const can_context_t * ctx = socketcan_ctx(iface);
	return ctx ? ctx->rx_overrun_count : 0;
}

uint32_t csp_can_socketcan_tx_eagain_count(const csp_iface_t * iface) {
	const can_context_t * ctx = socketcan_ctx(iface);
	return ctx ? ctx->tx_eagain_retries : 0;
}

uint32_t csp_can_socketcan_tx_deadline_giveup_count(const csp_iface_t * iface) {
	const can_context_t * ctx = socketcan_ctx(iface);
	return ctx ? ctx->tx_deadline_giveups : 0;
}

uint32_t csp_can_socketcan_tx_queue_full_count(const csp_iface_t * iface) {
	const can_context_t * ctx = socketcan_ctx(iface);
	return ctx ? ctx->tx_queue_full_drops : 0;
}

uint32_t csp_can_socketcan_tx_queue_depth_max(const csp_iface_t * iface) {
	const can_context_t * ctx = socketcan_ctx(iface);
	return ctx ? ctx->tx_queue_depth_max : 0;
}

uint32_t csp_can_socketcan_bus_off_count(const csp_iface_t * iface) {
	const can_context_t * ctx = socketcan_ctx(iface);
	return ctx ? ctx->bus_off_count : 0;
}

uint32_t csp_can_socketcan_tx_passive_count(const csp_iface_t * iface) {
	const can_context_t * ctx = socketcan_ctx(iface);
	return ctx ? ctx->tx_passive_count : 0;
}

uint32_t csp_can_socketcan_rx_latency_max_us(const csp_iface_t * iface, bool reset) {
	can_context_t * ctx = (can_context_t *)socketcan_ctx(iface);
	if (ctx == NULL) {
		return 0;
	}
	const uint32_t v = ctx->rx_latency_max_us;
	if (reset) {
		ctx->rx_latency_max_us = 0;
	}
	return v;
}

uint32_t csp_can_socketcan_rx_late_count(const csp_iface_t * iface) {
	const can_context_t * ctx = socketcan_ctx(iface);
	return ctx ? ctx->rx_late_count : 0;
}

csp_can_socketcan_tx_action_t csp_can_socketcan_tx_backoff(int errnum, uint32_t * waiting_ms) {
	if (waiting_ms == NULL) {
		return CSP_CAN_SOCKETCAN_TX_FAIL;
	}
	switch (errnum) {
		case EINTR:
			return CSP_CAN_SOCKETCAN_TX_RETRY_NOW;
		case ENOBUFS: /* qdisc (txqueuelen) full */
		case EAGAIN:  /* socket send buffer (SO_SNDBUF) full */
			if (*waiting_ms >= CSP_CAN_SOCKETCAN_TX_BUDGET_MS) {
				return CSP_CAN_SOCKETCAN_TX_GIVE_UP;
			}
			*waiting_ms += CSP_CAN_SOCKETCAN_TX_BACKOFF_MS;
			return CSP_CAN_SOCKETCAN_TX_RETRY_SLEEP;
		default:
			return CSP_CAN_SOCKETCAN_TX_FAIL;
	}
}

/* One line per GIVEUP_LOG_INTERVAL_MS per interface, carrying the real errno
 * and how long was actually waited. The previous unconditional line claimed
 * ">1000 ms" even when the loop had given up within a millisecond. */
static void socketcan_log_giveup(can_context_t * ctx, int errnum, uint32_t waited_ms) {
	const uint32_t now = csp_get_ms();
	if ((ctx->giveup_last_ms != 0) && ((now - ctx->giveup_last_ms) < GIVEUP_LOG_INTERVAL_MS)) {
		ctx->giveup_suppressed++;
		return;
	}
	csp_print("%s[%s]: write() gave up after %u ms of back-off, errno %d (%s); packet dropped (%u more give-ups not logged)\n",
			  __func__, ctx->name, (unsigned int)waited_ms, errnum, strerror(errnum), (unsigned int)ctx->giveup_suppressed);
	ctx->giveup_last_ms = now ? now : 1;
	ctx->giveup_suppressed = 0;
}

static void socketcan_free(can_context_t * ctx) {

	if (ctx) {
		if (ctx->txq != NULL) {
			/* Only reached once the TX thread is gone (or was never started). */
			socketcan_tx_item_t item;
			while (csp_queue_dequeue(ctx->txq, &item, 0) == CSP_QUEUE_OK) {
				csp_buffer_free(item.packet);
			}
			csp_queue_free(ctx->txq);
		}
		if (ctx->socket >= 0) {
			close(ctx->socket);
		}
		free(ctx);
	}
}

static void * socketcan_rx_thread(void * arg) {
	can_context_t * ctx = arg;

	while (1) {

		/* Use select for non blocking reads */
		fd_set input;
		FD_ZERO(&input);
		FD_SET(ctx->socket, &input);
		struct timeval timeout = {
			.tv_sec = 10,
		};
		int n = select(ctx->socket + 1, &input, NULL, NULL, &timeout);
		if (n == -1) {
			csp_print("CAN read error\n");
			continue;
		} else if (n == 0) {
			//printf("CAN idle\n");
			continue;
		}

		/* Read CAN frame, with the kernel receive timestamp when available */
		struct can_frame frame;
		struct iovec iov = {.iov_base = &frame, .iov_len = sizeof(frame)};
		union { struct cmsghdr align; char buf[CMSG_SPACE(sizeof(struct timespec))]; } cmsgbuf;
		struct msghdr msg = {.msg_iov = &iov, .msg_iovlen = 1, .msg_control = ctx->rx_timestamps ? cmsgbuf.buf : NULL, .msg_controllen = ctx->rx_timestamps ? sizeof(cmsgbuf.buf) : 0};
		int nbytes = recvmsg(ctx->socket, &msg, 0);
		if (nbytes > 0 && ctx->rx_timestamps) {
			for (struct cmsghdr * c = CMSG_FIRSTHDR(&msg); c != NULL; c = CMSG_NXTHDR(&msg, c)) {
				if ((c->cmsg_level == SOL_SOCKET) && (c->cmsg_type == SCM_TIMESTAMPNS)) {
					struct timespec rx_ts, now;
					memcpy(&rx_ts, CMSG_DATA(c), sizeof(rx_ts));
					clock_gettime(CLOCK_REALTIME, &now);
					const uint32_t lat = csp_can_socketcan_latency_us(&rx_ts, &now);
					if (lat > ctx->rx_latency_max_us) {
						ctx->rx_latency_max_us = lat;
					}
					if (lat > (uint32_t)CSP_CAN_PBUF_TIMEOUT_MS * 1000U) {
						ctx->rx_late_count++;
					}
					break;
				}
			}
		}
		if (nbytes < 0) {
			if (errno == EAGAIN || errno == EINTR) {
				/* This is acceptable, since something interrupted us, try again */
				continue;
			} else {
				csp_print("%s[%s]: read() failed, errno %d: %s\n", __func__, ctx->name, errno, strerror(errno));
				usleep(1*1E6);
				continue;
			}
		}

		if (nbytes != sizeof(frame)) {
			csp_print("%s[%s]: Read incomplete CAN frame, size: %d, expected: %u bytes\n", __func__, ctx->name, nbytes, (unsigned int)sizeof(frame));
			continue;
		}

		/* Drop frames with invalid size field */
		if (frame.can_dlc > CAN_MAX_DLEN) {
			continue;
		}

		/* Error frames must be claimed before the extended-id test: they
		 * set CAN_ERR_FLAG and leave CAN_EFF_FLAG clear, so testing for a
		 * standard id first would silently swallow every one of them. */
		if (frame.can_id & CAN_ERR_FLAG) {
			if ((frame.can_id & CAN_ERR_CRTL) && (frame.can_dlc > 1)) {
				if (frame.data[1] & CAN_ERR_CRTL_RX_OVERFLOW) {
					/* Dropped in hardware before reassembly: counted as rx_error
					 * like a socket-queue drop. The damage to the packet being
					 * reassembled is already counted under iface->frame. */
					ctx->rx_overrun_count++;
					ctx->iface.rx_error++;
					csp_dbg_can_errno = CSP_DBG_CAN_ERR_RX_OVF;
				}
				if (frame.data[1] & CAN_ERR_CRTL_TX_PASSIVE) {
					/* TX error counter past 127: this node is not getting
					 * ACKs. Transmits continue, slowly, so the send buffer
					 * fills -- the usual prelude to tx_error. */
					ctx->tx_passive_count++;
				}
			}
			if (frame.can_id & CAN_ERR_BUSOFF) {
				/* Everything queued is lost; the kernel restarts the
				 * controller after the configured restart-ms. */
				ctx->bus_off_count++;
			}
			/* Per-frame bus errors (lost arbitration, protocol violations)
			 * are reported by the kernel's own link statistics; not counted
			 * here to keep the RX thread cheap on a saturated bus. */
			continue;
		}

		/* Drop frames with standard id (CSP uses extended) */
		if (!(frame.can_id & CAN_EFF_FLAG)) {
			continue;
		}

		/* Drop remote frames */
		if (frame.can_id & CAN_RTR_FLAG) {
			csp_print("%s[%s]: discarding RTR frame\n", __func__, ctx->name);
			continue;
		}

		/* Strip flags */
		frame.can_id &= CAN_EFF_MASK;

		/* Call RX callbacsp_can_rx_frameck */
		csp_can_rx(&ctx->iface, frame.can_id, frame.data, frame.can_dlc, 0, NULL);
	}

	/* We should never reach this point */
	pthread_exit(NULL);
}

/* Runs on the interface's TX thread only (socketcan_tx_thread), never on a
 * sender's thread, so waiting here costs nobody a rate-group tick.
 *
 * Per-PACKET deadline, stamped by the sender in packet->timestamp_tx as a
 * csp_get_ms() value (0 = none). Normally a stale packet is dropped whole
 * before its first frame (socketcan_tx_thread); this check catches the
 * deadline passing while the kernel refuses a frame mid-packet, and the
 * per-frame CSP_CAN_SOCKETCAN_TX_BUDGET_MS is the final stop for a bus that
 * never recovers. Both leave a partial packet on the wire that every peer
 * times out after CSP_CAN_PBUF_TIMEOUT_MS: the price of not wedging on a dead
 * bus, paid only when it is dead. The CFP1 layer passes the packet on every
 * frame; CFP2 only on the last one, so CFP2 keeps the per-frame budget for
 * all but that frame. */
static int csp_can_tx_frame(void * driver_data, uint32_t id, const uint8_t * data, uint8_t dlc, const csp_packet_t *packet) {
	const uint32_t deadline_ms = (packet != NULL) ? packet->timestamp_tx : 0;
	if (dlc > CAN_MAX_DLEN) {
		return CSP_ERR_INVAL;
	}

	struct can_frame frame = {.can_id = id | CAN_EFF_FLAG,
							  .can_dlc = dlc};
	memcpy(frame.data, data, dlc);

	uint32_t waiting_ms = 0;
	can_context_t * ctx = driver_data;
	uintptr_t pdata = (uintptr_t)&frame;
	uintptr_t pend = ((uintptr_t)&frame + sizeof(frame));
	size_t length = sizeof(frame);

	while (pdata < pend) {
		int written;

		written = write(ctx->socket, (void *)pdata, length);
		if (written >= 0) {
			waiting_ms = 0;
			pdata += written;
			length -= written;
			continue;
		}

		const int errnum = errno;
		switch (csp_can_socketcan_tx_backoff(errnum, &waiting_ms)) {
			case CSP_CAN_SOCKETCAN_TX_RETRY_NOW:
				break;
			case CSP_CAN_SOCKETCAN_TX_RETRY_SLEEP:
				/* The socket is non-blocking: ENOBUFS is the qdisc full,
				 * EAGAIN the socket send buffer full. Either way the
				 * controller is behind the offered load; wait for it --
				 * unless the packet's deadline has passed, then drop it now. */
				if ((deadline_ms != 0) && ((int32_t)(csp_get_ms() - deadline_ms) >= 0)) {
					ctx->tx_deadline_giveups++;
					socketcan_log_giveup(ctx, errnum, waiting_ms);
					return CSP_ERR_TX;
				}
				if (errnum == ENOBUFS) {
					/* qdisc (txqueuelen) full: POLLOUT does not report it, so sleep. */
					ctx->tx_enobufs_retries++;
					usleep(CSP_CAN_SOCKETCAN_TX_BACKOFF_MS * 1000);
				} else {
					/* Socket send buffer full: wake as soon as the kernel drains
					 * some of it, at most one back-off step. */
					ctx->tx_eagain_retries++;
					struct pollfd pfd = {.fd = ctx->socket, .events = POLLOUT};
					(void)poll(&pfd, 1, CSP_CAN_SOCKETCAN_TX_BACKOFF_MS);
				}
				break;
			case CSP_CAN_SOCKETCAN_TX_GIVE_UP:
				socketcan_log_giveup(ctx, errnum, waiting_ms);
				return CSP_ERR_TX;
			case CSP_CAN_SOCKETCAN_TX_FAIL:
			default:
				csp_print("%s[%s]: write() failed, encountered an error during write(). %d - '%s'\n", __func__, ctx->name, errnum, strerror(errnum));
				return CSP_ERR_TX;
		}
	}

	return CSP_ERR_NONE;
}

/* The interface's tx_enqueue hook: called on the sender's thread, never
 * blocks, never touches the socket. Ownership of the packet passes to the
 * queue on success; on a full queue it stays with the caller, which frees it
 * and counts tx_error (csp_send_direct_iface()). */
static int socketcan_tx_enqueue(void * driver_data, csp_packet_t * packet, uint16_t via) {
	can_context_t * ctx = driver_data;
	const socketcan_tx_item_t item = {.packet = packet, .via = via};

	if (csp_queue_enqueue(ctx->txq, &item, 0) != CSP_QUEUE_OK) {
		ctx->tx_queue_full_drops++;
		return CSP_ERR_TX;
	}
	const int depth = csp_queue_size(ctx->txq);
	if ((depth > 0) && ((uint32_t)depth > ctx->tx_queue_depth_max)) {
		ctx->tx_queue_depth_max = (uint32_t)depth;
	}
	return CSP_ERR_NONE;
}

/* One writer per interface: a packet's frames leave contiguously, and every
 * wait for the kernel happens here. A packet whose deadline already passed
 * while it queued is dropped whole, so a saturated bus costs stale packets,
 * not partial ones. */
static void * socketcan_tx_thread(void * arg) {
	can_context_t * ctx = arg;
	socketcan_tx_item_t item;

	while (!atomic_load(&ctx->tx_stop)) {
		/* Bounded wait so a stop request is seen within one period. */
		if (csp_queue_dequeue(ctx->txq, &item, 100) != CSP_QUEUE_OK) {
			continue;
		}
		csp_packet_t * packet = item.packet;
		const uint32_t deadline_ms = packet->timestamp_tx;
		if ((deadline_ms != 0) && ((int32_t)(csp_get_ms() - deadline_ms) >= 0)) {
			ctx->tx_deadline_giveups++;
			ctx->iface.tx_error++;
			csp_buffer_free(packet);
			continue;
		}
		if (csp_can_tx_packet_sync(&ctx->iface, item.via, packet) != CSP_ERR_NONE) {
			/* The sync path does not free on failure; the frame-level
			 * reason (deadline, budget, hard error) was already counted. */
			ctx->iface.tx_error++;
			csp_buffer_free(packet);
		}
	}
	return NULL;
}

/* Create a driver thread, SCHED_FIFO at @p priority when > 0 (clamped to the
 * policy's range); a refusal -- typically EPERM without CAP_SYS_NICE -- falls
 * back to default attributes rather than failing the interface. Returns 0 or
 * the pthread_create() error. */
static int socketcan_start_thread(can_context_t * ctx, pthread_t * thread, void * (*fn)(void *), int priority, const char * what) {
	int created = -1;
	if (priority > 0) {
		pthread_attr_t attr;
		struct sched_param sp;
		const int lo = sched_get_priority_min(SCHED_FIFO), hi = sched_get_priority_max(SCHED_FIFO);
		int prio = priority;
		if ((lo >= 0) && (prio < lo)) prio = lo;
		if ((hi >= 0) && (prio > hi)) prio = hi;
		memset(&sp, 0, sizeof(sp));
		sp.sched_priority = prio;
		if ((pthread_attr_init(&attr) == 0) &&
			(pthread_attr_setschedpolicy(&attr, SCHED_FIFO) == 0) &&
			(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) == 0) &&
			(pthread_attr_setschedparam(&attr, &sp) == 0)) {
			created = pthread_create(thread, &attr, fn, ctx);
			if (created == 0) {
				csp_print("%s[%s]: %s thread SCHED_FIFO priority %d\n", __func__, ctx->name, what, prio);
			} else {
				csp_print("%s[%s]: SCHED_FIFO %d refused for the %s thread (%s), using default scheduling\n", __func__, ctx->name, prio, what, strerror(created));
			}
		}
		pthread_attr_destroy(&attr);
	}
	if (created != 0) {
		created = pthread_create(thread, NULL, fn, ctx);
	}
	if (created == 0) {
		/* Visible in ps -T / top -H; 15 chars max. */
		char tname[16];
		snprintf(tname, sizeof(tname), "%s-%.*s", what, (int)(sizeof(tname) - 1 - strlen(what) - 1), ctx->name);
		(void)pthread_setname_np(*thread, tname);
	}
	return created;
}


int csp_can_socketcan_build_filters(uint8_t version, uint16_t addr, uint16_t netmask, bool promisc, struct can_filter * filters, unsigned int max_filters) {

	if ((filters == NULL) || (max_filters < 3)) {
		return CSP_ERR_INVAL;
	}

	if (promisc) {
		filters[0].can_id = 0;
		filters[0].can_mask = 0; /* receive anything */
		return 1;
	}

	if (version == 1) {
		/* Own address OR the 5-bit broadcast address. A single filter on the
		 * own address alone would silently discard every CSP broadcast. */
		const uint16_t bcast = (1 << CFP_HOST_SIZE) - 1;
		int num_filters = 0;
		filters[num_filters].can_id = CFP_MAKE_DST(addr);
		filters[num_filters].can_mask = CFP_MAKE_DST(bcast);
		num_filters++;
		if (addr != bcast) {
			filters[num_filters].can_id = CFP_MAKE_DST(bcast);
			filters[num_filters].can_mask = CFP_MAKE_DST(bcast);
			num_filters++;
		}
		return num_filters;
	}

	filters[0].can_id = addr << CFP2_DST_OFFSET;
	filters[0].can_mask = CFP2_DST_MASK << CFP2_DST_OFFSET;
	filters[1].can_id = ((1 << (CFP2_DST_SIZE - netmask)) - 1) << CFP2_DST_OFFSET;
	filters[1].can_mask = CFP2_DST_MASK << CFP2_DST_OFFSET;
	filters[2].can_id = 0x3FFF << CFP2_DST_OFFSET;
	filters[2].can_mask = CFP2_DST_MASK << CFP2_DST_OFFSET;
	return 3;
}

static int csp_can_socketcan_set_promisc(const bool promisc, can_context_t * ctx) {

	struct can_filter filter[3];

	if (ctx->socket == 0) {
		return CSP_ERR_INVAL;
	}

	int num_filters = csp_can_socketcan_build_filters(csp_conf.version, ctx->iface.addr, ctx->iface.netmask, promisc, filter, 3);
	if (num_filters <= 0) {
		return CSP_ERR_INVAL;
	}

	if (setsockopt(ctx->socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, num_filters * sizeof(struct can_filter)) < 0) {
		csp_print("%s: setsockopt() failed, error: %s\n", __func__, strerror(errno));
		return CSP_ERR_INVAL;
	}

	return CSP_ERR_NONE;
}

static int csp_can_socketcan_add_alias(void * driver_data, uint16_t addr) {

	if (csp_conf.version == 1) {
		return -1;
	}

	can_context_t * ctx = driver_data;

	struct can_filter filter[10];
	socklen_t len = sizeof(filter);

	getsockopt(ctx->socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, &len);

	/* Current implementation has a defined maximum of filters available */
	if (len == sizeof(filter)) {
		return -2;
	}

	/* If only 1 filter exist for CSP v2, interface is promisc */
	if (len == sizeof(struct can_filter)) {
		return 0;
	}

	/* Add filter for specific additional receive address */
	filter[len/sizeof(struct can_filter)].can_id = addr << CFP2_DST_OFFSET;
	filter[len/sizeof(struct can_filter)].can_mask = CFP2_DST_MASK << CFP2_DST_OFFSET;;

	if (setsockopt(ctx->socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, len + sizeof(struct can_filter)) < 0) {
		return -2;
	}

	return 0;
}

int csp_can_socketcan_open_and_add_interface(const char * device, const char * ifname, unsigned int node_id, int bitrate, bool promisc, csp_iface_t ** return_iface) {
	if (ifname == NULL) {
		ifname = CSP_IF_CAN_DEFAULT_NAME;
	}

	csp_print("INIT %s: device: [%s], bitrate: %d, promisc: %d\n", ifname, device, bitrate, promisc);

	/* Set interface up - this may require increased OS privileges */
	if (bitrate > 0) {
		can_do_stop(device);
		can_set_bitrate(device, bitrate);
		can_set_restart_ms(device, 100);
		can_do_start(device);
	}

	can_context_t * ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL) {
		return CSP_ERR_NOMEM;
	}
	ctx->socket = -1;

	strncpy(ctx->name, ifname, sizeof(ctx->name) - 1);
	ctx->iface.name = ctx->name;
	ctx->iface.addr = node_id;
	ctx->iface.interface_data = &ctx->ifdata;
	ctx->iface.driver_data = ctx;
	ctx->ifdata.tx_func = csp_can_tx_frame;
	ctx->ifdata.tx_enqueue = socketcan_tx_enqueue;
	ctx->iface.add_alias = csp_can_socketcan_add_alias;
	ctx->ifdata.pbufs = NULL;
	atomic_init(&ctx->tx_stop, false);

	ctx->txq = csp_queue_create_static(CSP_CAN_SOCKETCAN_TX_QUEUE_LEN, sizeof(socketcan_tx_item_t), ctx->txq_buf, &ctx->txq_static);
	if (ctx->txq == NULL) {
		csp_print("%s[%s]: transmit queue allocation failed\n", __func__, ctx->name);
		socketcan_free(ctx);
		return CSP_ERR_NOMEM;
	}

	/* Create socket */
	if ((ctx->socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		csp_print("%s[%s]: socket() failed, error: %s\n", __func__, ctx->name, strerror(errno));
		socketcan_free(ctx);
		return CSP_ERR_INVAL;
	}

	/* Locate interface */
	struct ifreq ifr;
	strncpy(ifr.ifr_name, device, IFNAMSIZ - 1);
	if (ioctl(ctx->socket, SIOCGIFINDEX, &ifr) < 0) {
		csp_print("%s[%s]: device: [%s], ioctl() failed, error: %s\n", __func__, ctx->name, device, strerror(errno));
		socketcan_free(ctx);
		return CSP_ERR_INVAL;
	}

	fcntl(ctx->socket, F_SETFL, O_NONBLOCK);

	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	/* Bind the socket to CAN interface */
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(ctx->socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		csp_print("%s[%s]: bind() failed, error: %s\n", __func__, ctx->name, strerror(errno));
		socketcan_free(ctx);
		return CSP_ERR_INVAL;
	}

	/* Size the send buffer explicitly. The default (net.core.wmem_default,
	 * ~212 KB) holds only a few hundred frames in flight, so on a busy bus
	 * write() returns EAGAIN long before txqueuelen is reached. The kernel
	 * doubles the request for bookkeeping and clamps it to net.core.wmem_max;
	 * report the effective value so a clamp is visible at boot. */
	{
		int sndbuf = CSP_CAN_SOCKETCAN_SNDBUF_BYTES;
		int got = 0;
		socklen_t got_len = sizeof(got);
		if (setsockopt(ctx->socket, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
			csp_print("%s[%s]: SO_SNDBUF request failed: %s\n", __func__, ctx->name, strerror(errno));
		} else if (getsockopt(ctx->socket, SOL_SOCKET, SO_SNDBUF, &got, &got_len) == 0) {
			if (got < 2 * sndbuf) {
				csp_print("%s[%s]: socket sndbuf CLAMPED to %d bytes (asked %d, expect %d) -- raise net.core.wmem_max to at least %d\n",
						  __func__, ctx->name, got, sndbuf, 2 * sndbuf, sndbuf);
			} else {
				csp_print("%s[%s]: socket sndbuf %d bytes (asked %d)\n", __func__, ctx->name, got, sndbuf);
			}
		}
	}

	/* Size the receive buffer explicitly as well. Without this the socket
	 * inherits net.core.rmem_default (~212 KB, ~13k frames at the socketcan
	 * skb cost) and depends on a sysctl the deploy flow has to remember;
	 * with it the only knob is net.core.rmem_max, and a clamp is visible at
	 * boot. The kernel doubles the request as it does for SO_SNDBUF. */
	{
		int rcvbuf = CSP_CAN_SOCKETCAN_RCVBUF_BYTES;
		int got = 0;
		socklen_t got_len = sizeof(got);
		if (setsockopt(ctx->socket, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
			csp_print("%s[%s]: SO_RCVBUF request failed: %s\n", __func__, ctx->name, strerror(errno));
		} else if (getsockopt(ctx->socket, SOL_SOCKET, SO_RCVBUF, &got, &got_len) == 0) {
			if (got < 2 * rcvbuf) {
				csp_print("%s[%s]: socket rcvbuf CLAMPED to %d bytes (asked %d, expect %d) -- raise net.core.rmem_max to at least %d\n",
						  __func__, ctx->name, got, rcvbuf, 2 * rcvbuf, rcvbuf);
			} else {
				csp_print("%s[%s]: socket rcvbuf %d bytes (asked %d)\n", __func__, ctx->name, got, rcvbuf);
			}
		}
	}

	/* Kernel receive timestamps, so the RX thread can measure how long each
	 * frame waited in the socket before it was read: the one number that
	 * tells a starved RX thread from a slow bus. */
	{
		int on = 1;
		ctx->rx_timestamps = (setsockopt(ctx->socket, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on)) == 0);
		if (!ctx->rx_timestamps) {
			csp_print("%s[%s]: SO_TIMESTAMPNS not available, RX latency will read 0: %s\n", __func__, ctx->name, strerror(errno));
		}
	}

	/* Ask for controller error frames, restricted to controller state so a
	 * busy bus does not become a per-frame firehose. Without this a
	 * controller RX overrun is invisible: it is reported only this way. */
	const can_err_mask_t err_mask = CAN_ERR_CRTL | CAN_ERR_BUSOFF | CAN_ERR_RESTARTED;
	if (setsockopt(ctx->socket, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
		csp_print("%s[%s]: CAN_RAW_ERR_FILTER failed, controller overruns will be invisible: %s\n", __func__, ctx->name, strerror(errno));
	}

	/* Set filter mode */
	if (csp_can_socketcan_set_promisc(promisc, ctx) != CSP_ERR_NONE) {
		csp_print("%s[%s]: csp_can_socketcan_set_promisc() failed, error: %s\n", __func__, ctx->name, strerror(errno));
		socketcan_free(ctx);
		return CSP_ERR_INVAL;
	}

	/* Add interface to CSP */
	int res = csp_can_add_interface(&ctx->iface);
	if (res != CSP_ERR_NONE) {
		csp_print("%s[%s]: csp_can_add_interface() failed, error: %d\n", __func__, ctx->name, res);
		socketcan_free(ctx);
		return res;
	}

	/* Receive thread, then the transmit thread that drains ctx->txq. Both
	 * SCHED_FIFO when asked for (csp_can_socketcan_set_rx_priority /
	 * _set_tx_priority), with a fallback to default attributes. */
	int created = socketcan_start_thread(ctx, &ctx->rx_thread, socketcan_rx_thread, rx_thread_priority, "canrx");
	if (created != 0) {
		csp_print("%s[%s]: pthread_create() failed for the RX thread, error: %s\n", __func__, ctx->name, strerror(created));
		(void)csp_can_remove_interface(&ctx->iface);
		socketcan_free(ctx);
		return CSP_ERR_NOMEM;
	}
	created = socketcan_start_thread(ctx, &ctx->tx_thread, socketcan_tx_thread, csp_can_socketcan_tx_priority(), "cantx");
	if (created != 0) {
		csp_print("%s[%s]: pthread_create() failed for the TX thread, error: %s\n", __func__, ctx->name, strerror(created));
		(void)csp_can_remove_interface(&ctx->iface);
		(void)pthread_cancel(ctx->rx_thread);
		(void)pthread_join(ctx->rx_thread, NULL);
		socketcan_free(ctx);
		return CSP_ERR_NOMEM;
	}
	ctx->tx_thread_started = true;

	if (return_iface) {
		*return_iface = &ctx->iface;
	}

	return CSP_ERR_NONE;
}

csp_iface_t * csp_can_socketcan_init(const char * device, unsigned int node_id, int bitrate, bool promisc) {
	csp_iface_t * return_iface;
	int res = csp_can_socketcan_open_and_add_interface(device, CSP_IF_CAN_DEFAULT_NAME, node_id, bitrate, promisc, &return_iface);
	return (res == CSP_ERR_NONE) ? return_iface : NULL;
}

int csp_can_socketcan_stop(csp_iface_t * iface) {
	can_context_t * ctx = iface->driver_data;

	/* The TX thread is asked to stop and joined (not cancelled: a cancel
	 * inside the queue's condition wait would leave its mutex locked). It
	 * finishes the packet in hand first; whatever is still queued is freed by
	 * socketcan_free(). */
	if (ctx->tx_thread_started) {
		atomic_store(&ctx->tx_stop, true);
		int terr = pthread_join(ctx->tx_thread, NULL);
		if (terr != 0) {
			csp_print("%s[%s]: pthread_join() failed for the TX thread, error: %s\n", __func__, ctx->name, strerror(terr));
			return CSP_ERR_DRIVER;
		}
		ctx->tx_thread_started = false;
	}

	int error = pthread_cancel(ctx->rx_thread);
	if (error != 0) {
		csp_print("%s[%s]: pthread_cancel() failed, error: %s\n", __func__, ctx->name, strerror(errno));
		return CSP_ERR_DRIVER;
	}
	error = pthread_join(ctx->rx_thread, NULL);
	if (error != 0) {
		csp_print("%s[%s]: pthread_join() failed, error: %s\n", __func__, ctx->name, strerror(errno));
		return CSP_ERR_DRIVER;
	}
	socketcan_free(ctx);
	return CSP_ERR_NONE;
}
