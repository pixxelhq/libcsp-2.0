

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

/* A give-up is logged at most this often per interface. A starved sender
 * abandons packets in bursts, and one line per packet flooded the FSW logger. */
#define GIVEUP_LOG_INTERVAL_MS 1000

// CAN interface data, state, etc.
typedef struct {
	char name[CSP_IFLIST_NAME_MAX + 1];
	csp_iface_t iface;
	csp_can_interface_data_t ifdata;
	pthread_t rx_thread;
	int socket;
	uint32_t tx_enobufs_retries; /**< write() backoffs on ENOBUFS, see csp_can_socketcan_tx_enobufs_count() */
	uint32_t tx_eagain_retries;  /**< write() backoffs on EAGAIN, see csp_can_socketcan_tx_eagain_count() */
	uint32_t rx_overrun_count;   /**< CAN_ERR_CRTL_RX_OVERFLOW error frames, see csp_can_socketcan_rx_overrun_count() */
	uint32_t bus_off_count;      /**< CAN_ERR_BUSOFF error frames, see csp_can_socketcan_bus_off_count() */
	uint32_t tx_passive_count;   /**< CAN_ERR_CRTL_TX_PASSIVE error frames, see csp_can_socketcan_tx_passive_count() */
	uint32_t giveup_last_ms;     /**< csp_get_ms() of the last give-up line printed, 0 = never */
	uint32_t giveup_suppressed;  /**< give-ups not printed since giveup_last_ms */
} can_context_t;

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

uint32_t csp_can_socketcan_bus_off_count(const csp_iface_t * iface) {
	const can_context_t * ctx = socketcan_ctx(iface);
	return ctx ? ctx->bus_off_count : 0;
}

uint32_t csp_can_socketcan_tx_passive_count(const csp_iface_t * iface) {
	const can_context_t * ctx = socketcan_ctx(iface);
	return ctx ? ctx->tx_passive_count : 0;
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

		/* Read CAN frame */
		struct can_frame frame;
		int nbytes = read(ctx->socket, &frame, sizeof(frame)); 
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

static int csp_can_tx_frame(void * driver_data, uint32_t id, const uint8_t * data, uint8_t dlc, const csp_packet_t *packet) {
	(void)packet;
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
				 * controller is behind the offered load; wait for it. */
				if (errnum == ENOBUFS) {
					ctx->tx_enobufs_retries++;
				} else {
					ctx->tx_eagain_retries++;
				}
				usleep(CSP_CAN_SOCKETCAN_TX_BACKOFF_MS * 1000);
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
	ctx->iface.add_alias = csp_can_socketcan_add_alias;
	ctx->ifdata.pbufs = NULL;

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

	/* Create receive thread */
	if (pthread_create(&ctx->rx_thread, NULL, socketcan_rx_thread, ctx) != 0) {
		csp_print("%s[%s]: pthread_create() failed, error: %s\n", __func__, ctx->name, strerror(errno));
		(void)csp_can_remove_interface(&ctx->iface);
		socketcan_free(ctx);
		return CSP_ERR_NOMEM;
	}

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
