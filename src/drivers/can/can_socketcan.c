

#include <csp/drivers/can_socketcan.h>

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include <csp/csp_debug.h>
#include <sys/socket.h>
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

// CAN interface data, state, etc.
typedef struct {
	char name[CSP_IFLIST_NAME_MAX + 1];
	csp_iface_t iface;
	csp_can_interface_data_t ifdata;
	pthread_t rx_thread;
	int socket;
	uint32_t tx_enobufs_retries; /**< write() backoffs on ENOBUFS, see csp_can_socketcan_tx_enobufs_count() */
	uint32_t rx_overrun_count;   /**< CAN_ERR_CRTL_RX_OVERFLOW error frames, see csp_can_socketcan_rx_overrun_count() */
} can_context_t;

/* Resolve the driver context behind an interface without dereferencing a
 * foreign driver_data: a socketcan interface is embedded in its context, so
 * the context's iface member must be the interface itself. */
static const can_context_t * socketcan_ctx(const csp_iface_t * iface) {
	if ((iface == NULL) || (iface->driver_data == NULL)) {
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
			if ((frame.can_id & CAN_ERR_CRTL) && (frame.can_dlc > 1) && (frame.data[1] & CAN_ERR_CRTL_RX_OVERFLOW)) {
				/* Dropped in hardware before reassembly: counted as rx_error
				 * like a socket-queue drop. The damage to the packet being
				 * reassembled is already counted under iface->frame. */
				ctx->rx_overrun_count++;
				ctx->iface.rx_error++;
				csp_dbg_can_errno = CSP_DBG_CAN_ERR_RX_OVF;
			}
			/* Bus errors, bus-off and restarts are reported by the kernel's
			 * own link statistics; not logged here to keep the RX thread quiet. */
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
		if (written < 0) {
			if (errno == ENOBUFS) {
				/* If no space available, wait for 5 ms and try again */
				ctx->tx_enobufs_retries++;
				usleep(5000);
				waiting_ms += 5;
			} else if(errno == EAGAIN || errno == EINTR) {
				/* Acceptable, since something interrupted us, try again */
				waiting_ms += 5;
			} else {
				csp_print("%s[%s]: write() failed, encountered an error during write(). %d - '%s'\n", __func__, ctx->name, errno, strerror(errno));
				return CSP_ERR_TX;
			}

			if (waiting_ms >= 1000) {
				/* We finally got tired of waiting, give up */
				csp_print("%s[%s]: write() failed, we have been waiting for CAN buffers for too long (>1000 ms)\n", __func__, ctx->name);
				return CSP_ERR_TX;
			}
		} else {
			waiting_ms = 0;
			pdata += written;
			length -= written;
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
	filters[1].can_id = ((1 << (csp_id_get_host_bits() - netmask)) - 1) << CFP2_DST_OFFSET;
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
