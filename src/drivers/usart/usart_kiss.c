

#include <csp/interfaces/csp_if_kiss.h>

#include <csp/csp_debug.h>
#include <stdlib.h>
#include <string.h>

#include <csp/csp.h>
#include <csp/arch/csp_queue.h>
#include <csp/arch/csp_time.h>
#include <csp/drivers/usart.h>

typedef struct {
	char name[CSP_IFLIST_NAME_MAX + 1];
	csp_iface_t iface;
	csp_kiss_interface_data_t ifdata;
	csp_usart_fd_t fd;
	csp_queue_handle_t txq;       /**< transmit queue of csp_packet_t *, see CSP_USART_KISS_TX_QUEUE_LEN; NULL = synchronous */
	csp_static_queue_t txq_static;
	char txq_buf[CSP_USART_KISS_TX_QUEUE_LEN * sizeof(csp_packet_t *)];
	uint32_t tx_queue_full_drops; /**< packets refused because the transmit queue was full, see csp_usart_kiss_tx_queue_full_count() */
	uint32_t tx_queue_depth_max;  /**< deepest the transmit queue has been, see csp_usart_kiss_tx_queue_depth_max() */
	uint32_t tx_deadline_giveups; /**< packets dropped at dequeue because their timestamp_tx deadline passed, see csp_usart_kiss_tx_deadline_giveup_count() */
} kiss_context_t;

/* Scheduling priority for TX threads created from now on; 0 = default */
static int kiss_tx_thread_priority = 0;

void csp_usart_kiss_set_tx_priority(int priority) {
	kiss_tx_thread_priority = (priority < 0) ? 0 : priority;
}

int csp_usart_kiss_tx_priority(void) {
	return kiss_tx_thread_priority;
}

static int kiss_driver_tx(void * driver_data, const unsigned char * data, size_t data_length) {

	kiss_context_t * ctx = driver_data;
	if (csp_usart_write(ctx->fd, data, data_length) == (int)data_length) {
		return CSP_ERR_NONE;
	}
	return CSP_ERR_TX;
}

/* Resolve the driver context behind an interface without dereferencing a
 * foreign driver_data: the interface must be a KISS interface, its KISS
 * interface data must name this driver's transmit function, and the
 * context's iface member must be the interface itself. */
static const kiss_context_t * kiss_ctx(const csp_iface_t * iface) {
	if ((iface == NULL) || (iface->nexthop != csp_kiss_tx) || (iface->driver_data == NULL) || (iface->interface_data == NULL)) {
		return NULL;
	}
	const csp_kiss_interface_data_t * ifdata = iface->interface_data;
	if (ifdata->tx_func != kiss_driver_tx) {
		return NULL;
	}
	const kiss_context_t * ctx = iface->driver_data;
	return (&ctx->iface == iface) ? ctx : NULL;
}

uint32_t csp_usart_kiss_tx_queue_full_count(const csp_iface_t * iface) {
	const kiss_context_t * ctx = kiss_ctx(iface);
	return ctx ? ctx->tx_queue_full_drops : 0;
}

uint32_t csp_usart_kiss_tx_queue_depth_max(const csp_iface_t * iface) {
	const kiss_context_t * ctx = kiss_ctx(iface);
	return ctx ? ctx->tx_queue_depth_max : 0;
}

uint32_t csp_usart_kiss_tx_deadline_giveup_count(const csp_iface_t * iface) {
	const kiss_context_t * ctx = kiss_ctx(iface);
	return ctx ? ctx->tx_deadline_giveups : 0;
}

/* The interface's tx_enqueue hook: called on the sender's thread, never
 * blocks, never touches the tty. Ownership of the packet passes to the queue
 * on success; on a full queue it stays with the caller, which frees it and
 * counts tx_error (csp_send_direct_iface()). */
static int kiss_tx_enqueue(void * driver_data, csp_packet_t * packet) {
	kiss_context_t * ctx = driver_data;

	if (csp_queue_enqueue(ctx->txq, &packet, 0) != CSP_QUEUE_OK) {
		ctx->tx_queue_full_drops++;
		return CSP_ERR_TX;
	}
	const int depth = csp_queue_size(ctx->txq);
	if ((depth > 0) && ((uint32_t)depth > ctx->tx_queue_depth_max)) {
		ctx->tx_queue_depth_max = (uint32_t)depth;
	}
	return CSP_ERR_NONE;
}

/* One writer per interface: every wait for the tty happens here. A packet
 * whose deadline already passed while it queued is dropped whole, so a
 * stalled or slow serial link costs stale packets, not rate-group time. */
static void * kiss_tx_thread(void * arg) {
	kiss_context_t * ctx = arg;
	csp_packet_t * packet;

	while (1) {
		if (csp_queue_dequeue(ctx->txq, &packet, CSP_MAX_TIMEOUT) != CSP_QUEUE_OK) {
			continue;
		}
		const uint32_t deadline_ms = packet->timestamp_tx;
		if ((deadline_ms != 0) && ((int32_t)(csp_get_ms() - deadline_ms) >= 0)) {
			ctx->tx_deadline_giveups++;
			ctx->iface.tx_error++;
			csp_buffer_free(packet);
			continue;
		}
		if (csp_kiss_tx_packet_sync(&ctx->iface, packet) != CSP_ERR_NONE) {
			/* The sync path does not free on failure. */
			ctx->iface.tx_error++;
			csp_buffer_free(packet);
		}
	}
	return NULL;
}

static void kiss_driver_rx(void * user_data, uint8_t * data, size_t data_size, void * pxTaskWoken) {

	kiss_context_t * ctx = user_data;
	csp_kiss_rx(&ctx->iface, data, data_size, pxTaskWoken);
}

int csp_usart_open_and_add_kiss_interface(const csp_usart_conf_t * conf, const char * ifname, uint16_t addr, csp_iface_t ** return_iface) {

	if (ifname == NULL) {
		ifname = CSP_IF_KISS_DEFAULT_NAME;
	}

	kiss_context_t * ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL) {
		return CSP_ERR_NOMEM;
	}

	strncpy(ctx->name, ifname, sizeof(ctx->name) - 1);
	ctx->iface.name = ctx->name;
	ctx->iface.addr = addr;
	ctx->iface.driver_data = ctx;
	ctx->iface.interface_data = &ctx->ifdata;
	ctx->ifdata.tx_func = kiss_driver_tx;

#if (CSP_ZEPHYR)
	ctx->fd = NULL;
#else
	ctx->fd = -1;
#endif

	int res = csp_kiss_add_interface(&ctx->iface);
	if (res == CSP_ERR_NONE) {
		res = csp_usart_open(conf, kiss_driver_rx, ctx, &ctx->fd);
	}

	/* Transmit queue and thread. Not having them is not a failure: the
	 * interface then transmits synchronously on the caller's thread, which
	 * is what every KISS driver did before and what a platform without
	 * threads (zephyr) still does. */
	if (res == CSP_ERR_NONE) {
		ctx->txq = csp_queue_create_static(CSP_USART_KISS_TX_QUEUE_LEN, sizeof(csp_packet_t *), ctx->txq_buf, &ctx->txq_static);
		if (ctx->txq == NULL) {
			csp_print("%s[%s]: no transmit queue, sending synchronously\n", __func__, ctx->name);
		} else {
			const int thread_res = csp_usart_tx_thread_create(kiss_tx_thread, ctx, ctx->name, kiss_tx_thread_priority);
			if (thread_res == CSP_ERR_NONE) {
				ctx->ifdata.tx_enqueue = kiss_tx_enqueue;
			} else {
				if (thread_res != CSP_ERR_NOTSUP) {
					csp_print("%s[%s]: TX thread not started (%d), sending synchronously\n", __func__, ctx->name, thread_res);
				}
				csp_queue_free(ctx->txq);
				ctx->txq = NULL;
			}
		}
	}

	if (return_iface) {
		*return_iface = &ctx->iface;
	}

	return res;
}
