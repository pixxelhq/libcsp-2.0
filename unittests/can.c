#include <check.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <csp/csp.h>
#include <csp/interfaces/csp_if_can.h>
#include <csp/csp_interface.h>
#if (CSP_HAVE_LIBSOCKETCAN)
#include <linux/can.h>
#include <csp/drivers/can_socketcan.h>
#endif

#include "../src/csp_qfifo.h"

/* https://github.com/libcsp/libcsp/issues/1023
 *
 * Packets are sent through csp_can1_tx()/csp_can2_tx() into a fake driver
 * that records every CAN frame. The frames are then replayed into
 * csp_can_rx() on a second interface, in the order each test chooses, and
 * the router input FIFO is inspected for the result.
 */

#define MAX_FRAMES 64
#define TX_ADDR    1
#define RX_ADDR    2
#define SPORT      10
#define DPORT      11

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t dlc;
} frame_t;

static frame_t frames[MAX_FRAMES];
static unsigned int frame_count;

static csp_can_interface_data_t tx_ifdata;
static csp_can_interface_data_t rx_ifdata;
static csp_iface_t tx_iface = {.name = "CANTX", .addr = TX_ADDR, .interface_data = &tx_ifdata};
static csp_iface_t rx_iface = {.name = "CANRX", .addr = RX_ADDR, .interface_data = &rx_ifdata};

static int baseline;

static int capture_tx(void * driver_data, uint32_t id, const uint8_t * data, uint8_t dlc, const csp_packet_t * packet) {
	(void)driver_data;
	(void)packet;
	ck_assert_uint_lt(frame_count, MAX_FRAMES);
	ck_assert_uint_le(dlc, sizeof(frames[0].data));
	frames[frame_count].id = id;
	frames[frame_count].dlc = dlc;
	memcpy(frames[frame_count].data, data, dlc);
	frame_count++;
	return CSP_ERR_NONE;
}

static void can_test_init(unsigned int version) {

	csp_conf.version = version;
	csp_init();

	/* csp_init() recreates the buffer pool, so drop any pbuf left by a previous test */
	tx_ifdata.pbufs = NULL;
	rx_ifdata.pbufs = NULL;
	tx_ifdata.tx_func = capture_tx;
	rx_ifdata.tx_func = capture_tx;
	tx_iface.frame = tx_iface.rx_error = tx_iface.drop = 0;
	rx_iface.frame = rx_iface.rx_error = rx_iface.drop = 0;

	ck_assert_int_eq(csp_can_add_interface(&tx_iface), CSP_ERR_NONE);
	ck_assert_int_eq(csp_can_add_interface(&rx_iface), CSP_ERR_NONE);

	frame_count = 0;
	baseline = csp_buffer_remaining();
}

static uint8_t pattern(uint16_t i) {
	return (uint8_t)(i * 7 + 3);
}

static void send_packet(uint16_t length) {

	frame_count = 0;

	csp_packet_t * packet = csp_buffer_get(0);
	ck_assert_ptr_nonnull(packet);
	packet->id.pri = CSP_PRIO_NORM;
	packet->id.src = TX_ADDR;
	packet->id.dst = RX_ADDR;
	packet->id.sport = SPORT;
	packet->id.dport = DPORT;
	packet->id.flags = 0;
	packet->length = length;
	for (uint16_t i = 0; i < length; i++) {
		packet->data[i] = pattern(i);
	}

	ck_assert_int_eq(tx_iface.nexthop(&tx_iface, CSP_NO_VIA_ADDRESS, packet, 1), CSP_ERR_NONE);
}

static void feed_frame(const frame_t * f) {
	csp_can_rx(&rx_iface, f->id, f->data, f->dlc, 0, NULL);
}

static void feed(const unsigned int * order, unsigned int count) {
	for (unsigned int i = 0; i < count; i++) {
		ck_assert_uint_lt(order[i], frame_count);
		feed_frame(&frames[order[i]]);
	}
}

static void feed_all(void) {
	for (unsigned int i = 0; i < frame_count; i++) {
		feed_frame(&frames[i]);
	}
}

static void expect_delivered(uint16_t length) {

	csp_qfifo_t input;
	ck_assert_int_eq(csp_qfifo_read(&input), CSP_ERR_NONE);
	ck_assert_ptr_eq(input.iface, &rx_iface);
	ck_assert_uint_eq(input.packet->length, length);
	ck_assert_uint_eq(input.packet->id.src, TX_ADDR);
	ck_assert_uint_eq(input.packet->id.dst, RX_ADDR);
	ck_assert_uint_eq(input.packet->id.sport, SPORT);
	ck_assert_uint_eq(input.packet->id.dport, DPORT);
	for (uint16_t i = 0; i < length; i++) {
		ck_assert_uint_eq(input.packet->data[i], pattern(i));
	}
	csp_buffer_free(input.packet);
	ck_assert_int_eq(csp_buffer_remaining(), baseline);
}

static void expect_nothing(void) {
	csp_qfifo_t input;
	ck_assert_int_eq(csp_qfifo_read(&input), CSP_ERR_TIMEDOUT);
}

/* CFP 1.x MORE fragment k of N carries remain = N - 1 - k, so frame index = N - remain */
static void set_remain(frame_t * f, uint32_t remain) {
	f->id = (f->id & ~CFP_MAKE_REMAIN(0xFF)) | CFP_MAKE_REMAIN(remain);
}

static void test_in_order(unsigned int version) {

	static const uint16_t lengths[] = {0, 1, 2, 3, 4, 5, 8, 10, 11, 12, 13, 48, CSP_BUFFER_SIZE};

	can_test_init(version);

	for (size_t i = 0; i < sizeof(lengths) / sizeof(lengths[0]); i++) {
		send_packet(lengths[i]);
		feed_all();
		expect_delivered(lengths[i]);
	}
	ck_assert_uint_eq(rx_iface.frame, 0);
}

START_TEST(test_can_in_order_v1)
{
	test_in_order(1);
}
END_TEST

START_TEST(test_can_in_order_v2)
{
	test_in_order(2);
}
END_TEST

/* The capture from the issue: 7 frames, the last two MORE fragments swapped */
START_TEST(test_can_v1_last_two_swapped_1023)
{
	static const unsigned int order[] = {0, 1, 2, 3, 4, 6, 5};

	can_test_init(1);
	send_packet(48);
	ck_assert_uint_eq(frame_count, 7);
	feed(order, 7);

#if (CSP_CFP_OUT_OF_ORDER_RX)
	expect_delivered(48);
	ck_assert_uint_eq(rx_iface.frame, 0);
#else
	expect_nothing();
	ck_assert_uint_eq(rx_iface.frame, 2);
	ck_assert_int_eq(csp_buffer_remaining(), baseline);
#endif
}
END_TEST

/* Orderings observed on SocketCAN for 6 MORE fragments (remain 5..0), plus full reversal */
START_TEST(test_can_v1_reordered_1023)
{
	static const unsigned int orders[][7] = {
		{0, 2, 1, 3, 4, 5, 6}, /* remain 4 5 3 2 1 0 */
		{0, 1, 2, 4, 3, 5, 6}, /* remain 5 4 2 3 1 0 */
		{0, 1, 2, 6, 3, 4, 5}, /* remain 5 4 0 3 2 1 */
		{0, 1, 3, 2, 4, 5, 6}, /* remain 5 3 4 2 1 0 */
		{0, 6, 5, 4, 3, 2, 1}, /* fully reversed */
	};

	can_test_init(1);

	for (size_t i = 0; i < sizeof(orders) / sizeof(orders[0]); i++) {
		rx_iface.frame = 0;
		send_packet(48);
		ck_assert_uint_eq(frame_count, 7);
		feed(orders[i], 7);
#if (CSP_CFP_OUT_OF_ORDER_RX)
		expect_delivered(48);
		ck_assert_uint_eq(rx_iface.frame, 0);
#else
		expect_nothing();
		ck_assert_uint_ge(rx_iface.frame, 1);
		ck_assert_int_eq(csp_buffer_remaining(), baseline);
#endif
	}
}
END_TEST

START_TEST(test_can_v1_duplicate_fragment)
{
	static const unsigned int order[] = {0, 1, 2, 3, 3, 4, 5, 6};

	can_test_init(1);
	send_packet(48);
	feed(order, 8);

#if (CSP_CFP_OUT_OF_ORDER_RX)
	expect_delivered(48);
	ck_assert_uint_eq(rx_iface.frame, 1);
#else
	expect_nothing();
	ck_assert_uint_eq(rx_iface.frame, 4);
	ck_assert_int_eq(csp_buffer_remaining(), baseline);
#endif
	expect_nothing();
}
END_TEST

/* A MORE frame with remain >= N belongs to another packet that reused the id */
START_TEST(test_can_v1_straggler_fragment)
{
	static const unsigned int head[] = {0, 1, 2, 3};
	static const unsigned int tail[] = {4, 5, 6};

	can_test_init(1);
	send_packet(48);

	frame_t straggler = frames[1];
	set_remain(&straggler, 200);

	feed(head, 4);
	feed_frame(&straggler);
	feed(tail, 3);

#if (CSP_CFP_OUT_OF_ORDER_RX)
	expect_delivered(48);
	ck_assert_uint_eq(rx_iface.frame, 1);
#else
	expect_nothing();
	ck_assert_uint_ge(rx_iface.frame, 1);
	ck_assert_int_eq(csp_buffer_remaining(), baseline);
#endif
}
END_TEST

/* A missing fragment holds the pbuf until CSP_CAN_PBUF_TIMEOUT_MS, then the next BEGIN reclaims it */
START_TEST(test_can_v1_missing_fragment_times_out)
{
	static const unsigned int order[] = {0, 1, 2, 3, 4, 5};

	can_test_init(1);
	send_packet(48);
	feed(order, 6);

	expect_nothing();
	ck_assert_int_eq(csp_buffer_remaining(), baseline - 1);

	usleep((CSP_CAN_PBUF_TIMEOUT_MS + 50) * 1000);

	send_packet(8);
	feed_all();
	expect_delivered(8);
}
END_TEST

#if (CSP_CFP_OUT_OF_ORDER_RX)

START_TEST(test_can_v1_begin_remain_mismatch)
{
	can_test_init(1);
	send_packet(48);

	set_remain(&frames[0], CFP_REMAIN(frames[0].id) + 1);
	feed_all();

	expect_nothing();
	ck_assert_uint_ge(rx_iface.frame, 1);
	ck_assert_int_eq(csp_buffer_remaining(), baseline);
}
END_TEST

START_TEST(test_can_v1_short_middle_fragment)
{
	can_test_init(1);
	send_packet(48);

	frames[2].dlc = 7;
	feed_all();

	expect_nothing();
	ck_assert_uint_ge(rx_iface.frame, 1);
	ck_assert_int_eq(csp_buffer_remaining(), baseline);
}
END_TEST

#endif /* CSP_CFP_OUT_OF_ORDER_RX */

/* CFP 2.0: 48 bytes = BEGIN (4 bytes) + 6 fragments (fc 1..6, last is END) */
START_TEST(test_can_v2_reordered_within_window)
{
	static const unsigned int orders[][7] = {
		{0, 1, 2, 3, 4, 6, 5}, /* END one ahead */
		{0, 1, 2, 3, 6, 4, 5}, /* END two ahead */
		{0, 2, 1, 3, 4, 5, 6}, /* distance 1 */
		{0, 1, 5, 2, 3, 4, 6}, /* distance 3 */
		{0, 1, 6, 2, 3, 4, 5}, /* distance 4 (window edge) */
		{0, 4, 3, 2, 1, 5, 6}, /* several ahead, then fill */
	};

	can_test_init(2);

	for (size_t i = 0; i < sizeof(orders) / sizeof(orders[0]); i++) {
		rx_iface.frame = 0;
		send_packet(48);
		ck_assert_uint_eq(frame_count, 7);
		feed(orders[i], 7);
#if (CSP_CFP_OUT_OF_ORDER_RX)
		expect_delivered(48);
		ck_assert_uint_eq(rx_iface.frame, 0);
#else
		expect_nothing();
		ck_assert_uint_ge(rx_iface.frame, 1);
		ck_assert_int_eq(csp_buffer_remaining(), baseline);
#endif
	}
}
END_TEST

/* A fragment more than the window ahead is still reported as lost */
START_TEST(test_can_v2_beyond_window_is_lost)
{
	/* 80 bytes = BEGIN (4) + 10 fragments; fc 7 arrives while fc 2 is expected */
	static const unsigned int order[] = {0, 1, 7, 2, 3, 4, 5, 6, 8, 9, 10};

	can_test_init(2);
	send_packet(80);
	ck_assert_uint_eq(frame_count, 11);
	feed(order, 11);

	expect_nothing();
	ck_assert_uint_ge(rx_iface.frame, 1);
	ck_assert_int_eq(csp_buffer_remaining(), baseline);
}
END_TEST

START_TEST(test_can_v2_duplicate_ahead)
{
	static const unsigned int order[] = {0, 1, 3, 3, 2, 4, 5, 6};

	can_test_init(2);
	send_packet(48);
	feed(order, 8);

#if (CSP_CFP_OUT_OF_ORDER_RX)
	expect_delivered(48);
	ck_assert_uint_eq(rx_iface.frame, 1);
#else
	expect_nothing();
	ck_assert_uint_ge(rx_iface.frame, 1);
	ck_assert_int_eq(csp_buffer_remaining(), baseline);
#endif
	expect_nothing();
}
END_TEST

/* Per-interface reassembly timeout counter: a partial packet that is
 * reclaimed by the pbuf timeout is counted on its interface, so a host can
 * tell timeout fallout apart from independent frame losses. */
START_TEST(test_can_v1_missing_fragment_counts_timeout)
{
	static const unsigned int order[] = {0, 1, 2, 3, 4, 5};

	can_test_init(1);
	uint32_t before = csp_can_rx_timeout_count(&rx_iface);

	send_packet(48);
	feed(order, 6);
	expect_nothing();

	usleep((CSP_CAN_PBUF_TIMEOUT_MS + 50) * 1000);

	send_packet(8);
	feed_all();
	expect_delivered(8);

	ck_assert_uint_eq(csp_can_rx_timeout_count(&rx_iface), before + 1);
	ck_assert_uint_eq(csp_can_rx_timeout_count(NULL), 0);
}
END_TEST

/* csp_send_direct_iface() is public: a host that owns several interfaces
 * with the same address needs to pick the outgoing one per packet. */
START_TEST(test_send_direct_iface_forces_interface)
{
	can_test_init(1);

	csp_packet_t * packet = csp_buffer_get(0);
	ck_assert_ptr_nonnull(packet);
	packet->length = 4;
	for (uint16_t i = 0; i < packet->length; i++) {
		packet->data[i] = pattern(i);
	}

	csp_id_t idout = {.pri = CSP_PRIO_NORM, .flags = 0, .src = TX_ADDR, .dst = RX_ADDR, .dport = DPORT, .sport = SPORT};
	frame_count = 0;
	csp_send_direct_iface(&idout, packet, &tx_iface, CSP_NO_VIA_ADDRESS, 1);
	ck_assert_uint_gt(frame_count, 0);

	feed_all();
	expect_delivered(4);
}
END_TEST

#if (CSP_HAVE_LIBSOCKETCAN)

static int count_matches(uint32_t id, const struct can_filter * f, int n) {
	int matches = 0;
	for (int i = 0; i < n; i++) {
		if ((id & f[i].can_mask) == (f[i].can_id & f[i].can_mask)) {
			matches++;
		}
	}
	return matches;
}

/* CSP 1.x non-promiscuous filter: accept frames addressed to this interface
 * OR to the broadcast address, each exactly once. */
START_TEST(test_socketcan_filters_v1_own_or_broadcast)
{
	struct can_filter f[3];
	int n = csp_can_socketcan_build_filters(1, 4, 0, false, f, 3);
	ck_assert_int_eq(n, 2);

	for (uint32_t dst = 0; dst < (1U << CFP_HOST_SIZE); dst++) {
		uint32_t id = CFP_MAKE_DST(dst) | CFP_MAKE_SRC(7) | CFP_MAKE_TYPE(0);
		ck_assert_int_eq(count_matches(id, f, n), (dst == 4 || dst == 31) ? 1 : 0);
	}
}
END_TEST

START_TEST(test_socketcan_filters_v1_broadcast_address_collapses)
{
	struct can_filter f[3];
	int n = csp_can_socketcan_build_filters(1, 31, 0, false, f, 3);
	ck_assert_int_eq(n, 1);
	ck_assert_int_eq(count_matches(CFP_MAKE_DST(31), f, n), 1);
	ck_assert_int_eq(count_matches(CFP_MAKE_DST(4), f, n), 0);
}
END_TEST

START_TEST(test_socketcan_filters_promisc_accepts_all)
{
	struct can_filter f[3];
	int n = csp_can_socketcan_build_filters(1, 4, 0, true, f, 3);
	ck_assert_int_eq(n, 1);
	ck_assert_uint_eq(f[0].can_mask, 0);
}
END_TEST

START_TEST(test_socketcan_filters_v2_unchanged)
{
	struct can_filter f[3];
	int n = csp_can_socketcan_build_filters(2, 4, 0, false, f, 3);
	ck_assert_int_eq(n, 3);
	ck_assert_uint_eq(f[0].can_id, 4U << CFP2_DST_OFFSET);
	ck_assert_uint_eq(f[2].can_id, 0x3FFFU << CFP2_DST_OFFSET);
	ck_assert_int_eq(csp_can_socketcan_build_filters(1, 4, 0, false, f, 1), CSP_ERR_INVAL);
}
END_TEST

/* Driver counters are only meaningful on a socketcan interface; anything
 * else reads as zero rather than as garbage from a foreign driver_data. */
START_TEST(test_socketcan_counters_zero_for_foreign_iface)
{
	can_test_init(1);
	ck_assert_uint_eq(csp_can_socketcan_tx_enobufs_count(&rx_iface), 0);
	ck_assert_uint_eq(csp_can_socketcan_rx_overrun_count(&rx_iface), 0);
	ck_assert_uint_eq(csp_can_socketcan_tx_eagain_count(&rx_iface), 0);
	ck_assert_uint_eq(csp_can_socketcan_bus_off_count(&rx_iface), 0);
	ck_assert_uint_eq(csp_can_socketcan_tx_passive_count(&rx_iface), 0);
	ck_assert_uint_eq(csp_can_socketcan_tx_enobufs_count(NULL), 0);
	ck_assert_uint_eq(csp_can_socketcan_rx_overrun_count(NULL), 0);
	ck_assert_uint_eq(csp_can_socketcan_tx_eagain_count(NULL), 0);
	ck_assert_uint_eq(csp_can_socketcan_bus_off_count(NULL), 0);
	ck_assert_uint_eq(csp_can_socketcan_tx_passive_count(NULL), 0);
}
END_TEST

/* The KISS driver's context has the same {name, iface} prefix as the
 * socketcan one, with a smaller tail. A getter that identifies "its" context
 * by that prefix alone would read past the end of a KISS allocation. */
static int fake_kiss_nexthop(csp_iface_t * iface, uint16_t via, csp_packet_t * packet, int from_me) {
	(void)iface; (void)via; (void)from_me;
	csp_buffer_free(packet);
	return CSP_ERR_NONE;
}

START_TEST(test_socketcan_counters_zero_for_kiss_like_context)
{
	struct {
		char name[CSP_IFLIST_NAME_MAX + 1];
		csp_iface_t iface;
		uint8_t tail[64];
	} fake;
	memset(&fake, 0xAB, sizeof(fake));   /* whatever lies past iface is non-zero */
	memset(&fake.iface, 0, sizeof(fake.iface));
	fake.iface.name = fake.name;
	fake.iface.driver_data = &fake;
	fake.iface.nexthop = fake_kiss_nexthop;

	ck_assert_uint_eq(csp_can_socketcan_tx_enobufs_count(&fake.iface), 0);
	ck_assert_uint_eq(csp_can_socketcan_rx_overrun_count(&fake.iface), 0);
	ck_assert_uint_eq(csp_can_socketcan_tx_eagain_count(&fake.iface), 0);
	ck_assert_uint_eq(csp_can_socketcan_bus_off_count(&fake.iface), 0);
	ck_assert_uint_eq(csp_can_socketcan_tx_passive_count(&fake.iface), 0);
}
END_TEST

/* The TX retry policy. The socket is non-blocking, so a full per-socket send
 * buffer surfaces as EAGAIN just as a full qdisc surfaces as ENOBUFS, and both
 * must back off for real: charging the budget without sleeping gave up after
 * ~200 instantaneous retries while reporting a one-second wait. */
START_TEST(test_socketcan_backoff_eagain_sleeps_until_budget)
{
	uint32_t waiting_ms = 0;
	const unsigned int sleeps = CSP_CAN_SOCKETCAN_TX_BUDGET_MS / CSP_CAN_SOCKETCAN_TX_BACKOFF_MS;

	for (unsigned int i = 0; i < sleeps; i++) {
		ck_assert_int_eq(csp_can_socketcan_tx_backoff(EAGAIN, &waiting_ms), CSP_CAN_SOCKETCAN_TX_RETRY_SLEEP);
	}
	ck_assert_uint_eq(waiting_ms, CSP_CAN_SOCKETCAN_TX_BUDGET_MS);

	/* Only once the whole budget has actually been slept does it give up */
	ck_assert_int_eq(csp_can_socketcan_tx_backoff(EAGAIN, &waiting_ms), CSP_CAN_SOCKETCAN_TX_GIVE_UP);
	ck_assert_uint_eq(waiting_ms, CSP_CAN_SOCKETCAN_TX_BUDGET_MS);
}
END_TEST

START_TEST(test_socketcan_backoff_enobufs_same_policy)
{
	uint32_t waiting_ms = 0;
	const unsigned int sleeps = CSP_CAN_SOCKETCAN_TX_BUDGET_MS / CSP_CAN_SOCKETCAN_TX_BACKOFF_MS;

	for (unsigned int i = 0; i < sleeps; i++) {
		ck_assert_int_eq(csp_can_socketcan_tx_backoff(ENOBUFS, &waiting_ms), CSP_CAN_SOCKETCAN_TX_RETRY_SLEEP);
	}
	ck_assert_int_eq(csp_can_socketcan_tx_backoff(ENOBUFS, &waiting_ms), CSP_CAN_SOCKETCAN_TX_GIVE_UP);

	/* The two full-buffer conditions share one budget */
	waiting_ms = CSP_CAN_SOCKETCAN_TX_BUDGET_MS - CSP_CAN_SOCKETCAN_TX_BACKOFF_MS;
	ck_assert_int_eq(csp_can_socketcan_tx_backoff(EAGAIN, &waiting_ms), CSP_CAN_SOCKETCAN_TX_RETRY_SLEEP);
	ck_assert_int_eq(csp_can_socketcan_tx_backoff(ENOBUFS, &waiting_ms), CSP_CAN_SOCKETCAN_TX_GIVE_UP);
}
END_TEST

/* A signal is not buffer pressure: retry at once and leave the budget alone */
START_TEST(test_socketcan_backoff_eintr_is_free)
{
	uint32_t waiting_ms = 0;
	for (unsigned int i = 0; i < 1000; i++) {
		ck_assert_int_eq(csp_can_socketcan_tx_backoff(EINTR, &waiting_ms), CSP_CAN_SOCKETCAN_TX_RETRY_NOW);
	}
	ck_assert_uint_eq(waiting_ms, 0);
}
END_TEST

START_TEST(test_socketcan_backoff_hard_error_fails_immediately)
{
	uint32_t waiting_ms = 7;
	ck_assert_int_eq(csp_can_socketcan_tx_backoff(EINVAL, &waiting_ms), CSP_CAN_SOCKETCAN_TX_FAIL);
	ck_assert_int_eq(csp_can_socketcan_tx_backoff(ENETDOWN, &waiting_ms), CSP_CAN_SOCKETCAN_TX_FAIL);
	ck_assert_uint_eq(waiting_ms, 7);
	ck_assert_int_eq(csp_can_socketcan_tx_backoff(EAGAIN, NULL), CSP_CAN_SOCKETCAN_TX_FAIL);
}
END_TEST

#endif /* CSP_HAVE_LIBSOCKETCAN */

Suite * can_suite(void) {

	Suite * s = suite_create("CAN");
	TCase * tc = tcase_create("reassembly");
	tcase_set_timeout(tc, 30);

	tcase_add_test(tc, test_can_in_order_v1);
	tcase_add_test(tc, test_can_in_order_v2);
	tcase_add_test(tc, test_can_v1_last_two_swapped_1023);
	tcase_add_test(tc, test_can_v1_reordered_1023);
	tcase_add_test(tc, test_can_v1_duplicate_fragment);
	tcase_add_test(tc, test_can_v1_straggler_fragment);
	tcase_add_test(tc, test_can_v1_missing_fragment_times_out);
#if (CSP_CFP_OUT_OF_ORDER_RX)
	tcase_add_test(tc, test_can_v1_begin_remain_mismatch);
	tcase_add_test(tc, test_can_v1_short_middle_fragment);
#endif
	tcase_add_test(tc, test_can_v2_reordered_within_window);
	tcase_add_test(tc, test_can_v2_beyond_window_is_lost);
	tcase_add_test(tc, test_can_v2_duplicate_ahead);

	tcase_add_test(tc, test_can_v1_missing_fragment_counts_timeout);
	tcase_add_test(tc, test_send_direct_iface_forces_interface);
#if (CSP_HAVE_LIBSOCKETCAN)
	tcase_add_test(tc, test_socketcan_filters_v1_own_or_broadcast);
	tcase_add_test(tc, test_socketcan_filters_v1_broadcast_address_collapses);
	tcase_add_test(tc, test_socketcan_filters_promisc_accepts_all);
	tcase_add_test(tc, test_socketcan_filters_v2_unchanged);
	tcase_add_test(tc, test_socketcan_counters_zero_for_foreign_iface);
	tcase_add_test(tc, test_socketcan_counters_zero_for_kiss_like_context);
	tcase_add_test(tc, test_socketcan_backoff_eagain_sleeps_until_budget);
	tcase_add_test(tc, test_socketcan_backoff_enobufs_same_policy);
	tcase_add_test(tc, test_socketcan_backoff_eintr_is_free);
	tcase_add_test(tc, test_socketcan_backoff_hard_error_fails_immediately);
#endif

	suite_add_tcase(s, tc);
	return s;
}
