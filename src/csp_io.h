#pragma once

#include <csp/csp.h>
#include <csp/csp_interface.h>

/**
 * Send a packet, resolving the outgoing interface(s) from the destination
 * address in @p idout.
 *
 * The packet is always consumed by this call, either handed over to one or
 * more interfaces or freed on error.
 *
 * @param idout header (identifier) to send with
 * @param packet packet to send
 * @param routed_from interface the packet was received on, or NULL if the
 *        packet originates from the local node
 */
void csp_send_direct(csp_id_t* idout, csp_packet_t * packet, csp_iface_t * routed_from);
