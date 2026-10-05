/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * The operations: their entry points, which rf215_submit() dispatches to,
 * and their step tables, which rf215_step_name() searches. Kept apart from
 * rf215_core.h so the engine helpers the ops build on do not have to know
 * which ops exist; only rf215_dev.c and the op files include this.
 */

#ifndef KAONIC_DRIVERS_RF215_OP_H__
#define KAONIC_DRIVERS_RF215_OP_H__

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215/rf215_api.h"

/******************************************************************************/
/** Defines *******************************************************************/

/* Fills a table entry, written as `{ RF215_STEP(fn) },`: the name is the
 * step's own identifier, so a step is never listed under a stale name, and
 * the position in the table is the index. */
#define RF215_STEP(step) .fn = (step), .name = #step

/******************************************************************************/
/** Types *********************************************************************/

/* One step of an op, as reported by rf215_step_name(). */
struct rf215_step_name {
    rf215_step_fn fn;
    const char* name;
};

/******************************************************************************/
/** Step Tables ***************************************************************/

/* Each op owns its table; the accessor returns it and stores its length in
 * `*count`. */
const struct rf215_step_name* rf215_op_reset_steps(size_t* count);
const struct rf215_step_name* rf215_op_configure_steps(size_t* count);
const struct rf215_step_name* rf215_op_tx_steps(size_t* count);
const struct rf215_step_name* rf215_op_rx_steps(size_t* count);
const struct rf215_step_name* rf215_op_iqif_steps(size_t* count);

/******************************************************************************/
/** Entry Points **************************************************************/

enum rf215_step_result
rf215_op_reset_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
enum rf215_step_result
rf215_op_configure_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
enum rf215_step_result
rf215_op_tx_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
enum rf215_step_result
rf215_op_rx_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
enum rf215_step_result
rf215_op_rx_stop(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
enum rf215_step_result
rf215_op_rx_read_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
enum rf215_step_result
rf215_op_rx_rearm_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
enum rf215_step_result
rf215_op_ed_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
enum rf215_step_result
rf215_op_iqif_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);

/******************************************************************************/
/** Shared Sequences **********************************************************/

/*
 * Brings the transceiver into RX the way the datasheet and the errata want it:
 * through TXPREP, with the PLL confirmed locked (and kicked if it is not), and
 * with every commanded transition read back and re-issued if it was dropped.
 * Continues at `resume` once RFn_STATE reads RX; on failure the operation
 * ends with an error event. A transceiver already in RX with a locked PLL is
 * left alone. Uses op->tries and op->polls.
 */
enum rf215_step_result rf215_op_rx_arm(struct rf215_dev* dev,
                                       struct rf215_op* op,
                                       struct rf215_xfer* out,
                                       rf215_step_fn resume);

#endif /* KAONIC_DRIVERS_RF215_OP_H__ */
