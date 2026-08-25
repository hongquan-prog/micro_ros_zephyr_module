/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Process Mission
 *
 * Signal-chain public interface (business side, used by main() and the
 * "hb" shell commands).
 */

#ifndef SIGNAL_CHAIN_H
#define SIGNAL_CHAIN_H

#include <stdint.h>

struct signal_chain_diag;

int signal_chain_init(void);

/* Control period in ms, 1..1000.  Restarts the tick timer immediately. */
int signal_chain_set_period(uint32_t ms);
uint32_t signal_chain_get_period(void);

void signal_chain_get_diag(struct signal_chain_diag *out);

#endif /* SIGNAL_CHAIN_H */
