// Copyright 2026 Process Mission
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <uxr/client/transport.h>

#include <microros_transports.h>
#include <version.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/dt-bindings/interrupt-controller/arm-gic.h>

#include <zephyr/kernel/mm.h>

#include <string.h>

/*
 * micro-ROS transport over the ZVisor shared-memory endpoints.
 *
 * Wire protocol (must match the Linux zvisor-shmem peer):
 *   - One zvisor-shmem window from devicetree (zv_shm), split into two
 *     equal halves: first half is the request (TX) window, second half
 *     is the response (RX) window.
 *   - Frame layout in each half: [u32 payload length][u32 state][payload].
 *   - state: SLOT_FREE (slot consumed, peer may write) or SLOT_BUSY
 *     (frame pending, written by the slot owner). Only one message per
 *     direction is in flight at any time.
 *   - TX: wait for the request slot to become FREE, write the frame,
 *     flag it BUSY, then ring the doorbell with HVC 0x86000000 and the
 *     window's hypercall token; the hypervisor raises the peer's
 *     virtual IRQ (Linux IRQ 42).
 *   - RX: the hypervisor raises our virtual IRQ (default 41) after the
 *     peer wrote the response half; the ISR only wakes the transport
 *     reader, which
 *     copies the frame out and flags the slot FREE again. The slot is
 *     freed exclusively by the reader thread — never in the ISR — so a
 *     new frame can never overwrite a frame the framing layer has not
 *     fully consumed (ISR copy + early FREE loses frames under load).
 *
 * Both windows are mapped cache-none: the peer is another VM managed by
 * the hypervisor, so no cache maintenance is needed (and none is safe).
 */

#if !DT_NODE_EXISTS(DT_NODELABEL(zv_shm))
#error "ZVisor shmem transport requires a zv_shm zephyr,zvisor-shmem node"
#endif

#define ZV_NODE		DT_NODELABEL(zv_shm)

#define REQ_GPA		((uint32_t)DT_REG_ADDR(ZV_NODE))
#define RSP_GPA		(REQ_GPA + (uint32_t)DT_REG_SIZE(ZV_NODE) / 2)
#define DOORBELL_TOKEN	DT_PROP(ZV_NODE, zephyr_hypercall_token)
#define DOORBELL_IRQ	DT_PROP(ZV_NODE, zephyr_doorbell_irq)

/* ZVisor SMCCC doorbell function and success return value */
#define ZVISOR_SMCCC_FN_DOORBELL	0x86000000U
#define ZVISOR_SMCCC_RET_SUCCESS	0L

/* Only the first page of each window is used by the frame protocol */
#define WINDOW_MAP_SIZE		0x1000U

/* Frame layout: [u32 length][u32 state][payload] */
#define FRAME_OFF_LEN		0U
#define FRAME_OFF_STATE		4U
#define FRAME_OFF_PAYLOAD	8U
#define FRAME_HDR_SIZE		8U
#define FRAME_PAYLOAD_MAX	2048U

/* Slot ownership states */
#define SLOT_FREE		0U	/* consumed, writer may reuse */
#define SLOT_BUSY		1U	/* frame pending, waiting for peer */

static uint8_t *req_window;
static uint8_t *rsp_window;

/*
 * The windows are cache-none (device) memory: plain memcpy() may emit
 * ldp/stp pairs, which fault on device memory unless 16-byte aligned.
 * Copy in 4-byte granular accesses instead (the payload offset is 8,
 * so the window side is always 4-byte aligned).
 */
static void window_write(uint8_t *dst, const uint8_t *src, size_t len)
{
	size_t i;

	for (i = 0; i + sizeof(uint32_t) <= len; i += sizeof(uint32_t)) {
		uint32_t v;

		memcpy(&v, src + i, sizeof(v));
		sys_write32(v, (mem_addr_t)(dst + i));
	}
	for (; i < len; i++) {
		sys_write8(src[i], (mem_addr_t)(dst + i));
	}
}

static void window_read(uint8_t *dst, const uint8_t *src, size_t len)
{
	size_t i;

	for (i = 0; i + sizeof(uint32_t) <= len; i += sizeof(uint32_t)) {
		uint32_t v = sys_read32((mem_addr_t)(src + i));

		memcpy(dst + i, &v, sizeof(v));
	}
	for (; i < len; i++) {
		dst[i] = sys_read8((mem_addr_t)(src + i));
	}
}

/*
 * Reader-thread only (the ISR never touches these): rx_buf holds one
 * whole frame; the framing layer consumes it in small chunks via
 * rx_off. The slot is handed back to the peer only after the frame is
 * copied out, so rx_buf can never be overwritten mid-consume.
 */
static uint8_t rx_buf[FRAME_PAYLOAD_MAX];
static size_t rx_len;
static size_t rx_off;	/* consumed bytes; framing layer reads in small chunks */
K_SEM_DEFINE(rx_sem, 0, 1);

/* Debug counters for the stress app (printed by tests/stress). */
uint32_t zvisor_dbg_write_calls;
uint32_t zvisor_dbg_write_waits;
uint32_t zvisor_dbg_isr;
uint32_t zvisor_dbg_isr_badlen;
uint32_t zvisor_dbg_read_calls;
uint32_t zvisor_dbg_read_timeouts;

static bool transport_ready;

static void zvisor_shmem_irq(const void *arg)
{
	ARG_UNUSED(arg);

	/*
	 * Notify only: the payload copy and the slot hand-back happen in
	 * the reader thread, so a frame is never overwritten before the
	 * framing layer consumed it.
	 */
	if (sys_read32((mem_addr_t)rsp_window + FRAME_OFF_STATE) == SLOT_BUSY) {
		zvisor_dbg_isr++;
		k_sem_give(&rx_sem);
	}
}

static int zvisor_ring_doorbell(void)
{
	struct arm_smccc_res res;

	/* Make the frame visible to the peer VM before ringing */
	__asm__ volatile ("dsb sy" ::: "memory");

	arm_smccc_hvc(ZVISOR_SMCCC_FN_DOORBELL, DOORBELL_TOKEN,
		      0, 0, 0, 0, 0, 0, &res);
	if ((long)res.a0 != ZVISOR_SMCCC_RET_SUCCESS) {
		printk("micro-ROS: zvisor doorbell token %u failed (%ld)\n",
		       (unsigned int)DOORBELL_TOKEN, (long)res.a0);
		return -EIO;
	}

	return 0;
}

bool zephyr_transport_open(struct uxrCustomTransport * transport){
    (void) transport;

    if (transport_ready) {
        return true;
    }

    /* k_mem_map_phys_bare() has no failure return: it LOG_ERR + k_panic()
     * on exhaustion, so the pointers are always valid on return. */
    k_mem_map_phys_bare(&req_window, REQ_GPA, WINDOW_MAP_SIZE,
                        K_MEM_CACHE_NONE | K_MEM_PERM_RW);
    k_mem_map_phys_bare(&rsp_window, RSP_GPA, WINDOW_MAP_SIZE,
                        K_MEM_CACHE_NONE | K_MEM_PERM_RW);

    /* Initialize both slots: drop any stale frame left by a previous boot */
    sys_write32(0, (mem_addr_t)req_window + FRAME_OFF_LEN);
    sys_write32(SLOT_FREE, (mem_addr_t)req_window + FRAME_OFF_STATE);
    sys_write32(0, (mem_addr_t)rsp_window + FRAME_OFF_LEN);
    sys_write32(SLOT_FREE, (mem_addr_t)rsp_window + FRAME_OFF_STATE);
    __asm__ volatile ("dsb sy" ::: "memory");

    /* Doorbells are edge notifications (hypervisor-injected vIRQ style) */
    IRQ_CONNECT(DOORBELL_IRQ, 0, zvisor_shmem_irq, NULL, IRQ_TYPE_EDGE);
    irq_enable(DOORBELL_IRQ);

    printk("micro-ROS: ZVisor shmem transport open (req GPA 0x%x token %u, rsp GPA 0x%x irq %u)\n",
           REQ_GPA, (unsigned int)DOORBELL_TOKEN, RSP_GPA, DOORBELL_IRQ);

    transport_ready = true;
    return true;
}

bool zephyr_transport_close(struct uxrCustomTransport * transport){
    (void) transport;
    irq_disable(DOORBELL_IRQ);
    transport_ready = false;
    return true;
}

size_t zephyr_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    (void) transport;

    if (!transport_ready || len == 0 || len > FRAME_PAYLOAD_MAX) {
        if (err) {
            *err = 1;
        }
        return 0;
    }

    zvisor_dbg_write_calls++;

    /* Single message in flight: wait until the peer consumed the slot.
     * The peer (agent) may start much later than this guest, so wait
     * indefinitely — the same "block until the peer shows up" semantics
     * as the serial transport. */
    if (sys_read32((mem_addr_t)req_window + FRAME_OFF_STATE) != SLOT_FREE) {
        zvisor_dbg_write_waits++;
        printk("micro-ROS: zvisor shmem waiting for peer to consume TX slot\n");
        while (sys_read32((mem_addr_t)req_window + FRAME_OFF_STATE) != SLOT_FREE) {
            k_sleep(K_MSEC(1));
        }
    }

    window_write(req_window + FRAME_OFF_PAYLOAD, buf, len);
    sys_write32((uint32_t)len, (mem_addr_t)req_window + FRAME_OFF_LEN);
    sys_write32(SLOT_BUSY, (mem_addr_t)req_window + FRAME_OFF_STATE);

    if (zvisor_ring_doorbell() != 0) {
        /* Doorbell failed: release the slot so the next write can retry */
        sys_write32(SLOT_FREE, (mem_addr_t)req_window + FRAME_OFF_STATE);
        if (err) {
            *err = 1;
        }
        return 0;
    }

    return len;
}

size_t zephyr_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    (void) transport;
    (void) err;

    /* The framing layer reads the byte stream in small chunks; only
     * fetch a new frame once the previous one is fully consumed. */
    zvisor_dbg_read_calls++;
    if (rx_off >= rx_len) {
        /* Wait for the doorbell; on timeout still check the slot in
         * case an edge notification was missed. */
        if (k_sem_take(&rx_sem, K_MSEC(timeout)) != 0) {
            zvisor_dbg_read_timeouts++;
            if (sys_read32((mem_addr_t)rsp_window + FRAME_OFF_STATE) != SLOT_BUSY) {
                return 0;
            }
        } else if (sys_read32((mem_addr_t)rsp_window + FRAME_OFF_STATE) != SLOT_BUSY) {
            /* Spurious wakeup */
            return 0;
        }

        uint32_t frame_len = sys_read32((mem_addr_t)rsp_window + FRAME_OFF_LEN);

        if (frame_len > 0 && frame_len <= FRAME_PAYLOAD_MAX) {
            window_read(rx_buf, rsp_window + FRAME_OFF_PAYLOAD, frame_len);
            rx_len = frame_len;
            rx_off = 0;
        } else {
            zvisor_dbg_isr_badlen++;
            printk("micro-ROS: zvisor shmem RX bad frame len %u, dropped\n", frame_len);
        }

        /* Hand the slot back to the peer (reader thread only) */
        sys_write32(SLOT_FREE, (mem_addr_t)rsp_window + FRAME_OFF_STATE);

        if (rx_off >= rx_len) {
            return 0;
        }
    }

    size_t n = MIN(rx_len - rx_off, len);

    memcpy(buf, rx_buf + rx_off, n);
    rx_off += n;
    if (rx_off >= rx_len) {
        rx_len = 0;
        rx_off = 0;
    }

    return n;
}
