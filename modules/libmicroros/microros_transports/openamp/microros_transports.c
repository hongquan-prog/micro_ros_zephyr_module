#include <uxr/client/transport.h>

#include <microros_transports.h>
#include <version.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/ipc/rpmsg_service.h>

#if defined(CONFIG_MMU)
#include <zephyr/kernel/mm.h>
/*
 * Low-level MMU helper used to identity-map the RPMsg shared memory
 * window. Declared in the internal kernel_arch_interface.h, which is
 * not on the module include path.
 */
extern void arch_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags);
#endif

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define RING_BUF_SIZE 2048

#define RPMSG_ENDPOINT_NAME "microros"

/*
 * Shared memory layout is fixed by the Zephyr RPMsg service backend:
 * the virtio status page sits at the SHM base (0x400 bytes) and the
 * vring/buffer pool follows.
 */
#define SHM_NODE		    DT_CHOSEN(zephyr_ipc_shm)
#define SHM_BASE_ADDR		DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE_BYTES		DT_REG_SIZE(SHM_NODE)

#define RPMSG_SEND_TIMEOUT_MS		100

static uint8_t rpmsg_in_buffer[RING_BUF_SIZE];
static struct ring_buf in_ringbuf;

/* One semaphore count per received RPMsg message */
K_SEM_DEFINE(rx_sem, 0, RING_BUF_SIZE);

static int microros_endpoint_id = -1;

// --- micro-ROS OpenAMP/RPMsg Transport for Zephyr ---

static int rpmsg_rx_cb(struct rpmsg_endpoint *ept, void *data, size_t len,
		       uint32_t src, void *priv)
{
	ARG_UNUSED(ept);
	ARG_UNUSED(src);
	ARG_UNUSED(priv);

	size_t space = ring_buf_space_get(&in_ringbuf);

	if (space < len) {
		printk("micro-ROS: RX ring buffer overrun, dropping %zu bytes\n",
		       len - space);
		len = space;
	}

	if (len > 0) {
		ring_buf_put(&in_ringbuf, data, len);
		k_sem_give(&rx_sem);
	}

	return RPMSG_SUCCESS;
}

#if defined(CONFIG_MMU)
/*
 * The RPMsg backend dereferences the shared memory through its physical
 * address, so identity-map the SHM window. This must happen before the
 * rpmsg_service backend's PRE_KERNEL_1 init_status_flag() clears the
 * virtio status byte at the SHM base, hence PRE_KERNEL_1 prio 0.
 */
static int microros_openamp_map(void)
{
	arch_mem_map((void *)SHM_BASE_ADDR, SHM_BASE_ADDR, SHM_SIZE_BYTES,
		     K_MEM_PERM_RW | K_MEM_CACHE_WB);
	return 0;
}

SYS_INIT(microros_openamp_map, PRE_KERNEL_1, 0);
#endif

/*
 * Register the RPMsg endpoint before the RPMsg service itself initializes
 * (POST_KERNEL, CONFIG_RPMSG_SERVICE_INIT_PRIORITY = 48).
 */
static int microros_openamp_init(void)
{
	ring_buf_init(&in_ringbuf, sizeof(rpmsg_in_buffer), rpmsg_in_buffer);

	microros_endpoint_id =
		rpmsg_service_register_endpoint(RPMSG_ENDPOINT_NAME, rpmsg_rx_cb);
	if (microros_endpoint_id < 0) {
		printk("micro-ROS: failed to register RPMsg endpoint '%s' (%d)\n",
		       RPMSG_ENDPOINT_NAME, microros_endpoint_id);
		return 0;
	}

	printk("micro-ROS: RPMsg endpoint '%s' registered (id %d)\n",
	       RPMSG_ENDPOINT_NAME, microros_endpoint_id);

	return 0;
}

SYS_INIT(microros_openamp_init, POST_KERNEL, CONFIG_RPMSG_SERVICE_EP_REG_PRIORITY);

bool zephyr_transport_open(struct uxrCustomTransport * transport){
    (void) transport;

    if (microros_endpoint_id < 0) {
        printk("micro-ROS: RPMsg endpoint not registered\n");
        return false;
    }

    int64_t deadline = k_uptime_get() + CONFIG_MICROROS_OPENAMP_BIND_TIMEOUT_MS;

    while (!rpmsg_service_endpoint_is_bound(microros_endpoint_id)) {
        if (CONFIG_MICROROS_OPENAMP_BIND_TIMEOUT_MS > 0 &&
            k_uptime_get() >= deadline) {
            printk("micro-ROS: timeout waiting for RPMsg endpoint binding\n");
            return false;
        }
        k_msleep(10);
    }

    printk("micro-ROS: RPMsg endpoint bound, transport open\n");

    return true;
}

bool zephyr_transport_close(struct uxrCustomTransport * transport){
    (void) transport;
    // TODO: close OpenAMP transport here
    return true;
}

size_t zephyr_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    (void) transport;

    int64_t deadline = k_uptime_get() + RPMSG_SEND_TIMEOUT_MS;
    int ret;

    do {
        ret = rpmsg_service_send(microros_endpoint_id, buf, len);
        if (ret >= 0) {
            return (size_t)ret;
        }
        if (ret != -ENOMEM) {
            break;
        }
        // RPMsg buffer pool temporarily exhausted, retry shortly
        k_usleep(200);
    } while (k_uptime_get() < deadline);

    printk("micro-ROS: rpmsg_service_send failed (%d)\n", ret);
    if (err) {
        *err = 1;
    }

    return 0;
}

size_t zephyr_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    (void) transport;
    (void) err;

    if (ring_buf_is_empty(&in_ringbuf)) {
        if (k_sem_take(&rx_sem, K_MSEC(timeout)) != 0) {
            return 0;
        }
    }

    return ring_buf_get(&in_ringbuf, buf, len);
}
