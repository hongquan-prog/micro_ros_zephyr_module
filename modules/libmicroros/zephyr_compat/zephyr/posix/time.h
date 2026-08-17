/*
 * Compatibility shim for micro-ROS sources targeting older Zephyr.
 *
 * Zephyr >= 4.2 removed <zephyr/posix/time.h>: the C library now owns
 * <time.h> and provides the POSIX time API (clock_gettime, struct
 * itimerspec, ...) when _POSIX_C_SOURCE is set, which CONFIG_POSIX_API
 * does globally. Forward old-style includes straight to the libc header.
 * Including <zephyr/posix/posix_time.h> here would clash with the newlib
 * <time.h> pulled in via <unistd.h> (duplicate struct itimerspec).
 */

#ifndef MICROROS_ZEPHYR_COMPAT_POSIX_TIME_H_
#define MICROROS_ZEPHYR_COMPAT_POSIX_TIME_H_

#include <time.h>

#endif /* MICROROS_ZEPHYR_COMPAT_POSIX_TIME_H_ */
