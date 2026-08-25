# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# Backend selection: the real DDS (micro-ROS) backend is the default.
# Build the stub instead with -DHEARTBEAT_STUB=y.
if(HEARTBEAT_STUB)
  set(CONFIG_DEMO_HEARTBEAT_STUB y CACHE STRING "" FORCE)
endif()

# The micro-ROS library module is always registered so its Kconfig symbols
# exist during the configuration phase; the library is only linked when
# CONFIG_MICROROS=y (selected by CONFIG_DEMO_HEARTBEAT_MICROROS).
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/libmicroros)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pwm_dds_demo)

target_sources(app PRIVATE
	src/main.c
	src/signal_chain.c
)

if(CONFIG_DEMO_SHELL)
  target_sources(app PRIVATE src/dds_shell.c src/hb_shell.c)
endif()

if(CONFIG_DEMO_HEARTBEAT_STUB)
  target_sources(app PRIVATE src/dds_stub.c)
elseif(CONFIG_DEMO_HEARTBEAT_MICROROS)
  target_sources(app PRIVATE src/dds_microros.c)
endif()
