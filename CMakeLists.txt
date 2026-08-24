# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# The micro-ROS library module is only pulled in for the real DDS backend.
if(CONFIG_DEMO_HEARTBEAT_MICROROS)
  list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/libmicroros)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pwm_dds_demo)

target_sources(app PRIVATE
	src/main.c
	src/signal_chain.c
)

if(CONFIG_DEMO_HEARTBEAT_STUB)
  target_sources(app PRIVATE src/dds_stub.c)
elseif(CONFIG_DEMO_HEARTBEAT_MICROROS)
  target_sources(app PRIVATE src/dds_microros.c)
endif()
