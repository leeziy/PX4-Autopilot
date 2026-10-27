# SPDX-License-Identifier: BSD-3-Clause
px4_add_module(
	MODULE modules__trigger_bridge
	MAIN trigger_bridge
	SRCS TriggerBridge.cpp
	DEPENDS px4_work_queue
)
