/* SPDX-License-Identifier: BSD-3-Clause */
#pragma once

#include <px4_platform_common/px4_work_queue/ManualTrigger.hpp>
#include <stddef.h>
#include <stdint.h>

namespace trigger_protocol
{
static constexpr size_t PacketSize = 24;
static constexpr uint32_t Magic = 0x50585452; // "PXTR"
static constexpr uint8_t Version = 1;
static constexpr uint8_t SetMode = 1;
static constexpr uint8_t Trigger = 2;
static constexpr uint8_t ReplyBit = 0x80;

inline uint16_t read16(const uint8_t *p)
{
	return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

inline uint32_t read32(const uint8_t *p)
{
	return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

inline bool valid_header(const uint8_t *p)
{
	return read32(p) == Magic && p[4] == Version && p[7] == 0 && read16(p + 22) == 0;
}

inline void reply(uint8_t *packet, px4::TriggerResult result)
{
	packet[4] = Version;
	packet[5] |= ReplyBit;
	packet[6] = static_cast<uint8_t>(result);
	packet[7] = 0;
	packet[22] = 0;
	packet[23] = 0;
}
} // namespace trigger_protocol
