/* SPDX-License-Identifier: BSD-3-Clause */
#pragma once

#include <px4_platform_common/px4_config.h>
#include <stdint.h>

namespace px4
{

// Stable IDs match existing start markers; IDs of excluded modules stay unused.
enum class TriggerTarget : uint32_t {
	VehicleIMU = 0x11111240,
	VehicleAngularVelocity = 0x11111250,
	VehicleAcceleration = 0x11111260,
	VehicleMagnetometer = 0x11111270,
	VehicleAirData = 0x11111280,
	Sensors = 0x11111290,
	EKF2 = 0x11111300,
	HoverThrustEstimator = 0x11111310,
	RateControl = 0x11111330,
	AttitudeControl = 0x11111340,
	PositionControl = 0x11111350,
	ControlAllocator = 0x11111370,
	PWMSim = 0x11111380,
};

enum class TriggerMode : uint8_t { Native = 0, Manual = 1 };
enum class TriggerResult : uint8_t {
	Ok = 0,
	Accepted = 1, // ScheduleNow-equivalent request; may be coalesced by the original queue.
	UnknownTarget = 2,
	WrongMode = 3,
	Busy = 4,
	NotReady = 5,
	Stopping = 6,
	BadPacket = 7,
	BadCommand = 8,
};

// Optional POSIX feature. All participants take this lock BEFORE a WorkQueue lock.
// Recursive because lifecycle and timer helpers reuse normal WorkItem APIs.
class ManualTriggerLock
{
public:
	ManualTriggerLock();
	~ManualTriggerLock();
	ManualTriggerLock(const ManualTriggerLock &) = delete;
	ManualTriggerLock &operator=(const ManualTriggerLock &) = delete;
};

struct TriggerRunTicket {
	int slot{-1};
	uint64_t generation{0};
};

} // namespace px4
