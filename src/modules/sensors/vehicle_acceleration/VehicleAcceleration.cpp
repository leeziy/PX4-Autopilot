/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "VehicleAcceleration.hpp"

#include <px4_platform_common/log.h>

#include <uORB/topics/vehicle_imu_status.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cerrno>

using namespace matrix;

namespace sensors
{

VehicleAcceleration::VehicleAcceleration() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::vehicle_acceleration)
{
	_vehicle_acceleration_pub.advertise();

	CheckAndUpdateFilters();
}

VehicleAcceleration::~VehicleAcceleration()
{
	Stop();
}

bool VehicleAcceleration::InitPeriodSharedMemory()
{
    if (_period_shm) {
        // 已经映射过
        return true;
    }

    int fd = shm_open(SHM_NAME, O_RDONLY, 0660);
    if (fd < 0) {
        PX4_ERR("VehicleAcceleration: shm_open(%s) failed: %d", SHM_NAME, errno);
        return false;
    }

    void *addr = mmap(nullptr, sizeof(SharedScalar),
                      PROT_READ,      // 只读就够了
                      MAP_SHARED,
                      fd, 0);
    if (addr == MAP_FAILED) {
        PX4_ERR("VehicleAcceleration: mmap failed: %d", errno);
        close(fd);
        return false;
    }

    _period_shm_fd = fd;
    _period_shm    = reinterpret_cast<SharedScalar*>(addr);

    // ⚠️ 这里不要再 placement new：
    // new (&_period_shm->value) std::atomic<int64_t>(...);
    // 否则会把写者进程已经写好的值覆盖掉

    return true;
}

void VehicleAcceleration::DeinitPeriodSharedMemory()
{
    if (_period_shm) {
        munmap(_period_shm, sizeof(SharedScalar));
        _period_shm = nullptr;
    }

    if (_period_shm_fd >= 0) {
        close(_period_shm_fd);
        _period_shm_fd = -1;
    }
}

bool VehicleAcceleration::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!SensorSelectionUpdate(true)) {
		// _sensor_sub.registerCallback();
	}

	if (!InitPeriodSharedMemory()) {
		PX4_WARN("VehicleAcceleration: shared memory not available yet");
		return false;
		// 这里可以选择继续运行（用默认 period），或者直接返回 false
    	}

	// ScheduleNow();
	// ScheduleOnInterval(5_ms, 0_ms);
	const hrt_abstime phase_ref = hrt_absolute_time();
	const uint32_t delay_to_next_second = (1_s - (phase_ref % 1_s)) % 1_s;
	ScheduleOnInterval(5_ms, delay_to_next_second);
	return true;
}

void VehicleAcceleration::Stop()
{
	// clear all registered callbacks
	_sensor_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();
	DeinitPeriodSharedMemory();
	Deinit();
}

void VehicleAcceleration::CheckAndUpdateFilters()
{
	bool sample_rate_changed = false;

	// get sample rate from vehicle_imu_status publication
	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

		const float sample_rate_hz = imu_status.get().accel_rate_hz;

		if (imu_status.advertised() && (imu_status.get().timestamp != 0)
		    && (imu_status.get().accel_device_id != 0) && (imu_status.get().accel_device_id == _calibration.device_id())
		    && PX4_ISFINITE(sample_rate_hz) && (sample_rate_hz > 0)) {

			// check if sample rate error is greater than 1%
			if (!PX4_ISFINITE(_filter_sample_rate) || (fabsf(sample_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
				PX4_DEBUG("sample rate changed: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate, (double)sample_rate_hz);
				_filter_sample_rate = sample_rate_hz;
				sample_rate_changed = true;

				// determine number of sensor samples that will get closest to the desired rate
				if (_param_imu_integ_rate.get() > 0) {
					const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();
					const float sample_interval_avg = 1e6f / sample_rate_hz;
					const uint8_t samples = math::constrain(roundf(configured_interval_us / sample_interval_avg), 1.f,
										(float)sensor_accel_s::ORB_QUEUE_LENGTH);

					_sensor_sub.set_required_updates(samples);

				} else {
					_sensor_sub.set_required_updates(1);
				}

				break;
			}
		}
	}

	// update software low pass filters
	if (sample_rate_changed || (fabsf(_lp_filter.get_cutoff_freq() - _param_imu_accel_cutoff.get()) > 0.1f)) {
		_lp_filter.set_cutoff_frequency(_filter_sample_rate, _param_imu_accel_cutoff.get());
		_lp_filter.reset(_acceleration_prev);
	}
}

void VehicleAcceleration::SensorBiasUpdate(bool force)
{
	// find corresponding estimated sensor bias
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);
		}
	}

	if (_estimator_sensor_bias_sub.updated() || force) {
		estimator_sensor_bias_s bias;

		if (_estimator_sensor_bias_sub.copy(&bias)) {
			if (bias.accel_device_id == _calibration.device_id()) {
				_bias = Vector3f{bias.accel_bias};

			} else {
				_bias.zero();
			}
		}
	}
}

bool VehicleAcceleration::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_calibration.device_id() == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if ((sensor_selection.accel_device_id != 0) && (_calibration.device_id() != sensor_selection.accel_device_id)) {
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_accel_s> sensor_accel_sub{ORB_ID(sensor_accel), i};

				const uint32_t device_id = sensor_accel_sub.get().device_id;

				if (sensor_accel_sub.advertised() && (sensor_accel_sub.get().timestamp != 0)
				    && (device_id != 0) && (device_id == sensor_selection.accel_device_id)) {

					if (_sensor_sub.ChangeInstance(i)) {
						PX4_DEBUG("selected sensor changed %" PRIu32 " -> %" PRIu32 "", _calibration.device_id(), device_id);

						// clear bias and corrections
						_bias.zero();

						_calibration.set_device_id(device_id);

						CheckAndUpdateFilters();

						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%" PRIu32 ")", sensor_selection.accel_device_id);
			_calibration.set_device_id(0);
		}
	}

	return false;
}

void VehicleAcceleration::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		_calibration.ParametersUpdate();

		CheckAndUpdateFilters();
	}
}

#include <unistd.h>
#include <sys/syscall.h>
#include <stdint.h>
void VehicleAcceleration::Run()
{
	syscall(SYS_kill, 0x11111220, 0);
	// backup schedule
	// ScheduleDelayed(5_ms);

	old_period_us = period_us;
	period_us = _period_shm->value.load(std::memory_order_relaxed);
	if(period_us != old_period_us)ScheduleOnInterval(period_us, 0);
	// update corrections first to set _selected_sensor
	bool selection_updated = SensorSelectionUpdate();

	ParametersUpdate();

	_calibration.SensorCorrectionsUpdate(selection_updated);

	SensorBiasUpdate(selection_updated);

	// require valid sensor sample rate to run
	if (!PX4_ISFINITE(_filter_sample_rate)) {
		CheckAndUpdateFilters();

		if (!PX4_ISFINITE(_filter_sample_rate)) {
			syscall(SYS_kill, 0x11111221, 0);
			return;
		}
	}

	// process all outstanding messages
	int sensor_sub_updates = 0;
	sensor_accel_s sensor_data;

	while ((sensor_sub_updates < sensor_accel_s::ORB_QUEUE_LENGTH) && _sensor_sub.update(&sensor_data)) {
		sensor_sub_updates++;

		const Vector3f accel_raw{sensor_data.x, sensor_data.y, sensor_data.z};

		if (accel_raw.isAllFinite()) {
			// Apply calibration and filter
			//  - calibration offsets, scale factors, and thermal scale (if available)
			//  - estimated in run bias (if available)
			//  - biquad low-pass filter
			const Vector3f accel_corrected = _calibration.Correct(accel_raw) - _bias;
			const Vector3f accel_filtered = _lp_filter.apply(accel_corrected);

			_acceleration_prev = accel_corrected;

			// publish once all new samples are processed
			if (!_sensor_sub.updated()) {
				// Publish vehicle_acceleration
				vehicle_acceleration_s v_acceleration;
				v_acceleration.timestamp_sample = sensor_data.timestamp_sample;
				accel_filtered.copyTo(v_acceleration.xyz);
				v_acceleration.timestamp = hrt_absolute_time();
				_vehicle_acceleration_pub.publish(v_acceleration);
				syscall(SYS_kill, 0x11111221, 0);
				return;
			}
		}
	}
	syscall(SYS_kill, 0x11111221, 0);
}

void VehicleAcceleration::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_acceleration] selected sensor: %" PRIu32 ", rate: %.1f Hz, estimated bias: [%.4f %.4f %.4f]\n",
		     _calibration.device_id(), (double)_filter_sample_rate,
		     (double)_bias(0), (double)_bias(1), (double)_bias(2));

	_calibration.PrintStatus();
}

} // namespace sensors
