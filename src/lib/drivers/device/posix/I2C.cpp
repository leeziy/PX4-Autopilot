/****************************************************************************
 *
 *   Copyright (c) 2016-2021 PX4 Development Team. All rights reserved.
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

/**
 * @file I2C.cpp
 *
 * Base class for devices attached via the I2C bus.
 *
 * @todo Bus frequency changes; currently we do nothing with the value
 *       that is supplied.  Should we just depend on the bus knowing?
 */


 #include "I2C.hpp"

 #if defined(CONFIG_I2C)

 #ifdef __PX4_LINUX
 // #define __SYLIXOS_KERNEL
 #include <SylixOS.h>

 PLW_I2C_ADAPTER i2cAdpt;
 PLW_I2C_FUNCS   i2cFun;

 #include <px4_platform_common/i2c_spi_buses.h>

 namespace device
 {

 I2C::I2C(uint8_t device_type, const char *name, const int bus, const uint16_t address, const uint32_t frequency) :
	 CDev(name, nullptr)
 {
	 // fill in _device_id fields for a I2C device
	 _device_id.devid_s.devtype = device_type;
	 _device_id.devid_s.bus_type = DeviceBusType_I2C;
	 _device_id.devid_s.bus = bus;
	 _device_id.devid_s.address = address;
 }

 I2C::I2C(const I2CSPIDriverConfig &config)
	 : I2C(config.devid_driver_index, config.module_name, config.bus, config.i2c_address, config.bus_frequency)
 {
 }

 I2C::~I2C()
 {
	 if (_fd >= 0) {
	 	_fd = -1;
	 }
 }

 int
 I2C::init()
 {
	// Open the actual I2C device
	 char dev_path[16];
	 snprintf(dev_path, sizeof(dev_path), "/bus/i2c/%i", get_device_bus());
	 i2cAdpt = API_I2cAdapterGet(dev_path);
	 if (LW_NULL == i2cAdpt) {
		 DEVICE_DEBUG("failed to get I2C Adaptor");
		 _fd = -1;
		 goto out;
	 }
	 i2cFun = i2cAdpt->I2CADAPTER_pi2cfunc;
	 if (LW_NULL == i2cFun) {
		DEVICE_DEBUG("failed to get I2C Function");
		_fd = -1;
		goto out;
	}
	// _fd = probe();
	// if (_fd != 0) {
	// 	DEVICE_DEBUG("probe failed");
	// 	_fd = -1;
	// 	goto out;
	// }
	_fd = CDev::init();
	if (_fd != 0) {
		DEVICE_DEBUG("cdev init failed");
		_fd = -1;
		goto out;
	}
	 _fd = 0;
	 // tell the world where we are
	 DEVICE_DEBUG("on I2C bus %d", get_device_bus());

 out:

	 return _fd;

 }

 int
 I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
 {

	unsigned retry_count = 0;
	 if (_fd < 0) {
		 PX4_ERR("I2C device not opened");
		 return PX4_ERROR;
	 }

	 do {
		 DEVICE_DEBUG("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);

		 unsigned msgs = 0;
		 LW_I2C_MESSAGE msgv[2];

		 if (send_len > 0) {
			 msgv[msgs].I2CMSG_usAddr = get_device_address();
			 msgv[msgs].I2CMSG_usFlag = 0;
			 msgv[msgs].I2CMSG_pucBuffer = const_cast<uint8_t *>(send);
			 msgv[msgs].I2CMSG_usLen = send_len;
			 msgs++;
		 }

		 if (recv_len > 0) {
			 msgv[msgs].I2CMSG_usAddr = get_device_address();
			 msgv[msgs].I2CMSG_usFlag = LW_I2C_M_RD;
			 msgv[msgs].I2CMSG_pucBuffer = recv;
			 msgv[msgs].I2CMSG_usLen = recv_len;
			 msgs++;
		 }

		 if (msgs == 0) {
			 return -EINVAL;
		 }

		 _fd = i2cFun->I2CFUNC_pfuncMasterXfer(i2cAdpt, msgv, msgs);

		 if (_fd < 0) {
			 DEVICE_DEBUG("I2C transfer failed");

		 } else {
			 // success
			 _fd = PX4_OK;
			 break;
		 }

	 } while (retry_count++ < _retries);

	 return _fd;
 }

 } // namespace device

 #endif // __PX4_LINUX

 #endif // CONFIG_I2C
