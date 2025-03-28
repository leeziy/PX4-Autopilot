/****************************************************************************
 *
 *   Copyright (C) 2019-2021 PX4 Development Team. All rights reserved.
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
 * @file SPI.cpp
 *
 * Base class for devices connected via SPI.
 *
 */

#include "SPI.hpp"

#if defined(CONFIG_SPI)

#ifdef __PX4_LINUX
#include <SylixOS.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>

PLW_SPI_ADAPTER spiAdpt;
PLW_SPI_FUNCS   spiFun;

#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_config.h>

namespace device
{

SPI::SPI(uint8_t device_type, const char *name, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency) :
	CDev(name, nullptr),
	_device(device),
	_mode(mode),
	_frequency(frequency)
{
	_device_id.devid_s.devtype = device_type;
	// fill in _device_id fields for a SPI device
	_device_id.devid_s.bus_type = DeviceBusType_SPI;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = (uint8_t)device;
}

SPI::SPI(const I2CSPIDriverConfig &config)
	: SPI(config.devid_driver_index, config.module_name, config.bus, config.spi_devid, config.spi_mode,
	      config.bus_frequency)
{
}

SPI::~SPI()
{
	if (_fd >= 0) {
		_fd = -1;
	}
}

int
SPI::init()
{
	// Open the actual SPI device
	 char dev_path[16] {};
	 snprintf(dev_path, sizeof(dev_path), "/bus/spi/%i", get_device_bus());
	 spiAdpt = API_SpiAdapterGet(dev_path);
	 if (LW_NULL == spiAdpt) {
		 DEVICE_DEBUG("failed to get SPI Adaptor");
		 _fd = -1;
		 goto out;
	 }
	 spiFun = spiAdpt->SPIADAPTER_pspifunc;
	 if (LW_NULL == spiFun) {
		DEVICE_DEBUG("failed to get SPI Function");
		_fd = -1;
		goto out;
	}
	_fd = probe();
	if (_fd != 0) {
		DEVICE_DEBUG("probe failed");
		_fd = -1;
		goto out;
	}
	_fd = CDev::init();
	if (_fd != 0) {
		DEVICE_DEBUG("cdev init failed");
		_fd = -1;
		goto out;
	}
	 _fd = 0;
	 // tell the world where we are
	 DEVICE_DEBUG("on SPI bus %d", get_device_bus());

 out:

	 return _fd;

 }

int
SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	LW_SPI_MESSAGE  spiMsg;

	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	spiMsg.SPIMSG_pucRdBuffer   = recv;
	spiMsg.SPIMSG_pucWrBuffer   = send;
	spiMsg.SPIMSG_uiLen         = len;
	spiMsg.SPIMSG_pfuncComplete = LW_NULL;
	spiMsg.SPIMSG_usFlag         = LW_SPI_M_CPOL_1 | LW_SPI_M_CPHA_1|LW_SPI_M_CPOL_EN|LW_SPI_M_CPHA_EN;

	spiFun->SPIFUNC_pfuncMasterCtl(spiAdpt, LW_SPI_CTL_BAUDRATE, _frequency);
	_fd = spiFun->SPIFUNC_pfuncMasterXfer(spiAdpt, &spiMsg, 1);

	if (_fd < 0) {
		DEVICE_DEBUG("SPI transfer failed");
	}else{
		_fd = 0;
	}

	return _fd;
}

int
SPI::transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	LW_SPI_MESSAGE  spiMsg;

	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	spiMsg.SPIMSG_pucRdBuffer   = (UINT8*)&recv;
	spiMsg.SPIMSG_pucWrBuffer   = (UINT8*)&send;
	spiMsg.SPIMSG_uiLen         = len;
	spiMsg.SPIMSG_pfuncComplete = LW_NULL;
	spiMsg.SPIMSG_usFlag         = LW_SPI_M_CPOL_1 | LW_SPI_M_CPHA_1|LW_SPI_M_CPOL_EN|LW_SPI_M_CPHA_EN;

	spiFun->SPIFUNC_pfuncMasterCtl(spiAdpt, LW_SPI_CTL_BAUDRATE, _frequency);
	_fd = spiFun->SPIFUNC_pfuncMasterXfer(spiAdpt, &spiMsg, 1);

	if (_fd < 0) {
		DEVICE_DEBUG("SPI transfer failed");
	}else{
		_fd = 0;
	}

	return _fd;
}

} // namespace device

#endif // __PX4_LINUX
#endif // CONFIG_SPI
