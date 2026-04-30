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

#if defined(__PX4_LINUX)

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_config.h>

// #define __SYLIXOS_KERNEL

#include "SylixOS.h"

#define LW_SPI_CTL_BAUDRATE    4
#define LW_SPI_CTL_CS          3
#define SPI_DEVICE_CPOL_CPHA   5
#define LW_SPI_M_CPOL_0        0x0000   /*  CPOL 配置                   */
#define LW_SPI_M_CPOL_1        0x0001
#define LW_SPI_M_CPHA_0        0x0000   /*  CPHA 配置                   */
#define LW_SPI_M_CPHA_1        0x0002

namespace device
{

static int spi_mode_to_sylixos_mode(spi_mode_e mode)
{
	switch (mode) {
	case SPIDEV_MODE0:
		return LW_SPI_M_CPOL_0 | LW_SPI_M_CPHA_0;

	case SPIDEV_MODE1:
		return LW_SPI_M_CPOL_0 | LW_SPI_M_CPHA_1;

	case SPIDEV_MODE2:
		return LW_SPI_M_CPOL_1 | LW_SPI_M_CPHA_0;

	case SPIDEV_MODE3:
	default:
		return LW_SPI_M_CPOL_1 | LW_SPI_M_CPHA_1;
	}
}

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
		::close(_fd);
		_fd = -1;
	}
}

int
SPI::init()
{
	// Open the actual SPI device
	char dev_path[16];
	snprintf(dev_path, sizeof(dev_path), "/dev/spidev%i", get_device_bus());
	DEVICE_DEBUG("%s", dev_path);
	_fd = ::open(dev_path, O_RDWR);

	if (_fd < 0) {
		PX4_ERR("could not open %s", dev_path);
		return PX4_ERROR;
	}

	/* call the probe function to check whether the device is present */
	int ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("probe failed");
		return ret;
	}

	/* do base class init, which will create the device node, etc. */
	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("cdev init failed");
		return ret;
	}

	/* tell the world where we are */
	DEVICE_DEBUG("on SPI bus %d (%u KHz)", get_device_bus(), _frequency / 1000);

	return PX4_OK;
}

int
SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	// set write mode of SPI
	// int result = ::ioctl(_fd, SPI_IOC_WR_MODE, &_mode);

	// if (result == -1) {
	// 	PX4_ERR("can’t set spi mode");
	// 	return PX4_ERROR;
	// }

	// spi_ioc_transfer spi_transfer{};

	// spi_transfer.tx_buf = (uint64_t)send;
	// spi_transfer.rx_buf = (uint64_t)recv;
	// spi_transfer.len = len;
	// spi_transfer.speed_hz = _frequency;
	// spi_transfer.bits_per_word = 8;

	// result = ::ioctl(_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

	if (::ioctl(_fd, LW_SPI_CTL_BAUDRATE, _frequency) < 0) {
		PX4_ERR("set spi baudrate failed");
		return PX4_ERROR;
	}

	if (::ioctl(_fd, LW_SPI_CTL_CS, 0) < 0) {
		PX4_ERR("set spi cs failed");
		return PX4_ERROR;
	}

	if (::ioctl(_fd, SPI_DEVICE_CPOL_CPHA, spi_mode_to_sylixos_mode(_mode)) < 0) {
		PX4_ERR("set spi mode failed");
		return PX4_ERROR;
	}

	int result_w = ::write(_fd, send, len);
	int result_r = ::read(_fd, recv, len);

	if ((result_w < 0)||(result_r < 0)) {
		PX4_ERR("spi transfer failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
SPI::transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	// set write mode of SPI
	// int result = ::ioctl(_fd, SPI_IOC_WR_MODE, &_mode);

	// if (result == -1) {
	// 	PX4_ERR("can’t set spi mode");
	// 	return PX4_ERROR;
	// }

	// int bits = 16;
	// result = ::ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);

	// if (result == -1) {
	// 	PX4_ERR("can’t set 16 bit spi mode");
	// 	return PX4_ERROR;
	// }

	// spi_ioc_transfer spi_transfer[1] {};

	// spi_transfer[0].tx_buf = (uint64_t)send;
	// spi_transfer[0].rx_buf = (uint64_t)recv;
	// spi_transfer[0].len = len * 2;
	// spi_transfer[0].speed_hz = _frequency;
	//spi_transfer[0].bits_per_word = 8;
	//spi_transfer[0].delay_usecs = 10;
	// spi_transfer[0].cs_change = true;

	// result = ::ioctl(_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

	// if (result != (int)(len * 2)) {
	// 	PX4_ERR("write failed. Reported %d bytes written (%s)", result, strerror(errno));
	// 	return PX4_ERROR;
	// }

	// return PX4_OK;

	return transfer(reinterpret_cast<uint8_t *>(send), reinterpret_cast<uint8_t *>(recv),
			len * sizeof(uint16_t));
}

} // namespace device

#endif // __PX4_LINUX
#endif // CONFIG_SPI
