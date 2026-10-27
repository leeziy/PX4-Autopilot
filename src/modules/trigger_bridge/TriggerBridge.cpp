/* SPDX-License-Identifier: BSD-3-Clause */
#include "Protocol.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

class TriggerBridge : public ModuleBase<TriggerBridge>
{
public:
	~TriggerBridge() override { if (_socket >= 0) { close(_socket); } }
	static int task_spawn(int argc, char *argv[]);
	static TriggerBridge *instantiate(int argc, char *argv[]);
	static int custom_command(int, char *[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);
	void run() override;
	int print_status() override
	{
		PX4_INFO("UDP %s:%u (target modes persist when bridge stops)", _address, unsigned(_port));
		return 0;
	}

private:
	int _socket{-1};
	uint16_t _port{19000};
	char _address[INET_ADDRSTRLEN] {"127.0.0.1"};
};

int TriggerBridge::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("trigger_bridge", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 3000,
				      (px4_main_t)&run_trampoline, (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return wait_until_running();
}

TriggerBridge *TriggerBridge::instantiate(int argc, char *argv[])
{
	TriggerBridge *bridge = new TriggerBridge();

	if (!bridge) { return nullptr; }

	int index = 1;
	int option;
	const char *argument = nullptr;
	bool valid = true;

	while ((option = px4_getopt(argc, argv, "a:p:", &index, &argument)) != EOF) {
		switch (option) {
		case 'a':
			if (strlen(argument) >= sizeof(bridge->_address)) { valid = false; break; }

			strcpy(bridge->_address, argument);
			break;

		case 'p': {
				char *end = nullptr;
				long port = strtol(argument, &end, 10);

				if (!end || end == argument || *end != '\0' || port < 1 || port > 65535) { valid = false; break; }

				bridge->_port = static_cast<uint16_t>(port);
				break;
			}

		default:
			valid = false;
			break;
		}
	}

	sockaddr_in local{};
	local.sin_family = AF_INET;
	local.sin_port = htons(bridge->_port);

	if (!valid || inet_pton(AF_INET, bridge->_address, &local.sin_addr) != 1) {
		print_usage("invalid address or port");
		delete bridge;
		return nullptr;
	}

	bridge->_socket = socket(AF_INET, SOCK_DGRAM, 0);

	if (bridge->_socket < 0 || bind(bridge->_socket, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0
	    || fcntl(bridge->_socket, F_SETFL, O_NONBLOCK) < 0) {
		PX4_ERR("socket/bind failed (%i)", errno);
		delete bridge;
		return nullptr;
	}

	return bridge;
}

void TriggerBridge::run()
{
	using namespace trigger_protocol;
	PX4_INFO("UDP %s:%u", _address, unsigned(_port));
	pollfd descriptor{};
	descriptor.fd = _socket;
	descriptor.events = POLLIN;

	while (!should_exit()) {
		int ready = poll(&descriptor, 1, 100);

		if (ready < 0) {
			if (errno == EINTR) { continue; }

			PX4_ERR("poll failed (%i)", errno);
			break;
		}

		if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) { break; }

		if (ready == 0 || !(descriptor.revents & POLLIN)) { continue; }

		// One extra byte makes all oversized datagrams distinguishable from valid packets.
		uint8_t packet[PacketSize + 1] {};
		sockaddr_in peer{};
		socklen_t peer_length = sizeof(peer);
		ssize_t received = recvfrom(_socket, packet, sizeof(packet), 0,
					    reinterpret_cast<sockaddr *>(&peer), &peer_length);

		if (received != static_cast<ssize_t>(PacketSize) || read32(packet) != Magic) { continue; }

		// Never answer a reply packet; avoids response loops between bridges.
		if (packet[5] & ReplyBit) { continue; }

		px4::TriggerResult result = px4::TriggerResult::BadPacket;

		if (valid_header(packet)) {
			const uint32_t target = read32(packet + 16);
			const uint16_t instance = read16(packet + 20);

			if (packet[5] == SetMode && packet[6] <= 1) {
				result = px4::WorkItem::TriggerCommand(target, instance, true, static_cast<px4::TriggerMode>(packet[6]));

			} else if (packet[5] == Trigger && packet[6] == 0) {
				result = px4::WorkItem::TriggerCommand(target, instance, false, px4::TriggerMode::Native);

			} else {
				result = px4::TriggerResult::BadCommand;
			}
		}

		reply(packet, result);

		if (sendto(_socket, packet, PacketSize, 0, reinterpret_cast<sockaddr *>(&peer), peer_length) < 0) {
			PX4_WARN("reply failed (%i)", errno);
		}
	}
}

int TriggerBridge::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_USAGE_NAME("trigger_bridge", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('a', "127.0.0.1", "IPv4", "Bind IPv4 address", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 19000, 1, 65535, "UDP port", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int trigger_bridge_main(int argc, char *argv[])
{
	return TriggerBridge::main(argc, argv);
}
