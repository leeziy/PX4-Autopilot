/* SPDX-License-Identifier: BSD-3-Clause */
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <pthread.h>

namespace
{
pthread_mutex_t trigger_mutex;
pthread_once_t trigger_once = PTHREAD_ONCE_INIT;

void init_mutex()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&trigger_mutex, &attr);
	pthread_mutexattr_destroy(&attr);
}

struct Entry {
	px4::WorkItem *item{nullptr};
	uint32_t target{0};
	uint16_t instance{0};
	uint64_t generation{0};
	px4::TriggerMode mode{px4::TriggerMode::Native};
	bool ready{false};
	bool running{false};
	bool closing{false};
	bool retired{false};
	pthread_t runner{};
};

// Fixed capacity includes multi-instance VehicleIMU. No allocation on a trigger.
Entry entries[64];
uint64_t next_generation{0};
}

namespace px4
{

WorkItem::~WorkItem()
{
	UnregisterManualTrigger();
	ManualTriggerLock lock;
	Deinit();

	if (_trigger_slot >= 0) {
		auto &entry = entries[_trigger_slot];
		entry.item = nullptr;
		entry.retired = true;

		if (!entry.running) { entry = Entry{}; }
	}

}

bool WorkItem::ChangeWorkQueue(const wq_config_t &config)
{
	ManualTriggerLock lock;

	// A registered target cannot run concurrently on old and new queues.
	if (_trigger_slot >= 0 && entries[_trigger_slot].running) { return false; }

	return Init(config);
}

ManualTriggerLock::ManualTriggerLock()
{
	pthread_once(&trigger_once, init_mutex);
	pthread_mutex_lock(&trigger_mutex);
}

ManualTriggerLock::~ManualTriggerLock()
{
	pthread_mutex_unlock(&trigger_mutex);
}

void WorkItem::RegisterManualTrigger(TriggerTarget target, uint16_t instance)
{
	ManualTriggerLock lock;

	if (_trigger_slot >= 0) { return; }

	for (const auto &entry : entries) {
		if (entry.item && entry.target == static_cast<uint32_t>(target) && entry.instance == instance) {
			PX4_ERR("duplicate trigger target %s:%u", ItemName(), unsigned(instance));
			return;
		}
	}

	for (unsigned i = 0; i < sizeof(entries) / sizeof(entries[0]); ++i) {
		if (!entries[i].item && !entries[i].running) {
			entries[i] = Entry{};
			entries[i].item = this;
			entries[i].target = static_cast<uint32_t>(target);
			entries[i].instance = instance;
			entries[i].generation = ++next_generation;
			_trigger_slot = i;
			return;
		}
	}

	PX4_ERR("trigger registry full: %s", ItemName());
}

bool WorkItem::AutomaticSchedulingAllowed() const
{

	if (_trigger_slot >= 0) {
		const auto &entry = entries[_trigger_slot];
		return !entry.closing && entry.mode == TriggerMode::Native;
	}

	return true;
}

void WorkItem::ScheduleNow()
{
	ManualTriggerLock lock;

	if (_wq && AutomaticSchedulingAllowed()) { _wq->Add(this); }
}

void WorkItem::ScheduleForStop()
{
	ManualTriggerLock lock;

	if (_trigger_slot >= 0) { entries[_trigger_slot].closing = true; }


	if (_wq) { _wq->Add(this); }
}

TriggerResult WorkItem::TriggerCommand(uint32_t target, uint16_t instance, bool set_mode, TriggerMode mode)
{
	ManualTriggerLock lock;

	for (auto &entry : entries) {
		if (!entry.item || entry.target != target || entry.instance != instance) { continue; }

		if (entry.closing) { return TriggerResult::Stopping; }

		if (!entry.ready || !entry.item->_wq) { return TriggerResult::NotReady; }

		if (!set_mode) {
			if (entry.mode != TriggerMode::Manual) { return TriggerResult::WrongMode; }

			entry.item->_wq->Add(entry.item);
			return TriggerResult::Accepted;
		}

		if (mode != TriggerMode::Native && mode != TriggerMode::Manual) { return TriggerResult::BadCommand; }

		if (entry.mode == mode) { return TriggerResult::Ok; }

		// Do not wait while holding lifecycle/module locks. Caller can explicitly retry.
		if (entry.running) { return TriggerResult::Busy; }

		entry.mode = mode;
		entry.item->CancelAutomaticSchedule();
		entry.item->_wq->Remove(entry.item);

		if (mode == TriggerMode::Native) {
			entry.item->ResumeAutomaticSchedule();
			// One native run re-arms module-specific backup timers and subscriptions.
			entry.item->ScheduleNow();
		}

		return TriggerResult::Ok;
	}

	return TriggerResult::UnknownTarget;
}

TriggerRunTicket WorkItem::BeginTriggerRun()
{
	TriggerRunTicket ticket;

	if (_trigger_slot >= 0) {
		auto &entry = entries[_trigger_slot];
		entry.running = true;
		entry.runner = pthread_self();
		ticket.slot = _trigger_slot;
		ticket.generation = entry.generation;
	}

	return ticket;
}

void WorkItem::EndTriggerRun(const TriggerRunTicket &ticket)
{
	ManualTriggerLock lock;

	if (ticket.slot >= 0) {
		auto &entry = entries[ticket.slot];

		if (entry.generation == ticket.generation) {
			entry.running = false;
			entry.ready = !entry.closing;

			if (entry.retired) { entry = Entry{}; }
		}
	}

}

void WorkItem::UnregisterManualTrigger()
{

	// A Run() may delete itself. Never wait for that same worker thread.
	for (;;) {
		bool wait = false;
		{
			ManualTriggerLock lock;

			if (_trigger_slot < 0) { return; }

			auto &entry = entries[_trigger_slot];
			entry.closing = true;
			entry.ready = false;

			if (_wq) { _wq->Remove(this); }

			wait = entry.running && !pthread_equal(entry.runner, pthread_self());
		}

		if (!wait) { break; }

		px4_usleep(1000);
	}

	{
		ManualTriggerLock lock;
		CancelAutomaticSchedule();
	}

	// A timer callback may already be outside the HRT queue waiting for our lock.
	// Wait without the trigger lock before derived state / hrt_call can disappear.
	WaitForAutomaticSchedule();
}

} // namespace px4
