#!/usr/bin/env python3
"""PX4 UDP client and fixed-order periodic scheduler with a 1000 Hz host tick."""

import argparse
from collections import deque
from dataclasses import dataclass, field
from fractions import Fraction
import json
import math
from pathlib import Path
import secrets
import signal
import socket
import struct
import sys
import time

PACKET = struct.Struct("!IBBBBQIHH")
MAGIC = 0x50585452
VERSION = 1
SET_MODE = 1
TRIGGER = 2
TARGETS = {
    "vehicle_imu": 0x11111240,
    "vehicle_angular_velocity": 0x11111250,
    "vehicle_acceleration": 0x11111260,
    "vehicle_magnetometer": 0x11111270,
    "vehicle_air_data": 0x11111280,
    "sensors": 0x11111290,
    "ekf2": 0x11111300,
    "mc_hover_thrust_estimator": 0x11111310,
    "mc_rate_control": 0x11111330,
    "mc_att_control": 0x11111340,
    "mc_pos_control": 0x11111350,
    "control_allocator": 0x11111370,
    "pwm_out_sim": 0x11111380,
}
RESULTS = {
    0: "OK", 1: "ACCEPTED", 2: "UNKNOWN_TARGET", 3: "WRONG_MODE",
    4: "BUSY", 5: "NOT_READY", 6: "STOPPING", 7: "BAD_PACKET", 8: "BAD_COMMAND",
}
TICK_HZ = 1000
TICK_NS = 1_000_000


@dataclass
class ScheduledTask:
    name: str
    target: int
    instance: int
    rate: Fraction
    phase: int = 0
    basis: str = ""
    next_release: int = 0
    counters: dict = field(default_factory=lambda: {
        "sent": 0, "accepted": 0, "rejected": 0, "timed_out": 0, "skipped_releases": 0, "send_failed": 0,
    })

    def due(self, tick):
        """Release at ceil(k * 1000 / rate) + phase; skip stale releases, never burst."""
        if tick < self.phase:
            return False
        latest = (tick - self.phase) * self.rate.numerator // (TICK_HZ * self.rate.denominator)
        if latest < self.next_release:
            return False
        self.counters["skipped_releases"] += latest - self.next_release
        self.next_release = latest + 1
        return True


def parse_rate(value):
    try:
        rate = Fraction(str(value))
    except (ValueError, ZeroDivisionError) as exc:
        raise ValueError(f"invalid rate: {value}") from exc
    if not 0 < rate <= TICK_HZ:
        raise ValueError(f"rate must be in (0, {TICK_HZ}] Hz: {value}")
    return rate


def load_schedule(args):
    data = json.loads(Path(args.config).read_text())
    if data.get("tick_hz") != TICK_HZ:
        raise ValueError("schedule must use tick_hz=1000")
    if args.profile not in data["profiles"]:
        raise ValueError(f"profile not found: {args.profile}")
    overrides = {}
    for item in args.rate:
        name, separator, value = item.partition("=")
        if not separator or name not in TARGETS:
            raise ValueError(f"invalid --rate {item}; use TARGET=HZ")
        overrides[name] = parse_rate(value)
    tasks = []
    seen = set()
    names = set()
    for row in data["profiles"][args.profile]["tasks"]:
        name = row["target"]
        if name not in TARGETS:
            raise ValueError(f"unknown target in schedule: {name}")
        names.add(name)
        if name in args.exclude:
            continue
        instance = row.get("instance", 0)
        phase = row.get("phase_ticks", 0)
        if type(instance) is not int or not 0 <= instance <= 65535:
            raise ValueError(f"invalid instance for {name}")
        if type(phase) is not int or phase < 0:
            raise ValueError(f"invalid phase_ticks for {name}")
        if name == "ekf2" and instance != 0:
            raise ValueError("EKF2 manual target uses single instance 0")
        key = (name, instance)
        if key in seen:
            raise ValueError(f"duplicate scheduled target: {key}")
        seen.add(key)
        rate = overrides.get(name, parse_rate(row["rate_hz"]))
        tasks.append(ScheduledTask(name, TARGETS[name], instance, rate, phase, row.get("basis", "custom")))
    if not tasks:
        raise ValueError("schedule has no enabled targets")
    if set(overrides) - names:
        raise ValueError(f"--rate targets not in profile: {sorted(set(overrides) - names)}")
    return tasks, data


class SchedulerTransport:
    """One socket, pipelined TRIGGER replies, no packet retransmission or ID deduplication."""

    def __init__(self, sock, timeout):
        self.sock = sock
        self.timeout_ns = int(timeout * 1e9)
        self.request_id = secrets.randbits(64)
        self.pending = {}
        self.expirations = deque()

    def next_id(self):
        value = self.request_id
        self.request_id = (value + 1) & 0xFFFFFFFFFFFFFFFF
        return value

    def set_mode(self, task, mode, wait_seconds):
        """New SET_MODE requests only after explicit BUSY/NOT_READY, never after timeout."""
        deadline = time.monotonic() + wait_seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"mode transition not ready: {task.name}:{task.instance}")
            self.sock.settimeout(min(self.timeout_ns / 1e9, remaining))
            result = exchange(self.sock, SET_MODE, mode, self.next_id(), task.target, task.instance,
                              min(self.timeout_ns / 1e9, remaining),
                              on_unmatched=lambda packet: self.consume_reply(packet, strict=False))
            if result == 0:
                return
            if result not in (4, 5):
                raise RuntimeError(f"SET_MODE {task.name}:{task.instance}: {RESULTS.get(result, result)}")
            time.sleep(min(0.002, max(0, deadline - time.monotonic())))

    def send_trigger(self, task):
        if len(self.pending) >= 65536:
            raise RuntimeError("too many outstanding replies; scheduler stopped")
        request_id = self.next_id()
        packet = PACKET.pack(MAGIC, VERSION, TRIGGER, 0, 0, request_id, task.target, task.instance, 0)
        try:
            self.sock.send(packet)
        except BlockingIOError:
            task.counters["send_failed"] += 1
            return
        sent_at = time.monotonic_ns()
        task.counters["sent"] += 1
        self.pending[request_id] = task
        self.expirations.append((request_id, sent_at + self.timeout_ns))

    def receive(self):
        # Bound per-tick receive work; do not wait for any particular trigger ACK.
        for _ in range(256):
            try:
                response = self.sock.recv(PACKET.size + 1)
            except BlockingIOError:
                break
            self.consume_reply(response)

    def consume_reply(self, response, strict=True):
        if len(response) != PACKET.size:
            return
        magic, version, opcode, result, reserved, request_id, target, instance, tail = PACKET.unpack(response)
        if (magic, version, opcode, reserved, tail) != (MAGIC, VERSION, TRIGGER | 0x80, 0, 0):
            return
        task = self.pending.get(request_id)
        if task is None or (task.target, task.instance) != (target, instance):
            return
        del self.pending[request_id]
        if result == 1:
            task.counters["accepted"] += 1
        else:
            task.counters["rejected"] += 1
            if strict:
                raise RuntimeError(f"TRIGGER {task.name}:{task.instance}: {RESULTS.get(result, result)}")

    def expire(self, now_ns):
        while self.expirations:
            request_id, deadline = self.expirations[0]
            if request_id not in self.pending:
                self.expirations.popleft()
            elif deadline <= now_ns:
                task = self.pending.pop(request_id)
                task.counters["timed_out"] += 1
                self.expirations.popleft()
            else:
                break


def print_schedule(tasks, data, profile):
    print(f"profile={profile} tick={TICK_HZ}Hz reference={data.get('reference_branch', 'custom')} "
          f"commit={data.get('reference_commit', 'custom')}", flush=True)
    print("Rates are source-derived initial trigger settings, not measured native Run frequencies.")
    for order, task in enumerate(tasks):
        print(f"{order + 1:2d} {task.name}:{task.instance} {float(task.rate):9.3f} Hz "
              f"T={1000 / float(task.rate):9.3f} ms phase={task.phase} ticks")
    print("Same-tick sends follow this order; WorkQueue execution/completion order is not guaranteed.", flush=True)


def run_scheduler(args):
    tasks, data = load_schedule(args)
    print_schedule(tasks, data, args.profile)
    if args.dry_run:
        return 0
    attempted = []
    restore_errors = []
    exit_code = 0
    dispatch_started = None
    summary = {"profile": args.profile, "tick_hz": TICK_HZ, "skipped_ticks": 0,
               "max_wakeup_lateness_ns": 0, "executed_ticks": 0,
               "reference_branch": data.get("reference_branch"),
               "reference_commit": data.get("reference_commit"), "host": args.host, "port": args.port}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((args.host, args.port))
        transport = SchedulerTransport(sock, args.timeout)
        try:
            for task in tasks:
                # Track even a timeout: the mode command may have succeeded but its ACK was lost.
                attempted.append(task)
                transport.set_mode(task, 1, args.setup_timeout)
                print(f"MANUAL {task.name}:{task.instance}", flush=True)
            sock.setblocking(False)
            epoch = time.monotonic_ns() + TICK_NS
            dispatch_started = epoch
            end_tick = None if args.duration == 0 else math.ceil(args.duration * TICK_HZ)
            tick = 0
            report_at = epoch + int(args.report_interval * 1e9)
            while end_tick is None or tick < end_tick:
                deadline = epoch + tick * TICK_NS
                remaining = deadline - time.monotonic_ns()
                while remaining > 0:
                    time.sleep(remaining / 1e9)
                    remaining = deadline - time.monotonic_ns()
                now = time.monotonic_ns()
                summary["max_wakeup_lateness_ns"] = max(summary["max_wakeup_lateness_ns"], now - deadline)
                actual_tick = max(tick, (now - epoch) // TICK_NS)
                summary["skipped_ticks"] += actual_tick - tick
                if end_tick is not None and actual_tick >= end_tick:
                    break
                transport.receive()
                transport.expire(now)
                for task in tasks:
                    if task.due(actual_tick):
                        transport.send_trigger(task)
                transport.receive()
                transport.expire(time.monotonic_ns())
                summary["executed_ticks"] += 1
                tick = actual_tick + 1
                if time.monotonic_ns() >= report_at:
                    print(f"tick={actual_tick} sent={sum(t.counters['sent'] for t in tasks)} "
                          f"accepted={sum(t.counters['accepted'] for t in tasks)} "
                          f"pending={len(transport.pending)} skipped_ticks={summary['skipped_ticks']}", flush=True)
                    report_at = time.monotonic_ns() + int(args.report_interval * 1e9)
            transport.receive()
        except KeyboardInterrupt:
            print("Stopping scheduler; restoring NATIVE.", flush=True)
        except (OSError, TimeoutError, RuntimeError) as exc:
            print(str(exc), file=sys.stderr)
            exit_code = 1
        finally:
            summary["dispatch_elapsed_s"] = max(0, (time.monotonic_ns() - dispatch_started) / 1e9) if dispatch_started else 0
            # A normal exit, Ctrl-C or reported error restores every attempted target.
            # Reverse order releases downstream tasks before upstream publishers resume.
            for task in reversed(attempted):
                try:
                    transport.set_mode(task, 0, args.setup_timeout)
                except (OSError, TimeoutError, RuntimeError) as exc:
                    restore_errors.append(f"{task.name}:{task.instance}: {exc}")
                    print(f"NATIVE restore failed: {restore_errors[-1]}", file=sys.stderr)
            transport.expire(time.monotonic_ns())
            summary["outstanding_replies"] = len(transport.pending)
            summary["restore_errors"] = restore_errors
            summary["tasks"] = [dict(target=t.name, instance=t.instance, rate_hz=str(t.rate),
                                     phase_ticks=t.phase, **t.counters) for t in tasks]
            print(json.dumps(summary, indent=2), flush=True)
            if args.summary:
                Path(args.summary).write_text(json.dumps(summary, indent=2) + "\n")
    return 1 if restore_errors else exit_code


def target_id(value):
    if value in TARGETS:
        return TARGETS[value]
    try:
        number = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("use a documented target name or numeric ID") from exc
    if not 0 <= number <= 0xFFFFFFFF:
        raise argparse.ArgumentTypeError("target ID must fit uint32")
    return number


def exchange(sock, opcode, mode, request_id, target, instance, timeout, on_unmatched=None):
    request = PACKET.pack(MAGIC, VERSION, opcode, mode, 0, request_id, target, instance, 0)
    deadline = time.monotonic() + timeout
    sock.send(request)
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("no matching reply; execution outcome unknown; command NOT retransmitted")
        sock.settimeout(remaining)
        try:
            response = sock.recv(PACKET.size + 1)
        except socket.timeout as exc:
            raise TimeoutError("no reply; execution outcome unknown; command NOT retransmitted") from exc
        if len(response) != PACKET.size:
            continue
        magic, version, reply_op, result, reserved, reply_id, reply_target, reply_instance, tail = PACKET.unpack(response)
        if (magic, version, reply_op, reserved, reply_id, reply_target, reply_instance, tail) != (
            MAGIC, VERSION, opcode | 0x80, 0, request_id, target, instance, 0
        ):
            if on_unmatched is not None:
                on_unmatched(response)
            continue
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=19000)
    parser.add_argument("--timeout", type=float, default=1.0)
    sub = parser.add_subparsers(dest="command", required=True)
    mode_parser = sub.add_parser("mode")
    mode_parser.add_argument("target", type=target_id)
    mode_parser.add_argument("mode", choices=("native", "manual"))
    trigger_parser = sub.add_parser("trigger")
    trigger_parser.add_argument("target", type=target_id)
    trigger_parser.add_argument("--hz", type=float, default=0)
    trigger_parser.add_argument("--count", type=int, default=1)
    schedule_parser = sub.add_parser("schedule", help="automatic fixed-order scheduler, 1000 Hz tick")
    schedule_parser.add_argument("--profile", choices=("real", "hitl"), required=True)
    schedule_parser.add_argument("--config", default=str(Path(__file__).with_name("schedules.json")))
    schedule_parser.add_argument("--duration", type=float, default=0, help="seconds; 0 runs until Ctrl-C")
    schedule_parser.add_argument("--exclude", action="append", default=[], choices=tuple(TARGETS))
    schedule_parser.add_argument("--rate", action="append", default=[], metavar="TARGET=HZ")
    schedule_parser.add_argument("--setup-timeout", type=float, default=5.0)
    schedule_parser.add_argument("--report-interval", type=float, default=1.0)
    schedule_parser.add_argument("--summary", help="write final counters as JSON")
    schedule_parser.add_argument("--dry-run", action="store_true", help="print/validate schedule without UDP")
    for child in (mode_parser, trigger_parser):
        child.add_argument("--instance", type=int, default=0)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("invalid port")
    if hasattr(args, "instance") and not 0 <= args.instance <= 65535:
        parser.error("invalid port or instance")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout must be finite and positive")
    if args.command == "schedule":
        if not math.isfinite(args.duration) or args.duration < 0:
            parser.error("duration must be finite and nonnegative")
        if any(not math.isfinite(v) or v <= 0 for v in (args.setup_timeout, args.report_interval)):
            parser.error("setup-timeout and report-interval must be finite and positive")
        def stop_scheduler(_signum, _frame):
            raise KeyboardInterrupt
        previous_sigterm = signal.signal(signal.SIGTERM, stop_scheduler)
        try:
            return run_scheduler(args)
        except (ValueError, KeyError, TypeError) as exc:
            parser.error(str(exc))
        finally:
            signal.signal(signal.SIGTERM, previous_sigterm)
    if args.target == TARGETS["ekf2"] and args.instance != 0:
        parser.error("EKF2 uses single instance 0")
    opcode = SET_MODE if args.command == "mode" else TRIGGER
    mode = int(args.mode == "manual") if args.command == "mode" else 0
    count = args.count if args.command == "trigger" else 1
    hz = args.hz if args.command == "trigger" else 0
    if count < 1 or not math.isfinite(hz) or hz < 0 or (count > 1 and hz == 0):
        parser.error("use --count >= 1 and a positive --hz for periodic sends")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((args.host, args.port))
        request_id = secrets.randbits(64)
        next_send = time.monotonic()
        for index in range(count):
            if index:
                next_send = max(next_send + 1.0 / hz, time.monotonic())
                time.sleep(max(0, next_send - time.monotonic()))
            result = exchange(sock, opcode, mode, request_id, args.target, args.instance, args.timeout)
            print(f"request={request_id} target=0x{args.target:08x}:{args.instance} {RESULTS.get(result, result)}", flush=True)
            if result != (0 if opcode == SET_MODE else 1):
                return 1
            request_id = (request_id + 1) & 0xFFFFFFFFFFFFFFFF
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, TimeoutError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
