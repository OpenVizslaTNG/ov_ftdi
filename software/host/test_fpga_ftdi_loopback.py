#!/usr/bin/env python3
# Copyright (c) 2026 Tomasz Moń
# SPDX-License-Identifier: BSD-3-Clause

"""Verify FTDILoopback bitstream loopback with pseudo random data."""

import argparse
import os
import random
import signal
import sys
import threading
import time
import typing

from LibOV import FTDIDevice, HW_Init, FTDI_INTERFACE_A

DEFAULT_BIT = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    "..", "fpga", "ov3", "build_ftdi_loopback", "ftdi_loopback.bit")

BANNER = b"OV FTDI LOOPBACK READY\r\n"

MIB = 1024 * 1024

def read_with_timeout(dev, n, idle_timeout=2.0):
    buf = bytearray()
    last = [time.time()]

    def cb(b):
        if b:
            buf.extend(b)
            last[0] = time.time()
        if len(buf) >= n:
            return 1
        if time.time() - last[0] > idle_timeout:
            return 1
        return 0

    dev.read_async(FTDI_INTERFACE_A, cb, 8, 16)
    return bytes(buf)

def echo_rate(bytes, duration):
    return f"{bytes / (MIB * max(sys.float_info.epsilon, duration)):.2f} MiB/s"


class RandomSource:
    def __init__(self, seed, gen_chunk=1 << 16, limit_bytes=None):
        self._rng = random.Random(seed)
        self.gen_chunk = gen_chunk
        self.limit_bytes = limit_bytes
        self._buf = bytearray()
        self.bytes_generated = 0

    def generate(self, n):
        """Return next n bytes of the stream. Can return less bytes only if
        byte limit is reached."""
        if self.limit_bytes is None:
            wanted = n
        else:
            remaining = max(self.limit_bytes - self.bytes_generated, 0)
            wanted = min(remaining, n)

        while len(self._buf) < wanted:
            self._buf.extend(self._rng.randbytes(self.gen_chunk))

        data = bytes(self._buf[:wanted])
        del self._buf[:wanted]
        self.bytes_generated += wanted

        return data

    def skip(self, n):
        """Advance the stream past the next n bytes without returning them."""
        remaining = n
        while remaining > 0:
            generated = len(self.generate(min(remaining, self.gen_chunk)))
            if not generated:
                # limit_bytes reached, nothing left to skip over
                break
            remaining -= generated


class RandomVerifier(RandomSource):
    mismatch_found = False

    def verify(self, data):
        """Verify provided data against generator.

        Return the number of bytes that match expected stream. If len(data) is
        returned, it means that all data is expected and valid. Any other value
        indicates the index (relative to provided data) of first mismatch."""
        if self.mismatch_found:
            raise Exception("Verifier already indicated mismatch")

        expected = self.generate(len(data))
        if data == expected:
            return len(data)

        self.mismatch_found = True
        for i in range(len(expected)):
            if data[i] != expected[i]:
                return i

        # Unexpected data after generator reached limit
        return len(expected)


def analyze(result):
    print(f"  sent {result.bytes_sent} bytes, received {result.bytes_received} bytes")

    if result.bytes_received > result.bytes_sent:
        extra_bytes = result.bytes_received - result.bytes_sent
        print(f"  -> device sent {extra_bytes} byte(s) more than it was given")
        if result.bytes_verified == result.bytes_sent:
            print("     all sent bytes echoed correctly")
            print(f"     extra bytes: {result.mismatch_tail.hex(' ')}")
            return
        elif result.mismatch_tail is None:
            print("     all received bytes match, how did device predict pattern?")
            return
    elif result.bytes_verified == result.bytes_received:
        if result.bytes_received < result.bytes_sent:
            print(f"  -> first {result.bytes_received} byte(s) echoed correctly, "
                  "rest did not arrive")
        else:
            print("  -> exact match")
        return

    offset = result.bytes_verified

    # Absolute stream offset the printed window starts at
    window_start = max(0, offset - 4)
    window_end = offset + len(result.mismatch_tail)

    # Recreate window around the failure for divergence analysis
    result.generator.skip(window_start)
    sent_window = result.generator.generate(window_end - window_start)
    received_window = sent_window[:offset - window_start] + result.mismatch_tail

    # Indexes relative to windows. Print the whole window content before the
    # divergence and no more than 8 bytes afterwards.
    mismatch_idx = offset - window_start
    max_shown_length = mismatch_idx + 8
    shown_sent = min(max_shown_length, len(sent_window))
    shown_received = min(max_shown_length, len(received_window))
    print(f"  -> DIVERGES at offset {offset}")
    print(f"     sent[{window_start}:{window_start + shown_sent}] = "
          f"{sent_window[:shown_sent].hex(' ')}")
    print(f"     got [{window_start}:{window_start + shown_received}] = "
          f"{received_window[:shown_received].hex(' ')}")

    # Determine if this is a simple byte drop
    tail = received_window[mismatch_idx:shown_received]
    nxt = sent_window.find(tail, mismatch_idx) if tail else -1
    if nxt > mismatch_idx:
        print("     got[%d:] resumes at sent offset %d (a run of %d sent "
              "bytes was dropped)" % (offset, window_start + nxt,
                                      nxt - mismatch_idx))


class LoopbackResult(typing.NamedTuple):
    # Number of bytes that matched the expected stream. This value indicates the
    # offset of the first mismatch if it is less than bytes_received.
    bytes_verified: int
    # Number of bytes that were queued on FTDI OUT endpoint. Note that this may
    # be larger than number of bytes that FTDI actually ACKed on OUT endpoint.
    bytes_sent: int
    # Number of bytes read back from FTDI IN endpoint.
    bytes_received: int
    # Generator that can be used to recreate sent data stream.
    generator: RandomSource
    # Bytes actually received from the first mismatch onwards: the rest of the
    # chunk it was found in, plus everything the drain that follows brings in.
    # None if no mismatch was found.
    mismatch_tail: bytearray | None


def loopback(dev, seed, total_bytes, max_chunk, randomize_write_size, sleep_probability):
    status_every_mib = 10
    idle_timeout = 2.0
    sleep_max=0.003

    rng = random.Random(seed)
    verifier = RandomVerifier(seed, limit_bytes=total_bytes)
    generator = RandomSource(seed, limit_bytes=total_bytes)

    t0 = time.time()
    next_status = status_every_mib * MIB
    last_data_time = time.time()
    bytes_sent = 0
    bytes_received = 0
    bytes_verified = 0
    mismatch_tail = None

    stop = False
    handler_restored = False
    progress = threading.Condition()

    def sigint_handler(signum, frame):
        nonlocal stop, handler_restored

        signal.signal(signal.SIGINT, old_handler)
        handler_restored = True

        with progress:
            stop = True
            in_flight = bytes_sent - bytes_received
            progress.notify_all()

        print(f"Interrupted, waiting for remaining {in_flight / MIB:.1f} MiB "
              f"still in flight (Ctrl-C again to abort)")

    def cb(b):
        # Variables used only by this thread
        nonlocal last_data_time, next_status, mismatch_tail
        # Variables that can only be access with progress lock held
        nonlocal bytes_received, bytes_verified, stop

        if not b:
            if time.time() - last_data_time > idle_timeout:
                # Data is not arriving, most likely stalled
                return 1
            with progress:
                return stop and (bytes_received >= bytes_sent)

        last_data_time = time.time()

        first_mismatch_found = False
        if mismatch_tail is None:
            matched = verifier.verify(b)
            if matched < len(b):
                # Found first mismatch, stop generation and verify, but wait
                # until already queued data goes through the loopback loop
                mismatch_tail = bytearray(b[matched:])
                first_mismatch_found = True
        else:
            # Collect remaining data for later analysis
            mismatch_tail.extend(b)
            matched = 0

        with progress:
            bytes_received += len(b)
            bytes_verified += matched

            if first_mismatch_found:
                stop = True
                in_flight = bytes_sent - bytes_received

            progress.notify_all()
            done = stop and (bytes_received >= bytes_sent)

        # We can read bytes_verified without lock because only we update it
        if (bytes_verified >= next_status) and not stop:
            dt = time.time() - t0
            print(f"  {next_status // MIB} MiB OK  ({echo_rate(bytes_verified, dt)})")
            next_status += status_every_mib * MIB

        if first_mismatch_found:
            print(f"Mismatch at offset {bytes_verified}, waiting for remaining "
                  f"{in_flight / MIB:.1f} MiB to see how much of it arrives")

        return done

    def writer_can_continue():
        # Throttle down data generation if verifier lags by at least 1 MiB
        bytes_outstanding = bytes_sent - bytes_verified
        return stop or (bytes_outstanding < MIB)

    def writer():
        nonlocal bytes_sent, stop
        while True:
            if randomize_write_size:
                num = rng.randint(1, max_chunk)
            else:
                num = max_chunk

            chunk = generator.generate(num)
            if not chunk:
                # Writer has no more work to do
                break

            if sleep_probability and rng.random() < sleep_probability:
                time.sleep(rng.uniform(0.0, sleep_max))

            with progress:
                progress.wait_for(writer_can_continue)

                if stop:
                    break

                err = dev.write(FTDI_INTERFACE_A, chunk, async_=True)
                if err:
                    print(f"write failed with libusb error {err}",
                          file=sys.stderr)
                    break

                bytes_sent += len(chunk)

    def reader():
        err = None

        try:
            err = dev.read_async(FTDI_INTERFACE_A, cb, 8, 16)
        finally:
            if err is None:
                print("read_async never returned value", file=sys.stderr)
            elif err < 0:
                print(f"read stream aborted with libusb error {err}", file=sys.stderr)

            # Reader ends, make sure writer stops
            with progress:
                nonlocal stop
                stop = True
                progress.notify_all()

    # Prevent Ctrl-C from interrupting worker threads. Running both reader and
    # writer inside threads allows SIGINT handler running on main thread to take
    # progress condition and notify all after setting stop flag.
    old_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
    old_handler = signal.signal(signal.SIGINT, sigint_handler)
    rt = threading.Thread(target=reader, daemon=True)
    wt = threading.Thread(target=writer, daemon=True)
    rt.start()
    wt.start()
    signal.pthread_sigmask(signal.SIG_SETMASK, old_mask)

    wt.join()
    # Writer finished, notify reader it needs to stop (if it still runs)
    with progress:
        stop = True
        progress.notify_all()
    rt.join()
    if not handler_restored:
        signal.signal(signal.SIGINT, old_handler)

    return LoopbackResult(bytes_verified, bytes_sent, bytes_received,
                          RandomSource(seed, limit_bytes=total_bytes),
                          mismatch_tail)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bitstream", nargs="?", default=DEFAULT_BIT)
    ap.add_argument("--seed", type=int, default=0xDEADBEEF,
                    help="random number generator seed value")
    ap.add_argument("--max-chunk", type=int, default=4096,
                    help="maximum write chunk size")
    ap.add_argument("--random-write-size", action="store_true",
                    help="randomize write chunk size")
    ap.add_argument("--random-sleep", type=float,
                    help="probability of random sleeps between writes")
    ap.add_argument("--total", type=int, default=None,
                    help="number of bytes to loopback")
    ap.add_argument("--endless", action="store_true",
                    help="run forever until first divergence")
    args = ap.parse_args()

    if args.endless and args.total:
        print("--endless and --total are mutually exclusive")
        return 1

    if args.random_sleep is not None:
        if args.random_sleep < 0.0 or args.random_sleep > 1.0:
            print(f"--random-sleep takes probability in range [0.0; 1.0]")
            return 1

    dev = FTDIDevice()
    if dev.open():
        print("ERROR: could not open FTDI device", file=sys.stderr)
        return 1

    bit = os.path.realpath(args.bitstream)
    if not os.path.exists(bit):
        print(f"ERROR: bitstream not found: {bit}", file=sys.stderr)
        return 1
    print(f"Loading {bit}")
    HW_Init(dev, bit.encode("ascii"))

    # Read loopback banner
    banner = read_with_timeout(dev, len(BANNER), idle_timeout=1.0)
    if banner == BANNER:
        print(f"Banner OK: {banner}")
    else:
        if not banner:
            print("No startup banner", file=sys.stderr)
        else:
            print(f"Invalid banner: {banner}", file=sys.stderr)
        return 2

    total = args.total
    if args.endless:
        print("Endless loopback (Ctrl-C to stop)")
        total = None
    elif total is None:
        total = MIB

    t0 = time.time()
    result = loopback(dev, args.seed, total, args.max_chunk,
                      args.random_write_size, args.random_sleep)
    dt = time.time() - t0

    ok = (result.bytes_verified == result.bytes_received and
          result.bytes_received == result.bytes_sent)
    if not ok:
        print("FAILED:", file=sys.stderr)
        analyze(result)
        return 2
    print(f"{result.bytes_verified} bytes successfully echoed at "
          f"{echo_rate(result.bytes_verified, dt)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
