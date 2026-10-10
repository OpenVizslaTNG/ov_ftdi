#!/usr/bin/env python3
# Copyright (c) 2026 Tomasz Moń
# SPDX-License-Identifier: BSD-3-Clause

"""Verify FTDITXRamp bitstream output arrives correctly at host."""

import argparse
import os
import sys
import time

from LibOV import FTDIDevice, HW_Init, FTDI_INTERFACE_A

DEFAULT_BIT = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    "..", "fpga", "ov3", "build_ftdi_txramp", "ftdi_txramp.bit")


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bitstream", nargs="?", default=DEFAULT_BIT)
    ap.add_argument("--bytes", type=int, default=1 << 20)
    ap.add_argument("--max-breaks", type=int, default=20, dest="n",
                    help='maximum number of breaks to print')
    args = ap.parse_args()

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
    time.sleep(1.0)

    buf = read_with_timeout(dev, args.bytes)
    print(f"wanted {args.bytes}, read {len(buf)} bytes")
    if not buf:
        print("NO DATA: did not receive anything")
        return 2

    breaks = [i for i in range(1, len(buf)) if (buf[i] - buf[i - 1]) & 0xff != 1]
    if not breaks:
        if len(buf) >= args.bytes:
            print(f"OK: Received {len(buf)} contiguous ramp bytes")
            return 0

        print(f"FAIL: Stalled after {len(buf)} contiguous ramp bytes")
        return 0

    print(f"RAMP BROKE: {len(breaks)} discontinuities, first at offset {breaks[0]}")
    print(f"Data before and after first {min(args.n, len(breaks))} breaks:")
    for offset in breaks[:args.n]:
        before = buf[max(0, offset - 4):offset].hex(" ")
        after = buf[offset:offset + 4].hex(" ")
        print(f"  {before} [offset {offset}] {after}")

    return 2


if __name__ == "__main__":
    sys.exit(main())
