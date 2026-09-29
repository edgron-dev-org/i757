# SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
# Copyright (c) 2026 Edgron. See LICENSE at the SDK root.
"""u485_quickstart.py - first contact with Edgron expansion modules over a USB-RS485 adapter.

Scans slave addresses 1..16, prints each module's identity block, then shows the module's data:
  EX_16DI : input levels (FC02) + 32-bit counters (FC04 0x0200)
  EX_16DO : output read-back (FC01); with --toggle, blinks O1 for 10 s while refreshing every 1 s
            (the module drops all outputs after 3 s of bus silence - a master must keep polling)
  PH_EC   : the 8 engineering values (FC04 0x0108): pH x100, EC uS/cm, temperatures x10 C

Usage:
  python u485_quickstart.py --port COM7                  # 9600 8N1, scan addresses 1..16
  python u485_quickstart.py --port COM7 --baud scan      # try every rate a module can be set to
  python u485_quickstart.py --port COM7 --addr 3 --toggle
  python u485_quickstart.py --port /dev/ttyUSB0 --baud 1000000

Needs Python 3.8+ and pymodbus 3.x:  pip install pymodbus pyserial
Documentation: docs/Expansion_Modules_Standalone_Quick_Start.md and the module manuals in docs/.
"""
import argparse, inspect, sys, time
try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    sys.exit("pymodbus not installed: pip install pymodbus pyserial")

TYPES = {1: "EX_16DO", 2: "PH_EC", 3: "EX_16DI", 5: "EX_8AI"}
RATES = [9600, 19200, 38400, 57600, 115200, 4800, 250000, 500000, 1000000]   # every code of 0x0011/0x0281

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--port", required=True, help="serial port of the USB-RS485 adapter, e.g. COM7 or /dev/ttyUSB0")
ap.add_argument("--baud", default="9600", help="baud rate (modules ship at 9600), or 'scan' to try all rates")
ap.add_argument("--addr", type=int, default=0, help="only this slave address (default: scan 1..16)")
ap.add_argument("--toggle", action="store_true", help="EX_16DO: blink O1 for 10 s with a 1 s keep-alive")
a = ap.parse_args()


def open_client(baud):
    c = ModbusSerialClient(port=a.port, baudrate=baud, parity="N", stopbits=1, bytesize=8, timeout=0.3, retries=0)
    if not c.connect():
        sys.exit("cannot open %s" % a.port)
    # pymodbus renamed the unit-id keyword between 3.x releases
    kw = "device_id" if "device_id" in inspect.signature(c.read_input_registers).parameters else "slave"
    return c, kw


def ident(c, kw, addr):
    """FC04 0x0000..0x0008: map version, type, fw, caps, uptime hi/lo, crc errors, flags, hw. None if no reply."""
    try:
        r = c.read_input_registers(0, count=9, **{kw: addr})
    except Exception:
        return None
    if r.isError():
        return None
    return r.registers


def show_ident(addr, g):
    up = (g[4] << 16) | g[5]
    print("addr %2d: %-7s fw=0x%04X hw=0x%04X uptime=%ds crc_err=%d flags=0x%04X%s"
          % (addr, TYPES.get(g[1], "type%d" % g[1]), g[2], g[8], up, g[6], g[7],
             "  [SAFE STATE]" if g[7] & 1 else ""))


def demo_16di(c, kw, addr):
    d = c.read_discrete_inputs(0, count=16, **{kw: addr})
    r = c.read_input_registers(0x0200, count=32, **{kw: addr})
    if d.isError() or r.isError():
        print("   read failed:", d if d.isError() else r); return
    bits = "".join("1" if b else "0" for b in d.bits[:16])
    g = r.registers
    cnt = [(g[2 * i] << 16) | g[2 * i + 1] for i in range(16)]
    print("   DI1..16 =", bits, "(1 = loop closed)")
    print("   counters =", cnt, "(rising edges since power-up)")


def demo_16do(c, kw, addr):
    r = c.read_coils(0, count=16, **{kw: addr})
    if r.isError():
        print("   read failed:", r); return
    print("   O1..16 read-back =", "".join("1" if b else "0" for b in r.bits[:16]))
    if not a.toggle:
        print("   (--toggle blinks O1 for 10 s; note the 3 s safe-state rule in the manual, section 6)")
        return
    print("   blinking O1 for 10 s, refreshing every 1 s ...")
    for i in range(10):
        c.write_coil(0, i % 2 == 0, **{kw: addr})
        time.sleep(1.0)
        r = c.read_coils(0, count=1, **{kw: addr})
        print("   O1 =", int(r.bits[0]) if not r.isError() else "?")
    c.write_coil(0, False, **{kw: addr})
    print("   O1 off. Stop polling now and the module switches everything off by itself after 3 s.")


def demo_phec(c, kw, addr):
    r = c.read_input_registers(0x0108, count=8, **{kw: addr})
    if r.isError():
        print("   read failed:", r); return
    g = r.registers
    s16 = lambda v: v - 65536 if v > 32767 else v
    fmt = lambda v, div, unit: "invalid" if v == 0x7FFF else "%.2f %s" % (s16(v) / div, unit)
    print("   pH1 =", fmt(g[0], 100, ""), " pH2 =", fmt(g[1], 100, ""))
    print("   EC1 =", "invalid" if g[2] == 0x7FFF else "%d uS/cm" % g[2], " EC2 =", "invalid" if g[3] == 0x7FFF else "%d uS/cm" % g[3])
    print("   T_pH1 =", fmt(g[4], 10, "C"), " T_pH2 =", fmt(g[5], 10, "C"), " T_EC1 =", fmt(g[6], 10, "C"), " T_EC =", fmt(g[7], 10, "C"))
    print("   (0x7FFF = channel invalid: no probe, out of range, or fault. Poll every 2 s or slower.)")


DEMO = {1: demo_16do, 2: demo_phec, 3: demo_16di}

rates = RATES if a.baud == "scan" else [int(a.baud)]
addrs = [a.addr] if a.addr else range(1, 17)
found = []
for baud in rates:
    c, kw = open_client(baud)
    print("--- %s @ %d 8N1 ---" % (a.port, baud))
    for addr in addrs:
        g = ident(c, kw, addr)
        if g is None:
            continue
        show_ident(addr, g)
        found.append((baud, addr, g[1]))
        DEMO.get(g[1], lambda *x: print("   (no demo for this module type; see its manual)"))(c, kw, addr)
    c.close()
    if found:
        break
if not found:
    print("no module answered. Check: A/B polarity, DIP address, 24 V on EX_BUS, adapter GND to 0 V,")
    print("and the rate: a module that was ever on an i757 backplane runs at 1 Mbps (try --baud scan).")
    sys.exit(1)
print("found %d module(s) at %d baud" % (len(found), found[0][0]))
