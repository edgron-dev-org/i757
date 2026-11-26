# SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
"""Ask an i757 on the local network how it is doing - without the cloud.

Reads the network diagnostic block (Modbus TCP input registers 0x3000..), sets the network
parameters (holding registers 0x3200..) and, when the board's memory window is enabled
(`peek lan on`), dumps RAM. Contract: docs/Universal_Modbus_Port_Config.md 4.5 and 4.6.

    python lan_diag.py                      scan this /24 for boards and print each one's diagnostics
    python lan_diag.py 192.168.1.100        one board
    python lan_diag.py 192.168.1.100 20000000 64     hex dump of 64 bytes of RAM at 0x20000000
    python lan_diag.py 192.168.1.100 static 192.168.1.100 255.255.255.0 192.168.1.1 [dns]
                                            static mode: DHCP is not used any more (dns defaults to the gateway)
    python lan_diag.py 192.168.1.100 dhcp   back to DHCP (the addresses stay as the DHCP-timeout fallback)

The board accepts a change from the LAN only while it has been without its broker for a minute, or
after `peek lan on`; otherwise use the cloud or console command `netcfg`. A static address that
moves the board is applied at once: look for it at the new address.

Typical use: the board is running but offline on the dashboard. Look at "gateway" and "DHCP server":
if they are not your router, something else on the site is handing out addresses.
"""
import socket, struct, sys, concurrent.futures as cf

PORT = 502
STATE = {0: "idle", 1: "TCP connecting", 2: "MQTT connecting", 3: "connected", 4: "disconnecting", 0xFFFF: "no client yet"}


def tx(ip, pdu, uid=1, timeout=3.0):
    s = socket.create_connection((ip, PORT), timeout)
    try:
        s.sendall(struct.pack(">HHHB", 1, 0, len(pdu) + 1, uid) + pdu)
        d = s.recv(600)
    finally:
        s.close()
    return d[7:]


def ip4(hi, lo):
    v = (hi << 16) | lo
    return "%d.%d.%d.%d" % (v >> 24, (v >> 16) & 255, (v >> 8) & 255, v & 255)


def scan(prefix):
    def probe(i):
        ip = "%s.%d" % (prefix, i)
        try:
            s = socket.socket(); s.settimeout(0.5); s.connect((ip, PORT)); s.close()
            return ip
        except OSError:
            return None
    with cf.ThreadPoolExecutor(64) as ex:
        return [x for x in ex.map(probe, range(1, 255)) if x]


def own_prefix():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.0.2.1", 9))          # no packet is sent; this only picks the outgoing interface
        return s.getsockname()[0].rsplit(".", 1)[0]
    finally:
        s.close()


def show(ip):
    r = tx(ip, struct.pack(">BHH", 4, 0x0000, 8))
    if len(r) < 18 or r[0] != 4:
        print("%s: no i757 board map (%s)" % (ip, r.hex())); return
    w = struct.unpack(">8H", r[2:18])
    print("%s: board type 0x%04X, uptime %d s, %d C, cloud flag %d" % (ip, w[1], (w[4] << 16) | w[5], w[6], w[7] & 1))
    r = tx(ip, struct.pack(">BHH", 4, 0x3000, 25))
    if r[0] != 4:
        print("  no diagnostic block (firmware older than 2026-11-27): %s" % r.hex()); return
    w = struct.unpack(">25H", r[2:52])
    f = w[1]
    print("  link %s | DHCP lease %s | static fallback %s | broker %s | reached broker this boot %s | memory window %s | static mode %s" % tuple(
        "yes" if f & b else "no" for b in (1, 2, 4, 8, 16, 32, 64)))
    print("  ip %s  mask %s  gateway %s  dns %s" % (ip4(w[2], w[3]), ip4(w[4], w[5]), ip4(w[6], w[7]), ip4(w[8], w[9])))
    print("  DHCP server %s" % ip4(w[10], w[11]))
    print("  broker %s:%d (index %d)  client %s  last error %d  consecutive failures %d" % (
        ip4(w[12], w[13]), w[14], w[15], STATE.get(w[16], w[16]), struct.unpack(">h", struct.pack(">H", w[17]))[0], w[18]))
    print("  offline %d s  self resets this episode %d  publishes ok %d  last frame received %d s ago" % (
        (w[19] << 16) | w[20], w[21], (w[22] << 16) | w[23], w[24]))
    r = tx(ip, struct.pack(">BHH", 3, 0x3200, 10))
    if r[0] == 3:
        w = struct.unpack(">10H", r[2:22])
        print("  stored settings: %s  ip %s  mask %s  gateway %s  dns %s%s" % (
            "STATIC" if w[0] else "DHCP, fallback", ip4(w[1], w[2]), ip4(w[3], w[4]), ip4(w[5], w[6]), ip4(w[7], w[8]),
            "  (uncommitted change staged)" if w[9] else ""))


def u32(s):
    b = [int(x) for x in s.split(".")]
    if len(b) != 4 or any(x < 0 or x > 255 for x in b):
        raise SystemExit("not an IPv4 address: %s" % s)
    return (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]


def netset(ip, mode, addrs):
    """mode 1 = static, 0 = DHCP. One FC16 writes the block and the commit key, so nothing half-set is ever used."""
    if mode:
        if len(addrs) not in (3, 4):
            raise SystemExit("usage: lan_diag.py <board> static <ip> <mask> <gw> [dns]")
        v = [u32(a) for a in addrs] + ([0] if len(addrs) == 3 else [])
        regs = [1] + [x for a in v for x in (a >> 16, a & 0xFFFF)] + [0xA55A]
        pdu = struct.pack(">BHHB10H", 0x10, 0x3200, 10, 20, *regs)
    else:
        r = tx(ip, struct.pack(">BHH", 3, 0x3200, 9))          # keep the stored addresses, change the mode only
        if r[0] != 3:
            print("no settings block (firmware older than 2026-11-27b): %s" % r.hex()); return
        regs = [0] + list(struct.unpack(">8H", r[4:20])) + [0xA55A]
        pdu = struct.pack(">BHHB10H", 0x10, 0x3200, 10, 20, *regs)
    r = tx(ip, pdu)
    if r[0] == 0x10:
        print("accepted: %s" % ("static %s" % addrs[0] if mode else "DHCP"))
        if mode and addrs[0] != ip:
            print("the board is moving to %s now" % addrs[0])
    elif r[1] == 1:
        print("refused: the board is connected to its broker and the LAN is not marked trusted.")
        print("Use the cloud/console command `netcfg`, or `peek lan on` first.")
    elif r[1] == 3:
        print("refused: the addresses do not make a usable configuration (netmask, host address, gateway inside the subnet)")
    else:
        print("refused: %s (firmware older than 2026-11-27b has no settings block)" % r.hex())


def dump(ip, addr, length):
    done = 0
    while done < length:
        n = min(240, length - done)
        r = tx(ip, struct.pack(">BHHBI", 0x10, 0x3100, 2, 4, addr + done))
        if r[0] != 0x10:
            print("memory window is off (send `peek lan on` over the cloud or the console): %s" % r.hex()); return
        r = tx(ip, struct.pack(">BHH", 4, 0x3100, (n + 1) // 2))
        if r[0] != 4:
            print("%08x: not readable (RAM and the fault registers only; the TLS pool is excluded)" % (addr + done)); return
        data = r[2:2 + n]
        for o in range(0, len(data), 16):
            row = data[o:o + 16]
            print("%08x  %-47s  %s" % (addr + done + o, " ".join("%02x" % b for b in row),
                                         "".join(chr(b) if 32 <= b < 127 else "." for b in row)))
        done += n


def main():
    a = sys.argv[1:]
    if not a:
        pre = own_prefix()
        ips = scan(pre)
        if not ips:
            print("no Modbus TCP device on %s.0/24" % pre)
        for ip in ips:
            show(ip)
    elif len(a) == 1:
        show(a[0])
    elif a[1] in ("static", "dhcp"):
        netset(a[0], 1 if a[1] == "static" else 0, a[2:])
    else:
        dump(a[0], int(a[1], 16), int(a[2], 0) if len(a) > 2 else 64)


if __name__ == "__main__":
    main()
