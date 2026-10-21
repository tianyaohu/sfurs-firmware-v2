#!/usr/bin/env python3
"""src/tools/swd.py against a fake `openocd` executable that speaks OpenOCD's
documented Tcl RPC (Ctrl-Z framed commands, read_memory / write_memory)."""
import pathlib
import stat
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/tools"))
import swd  # noqa: E402

FAKE = '''#!/usr/bin/env python3
import re, socket, sys
a = sys.argv[1:]
if "FAIL" in " ".join(a):
    print("Error: open failed"); sys.exit(1)
open(__file__ + ".args", "w").write("\\n".join(a))
port = int(re.search(r"tcl_port (\\d+)", " ".join(a))[1])
mem = {}
srv = socket.socket(); srv.bind(("127.0.0.1", port)); srv.listen(1)
conn, _ = srv.accept()
buf = b""
while True:
    d = conn.recv(4096)
    if not d: break
    buf += d
    while b"\\x1a" in buf:
        cmd, buf = buf.split(b"\\x1a", 1)
        w = cmd.decode().split()
        if w[0] == "read_memory":
            addr, width, n = int(w[1], 0), int(w[2]), int(w[3])
            step = width // 8
            out = " ".join(hex(mem.get(addr + i * step, (addr + i * step) & (2**width - 1))) for i in range(n))
        elif w[0] == "write_memory":
            mem[int(w[1], 0)] = int(w[3].strip("{}"), 0); out = ""
        elif w[0] == "shutdown":
            conn.sendall(b"\\x1a"); sys.exit(0)
        else:
            out = "invalid command"
        conn.sendall(out.encode() + b"\\x1a")
'''


class Swd(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.fake = pathlib.Path(self.dir.name) / "openocd"
        self.fake.write_text(FAKE)
        self.fake.chmod(self.fake.stat().st_mode | stat.S_IXUSR)

    def tearDown(self):
        self.dir.cleanup()

    def test_read_write_and_blocks(self):
        with swd.Session("SN123", "stm32h755", str(self.fake)) as s:
            t = s.target
            self.assertEqual(t.read32(0x20000000), 0x20000000)
            t.write32(0x20000004, -1)
            self.assertEqual(t.read32(0x20000004), 0xFFFFFFFF)
            words = t.read_memory_block32(0x24000000, 600)  # spans three Tcl blocks
            self.assertEqual(words, [0x24000000 + 4 * i for i in range(600)])
            self.assertEqual(t.read_memory_block8(0x08000000, 3), [0, 1, 2])

    def test_probe_serial_and_chip_reach_openocd(self):
        for chip, script, speed in (("stm32h755", "stm32h7x.cfg", False), ("stm32g431", "stm32g4x.cfg", True)):
            with swd.Session("SN123", chip, str(self.fake)):
                args = pathlib.Path(str(self.fake) + ".args").read_text().split("\n")
            self.assertIn("adapter serial SN123", args)
            self.assertTrue(any(a.endswith(script) for a in args))
            self.assertEqual("adapter speed 1000" in args, speed)
            self.assertIn("gdb_port disabled", args)

    def test_reset_writes_aircr(self):
        with swd.Session("SN123", "stm32g431", str(self.fake)) as s:
            s.target.reset_sysreq()
            self.assertEqual(s.target.read32(swd.SCS_AIRCR), swd.AIRCR_SYSRESETREQ)

    def test_openocd_failure_reports_its_output(self):
        with self.assertRaisesRegex(swd.SwdError, "open failed"):
            swd.Session("FAIL", "stm32h755", str(self.fake)).open()

    def test_missing_binary_and_unknown_chip(self):
        with self.assertRaisesRegex(swd.SwdError, "not found"):
            swd.Session("SN", "stm32h755", "/nonexistent/openocd").open()
        with self.assertRaises(ValueError):
            swd.Session("SN", "stm32f407")


if __name__ == "__main__":
    unittest.main()
