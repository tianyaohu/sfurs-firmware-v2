"""SWD memory access through OpenOCD (one OpenOCD process per ST-LINK probe).

    with swd.Session(serial, "stm32h755") as s:
        s.target.read32(addr)

The session starts `openocd` bound to one probe serial with GDB and telnet
disabled, and talks to its Tcl RPC port. Attach only: it never halts, resets
or flashes (flashing goes through STM32CubeProgrammer). Target method names
follow what the tools already used, so call sites stay the same.
"""
import os
import re
import shutil
import socket
import subprocess
import tempfile
import time

OPENOCD = os.environ.get("OPENOCD", "openocd")
# chip -> OpenOCD target script, extra adapter speed (kHz, None = default)
CHIPS = {"stm32h755": ("target/stm32h7x.cfg", None),
         "stm32g431": ("target/stm32g4x.cfg", 1000)}
SCS_AIRCR, AIRCR_SYSRESETREQ = 0xE000ED0C, 0x05FA0004
EOM = b"\x1a"  # OpenOCD Tcl RPC terminates every command and reply with Ctrl-Z
BLOCK_WORDS = 256  # keeps each Tcl command short


class SwdError(RuntimeError):
    pass


def serial_command(binary, serial):
    """`adapter serial` exists from OpenOCD 0.12; 0.11 (Ubuntu 22.04) only has
    the ST-LINK-specific `hla_serial`."""
    out = subprocess.run([binary, "--version"], capture_output=True, text=True).stderr
    version = re.search(r"Open On-Chip Debugger (\d+)\.(\d+)", out)
    new = version and (int(version[1]), int(version[2])) >= (0, 12)
    return f"{'adapter serial' if new else 'hla_serial'} {serial}"


class Tcl:
    """Minimal OpenOCD Tcl RPC client."""

    def __init__(self, port, timeout=5.0):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.sock.settimeout(timeout)

    def run(self, cmd):
        self.sock.sendall(cmd.encode() + EOM)
        data = b""
        while not data.endswith(EOM):
            chunk = self.sock.recv(65536)
            if not chunk:
                raise SwdError(f"OpenOCD closed the connection during '{cmd}'")
            data += chunk
        return data[:-1].decode().strip()

    def close(self):
        self.sock.close()


class Target:
    """Memory access to the attached core; all values are unsigned."""

    def __init__(self, tcl):
        self.tcl = tcl

    def _read(self, addr, width, count):
        out = []
        for off in range(0, count, BLOCK_WORDS):
            n = min(BLOCK_WORDS, count - off)
            reply = self.tcl.run(f"read_memory 0x{addr + off * width // 8:x} {width} {n}")
            try:
                words = [int(w, 0) for w in reply.split()]
            except ValueError:
                raise SwdError(f"read_memory 0x{addr:x}: {reply}") from None
            if len(words) != n:
                raise SwdError(f"read_memory 0x{addr:x}: got {len(words)} of {n} items: {reply}")
            out += words
        return out

    def read32(self, addr):
        return self._read(addr, 32, 1)[0]

    def read_memory_block32(self, addr, count):
        return self._read(addr, 32, count)

    def read_memory_block8(self, addr, count):
        return self._read(addr, 8, count)

    def write32(self, addr, value):
        reply = self.tcl.run(f"write_memory 0x{addr:x} 32 {{0x{value & 0xFFFFFFFF:x}}}")
        if reply:
            raise SwdError(f"write_memory 0x{addr:x}: {reply}")

    def reset_sysreq(self):
        """Core reset through AIRCR.SYSRESETREQ. The probe may drop the link as
        the chip restarts, so a closed connection is not an error here."""
        try:
            self.write32(SCS_AIRCR, AIRCR_SYSRESETREQ)
        except (SwdError, OSError):
            pass


class Session:
    def __init__(self, serial, chip, binary=None):
        if chip not in CHIPS:
            raise ValueError(f"chip {chip}; one of {sorted(CHIPS)}")
        self.serial, self.chip, self.binary = serial, chip, binary or OPENOCD
        self.target = None
        self._proc = self._tcl = self._log = None

    def open(self, timeout=10.0):
        if shutil.which(self.binary) is None:
            raise SwdError(f"{self.binary} not found; install OpenOCD (apt install openocd)")
        script, speed = CHIPS[self.chip]
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        cmd = [self.binary, "-f", "interface/stlink.cfg",
               "-c", serial_command(self.binary, self.serial),
               "-c", f"tcl_port {port}", "-c", "gdb_port disabled", "-c", "telnet_port disabled"]
        if speed:
            cmd += ["-c", f"adapter speed {speed}"]
        cmd += ["-f", script, "-c", "reset_config none"]
        self._log = tempfile.TemporaryFile()
        self._proc = subprocess.Popen(cmd, stdout=self._log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + timeout
        while True:
            if self._proc.poll() is not None:
                message = f"probe {self.serial} ({self.chip}): {self._output()}"
                self.close()
                raise SwdError(message)
            try:
                self._tcl = Tcl(port)
                break
            except OSError:
                if time.monotonic() > deadline:
                    self.close()
                    raise SwdError(f"probe {self.serial}: OpenOCD not ready in {timeout:.0f} s")
                time.sleep(0.1)
        self.target = Target(self._tcl)
        return self

    def _output(self):
        self._log.seek(0)
        return self._log.read().decode(errors="replace").strip()[-800:]

    def close(self):
        """Detach. OpenOCD leaves the target running when it exits."""
        if self._tcl:
            try:
                self._tcl.run("shutdown")
            except (SwdError, OSError):
                pass
            self._tcl.close()
            self._tcl = None
        if self._proc:
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()
            self._proc = None
        if self._log:
            self._log.close()
            self._log = None
        self.target = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
