"""Fixtures for the twincat-ads integration tests.

Two clients talk to the system under test:
- ``plc``: a pyads client acting on the PLC like an engineering tool or HMI,
- ``ioc``: the test IOC (tests/ioc), driven through PVAccess and its iocsh.

Both connect to the ADS test server in server/ads_test_server.py.
"""

import contextlib
import ctypes
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pexpect
import pyads
import pytest
from p4p.client.thread import Context

from server.symbols import DEFAULT_PORT, find

TESTS_DIR = Path(__file__).parent
IOC_TOP = TESTS_DIR / "ioc"
IOC_BOOT = IOC_TOP / "iocBoot" / "iocAdsTest"
LOG_DIR = TESTS_DIR / "logs"

SERVER_IP = "127.0.0.2"
SERVER_NET_ID = "127.0.0.2.1.1"
SERVER_TCP_PORT = 48898
# pyads and the IOC would both derive 127.0.0.1.1.1 from their source address;
# a distinct NetId keeps the two ADS clients apart.
PYADS_NET_ID = "127.0.0.3.1.1"
# The emulated PLC clock runs behind the host, so PLC and EPICS timestamps differ
PLC_CLOCK_OFFSET = -1000.0


def pytest_collection_modifyitems(items):
    items.sort(key=lambda item: item.get_closest_marker("disruptive") is not None)


def wait_until(predicate, timeout, message, interval=0.05):
    deadline = time.monotonic() + timeout
    while True:
        result = predicate()
        if result:
            return result
        if time.monotonic() > deadline:
            raise AssertionError(f"timeout after {timeout} s: {message}")
        time.sleep(interval)


class AdsServer:
    """The ADS test server running as subprocess."""

    def __init__(self):
        self.proc = None

    def start(self):
        LOG_DIR.mkdir(exist_ok=True)
        log = open(LOG_DIR / "ads_server.log", "a")
        self.proc = subprocess.Popen(
            [sys.executable, "-m", "server.ads_test_server", "--host", SERVER_IP,
             "--clock-offset", str(PLC_CLOCK_OFFSET)],
            cwd=TESTS_DIR, stdout=log, stderr=subprocess.STDOUT,
        )
        wait_until(self._accepts, 10, "ADS test server did not start")

    def stop(self):
        self.proc.terminate()
        self.proc.wait(10)

    def restart(self):
        self.stop()
        self.start()

    def _accepts(self):
        if self.proc.poll() is not None:
            raise RuntimeError(f"ADS test server exited, see {LOG_DIR / 'ads_server.log'}")
        with contextlib.suppress(OSError), socket.create_connection((SERVER_IP, SERVER_TCP_PORT), 0.2):
            return True
        return False


class Plc:
    """pyads client: reads and writes PLC symbols by name, like an HMI would."""

    def __init__(self):
        self._connections = {}

    def connection(self, port=DEFAULT_PORT) -> pyads.Connection:
        if port not in self._connections:
            conn = pyads.Connection(SERVER_NET_ID, port, SERVER_IP)
            conn.open()
            self._connections[port] = conn
        return self._connections[port]

    def read(self, name, port=DEFAULT_PORT):
        sym = find(name, port)
        raw = self.connection(port).read_by_name(name, ctypes.c_ubyte * sym.size)
        return sym.decode(bytes(raw))

    def write(self, name, value, port=DEFAULT_PORT):
        sym = find(name, port)
        self.connection(port).write_by_name(name, list(sym.encode(value)), ctypes.c_ubyte * sym.size)

    def state(self, port=DEFAULT_PORT) -> int:
        return self.connection(port).read_state()[0]

    def set_state(self, state, port=DEFAULT_PORT):
        self.connection(port).write_control(state, 0, 0, pyads.PLCTYPE_INT)

    def online_change(self, port=DEFAULT_PORT):
        """Bump the symbol version, which the test server treats as an online change."""
        conn = self.connection(port)
        version = conn.read(pyads.constants.ADSIGRP_SYM_VERSION, 0, pyads.PLCTYPE_BYTE)
        conn.write(pyads.constants.ADSIGRP_SYM_VERSION, 0, (version + 1) % 256, pyads.PLCTYPE_BYTE)

    def close(self):
        for conn in self._connections.values():
            conn.close()
        self._connections.clear()


def _same(value, expected):
    if callable(expected):
        return expected(value)
    if hasattr(value, "tolist"):
        value = value.tolist()
    return value == expected


class Ioc:
    """The test IOC: an iocsh session plus a PVAccess client for its records."""

    PROMPT = "epics> "

    def __init__(self, name, script="st.cmd", prefix="ADS_TEST:", pva_port=15076, env=None):
        self.name = name
        self.script = script
        self.prefix = prefix
        self.env = {
            # Keep CA and PVA traffic on the loopback, away from other IOCs
            "EPICS_CAS_INTF_ADDR_LIST": "127.0.0.1",
            "EPICS_CA_AUTO_ADDR_LIST": "NO",
            "EPICS_CA_ADDR_LIST": "127.0.0.1",
            "EPICS_PVAS_INTF_ADDR_LIST": "127.0.0.1",
            "EPICS_PVAS_BROADCAST_PORT": str(pva_port),
            "PLC_IP": SERVER_IP,
            "PLC_AMS_NET_ID": SERVER_NET_ID,
            "P": prefix,
            **(env or {}),
        }
        self.pva = Context("pva", useenv=False, conf={
            "EPICS_PVA_ADDR_LIST": "127.0.0.1",
            "EPICS_PVA_AUTO_ADDR_LIST": "NO",
            "EPICS_PVA_BROADCAST_PORT": str(pva_port),
        })
        self.proc = None
        self.startup_log = ""

    def start(self, timeout=60):
        executable = next(IOC_TOP.glob("bin/*/adsTestIoc"), None)
        if executable is None:
            raise RuntimeError("test IOC not built, run: make -C tests/ioc")
        LOG_DIR.mkdir(exist_ok=True)
        self.proc = pexpect.spawn(
            str(executable), [self.script], cwd=IOC_BOOT, env={**os.environ, **self.env},
            encoding="utf-8", codec_errors="replace", timeout=timeout,
        )
        self.proc.logfile_read = open(LOG_DIR / f"{self.name}.log", "a")
        self.proc.expect_exact("iocRun: All initialization complete")
        self.startup_log = self.proc.before
        self.proc.expect_exact(self.PROMPT)

    def stop(self):
        if self.proc is None or not self.proc.isalive():
            return
        self.proc.sendline("exit")
        try:
            self.proc.expect(pexpect.EOF, timeout=15)
        except pexpect.TIMEOUT:
            self.proc.terminate(force=True)
            raise AssertionError(f"{self.name} did not exit within 15 s")

    def run(self, command, timeout=10) -> str:
        """Run an iocsh command and return its output."""
        self.proc.sendline(command)
        self.proc.expect_exact(self.PROMPT, timeout=timeout)
        return self.proc.before.split("\n", 1)[-1]  # drop the echoed command

    def pv(self, name) -> str:
        return self.prefix + name

    def get(self, name, timeout=5):
        return self.pva.get(self.pv(name), timeout=timeout)

    def put(self, name, value, timeout=5):
        self.pva.put(self.pv(name), value, wait=True, timeout=timeout)

    def wait_for(self, name, expected, timeout=5):
        """Wait until the PV equals ``expected`` (a value, pytest.approx or a predicate)."""
        last = []

        def check():
            last[:] = [self.get(name)]
            return _same(last[0], expected)

        try:
            wait_until(check, timeout, "")
        except AssertionError:
            raise AssertionError(f"{self.pv(name)} is {last[0]!r}, expected {expected!r}") from None
        return last[0]

    @contextlib.contextmanager
    def monitor(self, name):
        """Collect (arrival time, value) of every update while the block runs."""
        updates = []

        def on_update(value):
            if not isinstance(value, Exception):
                updates.append((time.time(), value))

        sub = self.pva.monitor(self.pv(name), on_update, request="record[queueSize=1000]")
        try:
            yield updates
        finally:
            sub.close()

    def close(self):
        self.stop()
        self.pva.close()


@pytest.fixture(scope="session")
def ads_server():
    server = AdsServer()
    server.start()
    yield server
    server.stop()


@pytest.fixture(scope="session")
def plc(ads_server):
    pyads.open_port()
    pyads.set_local_address(PYADS_NET_ID)
    client = Plc()
    yield client
    client.close()
    pyads.close_port()


@pytest.fixture(scope="session")
def ioc(ads_server):
    instance = Ioc("ioc")
    instance.start()
    yield instance
    instance.close()
