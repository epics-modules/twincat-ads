"""ADS test server emulating a TwinCAT PLC for the twincat-ads driver tests.

Socket handling builds on pyads' AdsTestServer. Request handling is our own:
the pyads handlers neither push notifications over the wire nor answer sum
reads in the layout AdsLib expects.

Emulated behaviour beyond plain read/write:
- ADS notifications are sampled every cycle time and buffered up to max delay,
  like a real runtime does.
- A PLC task increments Main.nCycleCounter and MAIN.fbSystemTime every ms.
- Writing ADSIGRP_SYM_VERSION simulates an online change: all handles and
  data notifications of that runtime become invalid.
- Write control changes the ADS state; notifications pause outside RUN.

Run from the tests directory: python -m server.ads_test_server
"""

import argparse
import itertools
import logging
import select
import signal
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, NamedTuple, Optional, Tuple

from pyads import constants as c
from pyads.testserver.testserver import ADS_PORT, AdsTestServer

from server.symbols import SYMBOLS, Symbol

log = logging.getLogger("ads_test_server")

ADSERR_DEVICE_SRVNOTSUPP = 0x701
ADSERR_DEVICE_SYMBOLNOTFOUND = 0x710
ADSERR_DEVICE_NOTIFYHNDINVALID = 0x714
GLOBALERR_TARGET_PORT = 0x6

SECONDS_1601_TO_1970 = 11644473600
STATE_REQUEST = 0x0004
STATE_RESPONSE = 0x0005

AMS_TCP_HEADER = struct.Struct("<HI")
# target netid, target port, source netid, source port, command, state, length, error, invoke id
AOE_HEADER = struct.Struct("<6sH6sHHHIII")


def u32(value: int) -> bytes:
    return struct.pack("<I", value)


class AdsError(Exception):
    def __init__(self, code: int):
        super().__init__(hex(code))
        self.code = code


class Request(NamedTuple):
    command: int
    port: int
    data: bytes
    client: Tuple[bytes, int]  # (netid, port) of the requester
    server: Tuple[bytes, int]  # (netid, port) the request was sent to


class Variable:
    def __init__(self, sym: Symbol, runtime: "Runtime", index_group: int, index_offset: int):
        self.sym = sym
        self.runtime = runtime
        self.index_group = index_group
        self.index_offset = index_offset
        self.value = sym.initial_bytes
        self.handle = 0

    def packed_info(self) -> bytes:
        """AdsSymbolEntry as returned for ADSIGRP_SYM_INFOBYNAMEEX."""
        name = self.sym.name.encode()
        type_name = self.sym.type_name.encode()
        comment = b""
        strings = name + b"\0" + type_name + b"\0" + comment + b"\0"
        header = struct.Struct("<IIIIIIHHH")
        return (
            header.pack(
                header.size + len(strings),
                self.index_group,
                self.index_offset,
                self.sym.size,
                self.sym.ads_type,
                0,
                len(name),
                len(type_name),
                len(comment),
            )
            + strings
        )


class Runtime:
    """One PLC runtime (AMS port) with its symbols, handles and ADS state."""

    DATA_AREA = 0x4040

    def __init__(self, port: int, symbols: List[Symbol], handle_ids):
        self.port = port
        self.state = c.ADSSTATE_RUN
        self._handle_ids = handle_ids
        self.version = Variable(Symbol("<symbol version>", "USINT"), self, c.ADSIGRP_SYM_VERSION, 0)
        self.by_index = {(c.ADSIGRP_SYM_VERSION, 0): self.version}
        self.by_name: Dict[str, Variable] = {}
        offset = 0
        for sym in symbols:
            index = sym.index or (self.DATA_AREA, offset)
            offset += sym.size
            var = Variable(sym, self, *index)
            self.by_index[index] = var
            self.by_name[sym.name.lower()] = var
        self.by_handle: Dict[int, Variable] = {}
        self.assign_handles()

    def assign_handles(self) -> None:
        self.by_handle = {}
        for var in self.by_name.values():
            var.handle = next(self._handle_ids)
            self.by_handle[var.handle] = var

    def resolve(self, index_group: int, index_offset: int) -> Variable:
        if index_group == c.ADSIGRP_SYM_VALBYHND:
            var = self.by_handle.get(index_offset)
        else:
            var = self.by_index.get((index_group, index_offset))
        if var is None:
            raise AdsError(ADSERR_DEVICE_SYMBOLNOTFOUND)
        return var

    def lookup(self, name: bytes) -> Variable:
        var = self.by_name.get(name.rstrip(b"\0").decode().lower())
        if var is None:
            raise AdsError(ADSERR_DEVICE_SYMBOLNOTFOUND)
        return var


@dataclass
class Notification:
    handle: int
    var: Variable
    conn: "Connection"
    request: Request
    cycle: float
    max_delay: float
    next_check: float = 0.0
    last_value: Optional[bytes] = None
    pending: List[Tuple[int, bytes]] = field(default_factory=list)
    pending_since: float = 0.0


class Plc:
    """State of the emulated PLC; every access happens under self.lock."""

    def __init__(self, clock_offset: float = 0.0):
        self.lock = threading.RLock()
        self.clock_offset = clock_offset
        handle_ids = itertools.count(1)
        self.runtimes = {port: Runtime(port, syms, handle_ids) for port, syms in SYMBOLS.items()}
        # Notification handles must be unique per server: AdsLib dispatches on them
        self._notification_ids = itertools.count(1)
        self.notifications: Dict[int, Notification] = {}

    def filetime(self) -> int:
        """PLC clock in 100 ns since 1601-01-01 (Windows FILETIME)."""
        return int((time.time() + self.clock_offset + SECONDS_1601_TO_1970) * 10_000_000)

    def handle(self, req: Request, conn: "Connection") -> Tuple[int, bytes]:
        """Return (AoE error code, response payload)."""
        runtime = self.runtimes.get(req.port)
        if runtime is None:
            return GLOBALERR_TARGET_PORT, b""
        handlers = {
            c.ADSCOMMAND_READDEVICEINFO: self._read_device_info,
            c.ADSCOMMAND_READ: self._read,
            c.ADSCOMMAND_WRITE: self._write,
            c.ADSCOMMAND_READWRITE: self._read_write,
            c.ADSCOMMAND_READSTATE: self._read_state,
            c.ADSCOMMAND_WRITECTRL: self._write_control,
            c.ADSCOMMAND_ADDDEVICENOTE: self._add_notification,
            c.ADSCOMMAND_DELDEVICENOTE: self._del_notification,
        }
        try:
            handler = handlers.get(req.command)
            if handler is None:
                raise AdsError(ADSERR_DEVICE_SRVNOTSUPP)
            with self.lock:
                return 0, u32(0) + handler(runtime, req, conn)
        except AdsError as err:
            log.debug("port %d command %d failed: %s", req.port, req.command, err)
            # READ and READWRITE responses always carry a length field
            length = u32(0) if req.command in (c.ADSCOMMAND_READ, c.ADSCOMMAND_READWRITE) else b""
            return 0, u32(err.code) + length

    def _read_device_info(self, rt: Runtime, req: Request, conn) -> bytes:
        return struct.pack("<BBH16s", 3, 1, 4024, b"TestServer")

    def _read_state(self, rt: Runtime, req: Request, conn) -> bytes:
        return struct.pack("<HH", rt.state, 0)

    def _write_control(self, rt: Runtime, req: Request, conn) -> bytes:
        rt.state, _ = struct.unpack_from("<HH", req.data)
        log.info("port %d: ADS state -> %d", rt.port, rt.state)
        return b""

    def _read(self, rt: Runtime, req: Request, conn) -> bytes:
        index_group, index_offset, length = struct.unpack_from("<III", req.data)
        value = rt.resolve(index_group, index_offset).value[:length]
        return u32(len(value)) + value

    def _write(self, rt: Runtime, req: Request, conn) -> bytes:
        index_group, index_offset, length = struct.unpack_from("<III", req.data)
        if index_group == c.ADSIGRP_SYM_RELEASEHND:
            return b""
        self.store(rt.resolve(index_group, index_offset), req.data[12 : 12 + length])
        return b""

    def store(self, var: Variable, data: bytes) -> None:
        # A short write only replaces the leading bytes, like on a real PLC
        var.value = var.sym.clamp(data[: var.sym.size] + var.value[len(data) :])
        if var is var.runtime.version:
            self._online_change(var.runtime)

    def _online_change(self, rt: Runtime) -> None:
        log.info("port %d: online change, handles and notifications invalidated", rt.port)
        rt.assign_handles()
        self.notifications = {
            h: n
            for h, n in self.notifications.items()
            if n.var.runtime is not rt or n.var is rt.version
        }

    def _read_write(self, rt: Runtime, req: Request, conn) -> bytes:
        index_group, index_offset, read_length, write_length = struct.unpack_from("<IIII", req.data)
        write_data = req.data[16 : 16 + write_length]
        if index_group == c.ADSIGRP_SYM_HNDBYNAME:
            result = u32(rt.lookup(write_data).handle)
        elif index_group == c.ADSIGRP_SYM_INFOBYNAMEEX:
            result = rt.lookup(write_data).packed_info()
        elif index_group == c.ADSIGRP_SUMUP_READ:
            result = self._sum_read(rt, index_offset, write_data)
        else:
            raise AdsError(ADSERR_DEVICE_SRVNOTSUPP)
        result = result[:read_length]
        return u32(len(result)) + result

    def _sum_read(self, rt: Runtime, count: int, write_data: bytes) -> bytes:
        """All error codes first, then the data of the successful reads."""
        errors, values = b"", b""
        for i in range(count):
            index_group, index_offset, size = struct.unpack_from("<III", write_data, 12 * i)
            try:
                value = rt.resolve(index_group, index_offset).value[:size]
                errors += u32(0)
                values += value.ljust(size, b"\0")
            except AdsError as err:
                errors += u32(err.code)
        return errors + values

    def _add_notification(self, rt: Runtime, req: Request, conn) -> bytes:
        index_group, index_offset, _length, _mode, max_delay, cycle = struct.unpack_from("<IIIIII", req.data)
        handle = next(self._notification_ids)
        self.notifications[handle] = Notification(
            handle=handle,
            var=rt.resolve(index_group, index_offset),
            conn=conn,
            request=req,
            cycle=max(cycle / 1e7, 0.001),
            max_delay=max_delay / 1e7,
        )
        return u32(handle)

    def _del_notification(self, rt: Runtime, req: Request, conn) -> bytes:
        (handle,) = struct.unpack_from("<I", req.data)
        if self.notifications.pop(handle, None) is None:
            raise AdsError(ADSERR_DEVICE_NOTIFYHNDINVALID)
        return b""

    def tick(self) -> None:
        """One cycle of the PLC task: update task variables, sample notifications."""
        with self.lock:
            now = time.monotonic()
            filetime = self.filetime()
            self._run_task(filetime)
            for handle, n in list(self.notifications.items()):
                if not n.conn.alive:
                    del self.notifications[handle]
                    continue
                if n.var.runtime.state != c.ADSSTATE_RUN:
                    continue
                if now >= n.next_check:
                    n.next_check = max(n.next_check + n.cycle, now)
                    if n.var.value != n.last_value:
                        if not n.pending:
                            n.pending_since = now
                        n.pending.append((filetime, n.var.value))
                        n.last_value = n.var.value
                if n.pending and now - n.pending_since >= n.max_delay:
                    self._send_notification(n)

    def _run_task(self, filetime: int) -> None:
        rt = self.runtimes[851]
        counter = rt.by_name["main.ncyclecounter"]
        counter.value = u32((struct.unpack("<I", counter.value)[0] + 1) & 0xFFFFFFFF)
        rt.by_name["main.fbsystemtime.timelodw"].value = u32(filetime & 0xFFFFFFFF)
        rt.by_name["main.fbsystemtime.timehidw"].value = u32(filetime >> 32)

    def _send_notification(self, n: Notification) -> None:
        stamps = b"".join(
            struct.pack("<QI", stamp, 1) + struct.pack("<II", n.handle, len(value)) + value
            for stamp, value in n.pending
        )
        payload = struct.pack("<II", 4 + len(stamps), len(n.pending)) + stamps
        n.pending = []
        # DEVICENOTE goes from the notified runtime back to the registering client
        n.conn.send(n.request.client, n.request.server, c.ADSCOMMAND_DEVICENOTE, STATE_REQUEST, payload)


class Connection(threading.Thread):
    """One TCP client (e.g. the IOC or a pyads client)."""

    def __init__(self, sock: socket.socket, plc: Plc):
        super().__init__(daemon=True)
        self.sock = sock
        self.plc = plc
        self.alive = True
        self._send_lock = threading.Lock()

    def run(self) -> None:
        try:
            while True:
                _, length = AMS_TCP_HEADER.unpack(self._recv(AMS_TCP_HEADER.size))
                self._dispatch(self._recv(length))
        except OSError:
            pass
        finally:
            self.alive = False
            self.sock.close()

    def close(self) -> None:
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

    def _recv(self, size: int) -> bytes:
        data = b""
        while len(data) < size:
            chunk = self.sock.recv(size - len(data))
            if not chunk:
                raise ConnectionResetError("client closed connection")
            data += chunk
        return data

    def _dispatch(self, frame: bytes) -> None:
        t_net, t_port, s_net, s_port, command, state, length, _err, invoke = AOE_HEADER.unpack_from(frame)
        if state & 0x1:
            return  # a response, e.g. to a DEVICENOTE
        data = frame[AOE_HEADER.size : AOE_HEADER.size + length]
        req = Request(command, t_port, data, client=(s_net, s_port), server=(t_net, t_port))
        log.debug("port %d command %d data %s", t_port, command, data[:16].hex())
        error, payload = self.plc.handle(req, self)
        self.send(req.client, req.server, command, STATE_RESPONSE, payload, error, invoke)

    def send(self, target, source, command, state, payload, error=0, invoke=0) -> None:
        header = AOE_HEADER.pack(*target, *source, command, state, len(payload), error, invoke)
        frame = AMS_TCP_HEADER.pack(0, len(header) + len(payload)) + header + payload
        try:
            with self._send_lock:
                self.sock.sendall(frame)
        except OSError:
            self.alive = False


class TestServer(AdsTestServer):
    def __init__(self, plc: Plc, ip_address: str, port: int):
        super().__init__(ip_address=ip_address, port=port, logging=False)
        self.plc = plc

    def run(self) -> None:
        self.server.listen(5)
        while self._run:
            ready, _, _ = select.select([self.server], [], [], 0.1)
            if ready:
                sock, address = self.server.accept()
                log.info("client connected from %s:%d", *address)
                conn = Connection(sock, self.plc)
                conn.start()
                self.clients.append(conn)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    # Not 127.0.0.1: a local TwinCAT router (e.g. via WSL mirrored networking) may own 48898 there
    parser.add_argument("--host", default="127.0.0.2")
    parser.add_argument("--port", type=int, default=ADS_PORT)
    parser.add_argument("--clock-offset", type=float, default=0.0, help="PLC clock offset to host time [s]")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    plc = Plc(clock_offset=args.clock_offset)
    server = TestServer(plc, args.host, args.port)
    server.start()
    log.info("listening on %s:%d", args.host, args.port)

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    while not stop.wait(0.001):
        plc.tick()
    server.close()


if __name__ == "__main__":
    main()
