"""PLC symbol table served by the ADS test server, shared with the tests."""

import struct
from dataclasses import dataclass
from typing import Optional, Tuple

from pyads import constants as c

# TwinCAT type name -> (ADS data type id, struct format of one element)
TYPES = {
    "BOOL": (c.ADST_BIT, "?"),
    "SINT": (c.ADST_INT8, "b"),
    "USINT": (c.ADST_UINT8, "B"),
    "BYTE": (c.ADST_UINT8, "B"),
    "INT": (c.ADST_INT16, "h"),
    "UINT": (c.ADST_UINT16, "H"),
    "WORD": (c.ADST_UINT16, "H"),
    "DINT": (c.ADST_INT32, "i"),
    "UDINT": (c.ADST_UINT32, "I"),
    "DWORD": (c.ADST_UINT32, "I"),
    "LINT": (c.ADST_INT64, "q"),
    "ULINT": (c.ADST_UINT64, "Q"),
    "REAL": (c.ADST_REAL32, "f"),
    "LREAL": (c.ADST_REAL64, "d"),
    "STRING": (c.ADST_STRING, None),
}

DEFAULT_PORT = 851
SECOND_PORT = 852


@dataclass(frozen=True)
class Symbol:
    name: str
    plc_type: str
    count: int = 1  # > 1 means ARRAY [0..count-1] OF plc_type
    initial: object = 0
    strlen: int = 80  # STRING(strlen) only
    limits: Optional[Tuple[float, float]] = None  # the "PLC" clamps written values
    index: Optional[Tuple[int, int]] = None  # fixed (index group, offset)

    @property
    def ads_type(self) -> int:
        return TYPES[self.plc_type][0]

    @property
    def type_name(self) -> str:
        base = f"STRING({self.strlen})" if self.plc_type == "STRING" else self.plc_type
        return f"ARRAY [0..{self.count - 1}] OF {base}" if self.count > 1 else base

    @property
    def size(self) -> int:
        if self.plc_type == "STRING":
            return self.strlen + 1
        return struct.calcsize("<" + TYPES[self.plc_type][1]) * self.count

    def _fmt(self) -> str:
        return f"<{self.count}{TYPES[self.plc_type][1]}"

    def encode(self, value) -> bytes:
        if self.plc_type == "STRING":
            return value.encode("latin-1")[: self.strlen].ljust(self.size, b"\0")
        values = list(value) if self.count > 1 else [value]
        return struct.pack(self._fmt(), *values)

    def decode(self, data: bytes):
        if self.plc_type == "STRING":
            return data.split(b"\0", 1)[0].decode("latin-1")
        values = struct.unpack(self._fmt(), data[: self.size])
        return list(values) if self.count > 1 else values[0]

    def clamp(self, data: bytes) -> bytes:
        if self.limits is None:
            return data
        low, high = self.limits
        return self.encode(min(max(self.decode(data), low), high))

    @property
    def initial_bytes(self) -> bytes:
        if self.plc_type != "STRING" and self.count > 1 and not isinstance(
            self.initial, (list, tuple)
        ):
            return self.encode([self.initial] * self.count)
        return self.encode(self.initial)


SCALAR_TYPES = [t for t in TYPES if t != "STRING"]
ARRAY_TYPES = ["SINT", "INT", "DINT", "LINT", "ULINT", "REAL", "LREAL"]
ARRAY_LEN = 10

SYMBOLS = {
    DEFAULT_PORT: [
        *(Symbol(f"Main.v{t}", t) for t in SCALAR_TYPES),
        *(Symbol(f"Main.a{t}", t, count=ARRAY_LEN) for t in ARRAY_TYPES),
        Symbol("Main.sString", "STRING", initial=""),
        Symbol("Main.fClamped", "LREAL", initial=50.0, limits=(0.0, 100.0)),
        Symbol("Main.fClampedPolled", "LREAL", initial=50.0, limits=(0.0, 100.0)),
        Symbol("Main.nPolled", "DINT"),
        Symbol("Main.nScanned", "DINT"),
        Symbol("Main.fTimebase", "LREAL"),
        Symbol("Main.fOnlineChange", "LREAL"),
        Symbol("Main.fAbsolute", "LREAL", index=(0x4020, 0x10)),
        # Driven by the server's PLC task, see ads_test_server.PlcTask
        Symbol("Main.nCycleCounter", "UDINT"),
        Symbol("MAIN.fbSystemTime.timeLoDW", "UDINT"),
        Symbol("MAIN.fbSystemTime.timeHiDW", "UDINT"),
    ],
    SECOND_PORT: [
        Symbol("Main.fPort852", "LREAL", initial=852.0),
    ],
}


def find(name: str, port: int = DEFAULT_PORT) -> Symbol:
    for sym in SYMBOLS[port]:
        if sym.name.lower() == name.lower():
            return sym
    raise KeyError(f"{name} not defined on port {port}")
