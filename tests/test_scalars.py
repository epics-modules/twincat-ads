"""Scalar PLC types, see docs/records.md "Scalars"."""

import pytest

from server.symbols import SCALAR_TYPES

# Per type: (value written by the PLC, value written by the IOC)
VALUES = {
    "BOOL": (1, 0),
    "SINT": (-100, 101),
    "USINT": (200, 7),
    "BYTE": (201, 8),
    "INT": (-30000, 30001),
    "UINT": (60000, 9),
    "WORD": (60001, 10),
    "DINT": (-2_000_000_000, 2_000_000_001),
    "UDINT": (2_000_000_000, 11),
    "DWORD": (2_000_000_001, 12),
    "LINT": (-(2**40), 2**41),
    "ULINT": (2**40, 13),
    "REAL": (1.5, -2.25),
    "LREAL": (-1234.5678, 8765.4321),
}


@pytest.mark.parametrize("plc_type", SCALAR_TYPES)
def test_input_follows_plc(plc, ioc, plc_type):
    value, _ = VALUES[plc_type]
    plc.write(f"Main.v{plc_type}", value)
    ioc.wait_for(f"v{plc_type}:Get", pytest.approx(value))


@pytest.mark.parametrize("plc_type", SCALAR_TYPES)
def test_write_only_output_writes_plc(plc, ioc, plc_type):
    first, second = VALUES[plc_type]
    plc.write(f"Main.v{plc_type}", first)
    ioc.put(f"v{plc_type}:Set", second)
    assert plc.read(f"Main.v{plc_type}") == pytest.approx(second)
