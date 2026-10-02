"""PLC arrays as waveform records, see docs/records.md "Arrays"."""

import pytest

from server.symbols import ARRAY_LEN, ARRAY_TYPES

# Per element type: one value per element
VALUES = {
    "SINT": [i - 5 for i in range(ARRAY_LEN)],
    "INT": [-1000 * i for i in range(ARRAY_LEN)],
    "DINT": [-100_000 * i for i in range(ARRAY_LEN)],
    "LINT": [-(2**40) * i for i in range(ARRAY_LEN)],
    "ULINT": [2**40 * i for i in range(ARRAY_LEN)],
    "REAL": [0.5 * i for i in range(ARRAY_LEN)],
    "LREAL": [0.25 * i - 1 for i in range(ARRAY_LEN)],
}


@pytest.mark.parametrize("plc_type", ARRAY_TYPES)
def test_input_follows_plc(plc, ioc, plc_type):
    plc.write(f"Main.a{plc_type}", VALUES[plc_type])
    ioc.wait_for(f"a{plc_type}:Get", pytest.approx(VALUES[plc_type]))


@pytest.mark.parametrize("plc_type", ARRAY_TYPES)
def test_write_only_output_writes_plc(plc, ioc, plc_type):
    plc.write(f"Main.a{plc_type}", [0] * ARRAY_LEN)
    reversed_values = VALUES[plc_type][::-1]
    ioc.put(f"a{plc_type}:Set", reversed_values)
    assert plc.read(f"Main.a{plc_type}") == pytest.approx(reversed_values)
