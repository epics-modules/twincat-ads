"""Special symbols .AMSPORTSTATE. and .ADR., see docs/records.md "Special symbols".

Writing .AMSPORTSTATE. stops the runtime, so that is tested in test_connection.py.
"""

import time

import pyads

from conftest import PLC_CLOCK_OFFSET


def test_ams_port_state_reads_run(ioc):
    ioc.wait_for("AmsPortState:Get", pyads.ADSSTATE_RUN)


def test_ams_port_state_uses_epics_timebase(ioc):
    """The driver default is the PLC timebase, whose clock is PLC_CLOCK_OFFSET off in these tests."""
    stamp = ioc.get("AmsPortState:Get").timestamp
    assert abs(stamp - time.time()) < abs(stamp - (time.time() + PLC_CLOCK_OFFSET))


def test_absolute_address_input_follows_plc(plc, ioc):
    plc.write("Main.fAbsolute", 3.25)
    ioc.wait_for("Absolute:Get", 3.25)


def test_absolute_address_output_writes_plc(plc, ioc):
    ioc.put("Absolute:Set", -6.5)
    assert plc.read("Main.fAbsolute") == -6.5
