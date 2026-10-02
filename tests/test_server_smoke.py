"""The ADS test server itself, checked through pyads.

Keeps server problems from being mistaken for driver problems.
"""

import time

import pyads
import pytest


def test_read_write_by_name(plc):
    plc.write("Main.vLREAL", 1.25)
    assert plc.read("Main.vLREAL") == 1.25


def test_unknown_symbol_is_rejected(plc):
    with pytest.raises(pyads.ADSError) as err:
        plc.connection().read_by_name("Main.doesNotExist", pyads.PLCTYPE_LREAL)
    assert err.value.err_code == 0x710  # ADSERR_DEVICE_SYMBOLNOTFOUND


def test_runtimes_on_two_ams_ports(plc):
    assert plc.read("Main.fPort852", port=852) == 852.0
    assert plc.state(port=851) == pyads.ADSSTATE_RUN
    assert plc.state(port=852) == pyads.ADSSTATE_RUN


def test_plc_task_increments_cycle_counter(plc):
    first = plc.read("Main.nCycleCounter")
    time.sleep(0.1)
    assert plc.read("Main.nCycleCounter") > first


def test_clamped_symbol_enforces_limits(plc):
    plc.write("Main.fClamped", 120.0)
    assert plc.read("Main.fClamped") == 100.0
    plc.write("Main.fClamped", 50.0)


def test_sum_read(plc):
    plc.write("Main.vDINT", 17)
    plc.write("Main.vLREAL", 2.5)
    assert plc.connection().read_list_by_name(["Main.vDINT", "Main.vLREAL"]) == {
        "Main.vDINT": 17,
        "Main.vLREAL": 2.5,
    }


def test_notifications_are_pushed(plc):
    received = []
    conn = plc.connection()

    @conn.notification(pyads.PLCTYPE_LREAL)
    def on_change(handle, name, timestamp, value):
        received.append(value)

    handles = conn.add_device_notification("Main.vLREAL", pyads.NotificationAttrib(8), on_change)
    try:
        plc.write("Main.vLREAL", 3.75)
        deadline = time.monotonic() + 2
        while 3.75 not in received and time.monotonic() < deadline:
            time.sleep(0.05)
    finally:
        conn.del_device_notification(*handles)
    assert 3.75 in received
