"""Bulk read (POLL_RATE) and EPICS scanned records, see docs/notifications-and-polling.md."""

import statistics
import time

import pytest


def update_intervals(ioc, pv, duration):
    with ioc.monitor(pv) as updates:
        time.sleep(duration)
    arrivals = [arrival for arrival, _ in updates[1:]]  # the first update is the current value
    return [b - a for a, b in zip(arrivals, arrivals[1:])]


def test_polled_input_follows_plc(plc, ioc):
    plc.write("Main.nPolled", 4711)
    ioc.wait_for("Polled:Get", 4711, timeout=3)


@pytest.mark.parametrize("pv", ["CycleCounter:Polled", "CycleCounter:Polled200ms"])
def test_all_polled_symbols_share_one_1hz_loop(ioc, pv):
    """POLL_RATE only selects bulk read mode; the value is not used (documented known issue)."""
    intervals = update_intervals(ioc, pv, duration=4.5)
    assert len(intervals) >= 2
    assert statistics.median(intervals) == pytest.approx(1.0, abs=0.2)


def test_scanned_record_reads_plc(plc, ioc):
    plc.write("Main.nScanned", 99)
    ioc.wait_for("Scanned:Get", 99, timeout=3)
