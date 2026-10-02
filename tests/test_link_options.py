"""Link options ADSPORT, TS_MS, T_DLY_MS and TIMEBASE, see docs/records.md "Options"."""

import statistics
import time

import pytest

from conftest import PLC_CLOCK_OFFSET


def test_adsport_selects_runtime(plc, ioc):
    plc.write("Main.fPort852", 1.5, port=852)
    ioc.wait_for("Port852:Get", 1.5)


@pytest.fixture(scope="module")
def counter_updates(ioc):
    """Updates of Main.nCycleCounter (changes every ms) with TS_MS=10 and TS_MS=200."""
    with ioc.monitor("CycleCounter:Fast") as fast, ioc.monitor("CycleCounter:Slow") as slow:
        time.sleep(3)
    return {"fast": fast[1:], "slow": slow[1:]}  # the first update is the current value


def plc_stamps(updates):
    return [value.timestamp for _, value in updates]


def test_sample_time_sets_plc_sampling_period(counter_updates):
    fast = plc_stamps(counter_updates["fast"])
    slow = plc_stamps(counter_updates["slow"])
    assert statistics.median(b - a for a, b in zip(fast, fast[1:])) == pytest.approx(0.010, abs=0.003)
    assert statistics.median(b - a for a, b in zip(slow, slow[1:])) == pytest.approx(0.200, abs=0.030)


def test_max_delay_buffers_samples(counter_updates):
    """With T_DLY_MS=500 samples arrive up to 500 ms after they were taken, in bursts."""
    latencies = [arrival - (stamp - PLC_CLOCK_OFFSET) for arrival, stamp in
                 zip((a for a, _ in counter_updates["fast"]), plc_stamps(counter_updates["fast"]))]
    assert 0.3 < max(latencies) < 0.8
    assert len(latencies) / 3 > 50  # several samples per telegram, otherwise this would be ~2/s


@pytest.mark.parametrize(
    "pv, clock_offset",
    [
        ("Timebase:Plc", PLC_CLOCK_OFFSET),
        ("Timebase:Epics", 0.0),
        ("Timebase:Default", PLC_CLOCK_OFFSET),  # argument 11 of adsAsynPortDriverConfigure is 0 (PLC)
        ("Timebase:NoTse", 0.0),  # without TSE=-2 the record stamps itself
    ],
)
def test_timebase(plc, ioc, pv, clock_offset):
    value = time.time() % 1000
    plc.write("Main.fTimebase", value)
    stamp = ioc.wait_for(pv, pytest.approx(value)).timestamp
    assert stamp == pytest.approx(time.time() + clock_offset, abs=2)
