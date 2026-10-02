"""Output records with info(asyn:READBACK), see docs/records.md "Output with readback".

The test server clamps Main.fClamped and Main.fClampedPolled to [0, 100].
"""

import time

import pytest


def test_output_follows_plc_change(plc, ioc):
    plc.write("Main.fClamped", 42.0)
    ioc.wait_for("Clamped:SetRB", pytest.approx(42.0))


def test_output_writes_plc(plc, ioc):
    ioc.put("Clamped:SetRB", 60.0)
    assert plc.read("Main.fClamped") == 60.0


def test_rejected_write_stays_visible_with_notifications(plc, ioc):
    """Documented pitfall: the PLC keeps 100, no notification fires, the record shows 120."""
    plc.write("Main.fClamped", 100.0)
    ioc.wait_for("Clamped:SetRB", pytest.approx(100.0))
    ioc.put("Clamped:SetRB", 120.0)
    time.sleep(2)
    assert plc.read("Main.fClamped") == 100.0
    assert ioc.get("Clamped:SetRB") == pytest.approx(120.0)


def test_polled_readback_shows_rejected_write(plc, ioc):
    plc.write("Main.fClampedPolled", 100.0)
    ioc.wait_for("ClampedPolled:SetRB", pytest.approx(100.0), timeout=3)
    ioc.put("ClampedPolled:SetRB", 120.0)
    ioc.wait_for("ClampedPolled:SetRB", pytest.approx(100.0), timeout=3)
