"""Connection supervision, see docs/troubleshooting.md "Connection supervision"."""

import pyads
import pytest

pytestmark = pytest.mark.disruptive

INVALID = 3


def is_invalid(value):
    return value.severity == INVALID


def is_valid(value):
    return value.severity == 0


def test_ams_port_state_write_controls_plc(plc, ioc):
    try:
        ioc.put("AmsPortState:Set", pyads.ADSSTATE_STOP)
        assert plc.state() == pyads.ADSSTATE_STOP
        ioc.wait_for("AmsPortState:Get", pyads.ADSSTATE_STOP)
    finally:
        plc.set_state(pyads.ADSSTATE_RUN)
    ioc.wait_for("AmsPortState:Get", pyads.ADSSTATE_RUN)


def test_only_run_state_counts_as_connected(plc, ioc):
    plc.write("Main.vLREAL", 1.0)
    ioc.wait_for("vLREAL:Get", 1.0)
    try:
        plc.set_state(pyads.ADSSTATE_STOP)
        ioc.wait_for("vLREAL:Get", is_invalid)
    finally:
        plc.set_state(pyads.ADSSTATE_RUN)
    ioc.wait_for("vLREAL:Get", is_valid, timeout=10)
    plc.write("Main.vLREAL", 2.0)
    ioc.wait_for("vLREAL:Get", 2.0)


def test_lost_connection_sets_comm_alarm(ads_server, plc, ioc):
    ads_server.stop()
    try:
        value = ioc.wait_for("vLREAL:Get", is_invalid)
        assert value.raw.alarm.message == "COMM"
        assert "ADS_TEST:vLREAL:Get" in ioc.run('dbgrep "*vLREAL*"')  # IOC still alive
    finally:
        plc.close()
        ads_server.start()


def test_reconnects_after_plc_comes_back(plc, ioc):
    """Runs after test_lost_connection_sets_comm_alarm restarted the server; retries every 5 s."""
    ioc.wait_for("vLREAL:Get", is_valid, timeout=15)
    plc.write("Main.vLREAL", 3.0)
    ioc.wait_for("vLREAL:Get", 3.0)
