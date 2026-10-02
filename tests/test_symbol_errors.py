"""Links that fail at record initialisation, see docs/troubleshooting.md "Common problems".

Uses its own IOC (st_errors.cmd), so failing links cannot affect the other tests.
"""

import pytest

from conftest import Ioc

INVALID = 3


@pytest.fixture(scope="module")
def error_ioc(ads_server):
    instance = Ioc("ioc_errors", script="st_errors.cmd", prefix="ADS_ERR:", pva_port=15077)
    instance.start()
    yield instance
    instance.close()


@pytest.mark.parametrize("pv", ["Error:MissingSymbol", "Error:WrongPort", "Error:MalformedAdr"])
def test_failing_link_leaves_record_invalid(error_ioc, pv):
    assert error_ioc.get(pv).severity == INVALID


def test_failing_links_are_reported_at_init(error_ioc):
    assert "Main.doesNotExist" in error_ioc.startup_log
    assert ".ADR.16#4020,10,8,5" in error_ioc.startup_log


def test_records_after_failing_links_still_work(plc, error_ioc):
    plc.write("Main.fOnlineChange", 7.5)
    error_ioc.wait_for("OnlineChange:Get", 7.5)
    error_ioc.put("OnlineChange:Set", 8.5)
    assert plc.read("Main.fOnlineChange") == 8.5
