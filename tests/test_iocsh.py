"""iocsh commands, see docs/ioc-configuration.md "iocsh commands"."""

import pytest

from conftest import Ioc

DOCUMENTED_CONFIGURE_ARGUMENTS = [
    "port name",
    "ip-addr",
    "ams-addr",
    "default-ams-port",
    "asyn param table size",
    "priority",
    "disable auto-connect",
    "default sample time ms",
    "max delay time ms",
    "ADS communication timeout ms",
    "default time source",
]


def test_configure_help_lists_documented_arguments(ioc):
    help_text = ioc.run("help adsAsynPortDriverConfigure")
    for argument in DOCUMENTED_CONFIGURE_ARGUMENTS:
        assert argument in help_text
    assert "PLC=0" in help_text and "EPICS=1" in help_text


def test_set_local_address_rejects_short_net_id(ioc):
    assert "local_ams_id parameter required" in ioc.run('adsSetLocalAddress("1.2.3.4")')


def test_startup_prints_bulk_read_time(ioc):
    assert "bulk read time: 1000 ms" in ioc.startup_log


def test_poll_info_lists_all_polled_symbols(ioc):
    output = ioc.run('adsPollInfo("")')
    assert "desired period = 1s" in output
    for symbol in ["MAIN.fbSystemTime.timeLoDW", "MAIN.fbSystemTime.timeHiDW", "Main.nPolled", "Main.fClampedPolled"]:
        assert symbol in output


def test_poll_info_filters_by_name(ioc):
    output = ioc.run('adsPollInfo("nPolled")')
    assert "Main.nPolled" in output
    assert "Main.fClampedPolled" not in output
    assert "fbSystemTime" not in output


def test_asyn_report_lists_parameters(ioc):
    assert "Main.vLREAL" in ioc.run('asynReport(2, "ADS_1")')


@pytest.fixture(scope="module")
def small_table_ioc(ads_server):
    instance = Ioc("ioc_small_table", prefix="ADS_SMALL:", pva_port=15078, env={"PARAM_TABLE_SIZE": "10"})
    instance.start()
    yield instance
    instance.close()


def test_full_parameter_table_is_reported(small_table_ioc):
    assert "Parameter table full" in small_table_ioc.startup_log
    assert "ADS_SMALL:vBOOL:Get" in small_table_ioc.run('dbgrep "*vBOOL*"')
