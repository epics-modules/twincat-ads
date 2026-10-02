"""PLC STRING access, see docs/records.md "Strings"."""

import pytest

LONG_TEXT = "a string longer than the 40 characters of stringin/stringout"


def chars(value) -> str:
    """Text of a CHAR waveform, up to the terminating NUL."""
    return bytes(x & 0xFF for x in value.tolist()).split(b"\0", 1)[0].decode("latin-1")


def test_stringin_follows_plc(plc, ioc):
    plc.write("Main.sString", "hello from the PLC")
    ioc.wait_for("String:Get", "hello from the PLC")


def test_stringout_writes_plc(plc, ioc):
    plc.write("Main.sString", "a previous, longer value")
    ioc.put("String:Set", "short")
    assert plc.read("Main.sString") == "short"


def test_lsi_follows_long_plc_string(plc, ioc):
    plc.write("Main.sString", LONG_TEXT)
    ioc.wait_for("String:GetLong", LONG_TEXT)


def test_lso_writes_long_string(plc, ioc):
    plc.write("Main.sString", "")
    ioc.put("String:SetLong", LONG_TEXT)
    assert plc.read("Main.sString") == LONG_TEXT


def test_char_waveform_follows_plc(plc, ioc):
    plc.write("Main.sString", "as bytes")
    ioc.wait_for("String:GetChars", lambda value: chars(value) == "as bytes")


def test_char_waveform_writes_plc(plc, ioc):
    plc.write("Main.sString", "a previous, longer value")
    ioc.put("String:SetChars", list(b"bytes\0"))
    assert plc.read("Main.sString") == "bytes"


@pytest.mark.parametrize("pv", ["String:Get", "String:GetLong"])
def test_string_records_see_the_same_plc_value(plc, ioc, pv):
    plc.write("Main.sString", "shared")
    ioc.wait_for(pv, "shared")
