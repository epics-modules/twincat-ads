"""Online change (new symbol version) in the PLC, see docs/troubleshooting.md.

The test server invalidates all handles and data notifications of the runtime,
like TwinCAT does after a download or online change.
"""

import time

import pytest

pytestmark = pytest.mark.disruptive


def test_values_flow_after_online_change(plc, ioc):
    plc.online_change()
    time.sleep(1)
    plc.write("Main.fOnlineChange", 11.0)
    ioc.wait_for("OnlineChange:Get", 11.0)
    ioc.put("OnlineChange:Set", 12.0)
    assert plc.read("Main.fOnlineChange") == 12.0
