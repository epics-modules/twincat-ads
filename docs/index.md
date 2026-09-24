# EPICS TwinCAT-ADS

EPICS module to communicate with Beckhoff TwinCAT controllers over the ADS
protocol.

The module provides an [asyn](https://epics-modules.github.io/asyn/) port
driver, `adsAsynPortDriver`, that maps EPICS records directly onto PLC symbols.
Values can be delivered either through ADS notifications (the PLC pushes
changes) or through periodic polling, configurable per record.

```{card} New here?
Start with {doc}`getting-started` to build the module, set up an ADS route and
link the driver into your IOC.
```

## Contents

| Page                             | Contents                                                          |
| -------------------------------- | ----------------------------------------------------------------- |
| {doc}`getting-started`           | Requirements, build, linking into an IOC, ADS routing.            |
| {doc}`ioc-configuration`         | Startup script and the `iocsh` commands registered by the driver. |
| {doc}`records`                   | Link syntax, options, data types and common record patterns.      |
| {doc}`notifications-and-polling` | Update modes, sample/delay times and timestamps.                  |
| {doc}`troubleshooting`           | Connection supervision, diagnostics and common problems.          |

```{toctree}
:maxdepth: 2
:hidden:

getting-started
ioc-configuration
records
notifications-and-polling
troubleshooting
release-notes
license
```
