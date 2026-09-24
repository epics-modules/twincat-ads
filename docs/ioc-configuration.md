# IOC configuration

Everything that concerns the connection as a whole is configured in the IOC
startup script. Per-record settings are described in {doc}`records`.

## Startup script

A minimal startup script, parameterised so that only the top block needs
editing:

```bash
dbLoadDatabase "dbd/adsExApp.dbd"
adsExApp_registerRecordDeviceDriver pdbbase

epicsEnvSet("ASYN_PORT",             "ADS_1")
epicsEnvSet("PLC_IP",                "192.168.88.63")
epicsEnvSet("PLC_AMS_NET_ID",        "$(PLC_IP).1.1")
epicsEnvSet("ADS_DEFAULT_PORT",      "851")
epicsEnvSet("PARAM_TABLE_SIZE",      "1000")
epicsEnvSet("PRIORITY",              "0")
epicsEnvSet("DISABLE_AUTOCONNECT",   "0")
epicsEnvSet("DEFAULT_SAMPLETIME_MS", "50")
epicsEnvSet("MAX_DELAY_TIME_MS",     "100")
epicsEnvSet("ADS_TIMEOUT_MS",        "5000")
epicsEnvSet("DEFAULT_TIME_SRC",      "0")

adsAsynPortDriverConfigure(${ASYN_PORT},${PLC_IP},${PLC_AMS_NET_ID},${ADS_DEFAULT_PORT},${PARAM_TABLE_SIZE},${PRIORITY},${DISABLE_AUTOCONNECT},${DEFAULT_SAMPLETIME_MS},${MAX_DELAY_TIME_MS},${ADS_TIMEOUT_MS},${DEFAULT_TIME_SRC})

asynOctetSetOutputEos(${ASYN_PORT}, -1, "\n")
asynOctetSetInputEos(${ASYN_PORT}, -1, "\n")
asynSetTraceMask(${ASYN_PORT}, -1, 0x41)

dbLoadRecords("db/adsTestAsyn.db","P=ADS_IOC:ASYN:,PORT=${ASYN_PORT},ADSPORT=${ADS_DEFAULT_PORT}")

iocInit
```

The complete version is in
[`iocBoot/iocexample/st.cmd`](https://github.com/epics-modules/twincat-ads/blob/master/iocBoot/iocexample/st.cmd),
together with the matching database
[`adsExApp/Db/adsTestAsyn.db`](https://github.com/epics-modules/twincat-ads/blob/master/adsExApp/Db/adsTestAsyn.db).

## `iocsh` commands

Loading `ads.dbd` registers three commands.

(iocsh-adsasynportdriverconfigure)=

### `adsAsynPortDriverConfigure`

Creates one asyn port connected to one TwinCAT controller. Call it once per PLC,
before `iocInit` and before any `dbLoadRecords` that reference the port.

```bash
adsAsynPortDriverConfigure(
    portName, ipAddr, amsAddr, defaultAmsPort, paramTableSize,
    priority, disableAutoConnect, defaultSampleTimeMS, maxDelayTimeMS,
    adsTimeoutMS, defaultTimeSource)
```

| #   | Argument                       | Type   | Example             | Description                                                                                         |
| --- | ------------------------------ | ------ | ------------------- | --------------------------------------------------------------------------------------------------- |
| 1   | `port name`                    | string | `ADS_1`             | asyn port name used in record `INP`/`OUT` links.                                                    |
| 2   | `ip-addr`                      | string | `192.168.88.63`     | IP address of the TwinCAT controller.                                                               |
| 3   | `ams-addr`                     | string | `192.168.88.63.1.1` | AMS NetId of the controller.                                                                        |
| 4   | `default-ams-port`             | int    | `851`               | Default ADS port of the runtime holding the symbols. Overridable per record with `ADSPORT=`.        |
| 5   | `asyn param table size`        | int    | `1000`              | Maximum number of PLC symbols (asyn parameters) for this port.                                      |
| 6   | `priority`                     | int    | `0`                 | asyn port thread priority (`0` = default).                                                          |
| 7   | `disable auto-connect`         | int    | `0`                 | `0` = asyn autoconnect enabled, `1` = disabled.                                                     |
| 8   | `default sample time ms`       | int    | `50`                | Default PLC-side notification sample period, overridable with `TS_MS=`.                             |
| 9   | `max delay time ms`            | int    | `100`               | Default maximum time the PLC buffers notifications before sending, overridable with `T_DLY_MS=`.    |
| 10  | `ADS communication timeout ms` | int    | `5000`              | Timeout for synchronous ADS requests.                                                               |
| 11  | `default time source`          | int    | `0`                 | `0` = PLC (`ADS_TIME_BASE_PLC`), `1` = EPICS (`ADS_TIME_BASE_EPICS`). Overridable with `TIMEBASE=`. |

```{note}
Records that take their timestamp from the driver must set `field(TSE, "-2")`.
See {ref}`timestamps`.
```

(iocsh-adssetlocaladdress)=

### `adsSetLocalAddress`

```bash
adsSetLocalAddress("192.168.114.129.1.1")
```

| Argument       | Type   | Description                                                          |
| -------------- | ------ | -------------------------------------------------------------------- |
| `local_ams_id` | string | AMS NetId the IOC presents to the PLC router, in `A.B.C.D.E.F` form. |

Use this when the host has several network interfaces, or when the AMS NetId
registered on the PLC does not match the automatically derived one. The string
must be at least 11 characters long; shorter values are rejected with a message.
Call it before `adsAsynPortDriverConfigure`.

(iocsh-adspollinfo)=

### `adsPollInfo`

```bash
adsPollInfo("")           # all polled symbols
adsPollInfo("fAmplitude") # only symbols whose name contains "fAmplitude"
```

| Argument | Type   | Description                                             |
| -------- | ------ | ------------------------------------------------------- |
| `name`   | string | Substring filter on the PLC symbol name, empty for all. |

Prints diagnostics for the bulk-read (polling) loop, see {doc}`troubleshooting`.
