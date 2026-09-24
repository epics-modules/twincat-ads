# Troubleshooting

## Connection supervision

A cyclic driver thread runs every 0.5 s and checks the ADS state of every AMS
port in use. On a state change it:

- marks the port connected/disconnected,
- sets `COMM_ALARM`/`INVALID_ALARM` on all parameters of that port when the
  connection is lost,
- re-registers symbolic handles and notification callbacks when the connection
  comes back,
- reads the device name and AMS router version on (re)connect.

If no AMS port is reachable and autoconnect is enabled (argument 7 of
`adsAsynPortDriverConfigure` set to `0`), the driver retries a full
disconnect/connect cycle every 5 s. The IOC stays alive across a PLC reboot.

```{note}
Older releases (up to v2.1.x) called `exit()` on connection loss, terminating
the IOC. That is no longer the case.
```

## Monitoring the PLC state from EPICS

The `.AMSPORTSTATE.` pseudo-symbol exposes the ADS state of an AMS port as a
`UINT16` parameter, updated by the supervision thread:

```
record(mbbi, "$(P)AmsPortState") {
    field(DTYP, "asynInt32")
    field(INP,  "@asyn($(PORT),0,1)ADSPORT=$(ADSPORT)/.AMSPORTSTATE.?")
    field(SCAN, "I/O Intr")
}
```

Only `ADSSTATE_RUN` counts as "connected" for the supervision logic.

## Bulk read statistics

```bash
adsPollInfo("")
```

prints the desired and measured period of the bulk-read loop, the number of
bulk-read groups, and for each polled symbol the index group, offset, size and
last timestamp. Pass a substring to filter by symbol name. The empty string also
lists the two internal `MAIN.fbSystemTime` entries used for the bulk timestamp.

At startup the driver also prints the loop period:

```text
bulk read time: 1000 ms
```

## asyn tracing

```bash
asynSetTraceMask("ADS_1", -1, 0x41)   # errors + flow (default in st.cmd)
asynSetTraceMask("ADS_1", -1, 0xFF)   # everything
asynReport(2, "ADS_1")                # parameter list and port state
```

`ASYN_TRACE_INFO` carries the connect/disconnect and ADS-state-change messages.

## Common problems

| Symptom                                                         | Likely cause                                                                                                                                                                              |
| --------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| IOC never connects, records `INVALID`/`COMM_ALARM`              | No ADS route on the PLC, or the route uses a different AMS NetId than the IOC. See {doc}`getting-started`; force the IOC NetId with {ref}`adsSetLocalAddress <iocsh-adssetlocaladdress>`. |
| Connects, but one symbol fails at `iocInit`                     | Symbol name misspelled or not exported by the PLC program, or the wrong `ADSPORT`.                                                                                                        |
| Record init fails on a `.ADR.` link                             | Group and offset must both use the `16#` prefix and all four fields must be present.                                                                                                      |
| Output record with readback shows a value the PLC does not have | The PLC clipped or ignored the write and generated no notification. Add `POLL_RATE=1.0/`, see {doc}`notifications-and-polling`.                                                           |
| Timestamps look wrong or jump                                   | `TIMEBASE=PLC` with an unsynchronised PLC clock, or `TSE` is not `-2`. See {ref}`timestamps`.                                                                                             |
| Parameter table full / symbols rejected                         | Increase argument 5 (`asyn param table size`) of `adsAsynPortDriverConfigure`.                                                                                                            |
| Flood of callbacks, high CPU                                    | `TS_MS` too small for the number of subscribed symbols. See {doc}`notifications-and-polling`.                                                                                             |
