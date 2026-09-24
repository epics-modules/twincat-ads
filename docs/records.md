# Records

The driver lets EPICS PVs peek and poke PLC variables. There is **no
handshake**: the PLC never acknowledges or rejects a write. If a write must be
confirmed, that protocol has to be built in the PLC code.

When a written value is picked up depends on how the PLC program is written and
on the PLC cycle time. The PLC may clip the value to a minimum/maximum, round it
to an integer, or ignore it entirely.

Because of that, it is often worth using more than one PLC variable per
quantity:

| Purpose              | PLC variable           | EPICS record  | Example PV          |
| -------------------- | ---------------------- | ------------- | ------------------- |
| Setpoint             | `Main.fAmplitudeSet`   | output record | `…SetFAmplitude_S`  |
| Readback of setpoint | `Main.fAmplitudeSetRB` | input record  | `…SetFAmplitude_RB` |
| Actual value         | `Main.fAmplitudeAct`   | input record  | `…FAmplitude_Act`   |

Example: the user enters 99.99, the PLC rounds it to 100.0 because that is what
the hardware can do, and while ramping the actual amplitude moves from 50.0 to
100.0. Only three variables make all three facts visible.

A related problem appears when the PLC has its own HMI (touch panel) or an
engineering tool is connected: the `_S` and `_RB` PVs diverge, which is
confusing and potentially dangerous. The readback pattern below addresses this
by letting the output record follow the PLC.

The example database
[`adsExApp/Db/adsTestAsyn.db`](https://github.com/epics-modules/twincat-ads/blob/master/adsExApp/Db/adsTestAsyn.db)
demonstrates every pattern described here.

## Link syntax

PLC symbols are addressed from the record `INP` or `OUT` field:

```
@asyn(<PORT>,<ADDR>,<TIMEOUT>)<OPTIONS>/<PLC_SYMBOL>[? | =]
```

| Part                      | Meaning                                                                                                                                 |
| ------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| `<PORT>,<ADDR>,<TIMEOUT>` | Standard asyn parameters, see the [asyn documentation](https://epics-modules.github.io/asyn/asynRecord.html). `<ADDR>` is normally `0`. |
| `<OPTIONS>`               | Zero or more slash-separated `KEY=VALUE` options (see below).                                                                           |
| `<PLC_SYMBOL>`            | The PLC variable name, e.g. `Main.fAmplitude`.                                                                                          |
| `?` or `=`                | Access direction.                                                                                                                       |

Everything after the last `/` is treated as the symbol name, so options must
always be terminated by `/`.

| Suffix | Behaviour                                                                                                                                              |
| ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `?`    | The driver reads the symbol. Used for input records, and for output records that should follow the PLC value — combine with `info(asyn:READBACK,"1")`. |
| `=`    | Write only. The driver never reads the symbol back.                                                                                                    |

### Options

All option names are case sensitive and each must end with `/`.

| Option                               | Default                                     | Description                                                                                                                                                                                       |
| ------------------------------------ | ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ADSPORT=<port>/`                    | Argument 4 of `adsAsynPortDriverConfigure`  | ADS port of the runtime holding the symbol.                                                                                                                                                       |
| `TS_MS=<ms>/`                        | Argument 8 of `adsAsynPortDriverConfigure`  | PLC-side sample period in milliseconds. The PLC checks the variable for changes at this rate. Accepts a floating point value.                                                                     |
| `T_DLY_MS=<ms>/`                     | Argument 9 of `adsAsynPortDriverConfigure`  | Maximum time the PLC buffers notifications before transmitting them. A larger value means fewer, larger ADS telegrams and more latency. Up to `T_DLY_MS / TS_MS` samples can arrive in one burst. |
| `TIMEBASE=PLC/` or `TIMEBASE=EPICS/` | Argument 11 of `adsAsynPortDriverConfigure` | Source of the record timestamp. Unrecognised values leave the default unchanged, see {ref}`timestamps`.                                                                                           |
| `POLL_RATE=<seconds>/`               | not set (notifications)                     | Switch this symbol from ADS notifications to the cyclic bulk read, see {doc}`notifications-and-polling`.                                                                                          |

```{warning}
Known issue: `POLL_RATE` currently only acts as a flag: it moves the symbol into the bulk read group. The requested value is stored but the whole bulk loop runs either at a
single period (1 s), or the port default sample time when that is larger than
1000 ms. Per-record poll rates are not implemented yet. Symbols larger than
1 MiB are never added to the bulk read.
```

### Special symbols

#### `.AMSPORTSTATE.`

Used instead of a PLC symbol to read or write the state of the AMS port itself.
The value is maintained inside the driver, is of type `UINT16`, and always uses
the EPICS timebase.

```
record(mbbi, "$(P)AmsPortState") {
    field(DTYP, "asynInt32")
    field(INP,  "@asyn($(PORT),0,1)ADSPORT=$(ADSPORT)/.AMSPORTSTATE.?")
    field(SCAN, "I/O Intr")
}
```

Writing to it issues an ADS write-control request, which can be used to
start/stop the PLC runtime.

#### `.ADR.` — absolute addressing

Access an index group/offset directly, bypassing the symbol table:

```
<options>/.ADR.16#<group>,16#<offset>,<size>,<type>[? | =]
```

| Field    | Format          | Meaning                                                |
| -------- | --------------- | ------------------------------------------------------ |
| `group`  | hex after `16#` | ADS index group.                                       |
| `offset` | hex after `16#` | ADS index offset.                                      |
| `size`   | decimal         | Size in bytes.                                         |
| `type`   | decimal         | ADS data type id (`ADST_*`, e.g. `5` = `ADST_REAL64`). |

```
ADSPORT=$(ADSPORT)/.ADR.16#5001,16#D,8,5?
```

Both the group and the offset must be written with the `16#` prefix; a malformed
`.ADR.` string is rejected when the record is initialised.

## Data types

The driver exposes the following asyn interfaces, so the matching `DTYP` values
are available on both input and output (including `I/O Intr`):

`asynInt32`, `asynInt64`, `asynFloat64`, `asynOctet`, `asynInt8Array`,
`asynInt16Array`, `asynInt32Array`, `asynFloat32Array`, `asynFloat64Array`.

```{note}
`asynInt64` is only compiled in when the EPICS base / asyn combination provides
it (it is disabled by `NO_ADS_ASYN_ASYNPARAMINT64`).
```

### Scalars

| PLC type (`ADST_…`)                                    | Typical `DTYP`                     | Typical record                                  |
| ------------------------------------------------------ | ---------------------------------- | ----------------------------------------------- |
| `BOOL` / `ADST_BIT`                                    | `asynInt32`                        | `bi`, `bo`                                      |
| `SINT`, `USINT`, `BYTE` (`ADST_INT8`, `ADST_UINT8`)    | `asynInt32`                        | `ai`, `ao`, `longin`, `longout`                 |
| `INT`, `UINT`, `WORD` (`ADST_INT16`, `ADST_UINT16`)    | `asynInt32`                        | `ai`, `ao`, `longin`, `longout`, `mbbi`, `mbbo` |
| `DINT`, `UDINT`, `DWORD` (`ADST_INT32`, `ADST_UINT32`) | `asynInt32`                        | `longin`, `longout`                             |
| `LINT`, `ULINT` (`ADST_INT64`, `ADST_UINT64`)          | `asynInt64`                        | `int64in`, `int64out`                           |
| `REAL` (`ADST_REAL32`)                                 | `asynFloat64`                      | `ai`, `ao`                                      |
| `LREAL` (`ADST_REAL64`)                                | `asynFloat64`                      | `ai`, `ao`                                      |
| `STRING` (`ADST_STRING`)                               | `asynOctetRead` / `asynOctetWrite` | `stringin`, `stringout`, `lsi`, `lso`           |

### Arrays

PLC arrays are mapped to `waveform` records. `NELM` must be at least the number
of PLC elements and `FTVL` must match the element type.

| PLC element type         | `DTYP` (in / out)                            | `FTVL`   |
| ------------------------ | -------------------------------------------- | -------- |
| `BYTE`, `SINT`, `STRING` | `asynInt8ArrayIn` / `asynInt8ArrayOut`       | `CHAR`   |
| `INT`, `WORD`            | `asynInt16ArrayIn` / `asynInt16ArrayOut`     | `SHORT`  |
| `DINT`, `DWORD`          | `asynInt32ArrayIn` / `asynInt32ArrayOut`     | `LONG`   |
| `REAL`                   | `asynFloat32ArrayIn` / `asynFloat32ArrayOut` | `FLOAT`  |
| `LREAL`                  | `asynFloat64ArrayIn` / `asynFloat64ArrayOut` | `DOUBLE` |

```
record(waveform,"$(P)GetFTestArray"){
    field(PINI, "1")
    field(TSE,  "-2")
    field(DTYP, "asynFloat64ArrayIn")
    field(INP,  "@asyn($(PORT),0,1)ADSPORT=$(ADSPORT)/Main.fTestArray?")
    field(NELM, "100")
    field(FTVL, "DOUBLE")
    field(SCAN, "I/O Intr")
}
```

```{note}
Waveform _output_ records use the `INP` field (not `OUT`) for the asyn link;
this is standard asyn behaviour for `waveform`.
```

### Strings

A PLC `STRING` can be accessed as a byte array:

```
record(waveform,"$(P)GetSTest"){
    field(DTYP, "asynInt8ArrayIn")
    field(INP,  "@asyn($(PORT),0,1)ADSPORT=$(ADSPORT)/Main.sTest?")
    field(NELM, "100")
    field(FTVL, "CHAR")
    field(SCAN, "I/O Intr")
}
```

or, since v2.2.0, as a native EPICS string through
`asynOctetRead`/`asynOctetWrite`, without client-side formatting:

| Record                   | Limit                                                           |
| ------------------------ | --------------------------------------------------------------- |
| `stringin` / `stringout` | up to 40 characters (EPICS string size)                         |
| `lsi` / `lso`            | longer strings; `SIZV` must be at least the PLC string size + 1 |

```
record(lsi,"$(P)GetSTestLsi"){
    field(DTYP, "asynOctetRead")
    field(INP,  "@asyn($(PORT),0,1)ADSPORT=$(ADSPORT)/Main.sTest?")
    field(SIZV, "100")
    field(SCAN, "I/O Intr")
}
```

## Common patterns

The examples use the macros of the example database: `$(P)` for the PV prefix,
`$(PORT)` for the asyn port and `$(ADSPORT)` for the ADS port.

### Plain input (notification driven)

The default: the PLC pushes changes, the record processes on `I/O Intr`.

```
record(ai,"$(P)GetFAmplitude"){
    field(PINI, "1")
    field(TSE,  "-2")
    field(DTYP, "asynFloat64")
    field(INP,  "@asyn($(PORT),0,1)ADSPORT=$(ADSPORT)/Main.fAmplitude?")
    field(PREC, "3")
    field(SCAN, "I/O Intr")
}
```

### Write-only output

Note the `=` suffix.

```
record(ao,"$(P)SetFAmplitude"){
    field(PINI, "1")
    field(TSE,  "-2")
    field(DTYP, "asynFloat64")
    field(OUT,  "@asyn($(PORT),0,1)ADSPORT=$(ADSPORT)/Main.fAmplitude=")
    field(PREC, "3")
    field(SCAN, "Passive")
}
```

### Output with readback

`info(asyn:READBACK,"1")` lets the driver update the `VAL` field of an output
record when the PLC value changes, keeping the PV in sync with a PLC-side HMI or
with values the PLC has clipped. The link ends with `?`, not `=`.

```
record(ao,"$(P)SetFAmplitudeRB"){
    field(PINI, "1")
    field(TSE,  "-2")
    field(DTYP, "asynFloat64")
    field(OUT,  "@asyn($(PORT),0,1)ADSPORT=$(ADSPORT)/Main.fAmplitude?")
    field(PREC, "3")
    field(SCAN, "Passive")

    info(asyn:READBACK,"1")
}
```

```{warning}
With notifications this can be tricky for slow control. If the current amplitude
is 50, the maximum is 100, and EPICS writes 120, the PLC may silently ignore the
write. No callback is fired, so the record keeps showing 120 while the PLC uses
50. Add `POLL_RATE=1.0/` in that case to force a PLC-side poll.
```

### Output with polled readback (recommended for slow control)

```
record(ao,"$(P)SetFAmplitudeRB"){
    field(PINI, "1")
    field(TSE,  "-2")
    field(DTYP, "asynFloat64")
    field(OUT,  "@asyn($(PORT),0,1)ADSPORT=$(ADSPORT)/POLL_RATE=1.0/Main.fAmplitude?")
    field(PREC, "3")
    field(SCAN, "Passive")

    info(asyn:READBACK,"1")
}
```

### Fast values with explicit sample and delay times

```
record(ai,"$(P)GetFTestPLCTime"){
    field(PINI, "1")
    field(TSE,  "-2")
    field(DTYP, "asynFloat64")
    field(INP,  "@asyn($(PORT),0,1)TIMEBASE=PLC/T_DLY_MS=500/TS_MS=10/ADSPORT=$(ADSPORT)/Main.fTest?")
    field(PREC, "3")
    field(SCAN, "I/O Intr")
}
```

`TS_MS=10` samples in the PLC every 10 ms, `T_DLY_MS=500` lets the PLC buffer up
to 500 ms of data. Up to 50 values may therefore arrive in a single burst every
500 ms. See {doc}`notifications-and-polling`.

### Periodic scan instead of `I/O Intr`

The record can also be scanned by EPICS. Each processing triggers a synchronous
ADS read.

```
record(ai,"$(P)GetICycleCounterSCAN"){
    field(PINI, "1")
    field(TSE,  "-2")
    field(DTYP, "asynInt32")
    field(INP,  "@asyn($(PORT),0,1)ADSPORT=$(ADSPORT)/Main.iCycleCounter?")
    field(SCAN, "1 second")
}
```

### Exposing the timestamp as a string

Useful to display the (PLC) timestamp of a value on an operator screen.

```
record(stringin, "$(P)GetFTestPLCTime:T") {
    field(DTYP, "Soft Timestamp")
    field(TSEL, "$(P)GetFTestPLCTime.TIME CP")
    field(INP,  "@%b %d, %Y %H:%M:%S.%09f")
}
```
