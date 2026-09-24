# Notifications, polling and timestamps

For EPICS input records the driver sets up ADS _notifications_ by default: the
PLC samples the variable every `TS_MS` milliseconds and pushes a telegram
whenever the value changed, buffering for at most `T_DLY_MS`. Adding
`POLL_RATE=1.0/` to the link switches the symbol to the cyclic bulk read
instead.

## When to use which

| Notifications work well for                                     | Notifications work badly for             |
| --------------------------------------------------------------- | ---------------------------------------- |
| Values that change seldom                                       | Noisy sensors, e.g. temperature readings |
| Values that need a PLC timestamp (technically the TwinCAT time) | Very many sensors                        |
|                                                                 | Output records with readback             |

The last point matters: if the PLC silently ignores or clips a written value, no
change notification is generated, so an output record with
`info(asyn:READBACK,"1")` would keep showing the rejected value. Polling always
re-reads the actual PLC value.

```{note}
`POLL_RATE` currently only selects bulk-read mode. All polled symbols share one
loop that runs at 1 Hz (or at the port default sample time if that is larger
than 1000 ms). Symbols larger than 1 MiB cannot be polled.
```

## Choosing parameters

| Goal                                 | Suggested settings                           |
| ------------------------------------ | -------------------------------------------- |
| Slow process value, archive only     | `POLL_RATE=1.0/` or `TS_MS=1000/`            |
| Output readback for slow control     | `POLL_RATE=1.0/` + `info(asyn:READBACK,"1")` |
| Fast transient capture with PLC time | `TIMEBASE=PLC/TS_MS=1/T_DLY_MS=500/`         |
| Noisy analogue sensor                | poll, or filter/deadband in the PLC          |

## A measured example

Two `longin` records read the same encoder counter through notifications, with
different sample times:

```
ADSPORT=852/TIMEBASE=PLC/T_DLY_MS=500/TS_MS=200/GVL_PILS.stEL5101CounterValue.nvalue?
ADSPORT=852/TIMEBASE=PLC/T_DLY_MS=500/TS_MS=1/GVL_PILS.stEL5101CounterValue.nvalue?
```

Running `camonitor` while moving the motor by 1 mm gives:

```text
# First "callback" when the system starts
EncoderRawP1ms   2026-03-17 10:58:56.732280 4787
EncoderRawP200ms 2026-03-17 10:58:56.732286 4787
# Move the motor
EncoderRawP1ms   2026-03-17 10:59:23.448157 4785
EncoderRawP1ms   2026-03-17 10:59:23.488586 4783
EncoderRawP1ms   2026-03-17 10:59:23.518551 4781
EncoderRawP1ms   2026-03-17 10:59:23.528207 4779
EncoderRawP200ms 2026-03-17 10:59:23.557945 4778
EncoderRawP1ms   2026-03-17 10:59:23.558489 4777
EncoderRawP1ms   2026-03-17 10:59:23.598498 4775
EncoderRawP1ms   2026-03-17 10:59:23.628552 4773
EncoderRawP1ms   2026-03-17 10:59:23.638531 4771
EncoderRawP1ms   2026-03-17 10:59:23.669819 4769
EncoderRawP1ms   2026-03-17 10:59:23.699587 4767
EncoderRawP1ms   2026-03-17 10:59:23.729942 4765
EncoderRawP200ms 2026-03-17 10:59:23.758619 4764
EncoderRawP1ms   2026-03-17 10:59:23.777669 4763
EncoderRawP1ms   2026-03-17 10:59:23.807956 4761
EncoderRawP1ms   2026-03-17 10:59:23.837835 4759
EncoderRawP1ms   2026-03-17 10:59:23.858510 4757
EncoderRawP1ms   2026-03-17 10:59:23.897407 4755
EncoderRawP200ms 2026-03-17 10:59:24.028100 4754
```

The 1 ms sampled PV produces a dense stream of callbacks for a single 1 mm move,
while the 200 ms sampled PV combines the motion into a handful of updates. On a
system with many such signals, the 1 ms variant multiplies network traffic,
archiver load and CA/PVA client load for very little extra information.

(timestamps)=

## Timestamps

Each value delivered by the driver carries a timestamp. Which clock it comes
from is selected by the `TIMEBASE` option, falling back to argument 11 of
{ref}`adsAsynPortDriverConfigure <iocsh-adsasynportdriverconfigure>`.

| Timebase | Enum value                | Meaning                                                                                            |
| -------- | ------------------------- | -------------------------------------------------------------------------------------------------- |
| `PLC`    | `ADS_TIME_BASE_PLC = 0`   | The TwinCAT timestamp of the sample, converted from the Windows FILETIME epoch to the EPICS epoch. |
| `EPICS`  | `ADS_TIME_BASE_EPICS = 1` | The IOC time at which the value was received/processed.                                            |

The record must ask asyn for the timestamp with `field(TSE, "-2")`. Without it,
the record timestamps itself when it processes and the `TIMEBASE` option has no
visible effect.

PLC time is the right choice when:

- the value is sampled fast in the PLC and buffered (`T_DLY_MS`), so that
  several samples arrive in one telegram and only the PLC timestamp tells them
  apart;
- EPICS data has to be correlated with PLC-internal logs or other ADS clients.

For PLC time to be meaningful, the PLC clock must be synchronised (NTP or the
EtherCAT distributed clock). An unsynchronised PLC produces timestamps that
drift against the rest of the facility.

Some values never have a PLC timestamp and always use the EPICS timebase
regardless of the option — in particular the `.AMSPORTSTATE.` pseudo-symbol,
which is generated inside the driver.
