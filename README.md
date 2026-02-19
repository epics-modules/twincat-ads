# EPICS TwinCAT-ADS

EPICS Module to communicate with TwinCAT controllers over ADS protocol.

## Install

1. Clone recursively:

```bash
$ git clone --recursive https://github.com/epics-modules/twincat-ads
```

2. Edit configure/RELEASE or create configure/RELEASE.local with:

```ini
EPICS_BASE=<path-to-your-base>
ASYN=<path-to-asyn>
```

3. Build and install:

```bash
make
make install
```

> Note: This module depends on
> [Beckhoff ADS library](https://github.com/Beckhoff/ADS), which is submoduled
> here and built from source.

## Prepare communication between IOC and TwinCAT Controller

To enable communication between the IOC and TwinCAT Controller, an ADS route
needs to be setup. This can be done in at least three different ways:

1. Login to the PLC remotely. Click on the TwinCAT runtime icon (in the field by
   the Windows clock) -> Router -> Edit Routes. Add the route through the popup
   window.

2. Login to the PLC remotely. Edit the C:\TwinCAT\3.1\Target\StaticRoutes.xml
   manually.

   ```
   <Route>
   	<Name>epics</Name>
   	<Address>192.168.114.129</Address>
   	<NetId>192.168.114.129.1.1</NetId>
   	<Type>TCP_IP</Type>
   	<Flags>32</Flags>
   </Route>
   ```

   _Note 1: Update with the correct IP address and AMSNETID._

   _Note 2: for TwincCAT 4024.0, this is the required method (see
   https://github.com/Beckhoff/ADS/issues/98)._

3. Use TwinCAT XAE. Click Solution -> SYSTEM -> Routes and add the route through
   the popup window.

## Usage

This module allows access to ADS Symbols through the following syntax:

```
@asyn(<PORT>,<ADDR>,<TIMEOUT>)<OPTIONS>/<PLC_SYMBOL>[? | =]
```

Where:

- `<PORT>,<ADDR> and <TIMEOUT>`: Standard asyn parameters, see
  [asyn docs](https://epics-modules.github.io/asyn/asynRecord.html#);
- `<OPTIONS>`: Zero or more slash-separated parameters:
  - `ADSPORT=<port>/`: Select AMS/ADS port (default typically 851).

  - `TS_MS=<sample_time_ms>/`: PLC sampling period in milliseconds.

  - `T_DLY_MS=<max_delay_ms>/`: Maximum buffering time before transmission (ms).

  - `TIMEBASE=PLC/` or `TIMEBASE=EPICS/`: Select timestamp source.

  - `POLL_RATE=<seconds>/`: Enable polling mode instead of I/O Intr. Value is
    polling period in seconds.

- `<PLC_SYMBOL>`: PLC variable name;
  - Note: An exception for this is `.AMSPORTSTATE.`, that can be used instead of
    a PLC Symbol to read / write the current ADSPORT state.
- `[? | =]:` End Symbol with `?` for read access (input or output with readback)
  or `=` for write access.

A database with several example records can be found in
[adsExApp/Db/adsTestAsyn.db](adsExApp/Db/adsTestAsyn.db).

The corresponding startup script with necessary configurations is available in
[startup/adsOnlyIO.cmd](startup/adsOnlyIO.cmd).

## License information

twincat-ads is free software: you can redistribute it and/or modify it under the
terms of the GNU Lesser General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along
with twincat-ads. If not, see <https://www.gnu.org/licenses/>.
