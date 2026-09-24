# Getting started

## Requirements

- EPICS base 7.0.3.1 or newer (see note below)
- [asyn](https://github.com/epics-modules/asyn)

```{note}
The module needs `iocInitHookAtShutdown` for properly closing PLC handles on
exit, and this hook was introduced in EPICS base 7.0.3.1.
```

## Build

1. Clone the repository

```bash
git clone --recursive https://github.com/epics-modules/twincat-ads
```

If the repository was cloned without `--recursive`, fetch the submodule
afterwards:

```bash
git submodule update --init --recursive
```

2. Point the build at EPICS base and asyn

Create a `configure/RELEASE.local` file:

```ini
EPICS_BASE=<path-to-your-base>
ASYN=<path-to-asyn>
```

3. Build and install

```bash
make
make install
```

The build produces:

| Artifact              | Description                              |
| --------------------- | ---------------------------------------- |
| `lib/<arch>/libads.*` | The asyn port driver library.            |
| `dbd/ads.dbd`         | Device support and `iocsh` registration. |
| `bin/<arch>/adsExApp` | Example IOC binary.                      |
| `db/adsTestAsyn.db`   | Example database.                        |

## Link the module to your IOC

1. Define `ADS` in your IOC `configure/RELEASE.local` file:

```ini
ADS=<path-to-twincat-ads>
```

2. Add the module and asyn to your IOC application `Makefile`:

```make
myIocApp_DBD  += ads.dbd
myIocApp_LIBS += asyn
myIocApp_LIBS += ads
```

3. Setup your database and ioc startup.

- The records linking PVs to PLC symbols are described in {doc}`records`.
- The startup script is covered in {doc}`ioc-configuration`.

## Configure the PLC route to your IOC host

Before the IOC can talk to the controller, the PLC-side ADS router must know
about the Linux host. Both sides are identified by an _AMS NetId_, which by
convention is the IP address of the machine with `.1.1` appended:

| Peer           | Example IP        | Example AMS NetId     |
| -------------- | ----------------- | --------------------- |
| TwinCAT PLC    | `192.168.88.63`   | `192.168.88.63.1.1`   |
| EPICS IOC host | `192.168.114.129` | `192.168.114.129.1.1` |

The route only has to be added on the PLC. The IOC-side AMS NetId is derived
automatically from the host IP, or can be forced with
{ref}`adsSetLocalAddress <iocsh-adssetlocaladdress>`.

There are at least three ways to add it.

### Method 1 — TwinCAT systray (remote desktop)

Log in to the PLC remotely. Click on the TwinCAT runtime icon in the system tray
next to the Windows clock → **Router** → **Edit Routes**, and add the route in
the pop-up window.

### Method 2 — StaticRoutes.xml

Log in to the PLC remotely and edit `C:\TwinCAT\3.1\Target\StaticRoutes.xml`:

```xml
<Route>
    <Name>epics</Name>
    <Address>192.168.114.129</Address>
    <NetId>192.168.114.129.1.1</NetId>
    <Type>TCP_IP</Type>
    <Flags>32</Flags>
</Route>
```

```{note}
Update the file with the IP address and AMS NetId of the IOC host.
```

```{important}
For TwinCAT 4024.0 this is the _required_ method, see
[Beckhoff/ADS#98](https://github.com/Beckhoff/ADS/issues/98).
```

### Method 3 — TwinCAT XAE

In the engineering environment: **Solution** → **SYSTEM** → **Routes**, then add
the route in the pop-up window.

### Verifying the route

With the route in place, the ADS command-line tool shipped with the submodule
can be used to check connectivity before starting an IOC:

```bash
adstool <plc-ip> --localams=<ioc-ams-netid> state
```

Inside a running IOC, the `.AMSPORTSTATE.` pseudo-symbol reports the same
information, see {doc}`troubleshooting`.
