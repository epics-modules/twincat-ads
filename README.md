# EPICS Module to communicate with TwinCAT controllers over ADS protocol

Originally developed for use with ESS EPICS Environment, but can also be compiled
for normal EPICS base.

```
$ git clone --recursive https://github.com/epics-modules/epics-twincat-ads
```

Start an e3 ioc with:
```
$ iocsh adsOnlyIO.cmd
$ iocsh adsMotorRecord.cmd
$ iocsh.bash adsMotorRecordOnly.cmd
```

## Prepare communication between IOC and TwinCAT Controller
To enable communication between the IOC and TwinCAT Controller, an ADS route needs to be setup. This can be done in at least three different ways:

1. Login to the PLC remotely. Click on the TwinCAT runtime icon (in the field by the Windows clock) -> Router -> Edit Routes. Add the route through the popup window.

2. Login to the PLC remotely. Edit the C:\TwinCAT\3.1\Target\StaticRoutes.xml manually.
	```
	<Route>
		<Name>epics</Name>
		<Address>192.168.114.129</Address>
		<NetId>192.168.114.129.1.1</NetId>
		<Type>TCP_IP</Type>
		<Flags>32</Flags>
	</Route>
	```
	*Note 1: Update with the correct IP address and AMSNETID.*

	*Note 2: for TwincCAT 4024.0, this is the required method (see https://github.com/Beckhoff/ADS/issues/98).*

3. Use TwinCAT XAE. Click Solution -> SYSTEM -> Routes and add the route through the popup window.




## License information

epics-twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

epics-twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with epics-twincat-ads. If not, see <https://www.gnu.org/licenses/>.

