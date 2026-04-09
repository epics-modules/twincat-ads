#!../../bin/linux-x86_64/adsExApp

#
#    This file is part of twincat-ads.
#
#    twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#
#    twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License along with twincat-ads. If not, see <https://www.gnu.org/licenses/>.
#

##############################################################################
# Demo file to link an EPICS IOC with some I/O in a TwinCAT plc
#
#  1. Open TwinCAT test project
#  2. The ams address of this linux client must be added to the TwinCAT ads router.
#     In TwinCAT: Systems->routes->add route, use ip of linux machine plus ".1.1"=> "192.168.88.44.1.1"
#  3. Download and start plc(s)
#  4. start ioc on linux machine with: ./st.cmd
#
##############################################################################
< envPaths
cd "${TOP}"

dbLoadDatabase "dbd/adsExApp.dbd"
adsExApp_registerRecordDeviceDriver pdbbase

############# Configure ads device driver:
# 1. Asyn port name                         :  "ADS_1"
# 2. IP                                     :  "192.168.88.44"
# 3. AMS of plc                             :  "192.168.88.44.1.1"
# 4. Default ams port                       :  851 for plc 1, 852 plc 2 ...
# 5. Parameter table size (max parameters)  :  1000
# 6. priority                               :  0
# 7. disable auto connnect                  :  0 (autoconnect enabled)
# 8. default sample time ms                 :  50
# 9. max delay time ms (buffer time in plc) :  100
# 10. ADS command timeout in ms             :  5000
# 11. default time source (PLC=0,EPICS=1)   :  0 (PLC) NOTE: record TSE field need to be set to -2 for timestamp in asyn ("field(TSE, -2)")
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

##############################################################################
############# Load records (asyn direct I/O intr):
dbLoadRecords("db/adsTestAsyn.db","P=ADS_IOC:ASYN:,PORT=${ASYN_PORT},ADSPORT=${ADS_DEFAULT_PORT}")

##############################################################################
############# Useful commands
#asynReport(2,"${ASYN_PORT}")
#asynSetTraceMask("${ASYN_PORT}", -1, 0xFF)
cd "${TOP}/iocBoot/${IOC}"
iocInit
