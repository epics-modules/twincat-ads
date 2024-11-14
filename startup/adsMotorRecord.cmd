#
#    This file is part of epics-twincat-ads.
#
#    epics-twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#
#    epics-twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License along with epics-twincat-ads. If not, see <https://www.gnu.org/licenses/>.
#

##############################################################################
# Demo file to run one motor record axis (or actually axis record).
#
#  1. Open TwinCAT test project
#  2. The ams address of this linux client must be added to the TwinCAT ads router.
#     In TwinCAT: Systems->routes->add route, use ip of linux machine plus ".1.1"=> "192.168.88.44.1.1"
#  3. Link axis 1 to hardware and make commisioning.
#  4. Ensure that NC axis 1 is linked to Main.M1Link.Axis
#  5. To run motor both limitswitches need to be linked to switeches or set to 1. (Main.bLimitFwd=1,Main.bLimitBwd=1)
#  6. Download and start plc(s)
#  7. start ioc on linux machine with: iocsh adsMotorRecord.cmd
#
##############################################################################
############# Configure ads device driver:
# 1. Asyn port name                          :  "ADS_1"
# 2. IP                                      :  "192.168.88.44"
# 3. AMS of plc                              :  "192.168.88.44.1.1"
# 4. Default ams port                        :  851 for plc 1, 852 plc 2 ...
# 5. Parameter table size (max parameters)   :  1000
# 6. priority                                :  0
# 7. disable auto connnect                   :  0 (autoconnect enabled)
# 8. default sample time ms                  :  500
# 9. max delay time ms (buffer time in plc)  :  1000
# 10. ADS command timeout in ms              :  5000
# 11. default time source (PLC=0,EPICS=1)    :  0 (PLC) NOTE: record TSE field need to be set to -2 for timestamp in asyn ("field(TSE, -2)")

epicsEnvSet(ADS_DEFAULT_PORT, 851)
adsAsynPortDriverConfigure("ADS_1","192.168.88.63","192.168.88.63.1.1",${ADS_DEFAULT_PORT},1000,0,0,50,100,1000,0)

asynOctetSetOutputEos("ADS_1", -1, "\n")
asynOctetSetInputEos("ADS_1", -1, "\n")
asynSetTraceMask("ADS_1", -1, 0x41)

##############################################################################
############# Configure motion (motor record)
EthercatMCCreateController("MCU1", "ADS_1", "32", "200", "1000", "")

epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=ADS_IOC:)")

############# Axis 1:
epicsEnvSet("AXISCONFIG",    "")
epicsEnvSet("EGU",           "mm")
epicsEnvSet("PREC",          "3")
epicsEnvSet("VELO",          "3.0")
epicsEnvSet("JVEL",          "3.0")
#JAR defaults to VELO/ACCL
epicsEnvSet("JAR",           "0.0")
epicsEnvSet("ACCL",          "1")
epicsEnvSet("MRES",          "0.001")
epicsEnvSet("MOTOR_NAME",    "M1")
epicsEnvSet("R",             "M1-")
epicsEnvSet("DESC",          "Motor 1")
epicsEnvSet("AXIS_NO",       "1")
epicsEnvSet("DLLM",          "0")
epicsEnvSet("DHLM",          "0")
epicsEnvSet("HOMEPROC",      "3")

EthercatMCCreateAxis(${MOTOR_PORT}, "${AXIS_NO}", "6", "stepSize=${MRES}")
dbLoadRecords("EthercatMC.template", "PREFIX=$(PREFIX), MOTOR_NAME=$(MOTOR_NAME), R=$(R), MOTOR_PORT=$(MOTOR_PORT), ASYN_PORT=$(ASYN_PORT), AXIS_NO=$(AXIS_NO), DESC=$(DESC), PREC=$(PREC), VELO=$(VELO), JVEL=$(JVEL), JAR=$(JAR), ACCL=$(ACCL), MRES=$(MRES), DLLM=$(DLLM), DHLM=$(DHLM), HOMEPROC=$(HOMEPROC)")
dbLoadRecords("EthercatMChome.template", "PREFIX=${PREFIX}, MOTOR_NAME=${MOTOR_NAME}, MOTOR_PORT=${MOTOR_PORT}, AXIS_NO=${AXIS_NO},HOMEPROC=${HOMEPROC}, HOMEPOS=0, HVELTO=3, HVELFRM=2, HOMEACC=0.1, HOMEDEC=0.01")

############# Axis 2: (Needs to be added in twincat project)
#epicsEnvSet("MOTOR_NAME",    "M2")
#epicsEnvSet("R",             "M2-")
#epicsEnvSet("DESC",          "Motor 2")
#epicsEnvSet("AXIS_NO",       "2")
#EthercatMCCreateAxis(${MOTOR_PORT}, "${AXIS_NO}", "6", "adsPort=851")
#dbLoadRecords("EthercatMC.template", "PREFIX=$(PREFIX), MOTOR_NAME=$(MOTOR_NAME), R=$(R), MOTOR_PORT=$(MOTOR_PORT), ASYN_PORT=$(ASYN_PORT), AXIS_NO=$(AXIS_NO), DESC=$(DESC), PREC=$(PREC), VELO=$(VELO), JVEL=$(JVEL), JAR=$(JAR), ACCL=$(ACCL), MRES=$(MRES), DLLM=$(DLLM), DHLM=$(DHLM), HOMEPROC=$(HOMEPROC)")
#dbLoadRecords("EthercatMChome.template", "PREFIX=${PREFIX}, MOTOR_NAME=${MOTOR_NAME}, MOTOR_PORT=${MOTOR_PORT}, AXIS_NO=${AXIS_NO},HOMEPROC=${HOMEPROC}, HOMEPOS=0, HVELTO=3, HVELFRM=2, HOMEACC=0.1, HOMEDEC=0.01")

##############################################################################
############# Load records (asyn direct I/O intr):
dbLoadRecords("../adsExApp/Db/adsTestAsyn.db","P=ADS_IOC:ASYN:,PORT=ADS_1,ADSPORT=${ADS_DEFAULT_PORT}")

##############################################################################
############# Motor/Axis record error message:
#
# Note: Motor/Axis record will try to read Main.M1.stAxisStatusV2 and use it if accessible. Otherwise fallback on original version ("Main.M1.stAxisStatus").
# The following error message will be displayed at startup if "Main.M1.stAxisStatusV2 "is not accessible (this error will not impact the driver):
#  "adsAsynPortDriver:adsGetSymInfoByName: Get symbolic information failed for Main.M1.stAxisStatusV2 with: ADSERR_DEVICE_SYMBOLNOTFOUND (1808)"
#  "adsApp/src/adsAsynPortDriver.cpp/octetCmdHandleInputLine:1237 motorHandleOneArg returned errorcode: 0x3"
#
##############################################################################
############# Useful commands
#asynReport(2,"ADS_1")
#asynSetTraceMask("ADS_1", -1, 0xFF)
iocInit
