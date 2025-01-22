#
#    This file is part of twincat-ads.
#
#    twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#
#    twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License along with twincat-ads. If not, see <https://www.gnu.org/licenses/>.
#
TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================


#=============================
# Build the IOC application

PROD_IOC = adsExApp
# ads.dbd will be created and installed
DBD += adsExApp.dbd

# ads.dbd will be made up from these files:
adsExApp_DBD += base.dbd
adsExApp_DBD += asyn.dbd
adsExApp_DBD += ads.dbd

# Build the main IOC entry point on workstation OSs.
adsExApp_SRCS += adsExApp_registerRecordDeviceDriver.cpp
adsExApp_SRCS_DEFAULT += adsMain.cpp
adsExApp_SRCS_vxWorks += -nil-

adsExApp_LIBS += asyn
adsExApp_LIBS += ads
# Finally link to the EPICS Base libraries
adsExApp_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


