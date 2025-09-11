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

USR_CXXFLAGS += -std=c++11
USR_CXXFLAGS += -DCONFIG_DEFAULT_LOGLEVEL=1
USR_CPPFLAGS += -I ../../../BeckhoffADS/AdsLib/
USR_CPPFLAGS += -I ../../../BeckhoffADS/AdsLib/bhf/
USR_LDFLAGS  += -lpthread

#=============================
# Build the IOC application

LIBRARY_IOC = ads

DBD += ads.dbd

# Add source files
-include $(TOP)/adsApp/src/ADS_FROM_BECKHOFF_SUPPORTSOURCES.mak

ads_SRCS += adsAsynPortDriver.cpp
ads_SRCS += adsAsynPortDriverUtils.cpp
ads_SRCS += ${ADS_FROM_BECKHOFF_SUPPORTSOURCES}

ads_LIBS += asyn
ads_LIBS += $(EPICS_BASE_IOC_LIBS)


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


