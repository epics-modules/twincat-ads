#
#    This file is part of epics-twincat-ads.
#
#    epics-twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#
#    epics-twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License along with epics-twincat-ads. If not, see <https://www.gnu.org/licenses/>.
#
# Makefile when running gnu make

ADS_FROM_BECKHOFF_SOURCES = \
  BeckhoffADS/AdsLib/AdsDef.cpp \
  BeckhoffADS/AdsLib/AdsLib.cpp \
  BeckhoffADS/AdsLib/AmsConnection.cpp \
  BeckhoffADS/AdsLib/AmsPort.cpp \
  BeckhoffADS/AdsLib/AmsRouter.cpp \
  BeckhoffADS/AdsLib/Log.cpp \
  BeckhoffADS/AdsLib/NotificationDispatcher.cpp \
  BeckhoffADS/AdsLib/Sockets.cpp \
  BeckhoffADS/AdsLib/Frame.cpp \



# download ADS if needed
build: adsApp/src/ADS_FROM_BECKHOFF_SUPPORTSOURCES.mak checkws

install: adsApp/src/ADS_FROM_BECKHOFF_SUPPORTSOURCES.mak checkws

clean: cleanadssources

checkws:
	./checkws.sh

cleanadssources:
	${PWD}/tools/downloadADS.sh clean ${ADS_FROM_BECKHOFF_SOURCES}


adsApp/src/ADS_FROM_BECKHOFF_SUPPORTSOURCES.mak: Makefile
	${PWD}/tools/downloadADS.sh build ${ADS_FROM_BECKHOFF_SOURCES}

include Makefile.epics

.PHONY: checkws
