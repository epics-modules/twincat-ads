#
#    This file is part of twincat-ads.
#
#    twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#
#    twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License along with twincat-ads. If not, see <https://www.gnu.org/licenses/>.
#
# Makefile when running gnu make

ADS_FROM_BECKHOFF_SOURCES = \
BeckhoffADS/example/example.cpp \
BeckhoffADS/AdsLib/Log.cpp \
BeckhoffADS/AdsLib/Frame.cpp \
BeckhoffADS/AdsLib/standalone/AmsRouter.cpp \
BeckhoffADS/AdsLib/standalone/AdsLib.cpp \
BeckhoffADS/AdsLib/standalone/NotificationDispatcher.cpp \
BeckhoffADS/AdsLib/standalone/AmsPort.cpp \
BeckhoffADS/AdsLib/standalone/AmsConnection.cpp \
BeckhoffADS/AdsLib/standalone/AmsNetId.cpp \
BeckhoffADS/AdsLib/AdsFile.cpp \
BeckhoffADS/AdsLib/TwinCAT/AdsLib.cpp \
BeckhoffADS/AdsLib/AdsLib.cpp \
BeckhoffADS/AdsLib/LicenseAccess.cpp \
BeckhoffADS/AdsLib/RTimeAccess.cpp \
BeckhoffADS/AdsLib/Sockets.cpp \
BeckhoffADS/AdsLib/AdsDef.cpp \
BeckhoffADS/AdsLib/RegistryAccess.cpp \
BeckhoffADS/AdsLib/SymbolAccess.cpp \
BeckhoffADS/AdsLib/bhf/ParameterList.cpp \
BeckhoffADS/AdsLib/AdsDevice.cpp \
BeckhoffADS/AdsLib/RouterAccess.cpp \

# Do not include the main programs:
#BeckhoffADS/AdsLibTest/main.cpp \
#BeckhoffADS/AdsLibOOITest/main.cpp \
#BeckhoffADS/AdsTool/main.cpp \
#BeckhoffADS/AdsLibTestRef/main.cpp \


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
