TOP=../..
include $(TOP)/configure/CONFIG

PROD_IOC = adsTestIoc
DBD += adsTestIoc.dbd

adsTestIoc_DBD += base.dbd
adsTestIoc_DBD += asyn.dbd
adsTestIoc_DBD += ads.dbd
# Serve the records over PVAccess for the p4p based tests
adsTestIoc_DBD += PVAServerRegister.dbd
adsTestIoc_DBD += qsrv.dbd

adsTestIoc_SRCS += adsTestIoc_registerRecordDeviceDriver.cpp
adsTestIoc_SRCS += adsTestIocMain.cpp

adsTestIoc_LIBS += ads
adsTestIoc_LIBS += asyn
adsTestIoc_LIBS += qsrv
adsTestIoc_LIBS += $(EPICS_BASE_PVA_CORE_LIBS)
adsTestIoc_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
