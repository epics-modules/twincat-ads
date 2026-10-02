# Shared setup of the test IOC startup scripts. Settings come from the
# environment (see tests/conftest.py); the defaults match the ADS test server.

cd "${TOP}"

dbLoadDatabase "dbd/adsTestIoc.dbd"
adsTestIoc_registerRecordDeviceDriver pdbbase

adsAsynPortDriverConfigure("ADS_1", "$(PLC_IP=127.0.0.2)", "$(PLC_AMS_NET_ID=127.0.0.2.1.1)", 851, $(PARAM_TABLE_SIZE=1000), 0, 0, $(DEFAULT_SAMPLETIME_MS=50), $(MAX_DELAY_TIME_MS=100), $(ADS_TIMEOUT_MS=2000), $(DEFAULT_TIME_SRC=0))
asynSetTraceMask("ADS_1", -1, 0x41)

epicsEnvSet("MACROS", "P=$(P=ADS_TEST:),PORT=ADS_1,ADSPORT=851")
