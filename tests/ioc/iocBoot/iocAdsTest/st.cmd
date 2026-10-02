#!../../bin/linux-x86_64/adsTestIoc

# Test IOC for the pytest suite in tests/, see common.cmd for the settings.

< envPaths
< ${TOP}/iocBoot/${IOC}/common.cmd

dbLoadRecords("db/types.db",        "$(MACROS)")
dbLoadRecords("db/strings.db",      "$(MACROS)")
dbLoadRecords("db/readback.db",     "$(MACROS)")
dbLoadRecords("db/polling.db",      "$(MACROS)")
dbLoadRecords("db/options.db",      "$(MACROS)")
dbLoadRecords("db/special.db",      "$(MACROS)")
dbLoadRecords("db/onlinechange.db", "$(MACROS)")

iocInit
