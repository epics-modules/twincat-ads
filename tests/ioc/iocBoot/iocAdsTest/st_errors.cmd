#!../../bin/linux-x86_64/adsTestIoc

# Records with links that fail at init, followed by valid ones that must still work.
# Kept apart from st.cmd so a failing link cannot affect the other tests.

< envPaths
< ${TOP}/iocBoot/${IOC}/common.cmd

dbLoadRecords("db/errors.db",       "$(MACROS)")
dbLoadRecords("db/onlinechange.db", "$(MACROS)")

iocInit
