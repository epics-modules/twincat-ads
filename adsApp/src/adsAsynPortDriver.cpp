//#define MCB_DEBUG
/*
* adsAsynPortDriver.cpp
*
* Class derived of asynPortDriver for ADS communication with TwinCAT plcs.
* AdsLib written by Beckhoff is used for communication: https://github.com/Beckhoff/ADS
*
* Author: Anders Sandström
* Edited to add bulk reads: Michael Browne
* Edited to add bulk symbol resolution: Yann Stephen Mandza
* Edited to resolve de-sync I/O Intr issues: Nicholas Lentz
* Edited to add static code analysis and CI pre-commit workflows: Yann Stephen Mandza
* Edited to Upgades ADS to release 113.0.32-1: Yann Stephen Mandza
* Edited to Stop trusting absent PLC timestamp in bulk reads: Yann Stephen Mandza
*
* Created January 25, 2018
* Edited  December 6, 2019
* Edited  March 10, 2026
* Edited  March 11, 2026
*/
#define USE_TYPED_RSET // Shut up about rset already!
#include "adsAsynPortDriver.h"

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <sys/time.h>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <iocsh.h>
#include <initHooks.h>

#include <epicsExport.h>
#include <dbStaticLib.h>
#include <dbAccess.h>
#include <alarm.h>

#include <fstream>
#include <algorithm>
#include <unordered_map>

static const char* driverName = "adsAsynPortDriver";
static adsAsynPortDriver* adsAsynPortObj;
static long oldTimeStamp               = 0;
static struct timeval oldTime          = {0};
static int allowCallbackEpicsState     = 0;
static initHookState currentEpicsState = initHookAtIocBuild;
static struct timeval s_iocStartTime; /* captured at initHookAtIocBuild */

#ifdef ADS_UNIT_TEST
#include <atomic>
std::atomic<int> g_callbackCount{0};

void adsAsynPortDriver::setGlobalInstance(adsAsynPortDriver* obj)
{
    adsAsynPortObj = obj;
}
#endif
/* drvInfo record-info cache
 *
 * getRecordInfoFromDrvInfo() initially walked the entire EPICS database
 * for every drvUserCreate() call — O(N²) in the number of records.
 *
 * The cache below is built once (O(N)) on the first call and consulted in
 * O(1) on every subsequent call.
 *
*/
struct RecordInfoCache
{
    std::string recordType;
    std::string recordName;
    std::string inp;
    std::string out;
    std::string dtyp;
    asynParamType asynType = asynParamNotDefined;
};


static std::unordered_map<std::string, RecordInfoCache> s_drvInfoCache;
static bool s_drvInfoCacheBuilt = false;

static void buildDrvInfoCache()
{
    if (s_drvInfoCacheBuilt)
        return;
    /* DB not loaded yet — caller will retry */
    if (!pdbbase)
        return;
    /* set only after confirming pdbbase is valid */
    s_drvInfoCacheBuilt = true;

    DBENTRY* pdb = dbAllocEntry(pdbbase);
    long st      = dbFirstRecordType(pdb);
    while (!st)
    {
        std::string rtype = dbGetRecordTypeName(pdb);
        long sr           = dbFirstRecord(pdb);
        while (!sr)
        {
            if (!dbIsAlias(pdb))
            {
                std::string rname = dbGetRecordName(pdb);

                std::string dtyp_str;
                asynParamType atype = asynParamNotDefined;
                if (!dbFindField(pdb, "DTYP"))
                {
                    const char* dtypVal = dbGetString(pdb);
                    if (dtypVal)
                    {
                        dtyp_str = dtypVal;

                        char dtypBuf[256] = {};
                        strncpy(dtypBuf, dtypVal, sizeof(dtypBuf) - 1);
                        atype = dtypStringToAsynType(dtypBuf);
                    }
                }

                /* parse "@asyn(port,adr,tmo)drvInfo" from INP or OUT field.
                 * Returns (port, drvInfo) strings, or ("","") if not an asyn link.
                 */
                auto tryLink = [&](const char* field)
                    -> std::pair<std::pair<std::string, std::string>, std::string>
                {
                    typedef std::pair<std::pair<std::string, std::string>, std::string> R;
                    if (dbFindField(pdb, field))
                        return R(std::make_pair(std::string(), std::string()), std::string());
                    const char* val = dbGetString(pdb);
                    if (!val)
                        return R(std::make_pair(std::string(), std::string()), std::string());
                    std::string linkStr(val);
                    char port[256] = {}, drvI[1024] = {};
                    int adr = 0, tmo = 0;
                    if (sscanf(val, "@asyn(%255[^,],%d,%d)%1023s", port, &adr, &tmo, drvI) == 4)
                        return R(std::make_pair(std::string(port), std::string(drvI)), linkStr);
                    return R(std::make_pair(std::string(), std::string()), std::string());
                };

                /* INP link */
                std::pair<std::pair<std::string, std::string>, std::string> inpRes = tryLink("INP");
                const std::string& inpPort = inpRes.first.first;
                const std::string& inpDrv  = inpRes.first.second;
                const std::string& inpStr  = inpRes.second;

                /* OUT link */
                std::pair<std::pair<std::string, std::string>, std::string> outRes = tryLink("OUT");
                const std::string& outPort = outRes.first.first;
                const std::string& outDrv  = outRes.first.second;
                const std::string& outStr  = outRes.second;

                /* store INP entry — link string already captured, no re-query */
                if (!inpPort.empty())
                {
                    std::string key = inpPort + '\0' + inpDrv;
                    if (!s_drvInfoCache.count(key))
                    {
                        RecordInfoCache e;
                        e.recordType        = rtype;
                        e.recordName        = rname;
                        e.dtyp              = dtyp_str;
                        e.asynType          = atype;
                        e.inp               = inpStr;
                        s_drvInfoCache[key] = std::move(e);
                    }
                }

                /* store OUT entry — link string already captured, no re-query */
                if (!outPort.empty())
                {
                    std::string key = outPort + '\0' + outDrv;
                    if (!s_drvInfoCache.count(key))
                    {
                        RecordInfoCache e;
                        e.recordType        = rtype;
                        e.recordName        = rname;
                        e.dtyp              = dtyp_str;
                        e.asynType          = atype;
                        e.out               = outStr;
                        s_drvInfoCache[key] = std::move(e);
                    }
                }
            }
            sr = dbNextRecord(pdb);
        }
        st = dbNextRecordType(pdb);
    }
    dbFreeEntry(pdb);
}


/** Callback hook for EPICS state.
 * \param[in] state EPICS state
 * \return void
 * Will be called be the EPICS framework with the current EPICS state as it changes.
 */
static void getEpicsState(initHookState state)
{
    const char* functionName = "getEpicsState";
    static struct timeval start;
    struct timeval now, diff;

    if (!adsAsynPortObj)
    {
        printf("%s:%s: ERROR: adsAsynPortObj==NULL\n", driverName, functionName);
        return;
    }

    asynUser* asynTraceUser = adsAsynPortObj->getTraceAsynUser();

    switch (state)
    {
        break;
    case initHookAtIocBuild:
        gettimeofday(&s_iocStartTime, NULL);
        break;
    case initHookAfterInitDevSup:
        gettimeofday(&start, NULL);
        break;
    case initHookAfterInitDatabase:
        gettimeofday(&now, NULL);
        timersub(&now, &start, &diff);
        printf("Database initialization took %ld.%06ld seconds.\n", diff.tv_sec, diff.tv_usec);
        break;
    case initHookAfterScanInit:
        allowCallbackEpicsState = 1;

        if (!adsAsynPortObj)
        {
            printf("%s:%s: ERROR: adsAsynPortObj==NULL\n", driverName, functionName);
            return;
        }

        adsAsynPortObj->bulkOK = 1;
        printf("Begin polling PLC!\n");
        break;
    case initHookAfterIocRunning:
        gettimeofday(&now, NULL);
        timersub(&now, &s_iocStartTime, &diff);
        printf("IOC fully running — total startup time %ld.%06ld seconds.\n",
               diff.tv_sec,
               diff.tv_usec);
        break;
    default:
        break;
    }

    currentEpicsState = state;
    asynPrint(asynTraceUser,
              ASYN_TRACEIO_DRIVER,
              "%s:%s: EPICS state: %s (%d). Allow ADS callbacks: %s.\n",
              driverName,
              functionName,
              epicsStateToString((int)state),
              (int)state,
              allowCallbackEpicsState ? "true" : "false");
}

/** Register EPICS hook function
 * \return void
 */
int initHook(void)
{
    return (initHookRegister(getEpicsState));
}

/** Callback from ads lib for symbols changed in PLC.
 * \param[in] pAddr AmsAddr of the system generating the callback.
 * \param[in] pNotification Data structure containing the updated data and timestamp information.
 * \param[in] hUser Identification index of the callback parameter.
 * \return void
 * This function will be called by the ADS lib if the symbol version in the PLC is changed.
 */
static void adsSymbolsChangedCallback(const AmsAddr* pAddr,
                                      const AdsNotificationHeader* pNotification,
                                      uint32_t hUser)
{
    const char* functionName = "adsSymbolsChangedCallback";

    if (!adsAsynPortObj)
    {
        printf("%s:%s: ERROR: adsAsynPortObj==NULL\n", driverName, functionName);
        return;
    }

    asynUser* asynTraceUser = adsAsynPortObj->getTraceAsynUser();

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*adsAsynPortObj, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = adsAsynPortObj->getAdsClientPortNumberForThreadId(0);
        asynPrint(asynTraceUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    asynPrint(asynTraceUser,
              ASYN_TRACE_INFO,
              "%s:%s: Symbols changed for Ams-port %u.\n",
              driverName,
              functionName,
              pAddr->port);

    adsAsynPortObj->invalidateParamsLock(pAddr->port);
    adsAsynPortObj->refreshParamsLock(amsClientPort, pAddr->port);
}

/** Callback from ads lib for updated data.
 * \param[in] pAddr AmsAddr of the system generating the callback.
 * \param[in] pNotification Data structure containing the updated data and timestamp information.
 * \param[in] hUser Identification index of the callback parameter.
 * \return void
 * This function will be called by the ADS lib when a registered parameter is updated (changed in PLC).
 */
static void
adsDataCallback(const AmsAddr* pAddr, const AdsNotificationHeader* pNotification, uint32_t hUser)
{
    const char* functionName = "adsDataCallback";
    if (!adsAsynPortObj)
    {
        printf("%s:%s: ERROR: adsAsynPortObj==NULL\n", driverName, functionName);
        return;
    }

    asynUser* asynTraceUser = adsAsynPortObj->getTraceAsynUser();
    asynPrint(
        asynTraceUser, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s:%s:\n", driverName, functionName);

    const uint8_t* data = reinterpret_cast<const uint8_t*>(pNotification + 1);
    struct timeval newTime;
    gettimeofday(&newTime, NULL);

    asynPrint(asynTraceUser,
              ASYN_TRACEIO_DRIVER,
              "TIME %ld.%06ld\n",
              (long)newTime.tv_sec,
              (long)newTime.tv_usec);

    long secs_used   = (newTime.tv_sec - oldTime.tv_sec); //avoid overflow by subtracting first
    long micros_used = ((secs_used * 1000000) + newTime.tv_usec) - (oldTime.tv_usec);
    oldTime          = newTime;

    //Ensure hUser is within range
    if (hUser > (uint32_t)(adsAsynPortObj->getParamTableSize() - 1))
    {
        asynPrint(asynTraceUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: hUser out of range: %u.\n",
                  driverName,
                  functionName,
                  hUser);
        return;
    }

    //Get paramInfo
    adsParamInfo* paramInfo = adsAsynPortObj->getAdsParamInfo(hUser);
    if (!paramInfo)
    {
        asynPrint(asynTraceUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: getAdsParamInfo() for hUser %u failed\n",
                  driverName,
                  functionName,
                  hUser);
        return;
    }

    if (adsAsynPortObj->datacbqueue.size() > MAXCBQSIZE)
    {
        asynPrint(asynTraceUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: datacbqueue at max size, skip %s (%d)\n",
                  driverName,
                  functionName,
                  paramInfo->drvInfo,
                  paramInfo->paramIndex);
        return;
    }

    asynPrint(asynTraceUser,
              ASYN_TRACEIO_DRIVER,
              "Process callback for parameter %s (%d).\n",
              paramInfo->drvInfo,
              paramInfo->paramIndex);
    asynPrint(asynTraceUser,
              ASYN_TRACEIO_DRIVER,
              "hUser 0x%x, data size[b]: %d.\n",
              hUser,
              pNotification->cbSampleSize);
    asynPrint(
        asynTraceUser,
        ASYN_TRACEIO_DRIVER,
        "time stamp [100ns]: %ld, since last plc [ms]: %4.2lf, since last ioc [ms]: %4.2lf.\n",
        pNotification->nTimeStamp,
        ((double)(pNotification->nTimeStamp - oldTimeStamp)) / 10000.0,
        (((double)(micros_used)) / 1000.0));
    oldTimeStamp = pNotification->nTimeStamp;

    //Ensure hUser is equal to parameter index
    if (hUser != (uint32_t)(paramInfo->paramIndex))
    {
        asynPrint(asynTraceUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: hUser not equal to parameter index (%u vs %d).\n",
                  driverName,
                  functionName,
                  hUser,
                  paramInfo->paramIndex);
        return;
    }

    // Copy data into memory here because the data pointer will not stay valid forever
    // This malloc is freed in dataCallbackThread

    void* local_data = malloc(pNotification->cbSampleSize);
    if (!local_data)
    {
        asynPrint(asynTraceUser,
                  ASYN_TRACE_ERROR,
                  "%s: malloc failed in data callback — dropping notification\n",
                  driverName);
        return;
    }
    memcpy(local_data, data, pNotification->cbSampleSize);
    adsAsynPortDriver::datacbinfo cbinfo = {paramInfo, local_data, *pNotification};
    adsAsynPortObj->datacbqueue.push(cbinfo);
} // cppcheck-suppress memleak

/** Start cyclic thread for supervision of connection.
 * \param[in] drvPvt adsAsynPortDriver object
 * \return void
 */
void cyclicThread(void* drvPvt)
{
    adsAsynPortDriver* pPvt = (adsAsynPortDriver*)drvPvt;
    pPvt->cyclicThread();
}

/** Start bulk read thread.
 * \param[in] drvPvt adsAsynPortDriver object
 * \return void
 */
void bulkReadThread(void* drvPvt)
{
    adsAsynPortDriver* pPvt = (adsAsynPortDriver*)drvPvt;
    pPvt->bulkReadThread();
}

/** Start data callback thread
 * \param[in] drvPvt adsAsynPortDriver object
 * \return void
 */
void dataCallbackThread(void* drvPvt)
{
    adsAsynPortDriver* pPvt = (adsAsynPortDriver*)drvPvt;
    pPvt->dataCallbackThread();
}

/** Start trigger IO Intr Callbacks thread
 * \param[in] drvPvt adsAsynPortDriver object
 * \return void
 */
void triggerEpicsIoIntrCallbacksThread(void* drvPvt)
{
    adsAsynPortDriver* pPvt = (adsAsynPortDriver*)drvPvt;
    pPvt->triggerEpicsIoIntrCallbacksThread();
}

/*
 * SUMUP handle resolution
 *
 * Issues a single ADS SUMUP write+read to resolve handles for every symbol
 * in symbolDict_ in one RTT.
 *
 * ADS SUMUP protocol:
 *   Group  : ADSIGRP_SUMUP_READWRITE  (0xF081)
 *   Offset : number of sub-requests   (N)
 *   Write  : N × (iGroup=0xF003, iOffs=nameLen, readLen=4)
 *            followed by N × name strings (no null terminator needed)
 *   Read   : N × (result_code uint32 + handle uint32)  = N×8 bytes
 * */
asynStatus adsAsynPortDriver::resolveSymbolHandles(uint16_t amsClientPort)
{
    static const char* functionName = "resolveSymbolHandles";
    if (symbolDict_.empty())
        return asynSuccess;

    /* collect entries in stable order */
    std::vector<AdsSymbolDictEntry*> entries;
    entries.reserve(symbolDict_.size());
    for (auto& kv : symbolDict_)
        entries.push_back(&kv.second);

    const size_t N          = entries.size();
    const size_t CHUNK_SIZE = 500;
    const size_t nChunks    = (N + CHUNK_SIZE - 1) / CHUNK_SIZE;

    size_t totalResolved = 0;

    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "%s::%s: resolving %zu handles in %zu chunk(s) of max %zu\n",
              driverName,
              functionName,
              N,
              nChunks,
              CHUNK_SIZE);

    for (size_t chunk = 0; chunk < nChunks; ++chunk)
    {
        const size_t chunkStart = chunk * CHUNK_SIZE;
        const size_t chunkEnd   = std::min(chunkStart + CHUNK_SIZE, N);
        const size_t chunkN     = chunkEnd - chunkStart;

        /* build write buffer */

        size_t namesTotal = 0;
        for (size_t i = chunkStart; i < chunkEnd; ++i)
            namesTotal += entries[i]->symbol.size();

        const size_t headerBytes = chunkN * 16;
        const size_t writeBytes  = headerBytes + namesTotal;

        /* read buffer layout */
        const size_t readBytes = chunkN * 12;

        std::vector<uint8_t> writeBuf(writeBytes, 0);
        std::vector<uint8_t> readBuf(readBytes, 0);

        uint8_t* hdr  = writeBuf.data();
        uint8_t* name = writeBuf.data() + headerBytes;

        for (size_t i = chunkStart; i < chunkEnd; ++i)
        {
            uint32_t iGroup   = ADSIGRP_SYM_HNDBYNAME;
            uint32_t iOffs    = 0;
            uint32_t readLen  = 4;
            uint32_t writeLen = (uint32_t)entries[i]->symbol.size();
            memcpy(hdr, &iGroup, 4);
            hdr += 4;
            memcpy(hdr, &iOffs, 4);
            hdr += 4;
            memcpy(hdr, &readLen, 4);
            hdr += 4;
            memcpy(hdr, &writeLen, 4);
            hdr += 4;
            memcpy(name, entries[i]->symbol.data(), entries[i]->symbol.size());
            name += entries[i]->symbol.size();
        }

        /* ADS SUMUP READWRITE call */
        AmsAddr remote;
        remote.netId = remoteNetId_;
        remote.port  = amsportDefault_; // Needs to chunk based on amsServer port but it isn't.
        // ^ This assumes everything is ams server port 851 which is probably almost everything but not quite.

        uint32_t bytesRead = 0;

        long rc = AdsSyncReadWriteReqEx2(amsClientPort,
                                         &remote,
                                         ADSIGRP_SUMUP_READWRITE, /* 0xF082 */
                                         (uint32_t)chunkN, /* iOffset = number of sub-requests */
                                         (uint32_t)readBytes,
                                         readBuf.data(),
                                         (uint32_t)writeBytes,
                                         writeBuf.data(),
                                         &bytesRead);

        asynPrint(pasynUserSelf,
                  ASYN_TRACE_FLOW,
                  "%s::%s: chunk %zu/%zu rc=0x%lx "
                  "bytesRead=%u expected=%zu\n",
                  driverName,
                  functionName,
                  chunk + 1,
                  nChunks,
                  rc,
                  bytesRead,
                  readBytes);

        if (rc != 0)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s::%s: chunk %zu/%zu SUMUP failed rc=0x%lx — "
                      "skipping chunk, ADS fallback for %zu symbols\n",
                      driverName,
                      functionName,
                      chunk + 1,
                      nChunks,
                      rc,
                      chunkN);
            continue; /* leave all symbols in chunk with resolved=false */
        }

        /* extract handles */
        const uint8_t* statusBlock = readBuf.data();
        const uint32_t* handleBlock =
            reinterpret_cast<const uint32_t*>(readBuf.data() + chunkN * 8);

        for (size_t i = 0; i < chunkN; ++i)
        {
            uint32_t result, retLen;
            memcpy(&result, statusBlock + i * 8 + 0, 4);
            memcpy(&retLen, statusBlock + i * 8 + 4, 4);
            uint32_t handle = handleBlock[i];

            asynPrint(pasynUserSelf,
                      ASYN_TRACE_FLOW,
                      "%s::%s: sub-req[%zu] '%s' "
                      "result=0x%x retLen=%u handle=0x%x\n",
                      driverName,
                      functionName,
                      chunkStart + i,
                      entries[chunkStart + i]->symbol.c_str(),
                      result,
                      retLen,
                      handle);

            if (result == 0 && handle != 0)
            {
                entries[chunkStart + i]->handle   = handle;
                entries[chunkStart + i]->resolved = true;
                ++totalResolved;
            }
            else
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s::%s: handle resolution failed for '%s' "
                          "result=0x%x retLen=%u handle=0x%x — ADS fallback\n",
                          driverName,
                          functionName,
                          entries[chunkStart + i]->symbol.c_str(),
                          result,
                          retLen,
                          handle);
            }
        }

        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s::%s: chunk %zu/%zu done — %zu/%zu resolved\n",
                  driverName,
                  functionName,
                  chunk + 1,
                  nChunks,
                  totalResolved,
                  N);
    }

    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "%s::%s: complete — %zu/%zu handles resolved, %zu will use ADS fallback\n",
              driverName,
              functionName,
              totalResolved,
              N,
              N - totalResolved);

    return asynSuccess;
}

/*
 * resolveSymbolInfo()
 *
 * Populates symbolDict_ directly from TwinCAT for every ADS symbol
 * referenced by a loaded DB record.  Issues a batched SUMUP_READWRITE
 * using ADSIGRP_SYM_INFOBYNAMEEX (0xF009) as the sub-request group.
 * TwinCAT returns the full AdsSymbolEntry for each symbol.
 *
 * Called once from drvUserCreate() after all DB records are loaded.
 * */
asynStatus adsAsynPortDriver::resolveSymbolInfo(uint16_t amsClientPort)
{
    static const char* functionName = "resolveSymbolInfo";

    symbolDict_.clear();

    /* Step 1: collect symbols from the DB cache */
    buildDrvInfoCache();

    std::vector<std::string> symbols;
    symbols.reserve(s_drvInfoCache.size());

    for (const auto& kv : s_drvInfoCache)
    {
        const std::string& key = kv.first;
        size_t sep             = key.find('\0');
        if (sep == std::string::npos)
            continue;
        std::string entryPort = key.substr(0, sep);
        std::string drvI      = key.substr(sep + 1);

        if (entryPort != std::string(portName))
            continue;

        if (drvI.find(".ADR.") != std::string::npos)
            continue;
        if (drvI.find(".AMSPORTSTATE.") != std::string::npos)
            continue;

        /* extract symbol name — last '/'-delimited token, trailing '?'/'=' stripped */
        std::string sym;
        size_t slash = drvI.rfind('/');
        if (slash != std::string::npos)
            sym = drvI.substr(slash + 1);
        else
            sym = drvI;

        if (!sym.empty() && (sym.back() == '?' || sym.back() == '='))
            sym.resize(sym.size() - 1);

        if (sym.empty())
            continue;

        symbols.push_back(sym);
    }

    if (symbols.empty())
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s::%s: no symbols to resolve (DB cache empty or all overridden)\n",
                  driverName,
                  functionName);
        return asynSuccess;
    }

    {
        std::unordered_map<std::string, int> seen;
        std::vector<std::string> unique;
        unique.reserve(symbols.size());
        for (auto& s : symbols)
        {
            std::string sl = s;
            std::transform(
                sl.begin(), sl.end(), sl.begin(), [](unsigned char c) { return std::tolower(c); });
            if (!seen.count(sl))
            {
                seen[sl] = 1;
                unique.push_back(s);
            }
        }
        symbols = std::move(unique);
    }

    const size_t N          = symbols.size();
    const size_t CHUNK_SIZE = 500;
    const size_t nChunks    = (N + CHUNK_SIZE - 1) / CHUNK_SIZE;
    size_t totalOK          = 0;
    size_t totalFail        = 0;

    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "%s::%s: resolving info for %zu symbols in %zu chunk(s)\n",
              driverName,
              functionName,
              N,
              nChunks);

    /* Step 2: batch SUMUP INFOBYNAMEEX */
    const size_t entrySize = sizeof(adsSymbolEntry);

    AmsAddr remote;
    remote.netId = remoteNetId_;
    remote.port  = amsportDefault_; // Needs to chunk based on amsServer port but it isn't.
    // ^ This assumes everything is ams server port 851 which is probably almost everything but not quite.

    for (size_t chunk = 0; chunk < nChunks; ++chunk)
    {
        const size_t chunkStart = chunk * CHUNK_SIZE;
        const size_t chunkEnd   = std::min(chunkStart + CHUNK_SIZE, N);
        const size_t chunkN     = chunkEnd - chunkStart;

        /* build write buffer */
        size_t namesTotal = 0;
        for (size_t i = chunkStart; i < chunkEnd; ++i)
            namesTotal += symbols[i].size();

        const size_t headerBytes = chunkN * 16;
        const size_t writeBytes  = headerBytes + namesTotal;
        const size_t readBytes   = chunkN * 8          /* status block */
                                 + chunkN * entrySize; /* data  block  */

        std::vector<uint8_t> writeBuf(writeBytes, 0);
        std::vector<uint8_t> readBuf(readBytes, 0);

        uint8_t* hdr  = writeBuf.data();
        uint8_t* name = writeBuf.data() + headerBytes;

        for (size_t i = chunkStart; i < chunkEnd; ++i)
        {
            uint32_t iGroup   = ADSIGRP_SYM_INFOBYNAMEEX;
            uint32_t iOffs    = 0;
            uint32_t readLen  = (uint32_t)entrySize;
            uint32_t writeLen = (uint32_t)symbols[i].size();
            memcpy(hdr, &iGroup, 4);
            hdr += 4;
            memcpy(hdr, &iOffs, 4);
            hdr += 4;
            memcpy(hdr, &readLen, 4);
            hdr += 4;
            memcpy(hdr, &writeLen, 4);
            hdr += 4;
            memcpy(name, symbols[i].data(), symbols[i].size());
            name += symbols[i].size();
        }

        /* ADS call */
        uint32_t bytesRead = 0;

        long rc = AdsSyncReadWriteReqEx2(amsClientPort,
                                         &remote,
                                         ADSIGRP_SUMUP_READWRITE,
                                         (uint32_t)chunkN,
                                         (uint32_t)readBytes,
                                         readBuf.data(),
                                         (uint32_t)writeBytes,
                                         writeBuf.data(),
                                         &bytesRead);

        if (rc != 0)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s::%s: chunk %zu/%zu SUMUP failed rc=0x%lx — "
                      "skipping chunk, ADS fallback for symbols in this chunk\n",
                      driverName,
                      functionName,
                      chunk + 1,
                      nChunks,
                      rc);
            totalFail += chunkN;
            continue;
        }

        /* sanity: TwinCAT must not write more than we allocated */
        if (bytesRead > readBytes)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s::%s: chunk %zu/%zu bytesRead=%u > readBytes=%zu — "
                      "buffer overflow, aborting. Increase entrySize cap.\n",
                      driverName,
                      functionName,
                      chunk + 1,
                      nChunks,
                      bytesRead,
                      readBytes);
            return asynError;
        }

        /* parse response */
        const uint8_t* statusBlock = readBuf.data();
        const uint8_t* dataPtr     = readBuf.data() + chunkN * 8;

        for (size_t i = 0; i < chunkN; ++i)
        {
            uint32_t result, retLen;
            memcpy(&result, statusBlock + i * 8 + 0, 4);
            memcpy(&retLen, statusBlock + i * 8 + 4, 4);

            const std::string& symName = symbols[chunkStart + i];

            if (result != 0)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s::%s: SYM_INFOBYNAMEEX failed for '%s' "
                          "result=0x%x — will use ADS fallback\n",
                          driverName,
                          functionName,
                          symName.c_str(),
                          result);
                ++totalFail;
                /* still advance past any partial response bytes */
                dataPtr += retLen;
                continue;
            }

            if (retLen < 36)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s::%s: '%s' retLen=%u too small — skipping\n",
                          driverName,
                          functionName,
                          symName.c_str(),
                          retLen);
                ++totalFail;
                dataPtr += retLen;
                continue;
            }

            /* copy up to entrySize bytes */
            adsSymbolEntry info;
            memset(&info, 0, sizeof(info));
            memcpy(&info, dataPtr, std::min((size_t)retLen, entrySize));
            info.variableName = info.buffer;
            info.symDataType  = info.buffer + info.nameLength + 1;
            info.symComment   = info.symDataType + info.typeLength + 1;

            /* populate symbolDict_ entry, handle resolved later by resolveSymbolHandles */
            std::string symLower = symName;
            std::transform(symLower.begin(),
                           symLower.end(),
                           symLower.begin(),
                           [](unsigned char c) { return std::tolower(c); });

            AdsSymbolDictEntry& e = symbolDict_[symLower];
            e.symbol              = symName;
            e.symbolLower         = symLower;
            e.size                = info.size;
            e.adst                = (uint32_t)info.dataType;
            e.datatype            = std::string(info.symDataType ? info.symDataType : "");
            e.resolved            = false; /* handle assigned by resolveSymbolHandles() */

            asynPrint(pasynUserSelf,
                      ASYN_TRACE_FLOW,
                      "%s::%s: '%s' → size=%u adst=%u type='%s'\n",
                      driverName,
                      functionName,
                      symName.c_str(),
                      e.size,
                      e.adst,
                      e.datatype.c_str());
            ++totalOK;
            /* advance by actual bytes TwinCAT wrote for this slot */
            dataPtr += retLen;
        }

        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s::%s: chunk %zu/%zu done — %zu OK so far\n",
                  driverName,
                  functionName,
                  chunk + 1,
                  nChunks,
                  totalOK);
    }

    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "%s::%s: complete — %zu resolved, %zu failed (ADS fallback for misses)\n",
              driverName,
              functionName,
              totalOK,
              totalFail);

    return asynSuccess;
}

/** Constructor for the adsAsynPortDriver class.
 * \param[in] portName Asyn port name.
 * \param[in] ipAddr Ip address of PLC.
 * \param[in] amsaddr Ams Address of PLC.
 * \param[in] amsport Default amsport in PLC (851 for first PLC).
 * \param[in] paramTableSize Maximum parameter/varaiable count.
 * \param[in] priority Asyn prio.
 * \param[in] autoConnect Enable auto connect.
 * \param[in] defaultSampleTimeMS Default sample of varaible (PLC ams router
 *            checks if variable changed, if changed then add to send buffer).
 * \param[in] maxDelayTimeMS Maximum delay before  variable that has changed is
 *            sent to client (linux). The variable can also be sent sooner if the
 *            ams router send buffer is filled.
 * \param[in] defaultTimeSource Default time stamp source of changed variable:\n
 *            defaultTimeSource=PLC: The PLC time stamp from when the value was
 *            changedis used and set as timestamp in the EPICS record
 *            (if record TSE field is set to -2 (enable asyn timestamp)).
 *            This is the preferred setting.\n
 *            defaultTimeSource=EPICS: The time stamp will be made when the
 *            updated data arrives in the EPICS client.\n

 * Initializes all variables and tries to connect to PLC system.
 */
adsAsynPortDriver::adsAsynPortDriver(const char* portName,
                                     const char* ipaddr,
                                     const char* amsaddr,
                                     unsigned int amsport,
                                     int paramTableSize,
                                     unsigned int priority,
                                     int autoConnect,
                                     int defaultSampleTimeMS,
                                     int maxDelayTimeMS,
                                     int adsTimeoutMS,
                                     ADSTIMESOURCE defaultTimeSource)
    : asynPortDriver(
          portName,
          1, /* maxAddr */
          asynInt32Mask | asynFloat64Mask | asynInt64Mask | asynInt8ArrayMask | asynInt16ArrayMask |
              asynInt32ArrayMask | asynInt64ArrayMask | asynFloat32ArrayMask |
              asynFloat64ArrayMask | asynDrvUserMask | asynOctetMask, /* Interface mask */
          asynInt32Mask | asynFloat64Mask | asynInt64Mask | asynInt8ArrayMask | asynInt16ArrayMask |
              asynInt32ArrayMask | asynInt64ArrayMask | asynFloat32ArrayMask |
              asynFloat64ArrayMask | asynDrvUserMask | asynOctetMask, /* Interrupt mask */
          ASYN_CANBLOCK, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
          autoConnect, /* Autoconnect */
          priority,    /* Default priority */
          0)           /* Default stack size*/
{
    const char* functionName = "adsAsynPortDriver";
    //Extra Debugging from the beginning: pasynTrace->setTraceMask(pasynUserSelf, 0x11);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    adsParamArray_.resize(paramTableSize);
    adsParamArrayCount_    = 0;
    paramTableSize_        = paramTableSize;
    ipaddr_                = strdup(ipaddr);
    amsaddr_               = strdup(amsaddr);
    amsportDefault_        = amsport;
    priority_              = priority;
    autoConnect_           = autoConnect;
    defaultSampleTimeMS_   = defaultSampleTimeMS;
    defaultMaxDelayTimeMS_ = maxDelayTimeMS;
    adsTimeoutMS_          = adsTimeoutMS;
    connectedAds_          = 0;
    defaultTimeSource_     = defaultTimeSource;
    routeAdded_            = 0;
    notConnectedCounter_   = 0;
    oneAmsConnectionOKold_ = 0;

    //Octet interface
    octetAsciiBuffer_.bufferSize = ADS_CMD_BUFFER_SIZE;
    octetAsciiBuffer_.bytesUsed  = 0;
    memset(&octetBinaryBuffer_, 0, ADS_CMD_BUFFER_SIZE);
    octetReturnVarName_ = 0;

    //ADS
    adsPort_     = 0; //handle
    remoteNetId_ = {0, 0, 0, 0, 0, 0};
    amsPortList_.clear();

    if (amsportDefault_ <= 0)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Invalid default AMS port: %d\n",
                  driverName,
                  functionName,
                  amsportDefault_);
        return;
    }
    addNewAmsPortToList(amsportDefault_);

    int nvals = sscanf(amsaddr_,
                       "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &remoteNetId_.b[0],
                       &remoteNetId_.b[1],
                       &remoteNetId_.b[2],
                       &remoteNetId_.b[3],
                       &remoteNetId_.b[4],
                       &remoteNetId_.b[5]);
    if (nvals != 6)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: AMS address invalid %s.\n",
                  driverName,
                  functionName,
                  amsaddr_);
        return;
    }

    if (nvals != 6)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Invalid AMS address: %s\n",
                  driverName,
                  functionName,
                  amsaddr_);
        return;
    }

    if (paramTableSize_ < 1)
    { //If paramTableSize_==1 then only stream device or motor record can use the driver through the "default access" param below.
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Param table size to small: %d\n",
                  driverName,
                  functionName,
                  paramTableSize_);
        return;
    }

    //Add first param for other access (like motor record or stream device).
    int index;
    asynStatus status = createParam("Default access", asynParamNotDefined, &index);
    if (status != asynSuccess)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: createParam for default access failed.\n",
                  driverName,
                  functionName);
        return;
    }
    adsParamArray_[0].recordName = strdup("Any record");
    adsParamArray_[0].recordType = strdup("No type");
    adsParamArray_[0].scan       = strdup("No scan");
    adsParamArray_[0].dtyp       = strdup("No dtyp");
    adsParamArray_[0].inp        = strdup("No inp");
    adsParamArray_[0].out        = strdup("No out");
    adsParamArray_[0].drvInfo    = strdup("No drvinfo");
    adsParamArray_[0].asynType   = asynParamNotDefined;
    adsParamArray_[0].paramIndex = index; //also used as hUser for ads callback
    adsParamArray_[0].plcAdrStr  = strdup("No adr str");
    adsParamArrayCount_++;

    if (status != asynSuccess)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: createParam for default access failed.\n",
                  driverName,
                  functionName);
        return;
    }

    // Open a client side ads port.
    // This doesn't require any communication with the server side, it is just to establish a unique
    // identifier for the ads communication calls coming from this thread.
    // All this really does is find the lowest available port number not currently open for the ads router on this client.
    // It would only fail if we have run out of available port numbers, which is any number between
    // 1 and UINT16_MAX = 65535 inclusive.
    // Each thread will get its own port number, and we will just make sure to close them when we don't
    // need them anymore. Most likely this class's destructor or earlier if we want.
    // Open a default client port.
    adsPort_ = addAdsClientPortNumberForThreadId(0);
    if (isInvalidPortNumber(adsPort_))
    {
        throw std::runtime_error(string_format(
            "%s:%s: failed to open default ads client port: %d.\n", driverName, __func__));
    }

    // Create an ads client port that will automatically be closed when this function leaves scope.
    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to get ads client port. Fallback to default client port.\n",
                  driverName,
                  __func__);
    }

    //* Create the thread that computes the waveforms in the background */
    {
        epicsThreadOpts opts = EPICS_THREAD_OPTS_INIT;
        opts.priority        = epicsThreadPriorityMedium;
        opts.stackSize       = epicsThreadGetStackSize(epicsThreadStackMedium);
        opts.joinable        = 1; // joined in ~adsAsynPortDriver()
        cyclicThreadId_      = epicsThreadCreateOpt(
            "adsAsynPortDriverCyclicThread", (EPICSTHREADFUNC)::cyclicThread, this, &opts);
    }
    if (cyclicThreadId_ == NULL)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }

    for (int i = 0; i < MAXBULK; i++)
        bulk[i].cnt = 0; // Entry is currently unused!!
    bulkTScnt = 0;
    if (defaultSampleTimeMS_ < 1000)
    {
        printf("Default Sample Time of %d ms is too small, defaulting to 1Hz.\n",
               defaultSampleTimeMS_);
        bulk_delay_us = 1000000; // 1 Hz
    }
    else
    {
        printf("Default bulk read time: %d ms\n", defaultSampleTimeMS_);
        bulk_delay_us = defaultSampleTimeMS_ * 1000;
    }
    bulkdatasize    = 4 * 1024 * 1024; // This is excessive!
    bulkdata        = (uint8_t*)malloc(bulkdatasize);
    bulkOK          = 0;
    bulk_elapsed_us = 0;

    //* Create the thread that does the bulk reads */
    {
        epicsThreadOpts opts = EPICS_THREAD_OPTS_INIT;
        opts.priority        = epicsThreadPriorityMedium;
        opts.stackSize       = epicsThreadGetStackSize(epicsThreadStackMedium);
        opts.joinable        = 1; // joined in ~adsAsynPortDriver()
        bulkReadThreadId_    = epicsThreadCreateOpt(
            "adsAsynPortDriverBulkReadThread", (EPICSTHREADFUNC)::bulkReadThread, this, &opts);
    }
    if (bulkReadThreadId_ == NULL)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
    //* Create the thread that does the ADS subscription callbacks */
    {
        epicsThreadOpts opts  = EPICS_THREAD_OPTS_INIT;
        opts.priority         = epicsThreadPriorityMedium;
        opts.stackSize        = epicsThreadGetStackSize(epicsThreadStackMedium);
        opts.joinable         = 1; // joined in ~adsAsynPortDriver()
        dataCallbackThreadId_ = epicsThreadCreateOpt("adsAsynPortDriverDataCallbackThread",
                                                     (EPICSTHREADFUNC)::dataCallbackThread,
                                                     this,
                                                     &opts);
    }
    if (dataCallbackThreadId_ == NULL)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }

    //* Create the thread that does I/O Intr callback triggers */
    {
        epicsThreadOpts opts = EPICS_THREAD_OPTS_INIT;
        opts.priority        = epicsThreadPriorityHigh;
        opts.stackSize       = epicsThreadGetStackSize(epicsThreadStackMedium);
        opts.joinable        = 1; // joined in ~adsAsynPortDriver()
        triggerIoIntrThreadId_ =
            epicsThreadCreateOpt("adsAsynPortDriverTriggerEpicsIoIntrCallbacksThread",
                                 (EPICSTHREADFUNC)::triggerEpicsIoIntrCallbacksThread,
                                 this,
                                 &opts);
    }
    if (triggerIoIntrThreadId_ == NULL)
    {
        throw std::runtime_error(string_format(
            "%s:%s: epicsThreadCreate failure for TriggerEpicsIoIntrCallbacksThread.\n",
            driverName,
            __func__));
    }

    //try to connect, and hang until we succeed!
    while (true)
    {
        if (connect(pasynUserSelf) != asynSuccess)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: connect failed for port %s.\n",
                      driverName,
                      functionName,
                      portName);
            epicsThreadSleep(1.0);
            continue;
        }
        long error        = 0;
        uint16_t adsState = 0;
        if (adsReadState(amsClientPort, amsport, &adsState, true, &error) != asynSuccess)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: adsReadStateLock failed for port %s.\n",
                      driverName,
                      functionName,
                      portName);
            disconnect(pasynUserSelf);
            continue;
        }
        if (adsState == ADSSTATE_RUN)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: connection established for port %s.\n",
                      driverName,
                      functionName,
                      portName);

            /* symbol info + handle resolution deferred to first drvUserCreate()
           * call — DB records are not loaded yet at this point in startup */

            return;
        }
    }
}

/** Destructor for the adsAsynPortDriver class.
 * Cleanup and deallocation of variables.
*/
adsAsynPortDriver::~adsAsynPortDriver()
{
    const char* functionName = "~adsAsynPortDriver";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    // Stop and join the worker threads BEFORE releasing symbolic handles,
    // freeing adsParamArray_ or closing the ADS port. The cyclic and bulk-read
    // threads issue synchronous ADS requests on the shared AmsRouter; if they
    // are still running while this destructor does the same and frees their
    // data, teardown either hangs (all threads blocked in AmsResponse::Wait) or
    // crashes (use-after-free on adsParamArray_/amsPortList_).
    stopThreads_ = true;
    if (cyclicThreadId_)
    {
        epicsThreadMustJoin(cyclicThreadId_);
        cyclicThreadId_ = nullptr;
    }
    if (bulkReadThreadId_)
    {
        epicsThreadMustJoin(bulkReadThreadId_);
        bulkReadThreadId_ = nullptr;
    }
    if (dataCallbackThreadId_)
    {
        epicsThreadMustJoin(dataCallbackThreadId_);
        dataCallbackThreadId_ = nullptr;
    }
    if (triggerIoIntrThreadId_)
    {
        epicsThreadMustJoin(triggerIoIntrThreadId_);
        triggerIoIntrThreadId_ = nullptr;
    }

    free(ipaddr_);
    free(amsaddr_);

    // Only default ams client port remains so we use that to delete the ads callbacks and release symbolic handles.
    for (int i = 0; i < adsParamArrayCount_; i++)
    {
        // Only release PLC-side notifications/handles while still connected. On
        // a dropped connection TwinCAT already discards this connection's
        // handles and notifications (the driver re-acquires them on reconnect
        // in refreshParams), and a synchronous ADS release would otherwise block
        // up to adsTimeoutMS per param. Matches the connectedAds_ guard in
        // refreshParams(). The free()s below must run regardless.
        if (connectedAds_)
        {
            adsDelDataCallback(adsPort_, &adsParamArray_[i], true);       //Block error messages
            adsReleaseSymbolicHandle(adsPort_, &adsParamArray_[i], true); //Block error messages
        }
        free(adsParamArray_[i].recordName);
        free(adsParamArray_[i].recordType);
        free(adsParamArray_[i].scan);
        free(adsParamArray_[i].dtyp);
        free(adsParamArray_[i].inp);
        free(adsParamArray_[i].out);
        free(adsParamArray_[i].drvInfo);
        free(adsParamArray_[i].plcAdrStr);
        if (adsParamArray_[i].plcDataIsArray)
        {
            free(adsParamArray_[i].arrayDataBuffer);
        }
    }

    for (amsPortInfo* port : amsPortList_)
    {
        delete port;
    }
    AdsPortCloseEx(adsPort_);
}

/** Cyclic thread for supervision of connection.
 * \return void
 * Check ads state of all connected ams ports and reconnects if needed.
 * At reconnect all symbolic handles and callbacks will be reregistered.
 */
void adsAsynPortDriver::cyclicThread()
{
    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    const char* functionName = "cyclicThread";
    double sampleTime        = 0.5;
    while (1)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_FLOW,
                  "%s:%s: Sample time [s]= %lf.\n",
                  driverName,
                  functionName,
                  sampleTime);

        epicsThreadSleep(sampleTime);
        if (stopThreads_)
        {
            break; // driver is being destroyed — exit before issuing ADS I/O
        }
        if (!allowCallbackEpicsState)
        {
            continue; //Epics not started
        }

        uint16_t adsState = 0;
        //Check state of all used ams ports
        bool oneAmsConnectionOK = false;
        for (amsPortInfo* port : amsPortList_)
        {
            long error      = 0;
            asynStatus stat = adsReadState(amsClientPort, port->amsPort, &adsState, true, &error);
            bool portConnected = (stat == asynSuccess && adsState == ADSSTATE_RUN);
            port->adsStateOld  = port->adsState;
            if (stat == asynSuccess)
            {
                port->adsState = (ADSSTATE)adsState;
            }
            else
            {
                port->adsState = ADSSTATE_INVALID;
            }

            port->connectedOld = port->connected;
            port->connected    = portConnected;
            port->paramsOK     = portConnected;

            oneAmsConnectionOK = oneAmsConnectionOK || portConnected;

            if (port->connected && port->refreshNeeded)
            {
                refreshParamsLock(amsClientPort, port->amsPort);
            }
            if (port->connectedOld && !port->connected)
            {
                invalidateParamsLock(port->amsPort);
                port->refreshNeeded = true;
                setAlarmPortLock(port->amsPort, COMM_ALARM, INVALID_ALARM);
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s:%s: connection lost for port %u\n",
                          driverName,
                          functionName,
                          port->amsPort);
            }
            if (!port->connectedOld && port->connected)
            {
                adsReadVersion(amsClientPort, port);
            }
        }

        //Printout state status
        for (amsPortInfo* port : amsPortList_)
        {
            if (port->connectedOld != port->connected)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_INFO,
                          "%s:%s: Device \"%s\" %s (Ams-port %u, Ams router version %u.%u.%u).\n",
                          driverName,
                          functionName,
                          port->devName,
                          port->connected ? "connected" : "disconnected",
                          port->amsPort,
                          port->version.version,
                          port->version.revision,
                          port->version.build);
            }
            if (port->adsStateOld != port->adsState)
            {
                //If Ams-router is a asyn paramter then update
                if (port->paramInfo)
                {
                    if (port->paramInfo->dataSource == ADS_DATASOURCE_AMS_STATE)
                    {
                        void* pData = (void*)&port->adsState;
                        adsUpdateParameterLock(port->paramInfo, pData, 2);
                    }
                }
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_INFO,
                          "%s:%s: Ams-port, %u, state change: \"%s\" -> \"%s\".\n",
                          driverName,
                          functionName,
                          port->amsPort,
                          adsStateToString(port->adsStateOld),
                          adsStateToString(port->adsState));
            }
        }


        if (!oneAmsConnectionOK)
        {
            notConnectedCounter_++;
            if (notConnectedCounter_ < 100 || notConnectedCounter_ % 100 == 0)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_FLOW,
                          "%s:%s: Not connected counter: %d.\n",
                          driverName,
                          functionName,
                          notConnectedCounter_);
            }
        }
        if (oneAmsConnectionOK)
        {
            notConnectedCounter_ = 0;
        }
        if (notConnectedCounter_ > 1)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "cyclicThread: failed to re-establish connection to ams server... exiting "
                      "ioc. will retry if on auto restart.\n");
            exit(-1);
        }

        if (!oneAmsConnectionOK && autoConnect_)
        {
            if (oneAmsConnectionOKold_)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s:%s: No connection! Try to reconnect...\n",
                          driverName,
                          functionName);
            }
            connectedAds_ = 0;
            if ((notConnectedCounter_ & 2) == 2)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "cyclicThread: try disconnect then reconnect.\n");
                disconnectLock(pasynUserSelf);
                connect(pasynUserSelf);
            }
        }
        oneAmsConnectionOKold_ = oneAmsConnectionOK;
    }
}

/* TBD - Poll at different rates depending on pollClass! */
void adsAsynPortDriver::bulkReadThread()
{
    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    const char* functionName = "bulkReadThread";
    struct timeval start, now;
    uint32_t bytesRead;
    long status;
    uint32_t cnt, readSize;
    asynUser* asynTraceUser = getTraceAsynUser();
    while (!bulkOK)
    {
        // Honour a shutdown request while still waiting for the first bulk-read
        // setup. Without this a driver torn down before bulkOK is ever set
        // (e.g. instantiated with no bulk params, then deleted) leaves this
        // thread spinning here forever, so the epicsThreadMustJoin() in the
        // destructor never returns. Completes the worker-thread stop/join path.
        if (stopThreads_)
        {
            return;
        }
        usleep(1000000);
    }
    {
#ifdef MCB_DEBUG
        int nChunks     = 0;
        int totalParams = 0;
        for (int i = 0; bulk[i].cnt; i++)
        {
            printf("bulkReadThread: chunk %d - %u params, amsPort=%u, readSize=%d\n",
                   i,
                   (unsigned)bulk[i].cnt,
                   bulk[i].amsPort,
                   bulk[i].readSize);
            totalParams += bulk[i].cnt;
            nChunks++;
        }
        printf("bulkReadThread: TOTAL %d chunk(s), %d param slots\n", nChunks, totalParams);
#endif
    }
    while (1)
    {
        if (stopThreads_)
        {
            break; // driver is being destroyed — stop polling
        }
        {
            std::lock_guard<std::mutex> lg(bulkReadInfoMutex_);
            gettimeofday(&start, NULL);
            for (int i = 0; bulk[i].cnt; i++)
            {
                if (!bulkOK || !bulk[i].cnt)
                {
                    break;
                }
#ifdef MCB_DEBUG
                static int first = 1;
                if (first)
                {
                    printf("Starting to poll!\n");
                    first = 0;
                }
#endif
                bytesRead         = 0;
                cnt               = bulk[i].cnt;
                readSize          = bulk[i].readSize;
                AmsAddr amsServer = {remoteNetId_, bulk[i].amsPort};
                status            = AdsSyncReadWriteReqEx2(amsClientPort,
                                                &amsServer,
                                                ADSIGRP_SUMUP_READ,
                                                cnt,
                                                readSize,
                                                bulkdata,
                                                sizeof(bulk[i].sum[0]) * cnt,
                                                &bulk[i].sum,
                                                &bytesRead);
                if (status)
                {
                    printf("Sum read %d failed: status %ld\n", i, status);
                    continue;
                }

                uint32_t* stat      = (uint32_t*)bulkdata;
                uint8_t* srd        = bulkdata + cnt * sizeof(uint32_t);
                uint64_t nTimeStamp = 0;
                /* The first *two* bulk parameters are reserved for a PLC
                 * timestamp (legacy MAIN.fbSystemTime.timeLoDW/HiDW). That
                 * symbol is NOT guaranteed to exist in a PLC project; when it is
                 * absent the timestamp slots return error status or a stale
                 * handle resolves to junk -- producing garbage times (e.g. year
                 * 2037) on every bulk record.
                 *
                 * Until a reliable, platform-wide PLC time symbol exists, do NOT
                 * trust the PLC timestamp here. Leave nTimeStamp = 0 so that
                 * refreshParamTime()'s `plcTimeStampRaw == 0` guard falls back to
                 * EPICS/IOC host time. The two reserved slots are still skipped
                 * below so srd stays aligned. */
                /* Always advance srd past the two timestamp data slots.
              * SUMUP_READ writes iSize response bytes for every slot regardless
              * of whether that slot's status is success or failure.  Skipping
              * advancement on failure (original code) shifts srd for every
              * subsequent param, producing zeros or garbage across the whole
              * poll cycle whenever the timestamp handles are transiently bad. */
                srd += bulk[i].sum[0].iSize;
                srd += bulk[i].sum[1].iSize;
                stat += 2;
                for (uint32_t j = 2; j < cnt; j++)
                {
                    auto paramInfo = getAdsParamInfo(bulk[i].paramID[j]);
                    if (!paramInfo)
                    {
                        asynPrint(asynTraceUser,
                                  ASYN_TRACE_ERROR,
                                  "%s:%s: getAdsParamInfo() for hUser %u failed\n",
                                  driverName,
                                  functionName,
                                  bulk[i].paramID[j]);
                        continue;
                    }
                    if (*stat++)
                    {
                        asynPrint(asynTraceUser,
                                  ASYN_TRACE_ERROR,
                                  "%s:%s: bulk read for %s (%d) failed\n",
                                  driverName,
                                  functionName,
                                  paramInfo->drvInfo,
                                  j);
                        paramInfo->refreshNeeded = true;
                    }
                    paramInfo->plcTimeStampRaw  = nTimeStamp;
                    paramInfo->lastCallbackSize = paramInfo->plcSize;
                    adsUpdateParameterLock(paramInfo, srd);
                    srd += paramInfo->lastCallbackSize;
                }
            }
        }
        gettimeofday(&now, NULL);
        bulk_elapsed_us = (now.tv_sec - start.tv_sec) * 1000000 + (now.tv_usec - start.tv_usec);
#ifdef MCB_DEBUG
        printf("ELAPSED: %g\n", bulk_elapsed_us / 1000000.0);
#endif
        // Always sleep at least 10ms
        usleep(std::max(bulk_delay_us - bulk_elapsed_us, 10000));
    }
}


/* Keeps possible slow data callbacks off of the ADS Recv queue*/
void adsAsynPortDriver::dataCallbackThread()
{
    const char* functionName = "dataCallbackThread";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);
    while (1)
    {
        if (stopThreads_)
        {
            break; // driver is being destroyed
        }
        if (datacbqueue.empty() || !allowCallbackEpicsState)
        {
            usleep(10000);
            continue;
        }
        adsAsynPortDriver::datacbinfo* info = &datacbqueue.front();
        asynPrint(pasynUserSelf,
                  ASYN_TRACEIO_DRIVER,
                  "%s:%s: Callback queue has %ld elements\n",
                  driverName,
                  functionName,
                  datacbqueue.size());
        asynPrint(pasynUserSelf,
                  ASYN_TRACEIO_DRIVER,
                  "%s:%s: Run callback for parameter %s (%d).\n",
                  driverName,
                  functionName,
                  info->paramInfo->drvInfo,
                  info->paramInfo->paramIndex);
        info->paramInfo->plcTimeStampRaw  = info->pNotification.nTimeStamp;
        info->paramInfo->lastCallbackSize = info->pNotification.cbSampleSize;
        adsUpdateParameterLock(info->paramInfo, info->data);
        // This free is for the malloc in adsDataCallback
        free(info->data);
        datacbqueue.pop();
#ifdef ADS_UNIT_TEST
        g_callbackCount++;
#endif
    }
}

// All IO Intr callbacks must be generated from this single thread.
void adsAsynPortDriver::triggerEpicsIoIntrCallbacksThread()
{
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, __func__);
    while (true)
    {
        if (stopThreads_)
        {
            break; // driver is being destroyed
        }
        if (!allowCallbackEpicsState)
        {
            epicsThreadSleep(0.5);
            continue;
        }
        // Batch callback calls into 100 ms intervals
        fireAllCallbacksLock();
        usleep(100000);
    }
}


/** Report of configured parameters.
 * \param[in] fp Output file.
 * \param[in] details Details of printout. A higher number results in more
 *            details.
 * \return void
 * Check ads state of all connected ams ports and reconnects if needed.
 */
void adsAsynPortDriver::report(FILE* fp, int details)
{
    const char* functionName = "report";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    if (!fp)
    {
        printf("%s:%s: ERROR: File NULL.\n", driverName, functionName);
        return;
    }

    if (details >= 1)
    {
        fprintf(fp, "General information:\n");
        fprintf(fp, "  Port:                        %s\n", portName);
        fprintf(fp, "  Ip-address:                  %s\n", ipaddr_);
        fprintf(fp, "  Ams-address:                 %s\n", amsaddr_);
        fprintf(fp, "  Default Ams-port :           %d\n", amsportDefault_);
        fprintf(fp, "  Auto-connect:                %s\n", autoConnect_ ? "true" : "false");
        fprintf(fp, "  Priority:                    %u\n", priority_);
        fprintf(fp, "  Param. table size:           %d\n", paramTableSize_);
        fprintf(fp, "  Param. count:                %d\n", adsParamArrayCount_);
        fprintf(fp, "  ADS command timeout [ms]:    %d\n", adsTimeoutMS_);
        fprintf(fp, "  Default sample time [ms]     %d\n", defaultSampleTimeMS_);
        fprintf(fp, "  Default max delay time [ms]: %d\n", defaultMaxDelayTimeMS_);
        fprintf(fp,
                "  Default time source:         %s\n",
                (defaultTimeSource_ == ADS_TIME_BASE_PLC) ? ADS_OPTION_TIMEBASE_PLC
                                                          : ADS_OPTION_TIMEBASE_EPICS);
        fprintf(fp, "  NOTE: Several records can be linked to the same parameter.\n");
        fprintf(fp, "\n");
    }
    if (details >= 2)
    {
        //print all parameters
        fprintf(fp, "Parameter details:\n");
        for (int i = 0; i < adsParamArrayCount_; i++)
        {
            auto paramInfo = &adsParamArray_[i];
            fprintf(fp, "  Parameter %d:\n", i);
            if (i == 0)
            {
                fprintf(fp,
                        "    Parameter 0 (pasynUser->reason==0) is reserved for Asyn octet "
                        "interface (Motor Record and Stream Device access).\n");
                fprintf(fp, "\n");
                continue;
            }
            fprintf(fp, "    Param name:                %s\n", paramInfo->drvInfo);
            fprintf(fp, "    Param index:               %d\n", paramInfo->paramIndex);
            fprintf(fp,
                    "    Param type:                %s (%d)\n",
                    asynTypeToString((long)paramInfo->asynType),
                    paramInfo->asynType);
            fprintf(fp, "    Param sample time [ms]:    %lf\n", paramInfo->sampleTimeMS);
            fprintf(fp, "    Param max delay time [ms]: %lf\n", paramInfo->maxDelayTimeMS);
            fprintf(
                fp, "    Param isIOIntr:            %s\n", paramInfo->isIOIntr ? "true" : "false");
            fprintf(fp, "    Param asyn addr:           %d\n", paramInfo->asynAddr);
            fprintf(fp,
                    "    Param time source:         %s\n",
                    (paramInfo->timeBase == ADS_TIME_BASE_PLC) ? ADS_OPTION_TIMEBASE_PLC
                                                               : ADS_OPTION_TIMEBASE_EPICS);
            fprintf(fp,
                    "    Param plc time:            %us:%uns\n",
                    paramInfo->plcTimeStamp.secPastEpoch,
                    paramInfo->plcTimeStamp.nsec);
            fprintf(fp,
                    "    Param epics time:          %us:%uns\n",
                    paramInfo->epicsTimestamp.secPastEpoch,
                    paramInfo->epicsTimestamp.nsec);
            fprintf(fp,
                    "    Param array buffer alloc:  %s\n",
                    paramInfo->arrayDataBuffer ? "true" : "false");
            fprintf(fp, "    Param array buffer size:   %zu\n", paramInfo->arrayDataBufferSize);
            fprintf(fp, "    Param alarm:               %d\n", paramInfo->alarmStatus);
            fprintf(fp, "    Param severity:            %d\n", paramInfo->alarmSeverity);
            fprintf(fp,
                    "    Param data source:         %s\n",
                    paramInfo->dataSource == ADS_DATASOURCE_PLC ? "PLC" : "DRIVER");
            fprintf(fp, "    Plc ams port:              %d\n", paramInfo->amsPort);
            fprintf(fp, "    Plc adr str:               %s\n", paramInfo->plcAdrStr);
            fprintf(fp,
                    "    Plc adr str is ADR cmd:    %s\n",
                    paramInfo->isAdrCommand ? "true" : "false");
            fprintf(fp,
                    "    Plc abs adr valid:         %s\n",
                    paramInfo->plcAbsAdrValid ? "true" : "false");
            fprintf(fp, "    Plc abs adr group:         16#%x\n", paramInfo->plcAbsAdrGroup);
            fprintf(fp, "    Plc abs adr offset:        16#%x\n", paramInfo->plcAbsAdrOffset);
            fprintf(
                fp, "    Plc data type:             %s\n", adsTypeToString(paramInfo->plcDataType));
            fprintf(
                fp, "    Plc data type size:        %zu\n", adsTypeSize(paramInfo->plcDataType));
            fprintf(fp, "    Plc data size:             %u\n", paramInfo->plcSize);
            fprintf(fp,
                    "    Plc data is array:         %s\n",
                    paramInfo->plcDataIsArray ? "true" : "false");
            fprintf(fp,
                    "    Plc data type warning:     %s\n",
                    paramInfo->plcDataTypeWarn ? "true" : "false");
            fprintf(fp, "    Ads hCallbackNotify:       %u\n", paramInfo->hCallbackNotify);
            fprintf(fp,
                    "    Ads CallbackNotify valid:  %s\n",
                    paramInfo->bCallbackNotifyValid ? "true" : "false");
            fprintf(fp, "    Ads hSymbHndle:            %u\n", paramInfo->hSymbolicHandle);
            fprintf(fp,
                    "    Ads hSymbHndleValid:       %s\n",
                    paramInfo->bSymbolicHandleValid ? "true" : "false");
            fprintf(fp, "    Record name:               %s\n", paramInfo->recordName);
            fprintf(fp, "    Record type:               %s\n", paramInfo->recordType);
            fprintf(fp, "    Record dtyp:               %s\n", paramInfo->dtyp);
            fprintf(fp, "\n");
        }
    }
}

/** Disconencts the PLC with asyn lock.
 * \param[in] pasynUser Asyn user.
 * \return asynSuccess or asynError.
 * Thread safe.
 */
asynStatus adsAsynPortDriver::disconnectLock(asynUser* pasynUser)
{
    lock();
    asynStatus stat = disconnect(pasynUser);
    unlock();
    return stat;
}

/** Disconencts the PLC.
 * \param[in] pasynUser Asyn user.
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::disconnect(asynUser* pasynUser)
{
    const char* functionName = "disconnect";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    // EtherCATMC appears to call disconnect sometimes.
    // adsDisconnect deletes the route, so this would cause all communication
    // in the ioc to fail.
    // Removed adsConnect call from here because we shouldn't ever need
    // to delete our route to achieve something.

    if (asynPortDriver::disconnect(pasynUser) != asynSuccess)
    {
        return asynError;
    }

    return asynSuccess;
}

/** Refreshes the parameters that need refresh after a reconnect or a
 * connection failure to a ams port.
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::refreshParams(uint16_t amsClientPort)
{
    return refreshParams(amsClientPort, 0);
}

/** Refreshes all parameters for a specific amsport (with asyn lock()).
 * \param[in] amsPort ams port.
 * \return asynSuccess or asynError.
 * Thread safe.
 */
asynStatus adsAsynPortDriver::refreshParamsLock(uint16_t amsClientPort, uint16_t amsPort)
{
    lock();
    asynStatus stat = refreshParams(amsClientPort, amsPort);
    unlock();
    return stat;
}

/** Refreshes all parameters for a specific amsport.
 * \param[in] amsPort ams port.
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::refreshParams(uint16_t amsClientPort, uint16_t amsPort)
{
    const char* functionName = "refreshParams";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    if (connectedAds_)
    {
        if (adsParamArrayCount_ > 1)
        {
            //Renew data notification callbacks
            for (int i = 1; i < adsParamArrayCount_; i++)
            { //Skip first param since used for motorrecord or stream device
                auto paramInfo = &adsParamArray_[i];
                if ((amsPort == 0 || paramInfo->amsPort == amsPort) && paramInfo->refreshNeeded)
                {
                    updateParamInfoWithPLCInfo(amsClientPort, paramInfo);
                }
            }
        }

        //Renew symbols changed notification callbacks
        for (amsPortInfo* port : amsPortList_)
        {
            if (port->amsPort == amsPort && port->refreshNeeded)
            {
                if (port->bCallbackNotifyValid)
                {
                    adsDelSymbolsChangedCallback(amsClientPort, port);
                }
                adsAddSymbolsChangedCallback(amsClientPort, port);
            }
        }
    }
    bulkOK = 1;
    return asynSuccess;
}

/** Invalidates all parameters for a specific amsport (with asyn lock()).
 * \param[in] amsPort ams port.
 * \return asynSuccess or asynError.
 * Thread safe.
 */
asynStatus adsAsynPortDriver::invalidateParamsLock(uint16_t amsPort)
{
    lock();
    asynStatus stat = invalidateParams(amsPort);
    unlock();
    return stat;
}

/** Invalidates all parameters for a specific amsport.
 * \param[in] amsPort ams port.
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::invalidateParams(uint16_t amsPort)
{
    const char* functionName = "invalidateParams";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    bulkOK = 0;
    if (adsParamArrayCount_ > 1)
    {
        for (int i = 1; i < adsParamArrayCount_; i++)
        { //Skip first param since used for motorrecord or stream device
            auto paramInfo = &adsParamArray_[i];
            if (amsPort == 0 || paramInfo->amsPort == amsPort)
            {
                paramInfo->refreshNeeded = true;
            }
        }
    }
    for (int i = 0; i < bulkTScnt; i++)
    {
        if (amsPort == 0 || bulkTS[i].amsPort == amsPort)
            bulkTS[i].refreshNeeded = 1;
    }
    for (int i = 0; i < MAXBULK; i++)
    {
        if (bulk[i].cnt == 0) // Quit if unused!
            break;
        if (amsPort == 0 || bulk[i].amsPort == amsPort)
            bulk[i].readSize = 4 * sizeof(uint32_t); // Initialize to just the timestamp!
    }
    return asynSuccess;
}

/** Connects to a PLC (with asyn lock()).
 * \param[in] pasynUser Asyn user
 * \return asynSuccess or asynError.
 * Thread safe.
 */
asynStatus adsAsynPortDriver::connectLock(asynUser* pasynUser)
{
    lock();
    asynStatus stat = connect(pasynUser);
    unlock();
    return stat;
}

/** Connects to a PLC.
 * \param[in] pasynUser Asyn user
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::connect(asynUser* pasynUser)
{
    const char* functionName = "connect";
    asynPrint(pasynUser,
              ASYN_TRACE_FLOW,
              "%s:%s: %s\n",
              driverName,
              functionName,
              epicsThreadGetNameSelf());

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    bool err        = false;
    asynStatus stat = adsConnect(amsClientPort);

    if (stat != asynSuccess)
    {
        return asynError;
    }

    if (asynPortDriver::connect(pasynUser) != asynSuccess)
    {
        return asynError;
    }

    connectedAds_ = 1;
    return err ? asynError : asynSuccess;
}

/** Validates drvInfo string
 * \param[in] drvInfo String containing information about the parameter.
 * \return asynSuccess or asynError.
 * The drvInfo string is what is after the asyn() in the "INP" or "OUT"
 * field of an record.
 */
asynStatus adsAsynPortDriver::validateDrvInfo(const char* drvInfo)
{
    const char* functionName = "validateDrvInfo";
    asynPrint(
        pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName, drvInfo);

    if (strlen(drvInfo) == 0)
    {
        asynPrint(
            pasynUserSelf, ASYN_TRACE_ERROR, "Invalid drvInfo string: Length 0 (%s).\n", drvInfo);
        return asynError;
    }

    //Check '?' mark last or '=' last
    const char* read = strrchr(drvInfo, '?');
    if (read)
    {
        if (strlen(read) == 1)
        {
            return asynSuccess;
        }
    }

    const char* write = strrchr(drvInfo, '=');
    if (write)
    {
        if (strlen(write) == 1)
        {
            return asynSuccess;
        }
    }

    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Invalid drvInfo string (%s).\n", drvInfo);
    return asynError;
}

/** Overrides asynPortDriver::drvUserCreate.
 * This function is called by the asyn-framework for each record that is linked to this asyn port.
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] drvInfo String containing information about the parameter.
 * \param[out] pptypeName
 * \param[out] psize size of pptypeName.
 * \return asynSuccess or asynError.
 * The drvInfo string is what is after the asyn() in the "INP" or "OUT"
 * field of an record.
 */
asynStatus adsAsynPortDriver::drvUserCreate(asynUser* pasynUser,
                                            const char* drvInfo,
                                            const char** pptypeName,
                                            size_t* psize)
{
    const char* functionName = "drvUserCreate";
    static int vcnt          = 0;

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    /* One-off: resolve all symbol info from TwinCAT on first call
   * resolveSymbolInfo() runs after dbLoadRecords() has populated pdbbase
   * so that buildDrvInfoCache() finds all records.
   * drvUserCreate is called during iocInit record initialisation, DB is fully loaded by then.
   * connectedAds_ guard ensures we don't attempt ADS calls if not yet up.
  */
    if (!symInfoResolved_ && connectedAds_)
    {
        symInfoResolved_ = true; /* set before call to prevent re-entry */
        if (resolveSymbolInfo(amsClientPort) != asynSuccess)
        {
            asynPrint(pasynUser,
                      ASYN_TRACE_ERROR,
                      "%s:%s: resolveSymbolInfo failed — "
                      "falling back to per-record ADS calls\n",
                      driverName,
                      functionName);
        }
        /* now resolve handles for everything we just populated */
        if (!symbolDict_.empty())
        {
            if (resolveSymbolHandles(amsClientPort) != asynSuccess)
            {
                asynPrint(pasynUser,
                          ASYN_TRACE_ERROR,
                          "%s:%s: resolveSymbolHandles failed\n",
                          driverName,
                          functionName);
                exit(-1);
            }
        }
    }
    /*───────────────────────── */

    asynPrint(
        pasynUser, ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName, drvInfo);

    if (validateDrvInfo(drvInfo) != asynSuccess)
    {
        return asynError;
    }

    std::string drvInfoStr = drvInfo;

    int index               = 0;
    auto createdParamsMapIt = createdParamsMap_.find(drvInfoStr);
    if (createdParamsMapIt != createdParamsMap_.end())
    {
        index = createdParamsMapIt->second;
        asynPrint(pasynUser,
                  ASYN_TRACE_FLOW,
                  "%s:%s: Parameter index found at: %d for %s. \n",
                  driverName,
                  functionName,
                  index,
                  drvInfo);
        return asynPortDriver::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
    }

    if (!vcnt++)
        printf("Linking EPICS PVs to PLC variables...\n");
    if (vcnt % 1000 == 0)
        printf("%d...\n", vcnt);

    struct timeval t0, t1;
    gettimeofday(&t0, NULL);

    //Ensure space left in param table
    if (adsParamArrayCount_ >= (paramTableSize_ - 1))
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Parameter table full. Parameter with drvInfo %s will be discarded.",
                  driverName,
                  functionName,
                  drvInfo);
        return asynError;
    }

    // Collect data from drvInfo string and recordpasynUser->reason=index;
    auto paramInfo            = &adsParamArray_[adsParamArrayCount_];
    paramInfo->sampleTimeMS   = defaultSampleTimeMS_;
    paramInfo->maxDelayTimeMS = defaultMaxDelayTimeMS_;
    paramInfo->refreshNeeded  = 1;
    paramInfo->bulkIndex      = -1;
    paramInfo->bulkOffset     = -1;

    auto status = parsePlcInfofromDrvInfo(drvInfo, paramInfo);
    if (status != asynSuccess)
    {
        return asynError;
    }

    paramInfo->drvInfo = strdup(drvInfo);

    status = getRecordInfoFromDrvInfo(drvInfo, paramInfo);
    if (status != asynSuccess)
    {
        return asynError;
    }

    status = createParam(drvInfo, paramInfo->asynType, &index);
    if (status != asynSuccess)
    {
        asynPrint(
            pasynUser, ASYN_TRACE_ERROR, "%s:%s: createParam() failed.", driverName, functionName);
        return asynError;
    }

    //Set default value for basic types...
    switch (paramInfo->asynType)
    {
    case asynParamInt32:
        setIntegerParam(index, 0);
        break;
    case asynParamFloat64:
        setDoubleParam(index, 0);
        break;
    case asynParamInt64:
        setInteger64Param(index, 0);
        break;
    default:
        break;
    }

    paramInfo->paramIndex = index;

    int addr = 0;
    status   = getAddress(pasynUser, &addr);
    if (status != asynSuccess)
    {
        asynPrint(
            pasynUser, ASYN_TRACE_ERROR, "%s:%s: getAddress() failed.", driverName, functionName);
        return (status);
    }

    paramInfo->asynAddr = addr;

    pasynUser->timeout = (paramInfo->maxDelayTimeMS * 2) / 1000;
    adsParamArrayCount_++;
    createdParamsMap_.insert(std::make_pair(drvInfoStr, index));

    if (!connectedAds_)
    {
        //try to connect without error handling
        connect(pasynUser);
    }

    if (connectedAds_ && !(paramInfo->dataSource == ADS_DATASOURCE_AMS_STATE))
    { //Do not read info from PLC if local variable (like ams-port state)
        status = updateParamInfoWithPLCInfo(amsClientPort, paramInfo);
        if (status != asynSuccess)
        {
            return asynError;
        }
    }

    gettimeofday(&t1, NULL);
    long elapsed_us = (t1.tv_sec - t0.tv_sec) * 1000000L + (t1.tv_usec - t0.tv_usec);
    asynPrint(pasynUser,
              ASYN_TRACE_FLOW,
              "%s:%s: Parameter created: \"%s\" (index %d) [%4ld us].\n",
              driverName,
              functionName,
              drvInfo,
              index,
              elapsed_us);

    return asynPortDriver::drvUserCreate(
        pasynUser, drvInfo, pptypeName, psize); //Assigns pasynUser->reason;
}

/** Update parameter with info from PLC (variable size, type and abs addr).
 * \param[in/out] paramInfo Parameter information structure.
 * \return asynSuccess or asynError.
 * If the PLC variable is an array then a buffer is allocated in the paramInfo to
 * hold the information.
 */
asynStatus adsAsynPortDriver::updateParamInfoWithPLCInfo(uint16_t amsClientPort,
                                                         adsParamInfo* paramInfo)
{
    const char* functionName = "updateParamInfoWithPLCInfo";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: : %s\n",
              driverName,
              functionName,
              paramInfo->drvInfo);

    asynStatus status;

    //Do not read information from PLC if "variable" in driver (like ams router state)
    if (paramInfo->dataSource != ADS_DATASOURCE_PLC)
    {
        paramInfo->refreshNeeded = false;
        return asynSuccess;
    }

    /* symbolDict_ is populated by resolveSymbolInfo()+resolveSymbolHandles() at
   * iocInit. Inject type/size/handle directly — zero ADS RTTs per record.
   * Falls back to individual ADS calls only for .ADR. commands, AMSPORTSTATE
   * locals, or any symbol that resolveSymbolInfo() did not find.
  */
    bool fromDict = false;
    if (!paramInfo->isAdrCommand && !symbolDict_.empty())
    {
        std::string key(paramInfo->plcAdrStr);
        std::transform(
            key.begin(), key.end(), key.begin(), [](unsigned char c) { return std::tolower(c); });
        auto it = symbolDict_.find(key);
        if (it != symbolDict_.end() && it->second.resolved && it->second.size > 0 &&
            it->second.adst != ADST_VOID)
        {
            const AdsSymbolDictEntry& e     = it->second;
            paramInfo->plcDataType          = e.adst;
            paramInfo->plcSize              = e.size;
            paramInfo->hSymbolicHandle      = e.handle;
            paramInfo->bSymbolicHandleValid = true;
            fromDict                        = true;
        }
    }

    if (!paramInfo->isAdrCommand && !fromDict)
    {
        status = adsGetSymInfoByName(amsClientPort, paramInfo);
        if (status != asynSuccess)
            return asynError;
    }

    // Check if array
    bool isArray = false;
    switch (paramInfo->plcDataType)
    {
    case ADST_VOID:
        isArray = false;
        break;
    case ADST_STRING:
        isArray = true; // Special case
        break;
    case ADST_WSTRING:
        isArray = true; // Special case?
        break;
    case ADST_BIGTYPE:
        isArray = false;
        break;
    case ADST_MAXTYPES:
        isArray = false;
        break;
    default:
        isArray = paramInfo->plcSize > adsTypeSize(paramInfo->plcDataType);
        break;
    }
    paramInfo->plcDataIsArray = isArray;

    // Allocate memory for array
    if (isArray)
    {
        if (paramInfo->plcSize != paramInfo->arrayDataBufferSize && paramInfo->arrayDataBuffer)
        { //new size of array
            free(paramInfo->arrayDataBuffer);
            paramInfo->arrayDataBuffer = NULL;
        }
        if (!paramInfo->arrayDataBuffer)
        {
            paramInfo->arrayDataBuffer     = calloc(paramInfo->plcSize, 1);
            paramInfo->arrayDataBufferSize = paramInfo->plcSize;
            if (!paramInfo->arrayDataBuffer)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s:%s: Failed to allocate memory for array data for %s.\n.",
                          driverName,
                          functionName,
                          paramInfo->drvInfo);
                unlock();
                return asynError;
            }
            memset(paramInfo->arrayDataBuffer, 0, paramInfo->plcSize);
        }
    }

    /* Handle already set by resolveSymbolHandles() for dict symbols.
   * For misses (ADR commands, locals, fallback), acquire individually. */
    if (!paramInfo->isAdrCommand && !fromDict)
    {
        adsReleaseSymbolicHandle(amsClientPort, paramInfo, true);
        status = adsGetSymHandleByName(amsClientPort, paramInfo);
        if (status != asynSuccess)
            return asynError;
    }

    if (paramInfo->isIOIntr)
    {
        /* If it's not a bulk read or if it's really big, just subscribe to it! */
        if (!paramInfo->isBulkRead || paramInfo->plcSize > 1024 * 1024)
        {
            adsDelDataCallback(amsClientPort, paramInfo, true); //try to delete
            status = adsAddDataCallback(amsClientPort, paramInfo);
            if (status != asynSuccess)
            {
                return asynError;
            }
        }
        else
        { /* Otherwise, put it in a bulk read! */
            status = adsAddToBulkRead(amsClientPort, paramInfo);
            if (status != asynSuccess)
            {
                return asynError;
            }
        }
    }

    paramInfo->refreshNeeded = false;
    return asynSuccess;
}

void adsAsynPortDriver::poll_info(char* name)
{
    int i;
    printf("Bulk read loop: desired period = %gs, last loop time = %gs\n",
           bulk_delay_us / 1000000.0,
           bulk_elapsed_us / 1000000.0);
    for (i = 0; bulk[i].cnt && i < MAXBULK; i++)
        ;
    printf("Bulk read count = %d\n", i);
    if (name[0] == 0)
        name = 0;
    for (i = 0; bulk[i].cnt && i < MAXBULK; i++)
    {
        printf("Bulk Read #%d:\n", i);
        if (!name)
        {
            printf("    0: MAIN.fbSystemTime.timeLoDW (G=0x%x, O=0x%x, S=%d)\n",
                   bulk[i].sum[0].iGroup,
                   bulk[i].sum[0].iOffset,
                   bulk[i].sum[0].iSize);
            printf("    1: MAIN.fbSystemTime.timeHiDW (G=0x%x, O=0x%x, S=%d)\n",
                   bulk[i].sum[1].iGroup,
                   bulk[i].sum[1].iOffset,
                   bulk[i].sum[1].iSize);
        }
        for (int j = 2; j < bulk[i].cnt; j++)
        {
            adsParamInfo* paramInfo = getAdsParamInfo(bulk[i].paramID[j]);
            if (!paramInfo)
                continue;
            if (!name || strstr(paramInfo->plcAdrStr, name))
                printf("  %3d: %s (G=0x%x, O=0x%x, S=%d, TS=%d.%09d)\n",
                       j,
                       paramInfo->plcAdrStr,
                       bulk[i].sum[j].iGroup,
                       bulk[i].sum[j].iOffset,
                       bulk[i].sum[j].iSize,
                       paramInfo->epicsTimestamp.secPastEpoch,
                       paramInfo->epicsTimestamp.nsec);
        }
    }
}

/* TBD - Use paramInfo->pollClass to separate into different poll rates!! */
asynStatus adsAsynPortDriver::adsAddToBulkRead(uint16_t amsClientPort, adsParamInfo* paramInfo)
{
    std::lock_guard<std::mutex> lg(bulkReadInfoMutex_);
    if (paramInfo->bulkIndex < 0)
    { /* Not assigned yet, find one! */
        int i;
        for (i = 0; i < MAXBULK; i++)
        {
            /* Look for an unused entry or a non-full entry for this port. */
            if (bulk[i].cnt == 0 ||
                (bulk[i].cnt != BULKSIZ && bulk[i].amsPort == paramInfo->amsPort))
                break;
        }
        if (i == MAXBULK)
        {
            return asynError; // No room at the inn.
        }
        if (!bulk[i].cnt)
        { // First variable in this bulk request!
            bulk[i].amsPort = paramInfo->amsPort;
            int j           = adsFindBulkTimeStamp(amsClientPort, paramInfo->amsPort);
            if (bulkTS[j].refreshNeeded)
            {                                    /* No TS variables!! */
                bulk[i].sum[0].iGroup  = 0x4020; // %M
                bulk[i].sum[0].iOffset = 0;
                bulk[i].sum[0].iSize   = sizeof(uint32_t);
                bulk[i].sum[1].iGroup  = 0x4020; // %M
                bulk[i].sum[1].iOffset = 0;
                bulk[i].sum[1].iSize   = sizeof(uint32_t);
            }
            else
            {
                bulk[i].sum[0].iGroup  = ADSIGRP_SYM_VALBYHND;
                bulk[i].sum[0].iOffset = bulkTS[j].iHandleH;
                bulk[i].sum[0].iSize   = sizeof(uint32_t);
                bulk[i].sum[1].iGroup  = ADSIGRP_SYM_VALBYHND;
                bulk[i].sum[1].iOffset = bulkTS[j].iHandleL;
                bulk[i].sum[1].iSize   = sizeof(uint32_t);
            }
            bulk[i].cnt      = 2;
            bulk[i].readSize = 4 * sizeof(uint32_t);
        }
        paramInfo->bulkIndex           = i;
        paramInfo->bulkOffset          = bulk[i].cnt;
        bulk[i].paramID[bulk[i].cnt++] = paramInfo->paramIndex;
    }
    /* Update the information for a previously allocated element.  Possibly *just*
       allocated, but that's still previously! */
    uint32_t group, offset;
    if (paramInfo->isAdrCommand)
    {
        group  = paramInfo->plcAbsAdrGroup;
        offset = paramInfo->plcAbsAdrOffset;
    }
    else
    {
        group  = ADSIGRP_SYM_VALBYHND;
        offset = paramInfo->hSymbolicHandle;
    }
    bulk[paramInfo->bulkIndex].sum[paramInfo->bulkOffset].iGroup  = group;
    bulk[paramInfo->bulkIndex].sum[paramInfo->bulkOffset].iOffset = offset;
    bulk[paramInfo->bulkIndex].sum[paramInfo->bulkOffset].iSize   = paramInfo->plcSize;
    bulk[paramInfo->bulkIndex].readSize += paramInfo->plcSize + sizeof(uint32_t);

    return asynSuccess;
}

// Assume locked!!
int adsAsynPortDriver::adsFindBulkTimeStamp(uint16_t amsClientPort, uint16_t amsPort)
{
    int i;
    for (i = 0; i < bulkTScnt; i++)
    {
        if (amsPort == bulkTS[i].amsPort)
            break;
    }
    if (i == bulkTScnt)
    {
        bulkTScnt++;
        bulkTS[i].amsPort       = amsPort;
        bulkTS[i].refreshNeeded = 1;
    }
#define TSLO "MAIN.fbSystemTime.timeLoDW"
#define TSHI "MAIN.fbSystemTime.timeHiDW"
    if (bulkTS[i].refreshNeeded)
    {
        long statL, statH;
        AmsAddr amsServer = {remoteNetId_, amsPort};
        statH             = AdsSyncReadWriteReqEx2(amsClientPort,
                                       &amsServer,
                                       ADSIGRP_SYM_HNDBYNAME,
                                       0,
                                       sizeof(uint32_t),
                                       &bulkTS[i].iHandleH,
                                       strlen(TSHI),
                                       TSHI,
                                       nullptr);
        statL             = AdsSyncReadWriteReqEx2(amsClientPort,
                                       &amsServer,
                                       ADSIGRP_SYM_HNDBYNAME,
                                       0,
                                       sizeof(uint32_t),
                                       &bulkTS[i].iHandleL,
                                       strlen(TSLO),
                                       TSLO,
                                       nullptr);
        if (!statH && !statL)
            bulkTS[i].refreshNeeded = 0;
    }
    return i;
}

/** Get asyn type from record.
 * \param[in] drvInfo String containing information about the parameter.
 * \param[in/out] paramInfo Parameter information structure.
 * \return asynSuccess or asynError.
 */

asynStatus adsAsynPortDriver::getRecordInfoFromDrvInfo(const char* drvInfo, adsParamInfo* paramInfo)
{
    const char* functionName = "getRecordInfoFromDrvInfo";
    asynPrint(
        pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName, drvInfo);


    /* Build cache on first call — O(N) scan of entire database, done once. */
    buildDrvInfoCache();

    std::string key = std::string(portName) + '\0' + drvInfo;
    auto it         = s_drvInfoCache.find(key);
    if (it == s_drvInfoCache.end())
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: no record found for drvInfo '%s' on port '%s'\n",
                  driverName,
                  functionName,
                  drvInfo,
                  portName);
        return asynError;
    }

    const RecordInfoCache& e = it->second;
    paramInfo->recordType    = e.recordType.empty() ? nullptr : strdup(e.recordType.c_str());
    paramInfo->recordName    = e.recordName.empty() ? nullptr : strdup(e.recordName.c_str());
    paramInfo->dtyp          = e.dtyp.empty() ? nullptr : strdup(e.dtyp.c_str());
    paramInfo->inp           = e.inp.empty() ? nullptr : strdup(e.inp.c_str());
    paramInfo->out           = e.out.empty() ? nullptr : strdup(e.out.c_str());
    paramInfo->asynType      = e.asynType;
    paramInfo->drvInfo       = strdup(drvInfo);
    return asynSuccess;
}

/** Get variable information from drvInfo string.
 * \param[in] drvInfo String containing information about the parameter.
 * \param[in/out] paramInfo Parameter information structure.
 * \return asynSuccess or asynError.
 * Methods checks if input or output ('?' or '=') and parses options:
 * - "ADSPORT" (Ams port for variable)\n
 * - "T_DLY_MS" (maximum delay time ms)\n
 * - "TS_MS" (sample time ms)\n
 * - "TIMEBASE" ("PLC" or "EPICS")\n
 * Also supports the following commands:
 * - ".AMSPORTSTATE." (Read/write AMS-port state)\n
 * - ".ADR.*" (absolute access)\n
 */
asynStatus adsAsynPortDriver::parsePlcInfofromDrvInfo(const char* drvInfo, adsParamInfo* paramInfo)
{
    const char* functionName = "parsePlcInfofromDrvInfo";
    asynPrint(
        pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName, drvInfo);

    //Check if input or output
    paramInfo->isIOIntr = false;
    const char* temp    = strrchr(drvInfo, '?');
    if (temp)
    {
        if (strlen(temp) == 1)
        {
            paramInfo->isIOIntr = true; //All inputs will be created I/O intr
        }
    }

    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: drvInfo %s is %s\n",
              driverName,
              functionName,
              drvInfo,
              paramInfo->isIOIntr ? "I/O Intr (end with ?)" : " not I/O Intr (end with =)");

    //take part after last "/" if option or complete string..
    char buffer[ADS_MAX_FIELD_CHAR_LENGTH];
    //See if option (find last '/')
    const char* drvInfoEnd = strrchr(drvInfo, '/');
    if (drvInfoEnd)
    { // found '/'
        int nvals = sscanf(drvInfoEnd, "/%127s", buffer);
        if (nvals == 1)
        {
            paramInfo->plcAdrStr                                   = strdup(buffer);
            paramInfo->plcAdrStr[strlen(paramInfo->plcAdrStr) - 1] = 0; //Strip ? or = from end
        }
        else
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse PLC address string from drvInfo (%s)\n",
                      driverName,
                      functionName,
                      drvInfo);
            return asynError;
        }
    }
    else
    {                                                               //No options
        paramInfo->plcAdrStr = strdup(drvInfo);                     //Symbolic or .ADR.
        paramInfo->plcAdrStr[strlen(paramInfo->plcAdrStr) - 1] = 0; //Strip ? or = from end
    }

    //Check if .ADR. command
    const char* option        = ADS_ADR_COMMAND_PREFIX;
    paramInfo->plcAbsAdrValid = false;
    paramInfo->isAdrCommand   = false;
    const char* isThere       = strstr(drvInfo, option);
    if (isThere)
    {
        if (strlen(isThere) < (strlen(option) + strlen("16#%x,16#%x,%u,%u")))
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s command from drvInfo (%s). String to short.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }
        paramInfo->isAdrCommand = true;

        int nvals = sscanf(isThere + strlen(option),
                           "16#%x,16#%x,%u,%u",
                           &paramInfo->plcAbsAdrGroup,
                           &paramInfo->plcAbsAdrOffset,
                           &paramInfo->plcSize,
                           &paramInfo->plcDataType);

        if (nvals == 4)
        {
            paramInfo->plcAbsAdrValid = true;
        }
        else
        {
            paramInfo->plcAbsAdrValid  = false;
            paramInfo->plcAbsAdrGroup  = -1;
            paramInfo->plcAbsAdrOffset = -1;
            paramInfo->plcSize         = -1;
            paramInfo->plcDataType     = -1;
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s command from drvInfo (%s). Wrong format.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }
    }

    //Check if ADS_OPTION_T_MAX_DLY_MS option
    option  = ADS_OPTION_T_MAX_DLY_MS;
    isThere = strstr(drvInfo, option);
    if (isThere)
    {
        if (strlen(isThere) < (strlen(option) + strlen("=0/")))
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }

        int nvals = sscanf(isThere + strlen(option), "=%lf/", &paramInfo->maxDelayTimeMS);

        if (nvals != 1)
        {
            paramInfo->maxDelayTimeMS = defaultMaxDelayTimeMS_;
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }
    }

    //Check if ADS_OPTION_T_SAMPLE_RATE_MS option
    option                  = ADS_OPTION_T_SAMPLE_RATE_MS;
    paramInfo->sampleTimeMS = defaultSampleTimeMS_;
    isThere                 = strstr(drvInfo, option);
    if (isThere)
    {
        if (strlen(isThere) < (strlen(option) + strlen("=0/")))
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }

        int nvals = sscanf(isThere + strlen(option), "=%lf/", &paramInfo->sampleTimeMS);

        if (nvals != 1)
        {
            paramInfo->sampleTimeMS = defaultSampleTimeMS_;
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }
    }

    //Check if ADS_OPTION_POLLRATE option
    option                = ADS_OPTION_POLLRATE;
    paramInfo->isBulkRead = false;
    paramInfo->pollClass  = 1.0;
    isThere               = strstr(drvInfo, option);
    if (isThere)
    {
        if (strlen(isThere) < (strlen(option) + strlen("=0/")))
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }
        paramInfo->isBulkRead = true;

        int nvals = sscanf(isThere + strlen(option), "=%lf/", &paramInfo->pollClass);

        if (nvals != 1)
        {
            paramInfo->pollClass = 1.0;
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }
    }

    //Check if ADS_OPTION_TIMEBASE option
    option              = ADS_OPTION_TIMEBASE;
    paramInfo->timeBase = defaultTimeSource_;
    isThere             = strstr(drvInfo, option);
    if (isThere)
    {
        int minLen   = strlen(ADS_OPTION_TIMEBASE_PLC);
        int epicsLen = strlen(ADS_OPTION_TIMEBASE_EPICS);
        if (epicsLen < minLen)
        {
            minLen = epicsLen;
        }
        if (strlen(isThere) < (strlen(option) + strlen("=/") + minLen))
        { //Allowed "PLC" or "EPICS"
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }

        int nvals = sscanf(isThere + strlen(option), "=%127[^/]/", buffer);
        if (nvals != 1)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }

        if (strcmp(ADS_OPTION_TIMEBASE_PLC, buffer) == 0)
        {
            paramInfo->timeBase = ADS_TIME_BASE_PLC;
        }

        if (strcmp(ADS_OPTION_TIMEBASE_EPICS, buffer) == 0)
        {
            paramInfo->timeBase = ADS_TIME_BASE_EPICS;
        }
    }

    //Check if ADS_OPTION_ADSPORT option
    option             = ADS_OPTION_ADSPORT;
    paramInfo->amsPort = amsportDefault_;
    isThere            = strstr(drvInfo, option);
    if (isThere)
    {
        if (strlen(isThere) < (strlen(option) + strlen("=0/")))
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }
        int nvals;
        int val;
        nvals = sscanf(isThere + strlen(option), "=%d/", &val);
        if (nvals == 1)
        {
            paramInfo->amsPort = (uint16_t)val;
        }
        else
        {
            paramInfo->amsPort = amsportDefault_;
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }
    }

    //Check if ADS_AMS_STATE_COMMAND option Local variable/parameter (not in PLC)
    option                = ADS_AMS_STATE_COMMAND;
    paramInfo->dataSource = ADS_DATASOURCE_PLC;
    isThere               = strstr(drvInfo, option);
    if (isThere)
    {
        addNewAmsPortToList(paramInfo->amsPort); //Only add if not already there
        amsPortInfo* port = getAmsPortObject(paramInfo->amsPort);
        if (!port)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
                      driverName,
                      functionName,
                      option,
                      drvInfo);
            return asynError;
        }
        paramInfo->dataSource =
            ADS_DATASOURCE_AMS_STATE; //This information is accessible in driver (not PLC)
        paramInfo->plcDataType    = ADST_UINT16;
        paramInfo->plcSize        = 2;
        paramInfo->plcDataIsArray = false;
        paramInfo->timeBase       = ADS_TIME_BASE_EPICS;
        port->paramInfo           = paramInfo;
    }

    return addNewAmsPortToList(paramInfo->amsPort); //Only add if not already there
}

/** Get ams port information object from ams-port list.
 * \param[in] amsPort ams-port
 *
 * \return amsPortInfo object.
 */
amsPortInfo* adsAsynPortDriver::getAmsPortObject(uint16_t amsPort)
{
    const char* functionName = "getAmsPortObject";
    asynPrint(
        pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: amsPort:%u\n", driverName, functionName, amsPort);

    for (amsPortInfo* port : amsPortList_)
    {
        if (port->amsPort == amsPort)
        {
            return port;
        }
    }
    return 0;
}
/** Add new ams port to ams-port list.
 * \param[in] amsPort ams-port
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::addNewAmsPortToList(uint16_t amsPort)
{
    const char* functionName = "addNewAmsPortToList";
    asynPrint(
        pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: amsPort:%u\n", driverName, functionName, amsPort);

    //See if new amsPort, then update list
    bool newAmsPort = true;
    for (amsPortInfo* port : amsPortList_)
    {
        if (port->amsPort == amsPort)
        {
            newAmsPort = false;
        }
    }

    if (!newAmsPort)
    {
        return asynSuccess;
    }

    try
    {
        amsPortInfo* newPort = new amsPortInfo();
        memset(newPort, 0, sizeof(amsPortInfo));
        newPort->amsPort       = amsPort;
        newPort->adsState      = (ADSSTATE)(ADSSTATE_MAXSTATES + 1); //Set unknown state..
        newPort->adsStateOld   = newPort->adsState;
        newPort->refreshNeeded = false; // This is actually all initialized!!
        amsPortList_.push_back(newPort);
    }
    catch (std::exception& e)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Failed to add new amsPort to list. Exception: %s.\n",
                  driverName,
                  functionName,
                  e.what());
        return asynError;
    }
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Added new amsPort to amsPortList: %d .\n",
              driverName,
              functionName,
              amsPort);

    return asynSuccess;
}

/** Checks if callback is allowed for a certain parameter.
 * \param[in] paramInfo Parameter info structure.
 *
 * \return true if parameter information and ams-port connection is OK
 *  otherwise false.
 */
bool adsAsynPortDriver::isCallbackAllowed(adsParamInfo* paramInfo)
{
    return !paramInfo->refreshNeeded;
}

/** Checks if callback is allowed for a certain ams-port.
 * \param[in] amsPort amsPort.
 *
 * \return true if connection ti ams-port is ok otherwise false.
 */
bool adsAsynPortDriver::isCallbackAllowed(uint16_t amsPort)
{
    const char* functionName = "isCallbackAllowed";
    asynPrint(
        pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: amsPort:%u\n", driverName, functionName, amsPort);

    for (amsPortInfo* port : amsPortList_)
    {
        if (port->amsPort == amsPort)
            return port->paramsOK;
    }
    return false;
}

/** Overrides asynPortDriver:readOctet.
 * This method, together with writeOctet, implements an ASCII command parser.
 * Mainly used for motor record and stream device access. pasynUser->reason==0
 * is reserved for this interface (and also pAdsParamArray_[0]).
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Buffer for read data.
 * \param[in] maxChars Size of value buffer.
 * \param[out] nActual Actual written chars to buffer.
 * \param[out] eomReason Read completed or not (Buffer to small
 * results in more reads needed).
 *
 * \return asynSuccess or asynError.
 *
 * \note: Example of a few ASCII commands:\n
 *  1. Symbolic read: "option1/option2/symbolicname?;":\n
 *      Read a var on ams-port 851: "ADSPORT=851/Main.M1.fPosition?;"\n
 *  2. Symbolic write: "option1/option2/symbolicname=<value>;":\n
 *      Write to a var on ams-port 851: "ADSPORT=851/Main.M1.fPosition=10;"\n
 *  3: Abs address read: "option1/.ADR.16#<group>,<offset>,<size>,<type>?;"\n
 *      Read low soflimit position in TwinCAT NC for axis 1:\n
 *      "ADSPORT=501/.ADR.16#5001,D,8,5?;"\n
 *  4: Abs address write: "option1/.ADR.16#<group>,<offset>,<size>,<type>=<value>;"\n
 *      Set low soflimit position in TwinCAT NC for axis 1 to 100:\n
 *      "ADSPORT=501/.ADR.16#5001,D,8,5=100;"\n
 */
asynStatus adsAsynPortDriver::readOctet(
    asynUser* pasynUser, char* value, size_t maxChars, size_t* nActual, int* eomReason)
{
    const char* functionName = "readOctet";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    size_t thisRead   = 0;
    int reason        = 0;
    asynStatus status = asynSuccess;

    *value = '\0';
    lock();
    int error = octetCMDreadIt(amsClientPort, value, maxChars);
    if (error)
    {
        status = asynError;
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: error, CMDreadIt failed (0x%x).\n",
                  driverName,
                  functionName,
                  error);
        unlock();
        return asynError;
    }

    thisRead = strlen(value);
    *nActual = thisRead;

    /* May be not enough space ? */
    if (thisRead > maxChars - 1)
    {
        reason |= ASYN_EOM_CNT;
    }
    else
    {
        reason |= ASYN_EOM_EOS;
    }

    if (thisRead == 0 && pasynUser->timeout == 0)
    {
        status = asynTimeout;
    }

    if (eomReason)
    {
        *eomReason = reason;
    }

    *nActual = thisRead;
    asynPrint(pasynUser,
              ASYN_TRACE_FLOW,
              "%s thisRead=%lu data=\"%s\"\n",
              portName,
              (unsigned long)thisRead,
              value);
    unlock();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:%s\n", driverName, functionName, value);

    return status;
}

/** Implements part of the asyn-octet ASCII command parser.
 * (see readOctet() and writeOctet for more info).
 * \param[in] outbuf Buffer for read data.
 * \param[in] outlen Size of value buffer.
 *
 * \return 0 for success or error code.
 */
int adsAsynPortDriver::octetCMDreadIt(uint16_t amsClientPort, char* outbuf, size_t outlen)
{
    const char* functionName = "octetCMDreadIt";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Buffer: %s, size: %d\n",
              driverName,
              functionName,
              outbuf,
              (int)outlen);

    int ret;
    if (!outbuf || !outlen)
    {
        return -1;
    }
    ret = snprintf(outbuf, outlen + 1, "%s", octetAsciiBuffer_.buffer);

    if (ret < 0)
    {
        octetClearBuffer(&octetAsciiBuffer_);
        return ret;
    }

    if (ret >= (int)outlen + 1)
    {
        ret = outlen;
    }
    octetRemoveFromBuffer(&octetAsciiBuffer_, ret);

    return 0;
}

/** Overrides asynPortDriver::writeOctet.
 * This method, together with readOctet, implements an ASCII command parser.
 * Mainly used for motor record and stream device access. pasynUser->reason==0
 * is reserved for this interface (and also pAdsParamArray_[0]).
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Buffer for read data.
 * \param[in] maxChars Size of value buffer.
 * \param[out] nActual Actual written chars to buffer.
 *
 * \return asynSuccess or asynError.
 *
 * \note: Example of a few ASCII commands:\n
 *  1. Symbolic read: "option1/option2/symbolicname?;":\n
 *      Read a var on ams-port 851: "ADSPORT=851/Main.M1.fPosition?;"\n
 *  2. Symbolic write: "option1/option2/symbolicname=<value>;":\n
 *      Write to a var on ams-port 851: "ADSPORT=851/Main.M1.fPosition=10;"\n
 *  3: Abs adress read: "option1/.ADR.16#<group>,<offset>,<size>,<type>?;"\n
 *      Read low softlimit position in TwinCAT NC for axis 1:\n
 *      "ADSPORT=501/.ADR.16#5001,D,8,5?;"\n
 *  4: Abs adress write: "option1/.ADR.16#<group>,<offset>,<size>,<type>=<value>;"\n
 *      Set low softlimit position in TwinCAT NC for axis 1 to 100:\n
 *      "ADSPORT=501/.ADR.16#5001,D,8,5=100;"\n
 */
asynStatus adsAsynPortDriver::writeOctet(asynUser* pasynUser,
                                         const char* value,
                                         size_t maxChars,
                                         size_t* nActual)
{
    const char* functionName = "writeOctet";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: %s\n", driverName, functionName, value);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    size_t thisWrite  = 0;
    asynStatus status = asynError;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s write.\n", portName);
    asynPrintIO(pasynUser,
                ASYN_TRACEIO_DRIVER,
                value,
                maxChars,
                "%s write %lu\n",
                portName,
                (unsigned long)maxChars);
    *nActual = 0;

    if (maxChars == 0)
    {
        return asynSuccess;
    }

    int errorCode = octetCMDwriteIt(amsClientPort, value, maxChars);
    if (errorCode)
    {
        /*Return asyn error if communication is down (all client errors) otherwise asynSuccess
     * but error message in buffer*/
        if (errorCode >= ADSERR_CLIENT_ERROR)
        {
            return asynError;
        }
    }
    status    = asynSuccess;
    thisWrite = maxChars;
    *nActual  = thisWrite;

    asynPrint(pasynUser,
              ASYN_TRACE_FLOW,
              "%s wrote %lu return %s.\n",
              portName,
              (unsigned long)*nActual,
              pasynManager->strStatus(status));
    return status;
}

/** Implements part of the asyn-octet ASCII command parser.
 * (see readOctet() and writeOctet for more info).
 * \param[in] inbuf Buffer for read data.
 * \param[in] inlen Size of value buffer.
 *
 * \return 0 for success or error code.
 */
int adsAsynPortDriver::octetCMDwriteIt(uint16_t amsClientPort, const char* inbuf, size_t inlen)
{
    const char* functionName = "octetCMDwriteIt";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Write command: %s, length: %d\n",
              driverName,
              functionName,
              inbuf,
              (int)inlen);

    int had_cr = 0;
    int had_lf = 0;
    int errorCode;
    char* new_buf = (char*)inbuf;
    if (!inbuf || !inlen)
        return -1;

    new_buf = (char*)malloc(inlen + 1);
    if (!new_buf)
        return -1;

    memcpy(new_buf, inbuf, inlen);
    new_buf[inlen] = 0;

    if (inlen > 1 && new_buf[inlen - 1] == '\n')
    {
        had_lf             = 1;
        new_buf[inlen - 1] = '\0';
        inlen--;
        if (inlen > 1 && new_buf[inlen - 1] == '\r')
        {
            had_cr             = 1;
            new_buf[inlen - 1] = '\0';
            inlen--;
        }
    }

    errorCode = octetCmdHandleInputLine(amsClientPort, new_buf, &octetAsciiBuffer_);
    free(new_buf);

    octetCmdBuf_printf(&octetAsciiBuffer_, "%s%s", had_cr ? "\r" : "", had_lf ? "\n" : "");

    return errorCode;
}

int adsAsynPortDriver::octetCmdHandleInputLine(uint16_t amsClientPort,
                                               const char* input_line,
                                               adsOctetOutputBufferType* buffer)
{
    const char* functionName = "octetCmdHandleInputLine";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Input line: %s\n",
              driverName,
              functionName,
              input_line);

    const char** my_argv = NULL;
    char** my_sepv       = NULL;
    int argc = octetCreateArgvSepv(input_line, (const char***)&my_argv, (char***)&my_sepv);

    int errorCodeLatch = 0;
    for (int i = 1; i <= argc; i++)
    {
        int errorCode = octetMotorHandleOneArg(
            amsClientPort, my_argv[i], buffer); //Continue with next cmd even if error
        if (errorCode && !errorCodeLatch)
        { //latch first error code for stacked commands
            errorCodeLatch = errorCode;
        }
        octetCmdBuf_printf(buffer, "%s", my_sepv[i]);
    }

    for (int i = 0; i <= argc; i++)
    {
        free((void*)my_argv[i]);
        free((void*)my_sepv[i]);
    }
    free(my_argv);
    free(my_sepv);

    return errorCodeLatch; //First encountered error code
}

/** Parse one ascii command.\
 * Implements part of the asyn-octet ASCII command parser.
 * (see readOctet() and writeOctet for more info).
 * \param[in] myarg_1 Command to parse.
 * \param[out] buffer Output buffer.
 *
 * \return 0 for success or error code.
 */
int adsAsynPortDriver::octetMotorHandleOneArg(uint16_t amsClientPort,
                                              const char* myarg_1,
                                              adsOctetOutputBufferType* buffer)
{
    const char* functionName = "octetMotorHandleOneArg";
    asynPrint(
        pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Command: %s\n", driverName, functionName, myarg_1);

    //const char *myarg = myarg_1;
    int err_code = 0;

    uint16_t amsPort =
        amsportDefault_; //should actually be called amsport ( 851 for first plc as default) ...

    /* ADSPORT= */
    if (!strncmp(myarg_1, ADS_OPTION_ADSPORT, strlen(ADS_OPTION_ADSPORT)))
    {
        myarg_1 += strlen(ADS_OPTION_ADSPORT) + 1; //+1 Because equal sign
        int nvals = sscanf(myarg_1, "%" SCNu16, &amsPort);
        if (nvals != 1)
        {
            err_code = ADS_COM_ERROR_OCTET_ADSPORT_OPTION_FAIL;
            OCTET_RETURN_ERROR(buffer, err_code, "%s", "ADS_COM_ERROR_OCTET_ADSPORT_OPTION_FAIL");
        }
        myarg_1 = strchr(myarg_1, '/');
        myarg_1++;
    }

    /* .THIS.sFeatures? */
    if (0 == strcmp(myarg_1, ADS_OCTET_FEATURES_COMMAND))
    {
#ifdef DUT_AXIS_STATUS
        const char* feature_str = "ads;stv1";
#else
        const char* feature_str = "ads";
#endif
        octetCmdBuf_printf(buffer, "%s", feature_str);
        return 0;
    }

    /*.ADR.*/
    const char* adr = strstr(myarg_1, ADS_ADR_COMMAND_PREFIX);
    if (adr)
    {
        myarg_1 = adr;

        err_code = octetMotorHandleADRCmd(amsClientPort, myarg_1, amsPort, buffer);
        if (err_code == -1 || err_code == 0)
        {
            return 0;
        }
        OCTET_RETURN_ERROR(buffer, err_code, "%s\n", adsErrorToString(err_code));
    }

    char variableName[255];
    memset(&variableName, 0, sizeof(variableName));

    //symbolic write
    adr = strchr(myarg_1, '=');
    if (adr)
    {
        //Copy variable name
        strncpy(variableName, myarg_1, adr - myarg_1);
        adr++; //Jump over '='
        err_code = octetAdsWriteByName(amsClientPort, amsPort, variableName, adr, buffer);
        if (err_code)
        {
            OCTET_RETURN_ERROR(buffer, err_code, "%s", adsErrorToString(err_code));
        }
        octetCmdBuf_printf(buffer, "OK");
        return 0;
    }

    //symbolic read
    adr = strchr(myarg_1, '?');
    if (adr)
    {
        //Copy variable name
        strncpy(variableName, myarg_1, adr - myarg_1);
        variableName[adr - myarg_1] = 0;
        err_code = octetAdsReadByName(amsClientPort, amsPort, variableName, buffer);
        if (err_code)
        {
            OCTET_RETURN_ERROR(buffer, err_code, "%s", adsErrorToString(err_code));
        }
        return 0;
    }
    /*  if we come here, it is a bad command */
    octetCmdBuf_printf(buffer, "Error: Bad command");
    return 0;
}

/** Parse one ASCII .ADR. command.\
 * Implements part of the asyn-octet ASCII command parser.
 * (see readOctet() and writeOctet for more info).
 * \param[in] arg Command to parse.
 * \param[in] amsport Ams-port.
 * \param[out] buffer Output buffer.
 *
 * \return 0 for success or error code.
 *
 * \note:  see octetAdsWriteByGroupOffset for more information.\n
 */
int adsAsynPortDriver::octetMotorHandleADRCmd(uint16_t amsClientPort,
                                              const char* arg,
                                              uint16_t amsport,
                                              adsOctetOutputBufferType* buffer)
{
    const char* functionName = "octetMotorHandleADRCmd";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Command: %s, amsPort: %d\n",
              driverName,
              functionName,
              arg,
              (int)amsport);

    const char* myarg_1      = NULL;
    unsigned group_no        = 0;
    unsigned offset_in_group = 0;
    unsigned len_in_PLC      = 0;
    unsigned type_in_PLC     = 0;
    int nvals;
    nvals = sscanf(
        arg, ".ADR.16#%x,16#%x,%u,%u=", &group_no, &offset_in_group, &len_in_PLC, &type_in_PLC);

    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: nvals=%d amsport=%u group_no=0x%x offset_in_group=0x%x len_in_PLC=%u "
              "type_in_PLC=%u\n",
              driverName,
              functionName,
              nvals,
              amsport,
              group_no,
              offset_in_group,
              len_in_PLC,
              type_in_PLC);

    if (nvals != 4)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Failed to parse .ADR. command.\n",
                  driverName,
                  functionName);
        return __LINE__;
    }

    //WRITE
    myarg_1 = strchr(arg, '=');
    if (myarg_1)
    {
        myarg_1++; /* Jump over '=' */

        int error = octetAdsWriteByGroupOffset(amsClientPort,
                                               amsport,
                                               (uint32_t)group_no,
                                               (uint32_t)offset_in_group,
                                               (uint16_t)type_in_PLC,
                                               (uint32_t)len_in_PLC,
                                               myarg_1,
                                               buffer);
        if (error)
        {
            OCTET_RETURN_ERROR(buffer, error, "%s", adsErrorToString(error));
        }
        octetCmdBuf_printf(buffer, "OK");
        return 0;
    }

    //READ
    myarg_1 = strchr(arg, '?');
    if (myarg_1)
    {
        myarg_1++; /* Jump over '?' */
        adsSymbolEntry info;
        memset(&info, 0, sizeof(info));
        info.dataType = type_in_PLC;
        info.size     = len_in_PLC;
        info.iGroup   = group_no;
        info.iOffset  = offset_in_group;

        int error = octetAdsReadByGroupOffset(amsClientPort, amsport, &info, buffer);
        if (error)
        {
            OCTET_RETURN_ERROR(buffer, error, "%s", adsErrorToString(error));
        }
        return 0;
    }
    return __LINE__;
}

/** Read a variable from PLC by symbolic addressing.\
 * Implements part of the asyn-octet ASCII command parser.
 * (see readOctet() and writeOctet for more info).
 * \param[in] amsport Ams-port.
 * \param[in] variableAddr Variable name ("Main.fTest")
 * \param[out] outBuffer Output buffer.
 *
 * \return 0 for success or error code.
 */
int adsAsynPortDriver::octetAdsReadByName(uint16_t amsClientPort,
                                          uint16_t amsPort,
                                          const char* variableAddr,
                                          adsOctetOutputBufferType* outBuffer)
{
    const char* functionName = "octetAdsReadByName";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Variable:%s, amsPort %u\n",
              driverName,
              functionName,
              variableAddr,
              amsPort);

    adsSymbolEntry infoStruct;
    memset(&infoStruct, 0, sizeof(infoStruct));

    long errorCode = 0;
    asynStatus stat =
        adsGetSymInfoByName(amsClientPort, amsPort, variableAddr, &infoStruct, &errorCode);
    if (stat != asynSuccess)
    {
        return errorCode;
    }

    return octetAdsReadByGroupOffset(amsClientPort, amsPort, &infoStruct, outBuffer);
}

/** Write a variable to PLC by symbolic addressing.\
 * Implements part of the asyn-octet ASCII command parser.
 * (see readOctet() and writeOctet for more info).
 * \param[in] amsport Ams-port.
 * \param[in] variableAddr Variable name ("Main.fTest")
 * \param[in] asciiValueToWrite Value to write in string format.
 * \param[out] outBuffer Output buffer.
 *
 * \return 0 for success or error code.
 */
int adsAsynPortDriver::octetAdsWriteByName(uint16_t amsClientPort,
                                           uint16_t amsPort,
                                           const char* variableAddr,
                                           const char* asciiValueToWrite,
                                           adsOctetOutputBufferType* outBuffer)
{
    const char* functionName = "octetAdsWriteByName";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Variable: %s, value: %s.\n",
              driverName,
              functionName,
              variableAddr,
              asciiValueToWrite);

    adsSymbolEntry infoStruct;
    memset(&infoStruct, 0, sizeof(infoStruct));

    long errorCode = 0;
    asynStatus stat =
        adsGetSymInfoByName(amsClientPort, amsPort, variableAddr, &infoStruct, &errorCode);
    if (stat != asynSuccess)
    {
        return errorCode;
    }

    return octetAdsWriteByGroupOffset(amsClientPort,
                                      amsPort,
                                      infoStruct.iGroup,
                                      infoStruct.iOffset,
                                      infoStruct.dataType,
                                      infoStruct.size,
                                      asciiValueToWrite,
                                      outBuffer);
}

/**Read a variable from PLC by absolute addressing.\
 * Implements part of the asyn-octet ASCII command parser.
 * (see readOctet() and writeOctet for more info).
 * \param[in] amsport Ams-port.
 * \param[in] info Variable information.
 * \param[out] outBuffer Output buffer.
 *
 * \return 0 for success or error code.
 */
int adsAsynPortDriver::octetAdsReadByGroupOffset(uint16_t amsClientPort,
                                                 uint16_t amsPort,
                                                 adsSymbolEntry* info,
                                                 adsOctetOutputBufferType* outBuffer)
{
    const char* functionName = "octetAdsReadByGroupOffset";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: amsPort: %d, group: %d, offset: %d, dataType: %s (%d), dataSize: %d.\n",
              driverName,
              functionName,
              (int)amsPort,
              (int)info->iGroup,
              (int)info->iOffset,
              adsTypeToString(info->dataType),
              (int)info->dataType,
              (int)info->size);

    uint32_t bytesRead = 0;
    AmsAddr amsServer  = {remoteNetId_, amsPort};

    int dataSize = info->size;
    if (info->size > ADS_CMD_BUFFER_SIZE)
    {
        dataSize = ADS_CMD_BUFFER_SIZE;
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_WARNING,
                  "%s:%s: Read buffer size smaller than size in plc.\n",
                  driverName,
                  functionName);
    }

    memset(&octetBinaryBuffer_, 0, ADS_CMD_BUFFER_SIZE);

    int error = AdsSyncReadReqEx2(amsClientPort,
                                  &amsServer,
                                  info->iGroup,
                                  info->iOffset,
                                  dataSize,
                                  &octetBinaryBuffer_,
                                  &bytesRead);

    if (error)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: ADS read failed with: %s (0x%x).\n",
                  driverName,
                  functionName,
                  adsErrorToString(error),
                  error);
        return error;
    }

    error = octetBinary2ascii(
        octetReturnVarName_, &octetBinaryBuffer_, ADS_CMD_BUFFER_SIZE, info, outBuffer);
    if (error)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Binary to ASCII conversion failed with: %d\n",
                  driverName,
                  functionName,
                  error);
        return error;
    }

    return 0;
}

/**Write a variable to PLC by absolute addressing.\
 * Implements part of the asyn-octet ASCII command parser.
 * (see readOctet() and writeOctet for more info).
 * \param[in] amsport Ams-port.
 * \param[in] group Group (address).
 * \param[in] offset Offset in group (address).
 * \param[in] dataType Data type to write (address).
 * \param[in] dataSize Bytes to write.
 * \param[out] asciiResponseBuffer Output buffer.
 *
 * \return 0 for success or error code.
 *
 * \note: dataType is defined in the adsLib as:
 *   Name:         dataType:  dataSize/element (bytes):\n
 *   ADST_VOID     0          0\n
 *   ADST_INT8     16         1\n
 *   ADST_UINT8    17         1\n
 *   ADST_INT16    2          2\n
 *   ADST_UINT16   18         2\n
 *   ADST_INT32    3          4\n
 *   ADST_UINT32   19         4\n
 *   ADST_INT64    20         8\n
 *   ADST_UINT64   21         8\n
 *   ADST_REAL32   4          4\n
 *   ADST_REAL64   5          8\n
 *   ADST_BIGTYPE  65         NAN\n
 *   ADST_STRING   30         1\n
 *   ADST_WSTRING  31         1\n
 *   ADST_REAL80   32         10\n
 *   ADST_BIT      33         1\n
 *   \n
 *   The data will be considered to be an array if dataSize is bigger than the\n
 *   size of the the type.
 */
int adsAsynPortDriver::octetAdsWriteByGroupOffset(uint16_t amsClientPort,
                                                  uint16_t amsPort,
                                                  uint32_t group,
                                                  uint32_t offset,
                                                  uint16_t dataType,
                                                  uint32_t dataSize,
                                                  const char* asciiValueToWrite,
                                                  adsOctetOutputBufferType* asciiResponseBuffer)
{
    const char* functionName = "octetAdsWriteByGroupOffset";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: amsPort: %d, group: %d, offset: %d, dataType: %s (%d), dataSize: %d.\n",
              driverName,
              functionName,
              (int)amsPort,
              (int)group,
              (int)offset,
              adsTypeToString(dataType),
              (int)dataType,
              (int)dataSize);

    uint32_t bytesToWrite = 0;
    AmsAddr amsServer     = {remoteNetId_, amsPort};

    memset(&octetBinaryBuffer_, 0, ADS_CMD_BUFFER_SIZE);

    int error = octetAscii2binary(
        asciiValueToWrite, dataType, &octetBinaryBuffer_, ADS_CMD_BUFFER_SIZE, &bytesToWrite);
    if (error)
    {
        octetCmdBuf_printf(asciiResponseBuffer, "Error: %x", error);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: ASCII to binary conversion failed with: %d.\n",
                  driverName,
                  functionName,
                  error);
        return error;
    }

    if (bytesToWrite > dataSize)
    {
        bytesToWrite = dataSize;
    }

    error = AdsSyncWriteReqEx(
        amsClientPort, &amsServer, group, offset, bytesToWrite, &octetBinaryBuffer_);

    if (error)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: ADS write failed with: %s (0x%x).\n",
                  driverName,
                  functionName,
                  adsErrorToString(error),
                  error);
        return error;
    }

    return 0;
}

/** Overrides asynPortDriver::readInt32.
 * Reads int32 directly from PLC via AdsSyncReadReqEx2, bypassing bulk thread.
 * \param[in]  pasynUser Pointer to asyn user structure.
 * \param[out] value     Value read from PLC.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::readInt32(asynUser* pasynUser, epicsInt32* value)
{
    const char* functionName = "readInt32";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    int paramIndex = pasynUser->reason;
    auto paramInfo = &adsParamArray_[paramIndex];

    // adsReadParam(..., 1) issues AdsSyncReadReqEx2, then calls
    // adsUpdateParameterLock -> setIntegerParam/setDoubleParam under the asyn
    // port lock.  The base-class readInt32 then retrieves the value from the
    // parameter table.  This completely bypasses the bulk poll thread.
    long errorCode = 0;
    if (adsReadParam(amsClientPort, paramInfo, &errorCode, 1) != asynSuccess)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: adsReadParam failed for %s (0x%lx)\n",
                  driverName,
                  functionName,
                  paramInfo->drvInfo,
                  errorCode);
        return setAlarmParam(paramInfo, READ_ALARM, INVALID_ALARM);
    }
    if (paramInfo->alarmStatus == READ_ALARM)
        setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);

    return asynPortDriver::readInt32(pasynUser, value);
}

/** Overrides asynPortDriver::readFloat64.
 * Reads float64 directly from PLC via AdsSyncReadReqEx2, bypassing bulk thread.
 * \param[in]  pasynUser Pointer to asyn user structure.
 * \param[out] value     Value read from PLC.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::readFloat64(asynUser* pasynUser, epicsFloat64* value)
{
    const char* functionName = "readFloat64";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    int paramIndex = pasynUser->reason;
    auto paramInfo = &adsParamArray_[paramIndex];

    long errorCode = 0;
    if (adsReadParam(amsClientPort, paramInfo, &errorCode, 1) != asynSuccess)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: adsReadParam failed for %s (0x%lx)\n",
                  driverName,
                  functionName,
                  paramInfo->drvInfo,
                  errorCode);
        return setAlarmParam(paramInfo, READ_ALARM, INVALID_ALARM);
    }
    if (paramInfo->alarmStatus == READ_ALARM)
        setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);

    return asynPortDriver::readFloat64(pasynUser, value);
}

/** Overrides asynPortDriver::writeInt32.
 * Writes int32 to PLC
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Value to write.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::writeInt32(asynUser* pasynUser, epicsInt32 value)
{
    const char* functionName = "writeInt32";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    int paramIndex = pasynUser->reason;

    if (paramIndex >= getAdsParamCount())
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: param index invalid. Greater than param count.\n",
                  driverName,
                  functionName);
        pasynUser->alarmStatus = WRITE_ALARM;
        return asynError;
    }

    auto paramInfo = &adsParamArray_[paramIndex];

    //Special case. Check if write ams port state
    if (paramInfo->dataSource == ADS_DATASOURCE_AMS_STATE)
    {
        if (adsWriteState(amsClientPort, paramInfo->amsPort, (uint16_t)value) != asynSuccess)
        {
            return setAlarmParam(paramInfo, WRITE_ALARM, INVALID_ALARM);
        }
        // Write OK -> reset write alarm
        if (paramInfo->alarmStatus == WRITE_ALARM)
        {
            return setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);
        }
        return asynSuccess;
    }

    uint8_t buffer[8]; //largest datatype is 8bytes
    uint32_t maxBytesToWrite = 0;
    // Convert epicsInt32 to plctype if possible..
    switch (paramInfo->plcDataType)
    {
    case ADST_INT8:
        int8_t* ADST_INT8Var;
        ADST_INT8Var    = ((int8_t*)buffer);
        *ADST_INT8Var   = (int8_t)value;
        maxBytesToWrite = 1;
        break;
    case ADST_INT16:
        int16_t* ADST_INT16Var;
        ADST_INT16Var   = ((int16_t*)buffer);
        *ADST_INT16Var  = (int16_t)value;
        maxBytesToWrite = 2;
        break;
    case ADST_INT32:
        int32_t* ADST_INT32Var;
        ADST_INT32Var   = ((int32_t*)buffer);
        *ADST_INT32Var  = (int32_t)value;
        maxBytesToWrite = 4;
        break;
    case ADST_UINT8:
        uint8_t* ADST_UINT8Var;
        ADST_UINT8Var   = ((uint8_t*)buffer);
        *ADST_UINT8Var  = (uint8_t)value;
        maxBytesToWrite = 1;
        break;
    case ADST_UINT16:
        uint16_t* ADST_UINT16Var;
        ADST_UINT16Var  = ((uint16_t*)buffer);
        *ADST_UINT16Var = (uint16_t)value;
        maxBytesToWrite = 2;
        break;
    case ADST_UINT32:
        uint32_t* ADST_UINT32Var;
        ADST_UINT32Var  = ((uint32_t*)buffer);
        *ADST_UINT32Var = (uint32_t)value;
        maxBytesToWrite = 4;
        break;
    case ADST_REAL32:
        float* ADST_REAL32Var;
        ADST_REAL32Var  = ((float*)buffer);
        *ADST_REAL32Var = (float)value;
        maxBytesToWrite = 4;
        break;
    case ADST_REAL64:
        double* ADST_REAL64Var;
        ADST_REAL64Var  = ((double*)buffer);
        *ADST_REAL64Var = (double)value;
        maxBytesToWrite = 8;
        break;
    case ADST_BIT:
        buffer[0]       = value > 0;
        maxBytesToWrite = 1;
        break;
    default:
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Data types not compatible (epicsInt32 and %s). Write canceled.\n",
                  driverName,
                  functionName,
                  adsTypeToString(paramInfo->plcDataType));
        return asynError;
        break;
    }

    // Warning. Risk of loss of data..
    if (sizeof(value) > maxBytesToWrite || sizeof(value) > paramInfo->plcSize)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_WARNING,
                  "%s:%s: WARNING. EPICS datatype size larger than PLC datatype size (%ld vs %d "
                  "bytes).\n",
                  driverName,
                  functionName,
                  sizeof(value),
                  paramInfo->plcSize);
        paramInfo->plcDataTypeWarn = true;
    }

    //Ensure that PLC datatype and number of bytes to write match
    if (maxBytesToWrite != paramInfo->plcSize || maxBytesToWrite == 0)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Data types size missmatch (%s and %d bytes). Write canceled.\n",
                  driverName,
                  functionName,
                  adsTypeToString(paramInfo->plcDataType),
                  maxBytesToWrite);
        setAlarmParam(paramInfo, WRITE_ALARM, INVALID_ALARM);
        return asynError;
    }

    //Do the write
    if (adsWriteParam(amsClientPort, paramInfo, (const void*)buffer, maxBytesToWrite) !=
        asynSuccess)
    {
        setAlarmParam(paramInfo, WRITE_ALARM, INVALID_ALARM);
        return asynError;
    }
    //Only reset if write alarm
    if (paramInfo->alarmStatus == WRITE_ALARM)
    {
        setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);
    }

    return asynPortDriver::writeInt32(pasynUser, value);
}

/** Overrides asynPortDriver::readInt64.
 * Reads int64 directly from PLC via AdsSyncReadReqEx2, bypassing bulk thread.
 * \param[in]  pasynUser Pointer to asyn user structure.
 * \param[out] value     Value read from PLC.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::readInt64(asynUser* pasynUser, epicsInt64* value)
{
    const char* functionName = "readInt64";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    int paramIndex = pasynUser->reason;
    if (paramIndex >= getAdsParamCount())
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: param index invalid. Greater than registered param count.\n",
                  driverName,
                  functionName);
        return asynError;
    }
    auto paramInfo = &adsParamArray_[paramIndex];

    long errorCode = 0;
    if (adsReadParam(amsClientPort, paramInfo, &errorCode, 1) != asynSuccess)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: adsReadParam failed for %s (0x%lx)\n",
                  driverName,
                  functionName,
                  paramInfo->drvInfo,
                  errorCode);
        return setAlarmParam(paramInfo, READ_ALARM, INVALID_ALARM);
    }
    if (paramInfo->alarmStatus == READ_ALARM)
        setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);

    return asynPortDriver::readInt64(pasynUser, value);
}

asynStatus adsAsynPortDriver::writeInt64(asynUser* pasynUser, epicsInt64 value)
{
    const char* functionName = "writeInt64";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    int paramIndex = pasynUser->reason;
    if (paramIndex >= getAdsParamCount())
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: param index invalid. Greater than registered param count.\n",
                  driverName,
                  functionName);
        return asynError;
    }
    auto paramInfo = &adsParamArray_[paramIndex];

    // Special case: AMS port state
    if (paramInfo->dataSource == ADS_DATASOURCE_AMS_STATE)
    {
        if (adsWriteState(amsClientPort, paramInfo->amsPort, (uint16_t)value) != asynSuccess)
        {
            return setAlarmParam(paramInfo, WRITE_ALARM, INVALID_ALARM);
        }
        if (paramInfo->alarmStatus == WRITE_ALARM)
        {
            return setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);
        }
        return asynSuccess;
    }

    uint8_t buffer[8]; // 8 bytes for int64_t/uint64_t/double
    uint32_t maxBytesToWrite = 0;

    switch (paramInfo->plcDataType)
    {
    case ADST_INT64:
    {
        int64_t* ADST_INT64Var = (int64_t*)buffer;
        *ADST_INT64Var         = (int64_t)value;
        maxBytesToWrite        = 8;
        break;
    }
    case ADST_UINT64:
    {
        uint64_t* ADST_UINT64Var = (uint64_t*)buffer;
        *ADST_UINT64Var          = (uint64_t)value; // User beware: signed->unsigned cast!
        maxBytesToWrite          = 8;
        break;
    }
    default:
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Data types not compatible (epicsInt64 and %s). Write canceled.\n",
                  driverName,
                  functionName,
                  adsTypeToString(paramInfo->plcDataType));
        return asynError;
    }

    // Sanity: Check PLC buffer sizes
    if (sizeof(value) > maxBytesToWrite || sizeof(value) > paramInfo->plcSize)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_WARNING,
                  "%s:%s: WARNING. EPICS datatype size larger than PLC datatype size (%ld vs %d "
                  "bytes).\n",
                  driverName,
                  functionName,
                  sizeof(value),
                  paramInfo->plcSize);
        paramInfo->plcDataTypeWarn = true;
    }

    // Ensure match
    if (maxBytesToWrite != paramInfo->plcSize || maxBytesToWrite == 0)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Data types size mismatch (%s and %d bytes). Write canceled.\n",
                  driverName,
                  functionName,
                  adsTypeToString(paramInfo->plcDataType),
                  maxBytesToWrite);
        setAlarmParam(paramInfo, WRITE_ALARM, INVALID_ALARM);
        return asynError;
    }

    // Write the value
    if (adsWriteParam(amsClientPort, paramInfo, buffer, maxBytesToWrite) != asynSuccess)
    {
        setAlarmParam(paramInfo, WRITE_ALARM, INVALID_ALARM);
        return asynError;
    }
    if (paramInfo->alarmStatus == WRITE_ALARM)
    {
        setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);
    }

    return asynPortDriver::writeInt64(pasynUser, value);
}


/** Overrides asynPortDriver::writeFloat64.
 * Writes float64 to PLC
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Value to write.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::writeFloat64(asynUser* pasynUser, epicsFloat64 value)
{
    const char* functionName = "writeFloat64";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    int paramIndex = pasynUser->reason;
    if (paramIndex >= getAdsParamCount())
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: param index invalid. Greater than registered param count.\n",
                  driverName,
                  functionName);
        return asynError;
    }
    auto paramInfo = &adsParamArray_[paramIndex];

    //Special case. Check if write ams port state
    if (paramInfo->dataSource == ADS_DATASOURCE_AMS_STATE)
    {
        if (adsWriteState(amsClientPort, paramInfo->amsPort, (uint16_t)value) != asynSuccess)
        {
            return setAlarmParam(paramInfo, WRITE_ALARM, INVALID_ALARM);
        }
        // Write OK -> reset write alarm
        if (paramInfo->alarmStatus == WRITE_ALARM)
        {
            return setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);
        }
        return asynSuccess;
    }

    uint8_t buffer[8]; //largest datatype is 8bytes
    uint32_t maxBytesToWrite = 0;
    // Convert epicsFloat64 to plctype if possible..
    switch (paramInfo->plcDataType)
    {
    case ADST_INT8:
        int8_t* ADST_INT8Var;
        ADST_INT8Var    = ((int8_t*)buffer);
        *ADST_INT8Var   = (int8_t)value;
        maxBytesToWrite = 1;
        break;
    case ADST_INT16:
        int16_t* ADST_INT16Var;
        ADST_INT16Var   = ((int16_t*)buffer);
        *ADST_INT16Var  = (int16_t)value;
        maxBytesToWrite = 2;
        break;
    case ADST_INT32:
        int32_t* ADST_INT32Var;
        ADST_INT32Var   = ((int32_t*)buffer);
        *ADST_INT32Var  = (int32_t)value;
        maxBytesToWrite = 4;
        break;
    case ADST_INT64:
        int64_t* ADST_INT64Var;
        ADST_INT64Var   = ((int64_t*)buffer);
        *ADST_INT64Var  = (int64_t)value;
        maxBytesToWrite = 8;
        break;
    case ADST_UINT8:
        uint8_t* ADST_UINT8Var;
        ADST_UINT8Var   = ((uint8_t*)buffer);
        *ADST_UINT8Var  = (uint8_t)value;
        maxBytesToWrite = 1;
        break;
    case ADST_UINT16:
        uint16_t* ADST_UINT16Var;
        ADST_UINT16Var  = ((uint16_t*)buffer);
        *ADST_UINT16Var = (uint16_t)value;
        maxBytesToWrite = 2;
        break;
    case ADST_UINT32:
        uint32_t* ADST_UINT32Var;
        ADST_UINT32Var  = ((uint32_t*)buffer);
        *ADST_UINT32Var = (uint32_t)value;
        maxBytesToWrite = 4;
        break;
    case ADST_UINT64:
        uint64_t* ADST_UINT64Var;
        ADST_UINT64Var  = ((uint64_t*)buffer);
        *ADST_UINT64Var = (uint64_t)value;
        maxBytesToWrite = 8;
        break;
    case ADST_REAL32:
        float* ADST_REAL32Var;
        ADST_REAL32Var  = ((float*)buffer);
        *ADST_REAL32Var = (float)value;
        maxBytesToWrite = 4;
        break;
    case ADST_REAL64:
        double* ADST_REAL64Var;
        ADST_REAL64Var  = ((double*)buffer);
        *ADST_REAL64Var = (double)value;
        maxBytesToWrite = 8;
        break;
    case ADST_BIT:
        buffer[0]       = value > 0;
        maxBytesToWrite = 1;
        break;
    default:
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Data types not compatible (epicsInt32 and %s). Write canceled.\n",
                  driverName,
                  functionName,
                  adsTypeToString(paramInfo->plcDataType));
        return asynError;
        break;
    }

    // Warning. Risk of loss of data..
    if (sizeof(value) > maxBytesToWrite || sizeof(value) > paramInfo->plcSize)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_WARNING,
                  "%s:%s: WARNING. EPICS datatype size larger than PLC datatype size (%ld vs %d "
                  "bytes).\n",
                  driverName,
                  functionName,
                  sizeof(value),
                  paramInfo->plcDataType);
        paramInfo->plcDataTypeWarn = true;
    }

    //Ensure that PLC datatype and number of bytes to write match
    if (maxBytesToWrite != paramInfo->plcSize || maxBytesToWrite == 0)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Data types size mismatch (%s and %d bytes). Write canceled.\n",
                  driverName,
                  functionName,
                  adsTypeToString(paramInfo->plcDataType),
                  maxBytesToWrite);
        setAlarmParam(paramInfo, WRITE_ALARM, INVALID_ALARM);
        return asynError;
    }

    //Do the write
    if (adsWriteParam(amsClientPort, paramInfo, (const void*)buffer, maxBytesToWrite) !=
        asynSuccess)
    {
        setAlarmParam(paramInfo, WRITE_ALARM, INVALID_ALARM);
        return asynError;
    }

    //Only reset if write alarm
    if (paramInfo->alarmStatus == WRITE_ALARM)
    {
        setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);
    }

    return asynPortDriver::writeFloat64(pasynUser, value);
}

/** Read array of a certain data type from PLC (or actually
 * paramlib,paraminfor->arrayDataBuffer, since all variables are updated
 * on-change by callbacks).
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] allowedType Allowed ads type to read.
 * \param[out] epicsDataBuffer Output buffer.
 * \param[in] nEpicsBufferBytes Output buffer size.
 * \param[out] nBytesRead Bytes read into buffer.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsGenericArrayRead(uint16_t amsClientPort,
                                                  asynUser* pasynUser,
                                                  long allowedType,
                                                  void* epicsDataBuffer,
                                                  size_t nEpicsBufferBytes,
                                                  size_t* nBytesRead)
{
    const char* functionName = "adsGenericArrayRead";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    int paramIndex = pasynUser->reason;
    if (paramIndex >= getAdsParamCount())
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: param index invalid. Greater than registered param count.\n",
                  driverName,
                  functionName);
        return asynError;
    }
    auto paramInfo = &adsParamArray_[paramIndex];

    //Only support same datatype as in PLC
    // Allow LWORD/ULINT as signed INT64
    bool extendedAllowedType = (paramInfo->plcDataType == allowedType) ||
                               (paramInfo->plcDataType == ADST_UINT64 && allowedType == ADST_INT64);

    if (!extendedAllowedType)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Data types not compatible (%s vs %s). Read canceled.\n",
                  driverName,
                  functionName,
                  adsTypeToString(paramInfo->plcDataType),
                  adsTypeToString(allowedType));
        setAlarmParam(paramInfo, READ_ALARM, INVALID_ALARM);
        return asynError;
    }

    size_t bytesToWrite = nEpicsBufferBytes;
    if (paramInfo->plcSize < nEpicsBufferBytes)
    {
        bytesToWrite = paramInfo->plcSize;
    }

    if (!paramInfo->arrayDataBuffer || !epicsDataBuffer)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Buffer(s) NULL. Read canceled.\n",
                  driverName,
                  functionName);
        setAlarmParam(paramInfo, READ_ALARM, INVALID_ALARM);
        return asynError;
    }

    memcpy(epicsDataBuffer, paramInfo->arrayDataBuffer, bytesToWrite);
    *nBytesRead = bytesToWrite;

    //Only reset if read alarm
    if (paramInfo->alarmStatus == READ_ALARM)
    {
        setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);
    }

    //update timestamp
    pasynUser->timestamp = paramInfo->epicsTimestamp;

    return asynSuccess;
}

/** Write array of a certain data type to PLC.
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] allowedType Allowed ads type to read.
 * \param[out] data Data to write.
 * \param[in] nEpicsBufferBytes Bytes to write.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsGenericArrayWrite(uint16_t amsClientPort,
                                                   asynUser* pasynUser,
                                                   long allowedType,
                                                   const void* data,
                                                   size_t nEpicsBufferBytes)
{
    const char* functionName = "adsGenericArrayWrite";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    int paramIndex = pasynUser->reason;
    if (paramIndex >= getAdsParamCount())
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: param index invalid. Greater than registered param count.\n",
                  driverName,
                  functionName);
        return asynError;
    }
    auto paramInfo = &adsParamArray_[paramIndex];

    //Only support same datatype as in PLC
    // Allow LWORD/ULINT as signed INT64
    bool extendedAllowedType = (paramInfo->plcDataType == allowedType) ||
                               (paramInfo->plcDataType == ADST_UINT64 && allowedType == ADST_INT64);

    if (!extendedAllowedType)
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Data types not compatible (%s vs %s). Write canceled.\n",
                  driverName,
                  functionName,
                  adsTypeToString(paramInfo->plcDataType),
                  adsTypeToString(allowedType));
        setAlarmParam(paramInfo, WRITE_ALARM, INVALID_ALARM);
        return asynError;
    }

    size_t bytesToWrite = nEpicsBufferBytes;
    if (paramInfo->plcSize < nEpicsBufferBytes)
    {
        bytesToWrite = paramInfo->plcSize;
    }

    //Write to ADS
    asynStatus stat = adsWriteParam(amsClientPort, paramInfo, data, bytesToWrite);
    if (stat != asynSuccess)
    {
        setAlarmParam(paramInfo, WRITE_ALARM, INVALID_ALARM);
        return asynError;
    }

    //copy data to buffer;
    if (paramInfo->arrayDataBuffer)
    {
        memcpy(paramInfo->arrayDataBuffer, data, bytesToWrite);
    }

    //Only reset if write alarm
    if (paramInfo->alarmStatus == WRITE_ALARM)
    {
        setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);
    }

    return asynSuccess;
}

/** Overrides asynPortDriver::readInt8Array.
 * Reads int8Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[out] value Output data buffer.
 * \param[in] nElements Output buffer size.
 * \param[out] nIn Bytes read into buffer.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::readInt8Array(asynUser* pasynUser,
                                            epicsInt8* value,
                                            size_t nElements,
                                            size_t* nIn)
{
    const char* functionName = "readInt8Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    int paramIndex = pasynUser->reason;
    if (paramIndex >= getAdsParamCount())
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: param index invalid. Greater than registered param count.\n",
                  driverName,
                  functionName);
        return asynError;
    }
    auto paramInfo = &adsParamArray_[paramIndex];

    long allowedType = ADST_INT8;

    //Also allow string and bool array as int8array (special case)
    if (paramInfo->plcDataType == ADST_STRING)
    {
        allowedType = ADST_STRING;
    }
    else if (paramInfo->plcDataType == ADST_BIT)
    {
        allowedType = ADST_BIT;
    }

    size_t nBytesRead = 0;
    asynStatus stat   = adsGenericArrayRead(amsClientPort,
                                          pasynUser,
                                          allowedType,
                                          (void*)value,
                                          nElements * sizeof(epicsInt8),
                                          &nBytesRead);
    if (stat != asynSuccess)
    {
        return asynError;
    }
    *nIn = nBytesRead / sizeof(epicsInt8);
    return asynSuccess;
}

/** Overrides asynPortDriver::writeInt8Array.
 * Writes int8Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Input data buffer.
 * \param[in] nElements Input data size.
 *
 * \return asynSuccess or asynError.
 */
asynStatus
adsAsynPortDriver::writeInt8Array(asynUser* pasynUser, epicsInt8* value, size_t nElements)
{
    const char* functionName = "writeInt8Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    int paramIndex = pasynUser->reason;
    if (paramIndex >= getAdsParamCount())
    {
        asynPrint(pasynUser,
                  ASYN_TRACE_ERROR,
                  "%s:%s: param index invalid. Greater than registered param count.\n",
                  driverName,
                  functionName);
        return asynError;
    }
    auto paramInfo = &adsParamArray_[paramIndex];

    long allowedType = ADST_INT8;

    //Also allow string and bool array as int8array (special case)
    if (paramInfo->plcDataType == ADST_STRING)
    {
        allowedType = ADST_STRING;
    }
    else if (paramInfo->plcDataType == ADST_BIT)
    {
        allowedType = ADST_BIT;
    }

    return adsGenericArrayWrite(
        amsClientPort, pasynUser, allowedType, (const void*)value, nElements * sizeof(epicsInt8));
}

/** Overrides asynPortDriver::readInt16Array.
 * Reads int16Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[out] value Output data buffer.
 * \param[in] nElements Output buffer size.
 * \param[out] nIn Bytes read into buffer.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::readInt16Array(asynUser* pasynUser,
                                             epicsInt16* value,
                                             size_t nElements,
                                             size_t* nIn)
{
    const char* functionName = "readInt16Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    long allowedType = ADST_INT16;

    size_t nBytesRead = 0;
    asynStatus stat   = adsGenericArrayRead(amsClientPort,
                                          pasynUser,
                                          allowedType,
                                          (void*)value,
                                          nElements * sizeof(epicsInt16),
                                          &nBytesRead);
    if (stat != asynSuccess)
    {
        return asynError;
    }
    *nIn = nBytesRead / sizeof(epicsInt16);
    return asynSuccess;
}

/** Overrides asynPortDriver::writeInt16Array.
 * Writes int16Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Input data buffer.
 * \param[in] nElements Input data size.
 *
 * \return asynSuccess or asynError.
 */
asynStatus
adsAsynPortDriver::writeInt16Array(asynUser* pasynUser, epicsInt16* value, size_t nElements)
{
    const char* functionName = "writeInt16Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    long allowedType = ADST_INT16;

    return adsGenericArrayWrite(
        amsClientPort, pasynUser, allowedType, (const void*)value, nElements * sizeof(epicsInt16));
}

/** Overrides asynPortDriver::readInt32Array.
 * Reads int32Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[out] value Output data buffer.
 * \param[in] nElements Output buffer size.
 * \param[out] nIn Bytes read into buffer.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::readInt32Array(asynUser* pasynUser,
                                             epicsInt32* value,
                                             size_t nElements,
                                             size_t* nIn)
{
    const char* functionName = "readInt32Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    long allowedType = ADST_INT32;

    size_t nBytesRead = 0;
    asynStatus stat   = adsGenericArrayRead(amsClientPort,
                                          pasynUser,
                                          allowedType,
                                          (void*)value,
                                          nElements * sizeof(epicsInt32),
                                          &nBytesRead);
    if (stat != asynSuccess)
    {
        return asynError;
    }
    *nIn = nBytesRead / sizeof(epicsInt32);
    return asynSuccess;
}


/** Overrides asynPortDriver::writeInt32Array.
 * Writes int32Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Input data buffer.
 * \param[in] nElements Input data size.
 *
 * \return asynSuccess or asynError.
 */
asynStatus
adsAsynPortDriver::writeInt32Array(asynUser* pasynUser, epicsInt32* value, size_t nElements)
{
    const char* functionName = "writeInt32Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    long allowedType = ADST_INT32;

    return adsGenericArrayWrite(
        amsClientPort, pasynUser, allowedType, (const void*)value, nElements * sizeof(epicsInt32));
}

/** Overrides asynPortDriver::readFloat32Array.
 * Reads float32Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[out] value Output data buffer.
 * \param[in] nElements Output buffer size.
 * \param[out] nIn Bytes read into buffer.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::readFloat32Array(asynUser* pasynUser,
                                               epicsFloat32* value,
                                               size_t nElements,
                                               size_t* nIn)
{
    const char* functionName = "readFloat32Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    long allowedType = ADST_REAL32;

    size_t nBytesRead = 0;
    asynStatus stat   = adsGenericArrayRead(amsClientPort,
                                          pasynUser,
                                          allowedType,
                                          (void*)value,
                                          nElements * sizeof(epicsFloat32),
                                          &nBytesRead);
    if (stat != asynSuccess)
    {
        return asynError;
    }
    *nIn = nBytesRead / sizeof(epicsFloat32);
    return asynSuccess;
}

/** Overrides asynPortDriver::writeFloat32Array.
 * Writes float32Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Input data buffer.
 * \param[in] nElements Input data size.
 *
 * \return asynSuccess or asynError.
 */
asynStatus
adsAsynPortDriver::writeFloat32Array(asynUser* pasynUser, epicsFloat32* value, size_t nElements)
{
    const char* functionName = "writeFloat32Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    long allowedType = ADST_REAL32;
    return adsGenericArrayWrite(amsClientPort,
                                pasynUser,
                                allowedType,
                                (const void*)value,
                                nElements * sizeof(epicsFloat32));
}


asynStatus adsAsynPortDriver::readInt64Array(asynUser* pasynUser,
                                             epicsInt64* value,
                                             size_t nElements,
                                             size_t* nIn)
{
    const char* functionName = "readInt64Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    long allowedType = ADST_INT64;

    size_t nBytesRead = 0;
    asynStatus stat   = adsGenericArrayRead(amsClientPort,
                                          pasynUser,
                                          allowedType,
                                          (void*)value,
                                          nElements * sizeof(epicsInt64),
                                          &nBytesRead);
    if (stat != asynSuccess)
    {
        return asynError;
    }
    *nIn = nBytesRead / sizeof(epicsInt64);
    return asynSuccess;
}

asynStatus
adsAsynPortDriver::writeInt64Array(asynUser* pasynUser, epicsInt64* value, size_t nElements)
{
    const char* functionName = "writeInt64Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    long allowedType = ADST_INT64;

    return adsGenericArrayWrite(
        amsClientPort, pasynUser, allowedType, (const void*)value, nElements * sizeof(epicsInt64));
}

/** Overrides asynPortDriver::readFloat64Array.
 * Reads float64Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[out] value Output data buffer.
 * \param[in] nElements Output buffer size.
 * \param[out] nIn Bytes read into buffer.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::readFloat64Array(asynUser* pasynUser,
                                               epicsFloat64* value,
                                               size_t nElements,
                                               size_t* nIn)
{
    const char* functionName = "readFloat64Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    long allowedType = ADST_REAL64;

    size_t nBytesRead = 0;
    asynStatus stat   = adsGenericArrayRead(amsClientPort,
                                          pasynUser,
                                          allowedType,
                                          (void*)value,
                                          nElements * sizeof(epicsFloat64),
                                          &nBytesRead);
    if (stat != asynSuccess)
    {
        return asynError;
    }
    *nIn = nBytesRead / sizeof(epicsFloat64);
    return asynSuccess;
}

/** Overrides asynPortDriver::writeFloat64Array.
 * Writes float64Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Input data buffer.
 * \param[in] nElements Input data size.
 *
 * \return asynSuccess or asynError.
 */
asynStatus
adsAsynPortDriver::writeFloat64Array(asynUser* pasynUser, epicsFloat64* value, size_t nElements)
{
    const char* functionName = "writeFloat64Array";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    long amsClientPort = 0;
    AdsClientPortGuard adsClientPortGuard(*this, amsClientPort);
    if (isInvalidPortNumber(amsClientPort))
    {
        amsClientPort = this->getAdsClientPortNumberForThreadId(0);
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: failed to open ads client port for this thread. Using default.\n",
                  driverName,
                  __func__);
    }

    long allowedType = ADST_REAL64;
    return adsGenericArrayWrite(amsClientPort,
                                pasynUser,
                                allowedType,
                                (const void*)value,
                                nElements * nElements * sizeof(epicsFloat64));
}

/** Returns pasynUserSelf for use in asynPrint().
 *
 * \return pasynUserSelf
 */
asynUser* adsAsynPortDriver::getTraceAsynUser()
{
    const char* functionName = "getTraceAsynUser";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    return pasynUserSelf;
}

/** Get handle to symbolic plc variable.
 *
 * \param[in/out] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsGetSymHandleByName(uint16_t amsClientPort, adsParamInfo* paramInfo)
{
    return adsGetSymHandleByName(amsClientPort, paramInfo, false);
}

/** Get handle to symbolic plc variable.
 *
 * \param[in/out] paramInfo Parameter information.
 * \param[in] blockErrorMsg Suppress error messages
 *            (used while trying to reconnect to avoid alot of error messages).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsGetSymHandleByName(uint16_t amsClientPort,
                                                    adsParamInfo* paramInfo,
                                                    bool blockErrorMsg)
{
    const char* functionName = "adsGetSymHandleByName";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    AmsAddr amsServer;
    amsServer = {remoteNetId_, paramInfo->amsPort};

    uint32_t symbolHandle = 0;
    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "%s:%s: using amsPort=%u for symbol '%s'\n",
              driverName,
              functionName,
              paramInfo->amsPort,
              paramInfo->plcAdrStr);

    const long handleStatus = AdsSyncReadWriteReqEx2(amsClientPort,
                                                     &amsServer,
                                                     ADSIGRP_SYM_HNDBYNAME,
                                                     0,
                                                     sizeof(paramInfo->hSymbolicHandle),
                                                     &symbolHandle,
                                                     strlen(paramInfo->plcAdrStr),
                                                     paramInfo->plcAdrStr,
                                                     nullptr);

    if (handleStatus)
    {
        if (!blockErrorMsg)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Create handle for %s failed with: %s (0x%lx)\n",
                      driverName,
                      functionName,
                      paramInfo->plcAdrStr,
                      adsErrorToString(handleStatus),
                      handleStatus);
        }
        return asynError;
    }

    //Add handle succeded
    paramInfo->hSymbolicHandle      = symbolHandle;
    paramInfo->bSymbolicHandleValid = true;

    return asynSuccess;
}

/** Register on-change callback for symbols version
 *
 * \param[in] port Structure containig Ams-port information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsAddSymbolsChangedCallback(uint16_t amsClientPort,
                                                           amsPortInfo* port)
{
    const char* functionName = "adsAddSymbolsChangedCallback";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Ams-port %u.\n",
              driverName,
              functionName,
              port->amsPort);

    AmsAddr amsServer;
    amsServer = {remoteNetId_, port->amsPort};

    AdsNotificationAttrib attrib;
    attrib.cbLength   = 1;
    attrib.nTransMode = ADSTRANS_SERVERONCHA;                       //Add option
    attrib.nMaxDelay  = (uint32_t)(defaultMaxDelayTimeMS_ * 10000); // 100ms
    attrib.nCycleTime = (uint32_t)(defaultSampleTimeMS_ * 10000);

    uint32_t hNotify = 0;

    long addStatus =
        AdsSyncAddDeviceNotificationReqEx(amsClientPort,
                                          &amsServer,
                                          ADSIGRP_SYM_VERSION,
                                          0,
                                          &attrib,
                                          &adsSymbolsChangedCallback,
                                          (uint32_t)port->amsPort, //Use amsPort as hUser
                                          &hNotify);

    if (addStatus)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Add device notification failed with: %s (0x%lx)\n",
                  driverName,
                  functionName,
                  adsErrorToString(addStatus),
                  addStatus);
        return asynError;
    }

    //Add was successful
    port->hCallbackNotify      = hNotify;
    port->bCallbackNotifyValid = true;
    port->refreshNeeded        = false;

    return asynSuccess;
}

/** Unregister on-change callback for symbols version
 *
 * \param[in] port Structure containig Ams-port information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsDelSymbolsChangedCallback(uint16_t amsClientPort,
                                                           amsPortInfo* port)
{
    const char* functionName = "adsDelSymbolsChangedCallback";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    AmsAddr amsServer;
    amsServer = {remoteNetId_, port->amsPort};

    const long delStatus =
        AdsSyncDelDeviceNotificationReqEx(amsClientPort, &amsServer, port->hCallbackNotify);

    port->bCallbackNotifyValid = false;
    port->hCallbackNotify      = -1;

    if (delStatus)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Delete device notification failed with: %s (0x%lx)\n",
                  driverName,
                  functionName,
                  adsErrorToString(delStatus),
                  delStatus);
        return asynError;
    }

    return asynSuccess;
}

/** Register on-change callback for parameter (plc-variable).
 *
 * \param[in/out] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsAddDataCallback(uint16_t amsClientPort, adsParamInfo* paramInfo)
{
    const char* functionName = "adsAddDataCallback";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    uint32_t group  = 0;
    uint32_t offset = 0;

    paramInfo->bCallbackNotifyValid = false;

    AmsAddr amsServer;
    amsServer = {remoteNetId_, paramInfo->amsPort};

    if (paramInfo->isAdrCommand)
    { // Abs access (ADR command)
        if (!paramInfo->plcAbsAdrValid)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Absolute address in paramInfo not valid.\n",
                      driverName,
                      functionName);
            return asynError;
        }

        group  = paramInfo->plcAbsAdrGroup;
        offset = paramInfo->plcAbsAdrOffset;
    }
    else
    { // Symbolic access

        // Read symbolic information if needed (to get paramInfo->plcSize)
        if (!paramInfo->plcAbsAdrValid)
        {
            asynStatus statusInfo = adsGetSymInfoByName(amsClientPort, paramInfo);
            if (statusInfo != asynSuccess)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s:%s: adsGetSymInfoByName failed.\n",
                          driverName,
                          functionName);
                return asynError;
            }
        }

        // Get symbolic handle if needed
        if (!paramInfo->bSymbolicHandleValid)
        {
            asynStatus statusHandle = adsGetSymHandleByName(amsClientPort, paramInfo);
            if (statusHandle != asynSuccess)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s:%s: adsGetSymHandleByName failed.\n",
                          driverName,
                          functionName);
                return asynError;
            }
        }

        group =
            ADSIGRP_SYM_VALBYHND; //Access via symbolic handle stored in paramInfo->hSymbolicHandle
        offset = paramInfo->hSymbolicHandle;
    }

    AdsNotificationAttrib attrib;
    /** Length of the data that is to be passed to the callback function. */
    attrib.cbLength = paramInfo->plcSize;
    /**
  * ADSTRANS_SERVERCYCLE: The notification's callback function is invoked cyclically.
  * ADSTRANS_SERVERONCHA: The notification's callback function is only invoked when the value changes.
  */
    attrib.nTransMode = ADSTRANS_SERVERONCHA; //Add option
    /** The notification's callback function is invoked at the latest when this time has elapsed. The unit is 100 ns. */
    attrib.nMaxDelay = (uint32_t)(paramInfo->maxDelayTimeMS * 10000); // 100ms
    /** The ADS server checks whether the variable has changed after this time interval. The unit is 100 ns. */
    attrib.nCycleTime = (uint32_t)(paramInfo->sampleTimeMS * 10000);

    uint32_t hNotify = 0;

    long addStatus = AdsSyncAddDeviceNotificationReqEx(adsPort_,
                                                       &amsServer,
                                                       group,
                                                       offset,
                                                       &attrib,
                                                       &adsDataCallback,
                                                       (uint32_t)paramInfo->paramIndex,
                                                       &hNotify);

    if (addStatus)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Add device notification failed with: %s (0x%lx)\n",
                  driverName,
                  functionName,
                  adsErrorToString(addStatus),
                  addStatus);
        return asynError;
    }

    //Add was successful
    paramInfo->hCallbackNotify      = hNotify;
    paramInfo->bCallbackNotifyValid = true;

    return asynSuccess;
}

/** Unregister on-change callback for parameter (plc-variable).
 *
 * \param[in/out] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsDelDataCallback(uint16_t amsClientPort, adsParamInfo* paramInfo)
{
    return adsDelDataCallback(amsClientPort, paramInfo, false);
}

/** Unregister on-change callback for parameter (plc-variable).
 *
 * \param[in/out] paramInfo Parameter information.
 * \param[in] blockErrorMsg Suppress error messages
 *            (used while trying to reconnect to avoid alot of error messages).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsDelDataCallback(uint16_t amsClientPort,
                                                 adsParamInfo* paramInfo,
                                                 bool blockErrorMsg)
{
    const char* functionName = "adsDelDataCallback";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    paramInfo->bCallbackNotifyValid = false;

    AmsAddr amsServer;
    amsServer = {remoteNetId_, paramInfo->amsPort};

    const long delStatus =
        AdsSyncDelDeviceNotificationReqEx(amsClientPort, &amsServer, paramInfo->hCallbackNotify);
    paramInfo->hCallbackNotify = -1;

    if (delStatus)
    {
        if (!blockErrorMsg)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Delete device notification failed with: %s (0x%lx)\n",
                      driverName,
                      functionName,
                      adsErrorToString(delStatus),
                      delStatus);
        }
        return asynError;
    }

    return asynSuccess;
}

/** Get symbolic information for a plc variable.
 *
 * \param[in] amsPort Ams-port
 * \param[in] varName Symbolic name of variable.
 * \param[out] info Information structure.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsGetSymInfoByName(uint16_t amsClientPort,
                                                  uint16_t amsPort,
                                                  const char* varName,
                                                  adsSymbolEntry* info)
{
    long errorCode = 0;
    return adsGetSymInfoByName(amsClientPort, amsPort, varName, info, &errorCode);
}

/** Get symbolic information for a plc variable.
 *
 * \param[in] amsPort Ams-port
 * \param[in] varName Symbolic name of variable.
 * \param[out] info Information structure.
 * \param[out] errorCode Ads error code.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsGetSymInfoByName(uint16_t amsClientPort,
                                                  uint16_t amsPort,
                                                  const char* varName,
                                                  adsSymbolEntry* info,
                                                  long* errorCode)
{
    const char* functionName = "adsGetSymInfoByName";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Variable name: %s, amsPort: %d.\n",
              driverName,
              functionName,
              varName,
              (int)amsPort);

    if (!info || !varName)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Info struct or varName NULL.\n",
                  driverName,
                  functionName);
        return asynError;
    }

    uint32_t bytesRead = 0;
    AmsAddr amsServer;

    amsServer = {remoteNetId_, amsPort};

    const long infoStatus = AdsSyncReadWriteReqEx2(amsClientPort,
                                                   &amsServer,
                                                   ADSIGRP_SYM_INFOBYNAMEEX,
                                                   0,
                                                   sizeof(adsSymbolEntry),
                                                   info,
                                                   strlen(varName),
                                                   varName,
                                                   &bytesRead);

    *errorCode = infoStatus;

    if (infoStatus)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Get symbolic information failed for %s with: %s (0x%lx)\n",
                  driverName,
                  functionName,
                  varName,
                  adsErrorToString(infoStatus),
                  infoStatus);
        if (infoStatus == GLOBALERR_TARGET_PORT)
        {
            /* Sigh.  It was up, now it's down.  Let's go home. */
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Port error, giving up!\n",
                      driverName,
                      functionName);
        }
        return asynError;
    }

    info->variableName = info->buffer;

    if (info->nameLength >= sizeof(info->buffer) - 1)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Missalignment of type in AdsSyncReadWriteReqEx2 return struct for %s\n",
                  driverName,
                  functionName,
                  varName);
        return asynError;
    }
    info->symDataType = info->buffer + info->nameLength + 1;

    if (info->nameLength + info->typeLength + 2 >= (uint16_t)(sizeof(info->buffer) - 1))
    {
        asynPrint(
            pasynUserSelf,
            ASYN_TRACE_ERROR,
            "%s:%s: Missalignment of comment in AdsSyncReadWriteReqEx2 return struct for %s\n",
            driverName,
            functionName,
            varName);
    }
    info->symComment = info->symDataType + info->typeLength + 1;

    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Symbolic information\n");
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "SymEntrylength: %d\n", info->entryLen);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "idxGroup: 0x%x\n", info->iGroup);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "idxOffset: 0x%x\n", info->iOffset);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "ByteSize: %d\n", info->size);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "adsDataType: %d\n", info->dataType);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Flags: %d\n", info->flags);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Name length: %d\n", info->nameLength);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Type length: %d\n", info->typeLength);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Type length: %d\n", info->commentLength);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Variable name: %s\n", info->variableName);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Data type: %s\n", info->symDataType);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Comment: %s\n", info->symComment);

    return asynSuccess;
}

/** Get symbolic information for a plc variable.
 *
 * \param[in/out] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsGetSymInfoByName(uint16_t amsClientPort, adsParamInfo* paramInfo)
{
    const char* functionName = "adsGetSymInfoByName";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    adsSymbolEntry infoStruct;
    memset(&infoStruct, 0, sizeof(infoStruct));

    asynStatus stat =
        adsGetSymInfoByName(amsClientPort, paramInfo->amsPort, paramInfo->plcAdrStr, &infoStruct);
    if (stat)
    {
        return asynError;
    }

    //fill paramInfo data structure
    paramInfo->plcAbsAdrGroup  = infoStruct.iGroup;
    paramInfo->plcAbsAdrOffset = infoStruct.iOffset;
    paramInfo->plcSize         = infoStruct.size;
    paramInfo->plcDataType     = infoStruct.dataType;
    paramInfo->plcAbsAdrValid  = true;

    return asynSuccess;
}

/** Connect to ads router (TwinCAT system).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsConnect(uint16_t amsClientPort)
{
    const char* functionName = "adsConnect";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    // add local route to your ADS Master
    if (!routeAdded_)
    {
        asynStatus stat = adsAddRoute();

        if (stat != asynSuccess)
        {
            adsDelRoute(1);
        }
    }

    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s:Open ADS port = %ld.\n",
              driverName,
              functionName,
              adsPort_);
    // Update timeout
    uint32_t defaultTimeout = 0;

    long status = AdsSyncGetTimeoutEx(amsClientPort, &defaultTimeout);

    if (status)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: AdsSyncGetTimeoutEx failed with: %s (0x%lx).\n",
                  driverName,
                  functionName,
                  adsErrorToString(status),
                  status);
        return asynError;
    }

    status = AdsSyncSetTimeoutEx(amsClientPort, (uint32_t)adsTimeoutMS_);

    if (status)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: AdsSyncSetTimeoutEx failed with: %s (0x%lx).\n",
                  driverName,
                  functionName,
                  adsErrorToString(status),
                  status);
        return asynError;
    }

    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Update ADS sync time out from %u to %u.\n",
              driverName,
              functionName,
              defaultTimeout,
              (uint32_t)adsTimeoutMS_);

    return asynSuccess;
}

/** Read Ams port version information
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReadVersion(uint16_t amsClientPort, amsPortInfo* port)
{
    const char* functionName = "adsReadVersion";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Ams-port %u\n",
              driverName,
              functionName,
              port->amsPort);

    AmsAddr amsServer;
    AdsVersion version;
    char devName[255];
    amsServer = {remoteNetId_, port->amsPort};

    long status = AdsSyncReadDeviceInfoReqEx(amsClientPort, &amsServer, devName, &version);

    if (status)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: AdsSyncReadDeviceInfoReqEx failed with: %s (0x%lx).\n",
                  driverName,
                  functionName,
                  adsErrorToString(status),
                  status);
        return asynError;
    }

    port->version = version;
    strncpy(port->devName, devName, sizeof(port->devName));
    return asynSuccess;
}

/** Disconnect ads router (TwinCAT system).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsDisconnect()
{
    const char* functionName = "adsDisconnect";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: adsPort_=%ld\n",
              driverName,
              functionName,
              adsPort_);

    adsDelRoute(1);

    return asynSuccess;
}

/** Release handle to symbolic variable (in TwinCAT plc)
 *
 * \param[in/out] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReleaseSymbolicHandle(uint16_t amsClientPort,
                                                       adsParamInfo* paramInfo)
{
    return adsReleaseSymbolicHandle(amsClientPort, paramInfo, false);
}

/** Release handle to symbolic variable (in TwinCAT plc)
 *
 * \param[in/out] paramInfo Parameter information.
 * \param[in] blockErrorMsg Suppress error messages
 *            (used while trying to reconnect to avoid alot of error messages).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReleaseSymbolicHandle(uint16_t amsClientPort,
                                                       adsParamInfo* paramInfo,
                                                       bool blockErrorMsg)
{
    const char* functionName = "adsReleaseHandle";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    paramInfo->bSymbolicHandleValid = false;

    AmsAddr amsServer;
    amsServer = {remoteNetId_, paramInfo->amsPort};

    const long releaseStatus = AdsSyncWriteReqEx(amsClientPort,
                                                 &amsServer,
                                                 ADSIGRP_SYM_RELEASEHND,
                                                 0,
                                                 sizeof(paramInfo->hSymbolicHandle),
                                                 &paramInfo->hSymbolicHandle);

    paramInfo->hSymbolicHandle = -1;
    if (releaseStatus && !blockErrorMsg)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Release of handle 0x%x failed with: %s (0x%lx)\n",
                  driverName,
                  functionName,
                  paramInfo->hSymbolicHandle,
                  adsErrorToString(releaseStatus),
                  releaseStatus);
        return asynError;
    }

    return asynSuccess;
}

/** Write value to variable in TwinCAT.
 *
 * \param[in] paramInfo Parameter information.
 * \param[in] binaryBuffer Data to write.
 * \param[in] bytesToWrite Bytes to write.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsWriteParam(uint16_t amsClientPort,
                                            adsParamInfo* paramInfo,
                                            const void* binaryBuffer,
                                            uint32_t bytesToWrite)
{
    const char* functionName = "adsWriteParam";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    // Reject I/O once teardown has begun. The destructor sets stopThreads_ and
    // then frees each paramInfo (plcAdrStr, buffers); a straggler record-scan
    // callback reaching this far would dereference freed memory. Bail before
    // touching paramInfo. In a live IOC the driver outlives the process, so
    // this only fires in the unit-test harness, which deletes the driver.
    if (stopThreads_)
    {
        return asynError;
    }

    // Calculate consumed time by this method
    struct timeval start, end;
    long secs_used, micros_used;
    gettimeofday(&start, NULL);

    uint32_t group  = 0;
    uint32_t offset = 0;

    AmsAddr amsServer;
    amsServer = {remoteNetId_, paramInfo->amsPort};

    if (paramInfo->isAdrCommand)
    { // Abs access (ADR command)
        if (!paramInfo->plcAbsAdrValid)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Absolute address in paramInfo not valid.\n",
                      driverName,
                      functionName);
            return asynError;
        }

        group  = paramInfo->plcAbsAdrGroup;
        offset = paramInfo->plcAbsAdrOffset;
    }
    else
    { // Symbolic access

        // Read symbolic information if needed (to get paramInfo->plcSize)
        if (!paramInfo->plcAbsAdrValid)
        {
            asynStatus statusInfo = adsGetSymInfoByName(amsClientPort, paramInfo);
            if (statusInfo == asynError)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s:%s: adsGetSymInfoByName failed.\n",
                          driverName,
                          functionName);
                return asynError;
            }
        }

        // Get symbolic handle if needed
        if (!paramInfo->bSymbolicHandleValid)
        {
            asynStatus statusHandle = adsGetSymHandleByName(amsClientPort, paramInfo);
            if (statusHandle != asynSuccess)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s:%s: adsGetSymHandleByName failed.\n",
                          driverName,
                          functionName);
                return asynError;
            }
        }

        group =
            ADSIGRP_SYM_VALBYHND; //Access via symbolic handle stored in paramInfo->hSymbolicHandle
        offset = paramInfo->hSymbolicHandle;
    }

    long writeStatus = AdsSyncWriteReqEx(
        amsClientPort, &amsServer, group, offset, paramInfo->plcSize, binaryBuffer);

    if (writeStatus)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: ADS write failed with: %s (0x%lx)\n",
                  driverName,
                  functionName,
                  adsErrorToString(writeStatus),
                  writeStatus);
        return asynError;
    }

    gettimeofday(&end, NULL);
    secs_used   = (end.tv_sec - start.tv_sec); //avoid overflow by subtracting first
    micros_used = ((secs_used * 1000000) + end.tv_usec) - (start.tv_usec);
    asynPrint(pasynUserSelf,
              ASYN_TRACEIO_DRIVER,
              "%s:%s: ADS write: micros used: 0x%lx\n",
              driverName,
              functionName,
              micros_used);

    return asynSuccess;
}

/** Read value of variable in TwinCAT.
 *
 * \param[in] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReadParam(uint16_t amsClientPort, adsParamInfo* paramInfo)
{
    long notused = 0;
    return adsReadParam(amsClientPort, paramInfo, &notused, 1);
}

/** Read value of variable in TwinCAT.
 *
 * \param[in] paramInfo Parameter information.
 * \param[out] error Error code.
 * \param[in] updateAsynPar Update asyn parameter.
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReadParam(uint16_t amsClientPort,
                                           adsParamInfo* paramInfo,
                                           long* error,
                                           int updateAsynPar)
{
    const char* functionName = "adsReadParam";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    uint32_t group  = 0;
    uint32_t offset = 0;
    *error          = 0;

    // Reject I/O once teardown has begun (see adsWriteParam for the rationale):
    // the destructor frees paramInfo after setting stopThreads_, so a straggler
    // scan callback must bail before dereferencing freed memory.
    if (stopThreads_)
    {
        return asynError;
    }

    AmsAddr amsServer;
    amsServer = {remoteNetId_, paramInfo->amsPort};

    if (paramInfo->isAdrCommand)
    { // Abs access (ADR command)
        if (!paramInfo->plcAbsAdrValid)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Absolute address in paramInfo not valid.\n",
                      driverName,
                      functionName);
            return asynError;
        }

        group  = paramInfo->plcAbsAdrGroup;
        offset = paramInfo->plcAbsAdrOffset;
    }
    else
    { // Symbolic access

        // Read symbolic information if needed (to get paramInfo->plcSize)
        if (!paramInfo->plcAbsAdrValid)
        {
            asynStatus statusInfo = adsGetSymInfoByName(amsClientPort, paramInfo);
            if (statusInfo == asynError)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s:%s: adsGetSymInfoByName failed.\n",
                          driverName,
                          functionName);
                return asynError;
            }
        }

        // Get symbolic handle if needed
        if (!paramInfo->bSymbolicHandleValid)
        {
            asynStatus statusHandle = adsGetSymHandleByName(amsClientPort, paramInfo);
            if (statusHandle != asynSuccess)
            {
                asynPrint(pasynUserSelf,
                          ASYN_TRACE_ERROR,
                          "%s:%s: adsGetSymHandleByName failed.\n",
                          driverName,
                          functionName);
                return asynError;
            }
        }

        group =
            ADSIGRP_SYM_VALBYHND; //Access via symbolic handle stored in paramInfo->hSymbolicHandle
        offset = paramInfo->hSymbolicHandle;
    }

    char* data         = new char[paramInfo->plcSize];
    uint32_t bytesRead = 0;

    *error = AdsSyncReadReqEx2(
        amsClientPort, &amsServer, group, offset, paramInfo->plcSize, (void*)data, &bytesRead);

    if (*error)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: AdsSyncReadReqEx2 failed: %s (%lu).\n",
                  driverName,
                  functionName,
                  adsErrorToString(*error),
                  *error);
        return asynError;
    }

    if (bytesRead != paramInfo->plcSize)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Read bytes differ from parameter plc size (%u vs %u).\n",
                  driverName,
                  functionName,
                  bytesRead,
                  paramInfo->plcSize);
        return asynError;
    }

    //No timestamp available
    paramInfo->plcTimeStampRaw = 0;
    paramInfo->firstReadDone   = true;

    asynStatus stat = asynSuccess;
    if (updateAsynPar)
    {
        stat = adsUpdateParameterLock(paramInfo, (const void*)data, bytesRead);
    }

    return stat;
}

/** Read state of amsport in TwinCAT
 *
 * \param[in] amsport Ams-prot.
 * \param[out] adsState State of ams-port (running, invalid, config..).
 * \param[in] blockErrorMsg Suppress error messages
 *            (used while trying to reconnect to avoid alot of error messages).
 *
 * Thread safe.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReadState(uint16_t amsClientPort,
                                           uint16_t amsport,
                                           uint16_t* adsState,
                                           bool blockErrorMsg)
{
    asynStatus stat;

    long error = 0;
    stat       = adsReadState(amsClientPort, amsport, adsState, blockErrorMsg, &error);

    return stat;
}

/** Read state of default amsport in TwinCAT
 *
 * \param[out] adsState State of ams-port (running, invalid, config..).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReadState(uint16_t amsClientPort, uint16_t* adsState)
{
    long error = 0;
    return adsReadState(amsClientPort, amsportDefault_, adsState, false, &error);
}

/** Read state of amsport in TwinCAT
 *
 * \param[in] amsport Ams-port.
 * \param[out] adsState State of ams-port (running, invalid, config..).
 * \param[in] blockErrorMsg Suppress error messages
 *            (used while trying to reconnect to avoid alot of error messages).
 * \param[out] error Error code.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReadState(
    uint16_t amsClientPort, uint16_t amsport, uint16_t* adsState, bool blockErrorMsg, long* error)
{
    const char* functionName = "adsReadState";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    AmsAddr amsServer = {remoteNetId_, amsport};

    uint16_t devState;

    const long status = AdsSyncReadStateReqEx(amsClientPort, &amsServer, adsState, &devState);
    *error            = status;

    if (status)
    {
        if (!blockErrorMsg)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: ADS read state failed with: %s (0x%lx)\n",
                      driverName,
                      functionName,
                      adsErrorToString(status),
                      status);
        }
        return asynError;
    }

    return asynSuccess;
}

/** Get parameter table size (max allowed parameter count).
 * \param[in] amsport Ams-port.
 * \param[in] adsState State of ams-port (running, invalid, config..).
 *
 * \return asynSuccess or asynError.
 */
asynStatus
adsAsynPortDriver::adsWriteState(uint16_t amsClientPort, uint16_t amsport, uint16_t adsState)
{
    const char* functionName = "adsWriteState";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: adsState = %s (%u)\n",
              driverName,
              functionName,
              adsStateToString(adsState),
              adsState);

    void* pData       = NULL;
    AmsAddr amsServer = {remoteNetId_, amsport};
    const long status = AdsSyncWriteControlReqEx(amsClientPort, &amsServer, adsState, 0, 0, pData);
    if (status)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: ADS write state failed with: %s (0x%lx)\n",
                  driverName,
                  functionName,
                  adsErrorToString(status),
                  status);
        return asynError;
    }

    return asynSuccess;
}

/** Get parameter table size (max allowed parameter count).
 *
 * \return Aysn -parameter table size.
 */
int adsAsynPortDriver::getParamTableSize()
{
    return paramTableSize_;
}

/** Get parameter info struct for a certain index/reason (pasynUser->reason).
 *
 * \param[in] index index/reason (pasynUser->reason).
 *
 * \return Parameter info structure.
 */
adsParamInfo* adsAsynPortDriver::getAdsParamInfo(int index)
{
    const char* functionName = "getAdsParamInfo";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: Get paramInfo for index: %d\n",
              driverName,
              functionName,
              index);

    if (index < getAdsParamCount())
    {
        return &adsParamArray_[index];
    }
    else
    {
        return NULL;
    }
}

/** Get current parameter count.
 *
 * \return Current parameter count.
 */
int adsAsynPortDriver::getAdsParamCount()
{
    return adsParamArrayCount_;
}

/** Update timestamp of parameter.
 *
 * \param[in] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 *
 * Refreshes and sets timestamp depending on time source (PLC or EPICS).
 */
asynStatus adsAsynPortDriver::refreshParamTime(adsParamInfo* paramInfo)
{
    const char* functionName = "refreshParamTime";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: plcTime %lu.\n",
              driverName,
              functionName,
              paramInfo->plcTimeStampRaw);

    //Convert plc timeStamp (windows format) to epicsTimeStamp
    if (windowsToEpicsTimeStamp(paramInfo->plcTimeStampRaw, &paramInfo->plcTimeStamp))
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: windowsToEpicsTimeStamp() failed.\n",
                  driverName,
                  functionName);
        return asynError;
    }

    epicsTimeStamp ts;

    //Update time stamp
    if (paramInfo->timeBase == ADS_TIME_BASE_EPICS || paramInfo->plcTimeStampRaw == 0)
    {
        if (updateTimeStamp() != asynSuccess)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: updateTimeStamp() failed.\n",
                      driverName,
                      functionName);
            return asynError;
        }
    }
    else
    { //ADS_TIME_BASE_PLC
        if (setTimeStamp(&paramInfo->plcTimeStamp) != asynSuccess)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: updateTimeStamp() failed.\n",
                      driverName,
                      functionName);
            return asynError;
        }
    }

    if (getTimeStamp(&ts) != asynSuccess)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: getTimeStamp() failed.\n",
                  driverName,
                  functionName);
        return asynError;
    }

    paramInfo->epicsTimestamp = ts;

    return asynSuccess;
}

/** Update asyn parameter or callback (for arrays).
 *
 * \param[in] paramInfo Parameter information.
 * \param[in] data Data to write to parameter (or callback to EPICS).
 *
 * \return asynSuccess or asynError.
 *
 * Thread safe.
 */
asynStatus adsAsynPortDriver::adsUpdateParameterLock(adsParamInfo* paramInfo, const void* data)
{
    lock();
    asynStatus stat = adsUpdateParameter(paramInfo, data);
    unlock();
    return stat;
}

/** Update asyn parameter or callback (for arrays).
 *
 * \param[in] paramInfo Parameter information.
 * \param[in] data Data to write to parameter (or callback to EPICS).
 * \param[in] dataSize Size of data to write.
 *
 * \return asynSuccess or asynError.
 *
 * Thread safe.
 */
asynStatus adsAsynPortDriver::adsUpdateParameterLock(adsParamInfo* paramInfo,
                                                     const void* data,
                                                     size_t dataSize)
{
    lock();
    asynStatus stat = adsUpdateParameter(paramInfo, data, dataSize);
    unlock();
    return stat;
}

/** Update asyn parameter or callback (for arrays).
 *
 * \param[in] paramInfo Parameter information.
 * \param[in] data Data to write to parameter (or callback to EPICS).
 *
 * \return asynSuccess or asynError.
 *
 */
asynStatus adsAsynPortDriver::adsUpdateParameter(adsParamInfo* paramInfo, const void* data)
{
    return adsUpdateParameter(paramInfo, data, paramInfo->lastCallbackSize);
}

/** Update asyn parameter or callback (for arrays).
 *
 * \param[in] paramInfo Parameter information.
 * \param[in] data Data to write to parameter (or callback to EPICS).
 * \param[in] dataSize Size of data to write.
 *
 * \return asynSuccess or asynError.
 *
 */
asynStatus
adsAsynPortDriver::adsUpdateParameter(adsParamInfo* paramInfo, const void* data, size_t dataSize)
{
    const char* functionName = "adsUpdateParameter";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    if (!paramInfo)
    {
        asynPrint(
            pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: paramInfo NULL.\n", driverName, functionName);
        return asynError;
    }

    if (!data)
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: data NULL.\n", driverName, functionName);
        return asynError;
    }

    if (refreshParamTime(paramInfo) != asynSuccess)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: refreshParamTime() failed.\n",
                  driverName,
                  functionName);
        return asynError;
    }

    asynStatus ret = asynError;

    //Ensure check if array
    if (paramInfo->plcDataIsArray)
    {
        if (!paramInfo->arrayDataBuffer)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Array but buffer is NULL.\n",
                      driverName,
                      functionName);
            return asynError;
        }
        //Copy data to param buffer
        memcpy(paramInfo->arrayDataBuffer, data, paramInfo->lastCallbackSize);
    }

    switch (paramInfo->plcDataType)
    {
    case ADST_INT8:
        int8_t* ADST_INT8Var;
        ADST_INT8Var = ((int8_t*)data);
        //Asyn types
        switch (paramInfo->asynType)
        {
        case asynParamInt32:
            ret = setIntegerParam(paramInfo->paramIndex, (int)(*ADST_INT8Var));
            break;
        case asynParamFloat64:
            ret = setDoubleParam(paramInfo->paramIndex, (double)(*ADST_INT8Var));
            break;
        case asynParamInt8Array:
            arrayParamsToCallCallbacksFor_.push(paramInfo);
            ret = asynSuccess;
            break;
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;

    case ADST_INT16:
        int16_t* ADST_INT16Var;
        ADST_INT16Var = ((int16_t*)data);
        //Asyn types
        switch (paramInfo->asynType)
        {
        case asynParamInt32:
            ret = setIntegerParam(paramInfo->paramIndex, (int)(*ADST_INT16Var));
            break;
        case asynParamFloat64:
            ret = setDoubleParam(paramInfo->paramIndex, (double)(*ADST_INT16Var));
            break;
        case asynParamInt16Array:
            arrayParamsToCallCallbacksFor_.push(paramInfo);
            ret = asynSuccess;
            break;
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;
    case ADST_INT32:
        int32_t* ADST_INT32Var;
        ADST_INT32Var = ((int32_t*)data);
        //Asyn types
        switch (paramInfo->asynType)
        {
        case asynParamInt32:
            ret = setIntegerParam(paramInfo->paramIndex, (int)(*ADST_INT32Var));
            break;
        case asynParamFloat64:
            ret = setDoubleParam(paramInfo->paramIndex, (double)(*ADST_INT32Var));
            break;
        case asynParamInt32Array:
            arrayParamsToCallCallbacksFor_.push(paramInfo);
            ret = asynSuccess;
            break;
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;
    case ADST_INT64:
        int64_t* ADST_INT64Var;
        ADST_INT64Var = ((int64_t*)data);
        //Asyn types
        switch (paramInfo->asynType)
        {
        case asynParamInt32:
            ret = setIntegerParam(paramInfo->paramIndex, (int)(*ADST_INT64Var));
            break;
        case asynParamFloat64:
            ret = setDoubleParam(paramInfo->paramIndex, (double)(*ADST_INT64Var));
            break;
        case asynParamInt64:
            ret = setInteger64Param(paramInfo->paramIndex, *ADST_INT64Var);
            break;
        // No 64 bit uint array callback type (also no 64bit uint in EPICS)
        case asynParamInt64Array:
            arrayParamsToCallCallbacksFor_.push(paramInfo);
            ret = asynSuccess;
            break;
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;
    case ADST_UINT8:
        uint8_t* ADST_UINT8Var;
        ADST_UINT8Var = ((uint8_t*)data);
        //Asyn types
        switch (paramInfo->asynType)
        {
        case asynParamInt32:
            ret = setIntegerParam(paramInfo->paramIndex, (int)(*ADST_UINT8Var));
            break;
        case asynParamFloat64:
            ret = setDoubleParam(paramInfo->paramIndex, (double)(*ADST_UINT8Var));
            break;
        // Arrays of unsigned not supported
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;
    case ADST_UINT16:
        uint16_t* ADST_UINT16Var;
        ADST_UINT16Var = ((uint16_t*)data);
        //Asyn types
        switch (paramInfo->asynType)
        {
        case asynParamInt32:
            ret = setIntegerParam(paramInfo->paramIndex, (int)(*ADST_UINT16Var));
            break;
        case asynParamFloat64:
            ret = setDoubleParam(paramInfo->paramIndex, (double)(*ADST_UINT16Var));
            break;
        // Arrays of unsigned not supported
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;
    case ADST_UINT32:
        uint32_t* ADST_UINT32Var;
        ADST_UINT32Var = ((uint32_t*)data);
        //Asyn types
        switch (paramInfo->asynType)
        {
        case asynParamInt32:
            ret = setIntegerParam(paramInfo->paramIndex, (int)(*ADST_UINT32Var));
            break;
        case asynParamFloat64:
            ret = setDoubleParam(paramInfo->paramIndex, (double)(*ADST_UINT32Var));
            break;
        // Arrays of unsigned not supported
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;
    case ADST_UINT64:
        uint64_t* ADST_UINT64Var;
        ADST_UINT64Var = ((uint64_t*)data);
        switch (paramInfo->asynType)
        {
        case asynParamInt32:
            ret = setIntegerParam(paramInfo->paramIndex, (int)(*ADST_UINT64Var));
            break;
        case asynParamFloat64:
            ret = setDoubleParam(paramInfo->paramIndex, (double)(*ADST_UINT64Var));
            break;
        case asynParamInt64:
            ret = setInteger64Param(paramInfo->paramIndex, (epicsInt64)(*ADST_UINT64Var));
            break;
        // No 64 bit uint array callback type (also no 64bit uint in EPICS)
        case asynParamInt64Array:
            arrayParamsToCallCallbacksFor_.push(paramInfo);
            ret = asynSuccess;
            break;
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;
    case ADST_REAL32:
        float* ADST_REAL32Var;
        ADST_REAL32Var = ((float*)data);
        switch (paramInfo->asynType)
        {
        case asynParamInt32:
            ret = setIntegerParam(paramInfo->paramIndex, (int)(*ADST_REAL32Var));
            break;
        case asynParamFloat64:
            ret = setDoubleParam(paramInfo->paramIndex, (double)(*ADST_REAL32Var));
            break;
        case asynParamFloat32Array:
            arrayParamsToCallCallbacksFor_.push(paramInfo);
            ret = asynSuccess;
            break;
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;
    case ADST_REAL64:
        double* ADST_REAL64Var;
        ADST_REAL64Var = ((double*)data);
        switch (paramInfo->asynType)
        {
        case asynParamInt32:
            ret = setIntegerParam(paramInfo->paramIndex, (int)(*ADST_REAL64Var));
            break;
        case asynParamFloat64:
            ret = setDoubleParam(paramInfo->paramIndex, (double)(*ADST_REAL64Var));
            break;
        case asynParamFloat64Array:
            arrayParamsToCallCallbacksFor_.push(paramInfo);
            ret = asynSuccess;
            break;
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;

    case ADST_BIT:
        int8_t* ADST_BitVar;
        ADST_BitVar = ((int8_t*)data);
        switch (paramInfo->asynType)
        {
        case asynParamInt32:
            ret = setIntegerParam(paramInfo->paramIndex, (int)(*ADST_BitVar));
            break;
        case asynParamFloat64:
            ret = setDoubleParam(paramInfo->paramIndex, (double)(*ADST_BitVar));
            break;
        case asynParamInt8Array:
            arrayParamsToCallCallbacksFor_.push(paramInfo);
            ret = asynSuccess;
            break;
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;
    case ADST_STRING:
        switch (paramInfo->asynType)
        {
        case asynParamInt8Array:
            arrayParamsToCallCallbacksFor_.push(paramInfo);
            ret = asynSuccess;
            break;
        default:
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                      driverName,
                      functionName,
                      adsTypeToString(paramInfo->plcDataType),
                      asynTypeToString(paramInfo->asynType));
            return asynError;
            break;
        }
        break;
    default:
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n",
                  driverName,
                  functionName,
                  adsTypeToString(paramInfo->plcDataType),
                  asynTypeToString(paramInfo->asynType));
        return asynError;
        break;
    }

    if (ret != asynSuccess)
    {
        return ret;
    }

    ret = setAlarmParam(paramInfo, NO_ALARM, NO_ALARM);
    if (ret != asynSuccess)
    {
        return ret;
    }

    return asynSuccess;
}

/** Call callbacks for all parameters.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::fireAllCallbacksLock()
{
    const char* functionName = "fireAllCallbacksLock";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    lock();
    callParamCallbacks();
    while (!arrayParamsToCallCallbacksFor_.empty())
    {
        fireCallbacksForArrayParam(arrayParamsToCallCallbacksFor_.front());
        arrayParamsToCallCallbacksFor_.pop();
    }
    unlock();
    return asynSuccess;
}

/** Call callbacks for a parameter.
 *
 * \param[in] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::fireCallbacksForArrayParam(adsParamInfo* paramInfo)
{
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, __func__);

    if (!paramInfo->plcDataIsArray)
    {
        return asynSuccess;
    }

    if (paramInfo->lastCallbackSize <= 0)
    {
        return asynSuccess;
    }

    asynStatus ret = asynError;

    //Array
    switch (paramInfo->plcDataType)
    {
    case ADST_INT8:
        switch (paramInfo->asynType)
        {
        case asynParamInt8Array:
            ret = doCallbacksInt8Array((epicsInt8*)paramInfo->arrayDataBuffer,
                                       paramInfo->lastCallbackSize,
                                       paramInfo->paramIndex,
                                       paramInfo->asynAddr);
            break;
        default:
            asynPrint(
                pasynUserSelf,
                ASYN_TRACE_ERROR,
                "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s, Name= %s\n",
                driverName,
                __func__,
                adsTypeToString(paramInfo->plcDataType),
                asynTypeToString(paramInfo->asynType),
                paramInfo->plcAdrStr);
            return asynError;
            break;
        }
        break;

    case ADST_INT16:
        switch (paramInfo->asynType)
        {
        case asynParamInt16Array:
            ret = doCallbacksInt16Array((epicsInt16*)paramInfo->arrayDataBuffer,
                                        paramInfo->lastCallbackSize,
                                        paramInfo->paramIndex,
                                        paramInfo->asynAddr);
            break;
        default:
            asynPrint(
                pasynUserSelf,
                ASYN_TRACE_ERROR,
                "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s, Name= %s\n",
                driverName,
                __func__,
                adsTypeToString(paramInfo->plcDataType),
                asynTypeToString(paramInfo->asynType),
                paramInfo->plcAdrStr);
            return asynError;
            break;
        }
        break;
    case ADST_INT32:
        switch (paramInfo->asynType)
        {
        case asynParamInt32Array:
            ret = doCallbacksInt32Array((epicsInt32*)paramInfo->arrayDataBuffer,
                                        paramInfo->lastCallbackSize,
                                        paramInfo->paramIndex,
                                        paramInfo->asynAddr);
            break;
        default:
            asynPrint(
                pasynUserSelf,
                ASYN_TRACE_ERROR,
                "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s, Name= %s\n",
                driverName,
                __func__,
                adsTypeToString(paramInfo->plcDataType),
                asynTypeToString(paramInfo->asynType),
                paramInfo->plcAdrStr);
            return asynError;
            break;
        }
        break;
    case ADST_INT64:
        switch (paramInfo->asynType)
        {
        case asynParamInt64Array:
            ret = doCallbacksInt64Array((epicsInt64*)paramInfo->arrayDataBuffer,
                                        paramInfo->lastCallbackSize / sizeof(epicsInt64),
                                        paramInfo->paramIndex,
                                        paramInfo->asynAddr);
            break;
        default:
            asynPrint(
                pasynUserSelf,
                ASYN_TRACE_ERROR,
                "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s, Name= %s\n",
                driverName,
                __func__,
                adsTypeToString(paramInfo->plcDataType),
                asynTypeToString(paramInfo->asynType),
                paramInfo->plcAdrStr);
            return asynError;
            break;
        }
        break;
    // No 64 bit uint array callback type -> cast into int64_t and use doCallbacksInt64Array
    case ADST_UINT64:
        switch (paramInfo->asynType)
        {
        case asynParamInt64Array:
            ret = doCallbacksInt64Array((epicsInt64*)paramInfo->arrayDataBuffer,
                                        paramInfo->lastCallbackSize / sizeof(epicsInt64),
                                        paramInfo->paramIndex,
                                        paramInfo->asynAddr);
            break;
        default:
            asynPrint(
                pasynUserSelf,
                ASYN_TRACE_ERROR,
                "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s, Name= %s\n",
                driverName,
                __func__,
                adsTypeToString(paramInfo->plcDataType),
                asynTypeToString(paramInfo->asynType),
                paramInfo->plcAdrStr);
            return asynError;
            break;
        }
        break;
    case ADST_REAL32:
        switch (paramInfo->asynType)
        {
        case asynParamFloat32Array:
            ret = doCallbacksFloat32Array((epicsFloat32*)paramInfo->arrayDataBuffer,
                                          paramInfo->lastCallbackSize,
                                          paramInfo->paramIndex,
                                          paramInfo->asynAddr);
            break;
        default:
            asynPrint(
                pasynUserSelf,
                ASYN_TRACE_ERROR,
                "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s, Name= %s\n",
                driverName,
                __func__,
                adsTypeToString(paramInfo->plcDataType),
                asynTypeToString(paramInfo->asynType),
                paramInfo->plcAdrStr);
            return asynError;
            break;
        }
        break;

    case ADST_REAL64:
        switch (paramInfo->asynType)
        {
        case asynParamFloat64Array:
            ret = doCallbacksFloat64Array((epicsFloat64*)paramInfo->arrayDataBuffer,
                                          paramInfo->lastCallbackSize,
                                          paramInfo->paramIndex,
                                          paramInfo->asynAddr);
            break;
        default:
            asynPrint(
                pasynUserSelf,
                ASYN_TRACE_ERROR,
                "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s, Name= %s\n",
                driverName,
                __func__,
                adsTypeToString(paramInfo->plcDataType),
                asynTypeToString(paramInfo->asynType),
                paramInfo->plcAdrStr);
            return asynError;
            break;
        }
        break;

    case ADST_BIT:
        switch (paramInfo->asynType)
        {
        case asynParamInt8Array:
            ret = doCallbacksInt8Array((epicsInt8*)paramInfo->arrayDataBuffer,
                                       paramInfo->lastCallbackSize,
                                       paramInfo->paramIndex,
                                       paramInfo->asynAddr);
            break;
        default:
            asynPrint(
                pasynUserSelf,
                ASYN_TRACE_ERROR,
                "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s, Name= %s\n",
                driverName,
                __func__,
                adsTypeToString(paramInfo->plcDataType),
                asynTypeToString(paramInfo->asynType),
                paramInfo->plcAdrStr);
            return asynError;
            break;
        }
        break;
    case ADST_STRING:
        switch (paramInfo->asynType)
        {
        case asynParamInt8Array:
            ret = doCallbacksInt8Array((epicsInt8*)paramInfo->arrayDataBuffer,
                                       paramInfo->lastCallbackSize,
                                       paramInfo->paramIndex,
                                       paramInfo->asynAddr);
            break;
        default:
            asynPrint(
                pasynUserSelf,
                ASYN_TRACE_ERROR,
                "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s, Name= %s\n",
                driverName,
                __func__,
                adsTypeToString(paramInfo->plcDataType),
                asynTypeToString(paramInfo->asynType),
                paramInfo->plcAdrStr);
            return asynError;
            break;
        }
        break;

    default:
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s, Name= %s\n",
                  driverName,
                  __func__,
                  adsTypeToString(paramInfo->plcDataType),
                  asynTypeToString(paramInfo->asynType),
                  paramInfo->plcAdrStr);
        return asynError;
        break;
    }
    return ret;
}

/** Set parameter alarm state.
 *
 * \param[in] paramInfo Parameter information.
 * \param[in] alarm Alarm type (EPICS def).
 * \param[in] severity Alarm severity (EPICS def).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::setAlarmParam(adsParamInfo* paramInfo, int alarm, int severity)
{
    const char* functionName = "setAlarmParam";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    if (!paramInfo)
    {
        asynPrint(
            pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: paramInfo==NULL.\n", driverName, functionName);
        return asynError;
    }

    asynStatus stat;
    int oldAlarmStatus = 0;
    stat               = getParamAlarmStatus(paramInfo->paramIndex, &oldAlarmStatus);
    if (stat != asynSuccess)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: getParamAlarmStatus failed for parameter %s (%d).\n",
                  driverName,
                  functionName,
                  paramInfo->drvInfo,
                  paramInfo->paramIndex);
        return asynError;
    }

    if (oldAlarmStatus != alarm)
    {
        stat = setParamAlarmStatus(paramInfo->paramIndex, alarm);
        if (stat != asynSuccess)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed set alarm status for parameter %s (%d).\n",
                      driverName,
                      functionName,
                      paramInfo->drvInfo,
                      paramInfo->paramIndex);
            return asynError;
        }
        paramInfo->alarmStatus = alarm;
    }

    int oldAlarmSeverity = 0;
    stat                 = getParamAlarmSeverity(paramInfo->paramIndex, &oldAlarmSeverity);
    if (stat != asynSuccess)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: getParamAlarmStatus failed for parameter %s (%d).\n",
                  driverName,
                  functionName,
                  paramInfo->drvInfo,
                  paramInfo->paramIndex);
        return asynError;
    }

    if (oldAlarmSeverity != severity)
    {
        stat = setParamAlarmSeverity(paramInfo->paramIndex, severity);
        if (stat != asynSuccess)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_ERROR,
                      "%s:%s: Failed set alarm severity for parameter %s (%d).\n",
                      driverName,
                      functionName,
                      paramInfo->drvInfo,
                      paramInfo->paramIndex);
            return asynError;
        }
        paramInfo->alarmSeverity = severity;
    }

    return stat;
}

/** Set parameter alarm state.
 *
 * \param[in] amsPort Ams-port.
 * \param[in] alarm Alarm type (EPICS def).
 * \param[in] severity Alarm severity (EPICS def).
 *
 * \return asynSuccess or asynError.
 *
 * Thread safe.
 */
asynStatus adsAsynPortDriver::setAlarmPortLock(uint16_t amsPort, int alarm, int severity)
{
    asynStatus stat;
    lock();
    stat = setAlarmPort(amsPort, alarm, severity);
    unlock();
    return stat;
}

/** Set alarm for all parameter on a ams-port.
 *
 * \param[in] amsPort Ams-port.
 * \param[in] alarm Alarm type (EPICS def).
 * \param[in] severity Alarm severity (EPICS def).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::setAlarmPort(uint16_t amsPort, int alarm, int severity)
{
    const char* functionName = "setAlarmPort";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    for (int i = 0; i < adsParamArrayCount_; i++)
    {
        if (adsParamArray_[i].amsPort == amsPort)
        {
            if (setAlarmParam(&adsParamArray_[i], alarm, severity) != asynSuccess)
            {
                return asynError;
            }
        }
    }
    return asynSuccess;
}

/** Delete ads route
 *
 * \param[in] force Force delete.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsDelRoute(int force)
{
    const char* functionName = "adsDelRoute";
    asynPrint(pasynUserSelf,
              ASYN_TRACE_FLOW,
              "%s:%s: force = %s\n",
              driverName,
              functionName,
              force ? "true" : "false");
    if (routeAdded_ || force)
    {
        routeAdded_ = 0;
        AdsDelRoute(remoteNetId_);
    }
    return asynSuccess;
}

/** Add ads route
 *
 * \return asynSuccess or asynError.
 *
 */
asynStatus adsAsynPortDriver::adsAddRoute()
{
    const char* functionName = "adsAddRouteLock";
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

    // add local route to your ADS Master
    const long addRouteStatus = AdsAddRoute(remoteNetId_, ipaddr_);
    if (addRouteStatus)
    {
        asynPrint(pasynUserSelf,
                  ASYN_TRACE_ERROR,
                  "%s:%s: Adding ADS route failed with: %s (0x%lx).\n",
                  driverName,
                  functionName,
                  adsErrorToString(addRouteStatus),
                  addRouteStatus);
        return asynError;
    }
    routeAdded_ = 1;
    return asynSuccess;
}


/* Configuration routine.  Called directly, or from the iocsh function below */

extern "C"
{
    asynUser* pPrintOutAsynUser;

    static void printHelp()
    {
        printf("\n");
        printf(" EPICS integration of TwinCAT PLC:s by ADS communication.\n");
        printf("\n");
        printf(" Command usage:\n");
        printf(" adsAsynPortDriverConfigure(<Asyn port name>,\n");
        printf("                            <IP address of PLC>,\n");
        printf("                            <AMS address of PLC>,\n");
        printf("                            <Default AMS port>,\n");
        printf("                            <Maximum parameter count>,\n");
        printf("                            <Asyn priority>,\n");
        printf("                            <Asyn disable auto connect>,\n");
        printf("                            <Default sample time> [ms],\n");
        printf("                            <Default max delay time [ms]>,\n");
        printf("                            <ADS command timeout [ms]>,\n");
        printf("                            <Default time source>)\n");
        printf("\n");
        printf(" Example configuration:\n");
        printf(" 0. Asyn port name                             : \"ADS_1\"\n");
        printf(" 1. IP                                         : \"192.168.88.44\"\n");
        printf(" 2. AMS of plc                                 : \"192.168.88.44.1.1\"\n");
        printf(" 3. Default ams port                           : 851 for plc 1, 852 plc 2 ...\n");
        printf(" 4. Parameter table size (max parameters)      : 1000 example\n");
        printf(" 5. priority                                   : 0\n");
        printf(" 6. disable auto connect                       : 0 (autoconnect enabled)\n");
        printf(" 7. default sample time ms                     : 500 (check if variable changed "
               "each 500ms)\n");
        printf(" 8. max delay time ms (buffer time in plc)     : 1000 (if changed, send data "
               "atleast each 1000ms or faster if send buffer is full)\n");
        printf(
            " 9. ADS command timeout in ms                 : 1000 (timeout for adsLib commands)\n");
        printf(" 10. default time source (PLC=0,EPICS=1).      : 0 (PLC) NOTE: record TSE field "
               "need to be set to -2 for timestamp in asyn (field(TSE, -2))\n");
        printf("\n");
        printf(" Resulting adsAsynPortDriverConfigure() command: \n");
        printf(" adsAsynPortDriverConfigure(\"ADS_1\",\"192.168.88.44\",\"192.168.88.44.1.1\",851,"
               "1000,0,0,50,100,1000,0)\n");
        printf("\n");
        printf("\n");
        printf(
            " NOTE: An ADS route needs to be added to the TwinCAT router of the controller/PLC:\n");
        printf("       1. \"TwinCAT->System->Routes->Static Routes\": Press \"Add\" button.\n");
        printf("       2. \"Route Name (Target)\": Enter name of EPICS machine.\n");
        printf("       3. \"AMSNetId\": Enter IP of EPICS machine. Add \".1.1\" in the end "
               "(x.x.x.x.1.1).\n");
        printf("       4. \"Address Info\": Enter IP of EPICS machine (x.x.x.x).\n");
        printf("       5. Choose \"IP Address\" checkbox.\n");
        printf("       6. Choose \"Remote Route\"->\"None\" checkbox.\n");
        printf("       7. Press \"Add Route\" button.\n");
        printf("       8. Close \"Add Route Dialog\".\n");
        printf("       9. Ensure that the route was successfully added in the \"Static Routes\" "
               "list.\n");
        printf("\n");

        return;
    }
    /*
   * Configure and register
   */
    epicsShareFunc int adsAsynPortDriverConfigure(const char* portName,
                                                  const char* ipaddr,
                                                  const char* amsaddr,
                                                  unsigned int amsport,
                                                  unsigned int asynParamTableSize,
                                                  unsigned int priority,
                                                  int noAutoConnect,
                                                  int defaultSampleTimeMS,
                                                  int maxDelayTimeMS,
                                                  int adsTimeoutMS,
                                                  int defaultTimeSource)
    {
        if (!portName)
        {
            printHelp();
            return -1;
        }

        if (strlen(portName) == 0 || strcmp(portName, "-h") == 0)
        {
            printHelp();
            return -1;
        }

        if (!ipaddr)
        {
            printf("adsAsynPortDriverConfigure bad ipaddr: %s\n", ipaddr ? ipaddr : "");
            return -1;
        }
        if (!amsaddr)
        {
            printf("adsAsynPortDriverConfigure bad amsaddr: %s\n", amsaddr ? amsaddr : "");
            return -1;
        }
        if (defaultSampleTimeMS < 0)
        {
            printf("adsAsynPortDriverConfigure bad defaultSampleTimeMS: %dms. Standard value of "
                   "100ms will be used.\n",
                   defaultSampleTimeMS);
            defaultSampleTimeMS = 100;
        }

        if (maxDelayTimeMS < 0)
        {
            printf("adsAsynPortDriverConfigure bad maxDelayTimeMS: %dms. Standard value of 500ms "
                   "will be used.\n",
                   maxDelayTimeMS);
            maxDelayTimeMS = 500;
        }

        if (adsTimeoutMS < 0)
        {
            printf("adsAsynPortDriverConfigure bad adsTimeoutMS: %dms. Standard value of 2000ms "
                   "will be used.\n",
                   adsTimeoutMS);
            adsTimeoutMS = 2000;
        }

        if (defaultTimeSource < 0 || defaultTimeSource >= ADS_TIME_BASE_MAX)
        {
            printf("adsAsynPortDriverConfigure bad default time source: %d. EPICS (IOC) time "
                   "stamps will be used. Valid options are: EPICS=%d and PLC=%d.\n",
                   defaultTimeSource,
                   (int)ADS_TIME_BASE_EPICS,
                   (int)ADS_TIME_BASE_PLC);
            /* Default to EPICS/IOC host time. PLC time requires a reliable PLC
             * time symbol that every PLC app exposes; until that exists, IOC
             * time is the only trustworthy default. (Legacy
             * MAIN.fbSystemTime is not a platform guarantee.) */
            defaultTimeSource = ADS_TIME_BASE_EPICS;
        }

        adsAsynPortObj = new adsAsynPortDriver(portName,
                                               ipaddr,
                                               amsaddr,
                                               amsport,
                                               asynParamTableSize,
                                               priority,
                                               noAutoConnect == 0,
                                               defaultSampleTimeMS,
                                               maxDelayTimeMS,
                                               adsTimeoutMS,
                                               (ADSTIMESOURCE)defaultTimeSource);
        if (adsAsynPortObj)
        {
            asynUser* traceUser = adsAsynPortObj->getTraceAsynUser();
            if (!traceUser)
            {
                printf(
                    "adsAsynPortDriverConfigure: ERROR: Failed to retrieve asynUser for trace. \n");
                return (asynError);
            }
            pPrintOutAsynUser = traceUser;
        }

        initHook();

        return asynSuccess;
    }

    /*
   * IOC shell command registration
   */
    static const iocshArg adsAsynPortDriverConfigureArg0  = {"port name", iocshArgString};
    static const iocshArg adsAsynPortDriverConfigureArg1  = {"ip-addr", iocshArgString};
    static const iocshArg adsAsynPortDriverConfigureArg2  = {"ams-addr", iocshArgString};
    static const iocshArg adsAsynPortDriverConfigureArg3  = {"default-ams-port", iocshArgInt};
    static const iocshArg adsAsynPortDriverConfigureArg4  = {"asyn param table size", iocshArgInt};
    static const iocshArg adsAsynPortDriverConfigureArg5  = {"priority", iocshArgInt};
    static const iocshArg adsAsynPortDriverConfigureArg6  = {"disable auto-connect", iocshArgInt};
    static const iocshArg adsAsynPortDriverConfigureArg7  = {"default sample time ms", iocshArgInt};
    static const iocshArg adsAsynPortDriverConfigureArg8  = {"max delay time ms", iocshArgInt};
    static const iocshArg adsAsynPortDriverConfigureArg9  = {"ADS communication timeout ms",
                                                             iocshArgInt};
    static const iocshArg adsAsynPortDriverConfigureArg10 = {"default time source (EPICS=0,PLC=1)",
                                                             iocshArgInt};
    static const iocshArg* adsAsynPortDriverConfigureArgs[] = {&adsAsynPortDriverConfigureArg0,
                                                               &adsAsynPortDriverConfigureArg1,
                                                               &adsAsynPortDriverConfigureArg2,
                                                               &adsAsynPortDriverConfigureArg3,
                                                               &adsAsynPortDriverConfigureArg4,
                                                               &adsAsynPortDriverConfigureArg5,
                                                               &adsAsynPortDriverConfigureArg6,
                                                               &adsAsynPortDriverConfigureArg7,
                                                               &adsAsynPortDriverConfigureArg8,
                                                               &adsAsynPortDriverConfigureArg9,
                                                               &adsAsynPortDriverConfigureArg10};

    static const iocshFuncDef adsAsynPortDriverConfigureFuncDef = {
        "adsAsynPortDriverConfigure", 11, adsAsynPortDriverConfigureArgs};

    static void adsAsynPortDriverConfigureCallFunc(const iocshArgBuf* args)
    {
        adsAsynPortDriverConfigure(args[0].sval,
                                   args[1].sval,
                                   args[2].sval,
                                   args[3].ival,
                                   args[4].ival,
                                   args[5].ival,
                                   args[6].ival,
                                   args[7].ival,
                                   args[8].ival,
                                   args[9].ival,
                                   args[10].ival);
    }

    /*
   * adsSetLocalAddress("ams_net_id")
   */
    static const iocshArg adsSetLocalAddressArg0        = {"local_ams_id", iocshArgString};
    static const iocshArg* adsSetLocalAddressArgs[]     = {&adsSetLocalAddressArg0};
    static const iocshFuncDef adsSetLocalAddressFuncDef = {
        "adsSetLocalAddress", 1, adsSetLocalAddressArgs};

    static void adsSetLocalAddressCallFunc(const iocshArgBuf* args)
    {
        const char* functionName = "adsSetLocalAddress";
        if (!args[0].sval || strlen(args[0].sval) < 11)
        {
            printf("%s:%s: local_ams_id parameter required (of the form A.B.C.D.E.F)\n",
                   driverName,
                   functionName);
            return;
        }
        printf("%s:%s: Setting local AMS Net ID to: %s\n", driverName, functionName, args[0].sval);
        AdsSetLocalAddress(std::string(args[0].sval));
    }

    /*
   * adsPollInfo("name")
   */
    static const iocshArg adsPollInfoArg0        = {"name", iocshArgString};
    static const iocshArg* adsPollInfoArgs[]     = {&adsPollInfoArg0};
    static const iocshFuncDef adsPollInfoFuncDef = {"adsPollInfo", 1, adsPollInfoArgs};

    static void adsPollInfoCallFunc(const iocshArgBuf* args)
    {
        adsAsynPortObj->poll_info(args[0].sval);
    }

    /*
   * This routine is called before multitasking has started, so there's
   * no race condition in the test/set of firstTime.
   */

    static void adsAsynPortDriverRegister(void)
    {
        iocshRegister(&adsAsynPortDriverConfigureFuncDef, adsAsynPortDriverConfigureCallFunc);
        iocshRegister(&adsSetLocalAddressFuncDef, adsSetLocalAddressCallFunc);
        iocshRegister(&adsPollInfoFuncDef, adsPollInfoCallFunc);
    }

    epicsExportRegistrar(adsAsynPortDriverRegister);
}

long adsAsynPortDriver::getAdsClientPortNumberForThreadId(epicsThreadId threadId)
{
    std::lock_guard<std::recursive_mutex> lockGuard(threadIdToAmsClientPortMapMutex_);

    // If the map doesn't contain the thread id already, return the default one.
    if (threadIdToAmsClientPortMap_.find(threadId) != threadIdToAmsClientPortMap_.end())
        return threadIdToAmsClientPortMap_.at(0).port;

    // Otherwise, get the ads client port assigned to the input threadId.
    return threadIdToAmsClientPortMap_.at(threadId).port;
}

long adsAsynPortDriver::addAdsClientPortNumberForThreadId(epicsThreadId threadId)
{
    std::lock_guard<std::recursive_mutex> lockGuard(threadIdToAmsClientPortMapMutex_);

    // If the map contains the thread id already, increment the live count
    // and return the port number used for it.
    auto it = threadIdToAmsClientPortMap_.find(threadId);
    if (it != threadIdToAmsClientPortMap_.end())
    {
        it->second.liveCount++;
        return it->second.port;
    }

    // If we get here, the thread id doesn't already have an ads client port assigned for it.
    // In this case, open a new port for it.
    auto adsClientPort = AdsPortOpenEx();
    if (isInvalidPortNumber(adsClientPort))
        return 0;

    // Insert the new pair.
    threadIdToAmsClientPortMap_.insert(
        std::make_pair(threadId, AmsClientPortEntry(adsClientPort, 1)));
    return adsClientPort;
}

asynStatus adsAsynPortDriver::delAdsClientPortNumberForThreadId(epicsThreadId threadId)
{
    std::lock_guard<std::recursive_mutex> lockGuard(threadIdToAmsClientPortMapMutex_);

    // If the map contains the thread id erase it.
    auto it = threadIdToAmsClientPortMap_.find(threadId);
    if (it != threadIdToAmsClientPortMap_.end())
    {
        it->second.liveCount--;
        if (it->second.liveCount > 0)
        {
            // Do not close the port if the live count is still above 0.
            // This means this thread is still using this port somewhere.
            return asynSuccess;
        }
        auto result = AdsPortCloseEx(it->second.port);
        if (result)
        {
            asynPrint(pasynUserSelf,
                      ASYN_TRACE_WARNING,
                      "%s:%s: found thread id in map but failed to close port.\n",
                      driverName,
                      __func__);
            return asynError;
        }
        threadIdToAmsClientPortMap_.erase(it);
    }
    return asynSuccess;
}

AdsClientPortGuard::AdsClientPortGuard(adsAsynPortDriver& adsAsynPortDriver, long& adsClientPort)
    : adsAsynPortDriver_(adsAsynPortDriver)
{
    threadId_      = epicsThreadGetIdSelf();
    adsClientPort_ = adsAsynPortDriver_.addAdsClientPortNumberForThreadId(threadId_);
    adsClientPort  = adsClientPort_;
}
AdsClientPortGuard::~AdsClientPortGuard()
{
    adsAsynPortDriver_.delAdsClientPortNumberForThreadId(threadId_);
}
long AdsClientPortGuard::getAdsClientPort() const
{
    return adsClientPort_;
}
