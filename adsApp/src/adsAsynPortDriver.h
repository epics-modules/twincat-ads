#ifndef ADSASYNPORTDRIVER_H_
#define ADSASYNPORTDRIVER_H_

#include "asynPortDriver.h"
#include <epicsEvent.h>
#include <dbCommon.h>
#include <dbBase.h>
#include <dbStaticLib.h>
#include "AdsLib.h"
#include <vector>
#include "adsAsynPortDriverUtils.h"
#include "adsSymbolTable.h"
#include <mutex>
#include <queue>
#include <atomic>
#include <algorithm>

/** Class derived of asynPortDriver for ads communication with TwinCAT plc:s */

class adsAsynPortDriver : public asynPortDriver
{
  public:
    adsAsynPortDriver(const char* portName,
                      const char* ipaddr,
                      const char* amsaddr,
                      unsigned int amsport,
                      int paramTableSize,
                      unsigned int priority,
                      int autoConnect,
                      int defaultSampleTimeMS,
                      int maxDelayTimeMS,
                      int adsTimeoutMS,
                      ADSTIMESOURCE defaultTimeSource);

    virtual ~adsAsynPortDriver();
    virtual void report(FILE* fp, int details);
    virtual asynStatus disconnect(asynUser* pasynUser);
    virtual asynStatus connect(asynUser* pasynUser);
    virtual asynStatus
    drvUserCreate(asynUser* pasynUser, const char* drvInfo, const char** pptypeName, size_t* psize);
    virtual asynStatus
    writeOctet(asynUser* pasynUser, const char* value, size_t maxChars, size_t* nActual);
    virtual asynStatus
    readOctet(asynUser* pasynUser, char* value, size_t maxChars, size_t* nActual, int* eomReason);
    virtual asynStatus readInt32(asynUser* pasynUser, epicsInt32* value);

    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    virtual asynStatus readFloat64(asynUser* pasynUser, epicsFloat64* value);
    virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);
    virtual asynStatus
    readInt8Array(asynUser* pasynUser, epicsInt8* value, size_t nElements, size_t* nIn);
    virtual asynStatus writeInt8Array(asynUser* pasynUser, epicsInt8* value, size_t nElements);
    virtual asynStatus
    readInt16Array(asynUser* pasynUser, epicsInt16* value, size_t nElements, size_t* nIn);
    virtual asynStatus writeInt16Array(asynUser* pasynUser, epicsInt16* value, size_t nElements);
    virtual asynStatus
    readInt32Array(asynUser* pasynUser, epicsInt32* value, size_t nElements, size_t* nIn);
    virtual asynStatus writeInt32Array(asynUser* pasynUser, epicsInt32* value, size_t nElements);
    virtual asynStatus
    readFloat32Array(asynUser* pasynUser, epicsFloat32* value, size_t nElements, size_t* nIn);
    virtual asynStatus
    writeFloat32Array(asynUser* pasynUser, epicsFloat32* value, size_t nElements);
    virtual asynStatus
    readFloat64Array(asynUser* pasynUser, epicsFloat64* value, size_t nElements, size_t* nIn);
    virtual asynStatus
    writeFloat64Array(asynUser* pasynUser, epicsFloat64* value, size_t nElements);
    // 64-bit integer interface support (asynInt64 and asynInt64Array)
    virtual asynStatus readInt64(asynUser* pasynUser, epicsInt64* value);

    // 64-bit integer interface support (asynInt64 and asynInt64Array)
    virtual asynStatus writeInt64(asynUser* pasynUser, epicsInt64 value);
    virtual asynStatus
    readInt64Array(asynUser* pasynUser, epicsInt64* value, size_t nElements, size_t* nIn);
    virtual asynStatus writeInt64Array(asynUser* pasynUser, epicsInt64* value, size_t nElements);

    asynStatus adsUpdateParameterLock(adsParamInfo* paramInfo, const void* data);
    asynStatus invalidateParamsLock(uint16_t amsPort);
    asynStatus refreshParamsLock(uint16_t amsClientPort, uint16_t amsPort);
    asynStatus adsAddRoute();
    asynStatus fireAllCallbacksLock();
    asynUser* getTraceAsynUser();
    int getParamTableSize();
    adsParamInfo* getAdsParamInfo(int index);
    int getAdsParamCount();
    bool isCallbackAllowed(adsParamInfo* paramInfo);
    bool isCallbackAllowed(uint16_t amsPort);

    void cyclicThread();
    void bulkReadThread();
    void dataCallbackThread();
    void triggerEpicsIoIntrCallbacksThread();
    void poll_info(char* name);
    // data callback thread
#define MAXCBQSIZE 10000
    struct datacbinfo
    {
        adsParamInfo* paramInfo;
        void* data;
        const AdsNotificationHeader pNotification;
    };
    std::queue<datacbinfo> datacbqueue;

    long getAdsClientPortNumberForThreadId(epicsThreadId threadId);
    long addAdsClientPortNumberForThreadId(epicsThreadId threadId);
    asynStatus delAdsClientPortNumberForThreadId(epicsThreadId threadId);

  protected:
  private:
    //Asyn and EPICS methods
    asynStatus connectLock(asynUser* pasynUser);
    asynStatus disconnectLock(asynUser* pasynUser);

    asynStatus validateDrvInfo(const char* drvInfo);
    asynStatus getRecordInfoFromDrvInfo(const char* drvInfo, adsParamInfo* paramInfo);
    asynStatus parsePlcInfofromDrvInfo(const char* drvInfo, adsParamInfo* paramInfo);
    asynStatus refreshParams(uint16_t amsClientPort);
    asynStatus refreshParams(uint16_t amsClientPort, uint16_t amsPort);
    asynStatus invalidateParams(uint16_t amsPort);
    asynStatus adsUpdateParameter(adsParamInfo* paramInfo, const void* data);
    asynStatus adsUpdateParameter(adsParamInfo* paramInfo, const void* data, size_t dataSize);
    asynStatus adsUpdateParameterLock(adsParamInfo* paramInfo, const void* data, size_t dataSize);

    // ADS methods
    asynStatus adsAddDataCallback(uint16_t amsClientPort, adsParamInfo* paramInfo);

    asynStatus adsDelDataCallback(uint16_t amsClientPort, adsParamInfo* paramInfo);
    asynStatus
    adsDelDataCallback(uint16_t amsClientPort, adsParamInfo* paramInfo, bool blockErrorMsg);
    asynStatus adsAddSymbolsChangedCallback(uint16_t amsClientPort, amsPortInfo* port);
    asynStatus adsDelSymbolsChangedCallback(uint16_t amsClientPort, amsPortInfo* port);
    asynStatus adsGetSymInfoByName(uint16_t amsClientPort, adsParamInfo* paramInfo);
    asynStatus adsGetSymInfoByName(uint16_t amsClientPort,
                                   uint16_t amsPort,
                                   const char* varName,
                                   adsSymbolEntry* info);
    asynStatus adsGetSymInfoByName(uint16_t amsClientPort,
                                   uint16_t amsPort,
                                   const char* varName,
                                   adsSymbolEntry* info,
                                   long* errorCode);
    asynStatus adsGetSymHandleByName(uint16_t amsClientPort, adsParamInfo* paramInfo);
    asynStatus
    adsGetSymHandleByName(uint16_t amsClientPort, adsParamInfo* paramInfo, bool blockErrorMsg);
    asynStatus adsReleaseSymbolicHandle(uint16_t amsClientPort, adsParamInfo* paramInfo);
    asynStatus
    adsReleaseSymbolicHandle(uint16_t amsClientPort, adsParamInfo* paramInfo, bool blockErrorMsg);
    asynStatus adsConnect(uint16_t amsClientPort);
    asynStatus adsDisconnect();
    asynStatus adsWriteParam(uint16_t amsClientPort,
                             adsParamInfo* paramInfo,
                             const void* binaryBuffer,
                             uint32_t bytesToWrite);
    asynStatus adsReadParam(uint16_t amsClientPort, adsParamInfo* paramInfo);
    asynStatus
    adsReadParam(uint16_t amsClientPort, adsParamInfo* paramInfo, long* error, int updateAsynPar);
    asynStatus adsReadState(uint16_t amsClientPort, uint16_t* adsState);
    asynStatus
    adsReadState(uint16_t amsClientPort, uint16_t amsport, uint16_t* adsState, bool blockErrorMsg);
    asynStatus adsReadState(uint16_t amsClientPort,
                            uint16_t amsport,
                            uint16_t* adsState,
                            bool blockErrorMsg,
                            long* error);
    asynStatus adsWriteState(uint16_t amsClientPort, uint16_t amsport, uint16_t adsState);

    asynStatus adsDelRoute(int force);
    asynStatus adsGenericArrayWrite(uint16_t amsClientPort,
                                    asynUser* pasynUser,
                                    long allowedType,
                                    const void* epicsDataBuffer,
                                    size_t nEpicsBufferBytes);
    asynStatus adsGenericArrayRead(uint16_t amsClientPort,
                                   asynUser* pasynUser,
                                   long allowedType,
                                   void* epicsDataBuffer,
                                   size_t nEpicsBufferBytes,
                                   size_t* nBytesRead);
    asynStatus adsReadVersion(uint16_t amsClientPort, amsPortInfo* port);
    asynStatus updateParamInfoWithPLCInfo(uint16_t amsClientPort, adsParamInfo* paramInfo);
    asynStatus refreshParamTime(adsParamInfo* paramInfo);
    asynStatus setAlarmPortLock(uint16_t amsPort, int alarm, int severity);
    asynStatus setAlarmPort(uint16_t amsPort, int alarm, int severity);
    asynStatus setAlarmParam(adsParamInfo* paramInfo, int alarm, int severity);
    asynStatus fireCallbacksForArrayParam(adsParamInfo* paramInfo);
    asynStatus addNewAmsPortToList(uint16_t amsPort);
    amsPortInfo* getAmsPortObject(uint16_t amsPort);
    asynStatus adsAddToBulkRead(uint16_t amsClientPort, adsParamInfo* paramInfo);
    int adsFindBulkTimeStamp(uint16_t amsClientPort, uint16_t amsPort);

    //Octet interface methods (ascii command parser through readoctet() and writeoctet())
    int octetCMDreadIt(uint16_t amsClientPort, char* outbuf, size_t outlen);
    int octetCMDwriteIt(uint16_t amsClientPort, const char* inbuf, size_t inlen);
    int octetCmdHandleInputLine(uint16_t amsClientPort,
                                const char* input_line,
                                adsOctetOutputBufferType* buffer);
    int octetMotorHandleOneArg(uint16_t amsClientPort,
                               const char* myarg_1,
                               adsOctetOutputBufferType* buffer);
    int octetMotorHandleADRCmd(uint16_t amsClientPort,
                               const char* arg,
                               uint16_t adsport,
                               adsOctetOutputBufferType* buffer);
    int octetAdsReadByName(uint16_t amsClientPort,
                           uint16_t amsPort,
                           const char* variableAddr,
                           adsOctetOutputBufferType* outBuffer);
    int octetAdsWriteByName(uint16_t amsClientPort,
                            uint16_t amsPort,
                            const char* variableAddr,
                            const char* asciiValueToWrite,
                            adsOctetOutputBufferType* outBuffer);
    int octetAdsReadByGroupOffset(uint16_t amsClientPort,
                                  uint16_t amsPort,
                                  adsSymbolEntry* info,
                                  adsOctetOutputBufferType* outBuffer);
    int octetAdsWriteByGroupOffset(uint16_t amsClientPort,
                                   uint16_t amsPort,
                                   uint32_t group,
                                   uint32_t offset,
                                   uint16_t dataType,
                                   uint32_t dataSize,
                                   const char* asciiValueToWrite,
                                   adsOctetOutputBufferType* asciiResponseBuffer);

    char* ipaddr_;
    char* amsaddr_;
    int autoConnect_;
    int adsParamArrayCount_;
    int paramTableSize_;
    int defaultSampleTimeMS_;
    int defaultMaxDelayTimeMS_;
    int adsTimeoutMS_;
    int connectedAds_;
    long adsPort_;
    int routeAdded_;
    int notConnectedCounter_;
    int oneAmsConnectionOKold_;
    uint16_t amsportDefault_;
    unsigned int priority_;
    AmsNetId remoteNetId_;
    std::vector<adsParamInfo> adsParamArray_;
    std::vector<amsPortInfo*> amsPortList_;
    ADSTIMESOURCE defaultTimeSource_;
    std::unordered_map<epicsThreadId, AmsClientPortEntry> threadIdToAmsClientPortMap_;
    std::recursive_mutex threadIdToAmsClientPortMapMutex_;

    // Worker-thread lifecycle
    // Cooperative shutdown so the destructor can stop and join the worker
    // threads before releasing handles / freeing adsParamArray_ / closing the
    // ADS port. Without this, the still-running cyclic/bulk threads race the
    // destructor on the shared AmsRouter (teardown hang) or it frees memory out
    // from under them (use-after-free crash).
    std::atomic<bool> stopThreads_{false};
    epicsThreadId cyclicThreadId_{nullptr};
    epicsThreadId bulkReadThreadId_{nullptr};
    epicsThreadId dataCallbackThreadId_{nullptr};
    epicsThreadId triggerIoIntrThreadId_{nullptr};
    std::unordered_map<std::string, int> createdParamsMap_;
    std::mutex bulkReadInfoMutex_;
    std::queue<adsParamInfo*> arrayParamsToCallCallbacksFor_;

    //octet
    adsOctetOutputBufferType octetAsciiBuffer_;
    uint8_t octetBinaryBuffer_[ADS_CMD_BUFFER_SIZE];
    int octetReturnVarName_;

    //bulk read
#define MAXTSENTRY 10
    struct tsentry
    {
        uint16_t amsPort;
        uint32_t iHandleH;
        uint32_t iHandleL;
        int refreshNeeded;
    } bulkTS[MAXTSENTRY];
    int bulkTScnt;
#define MAXBULK 2000
#define BULKSIZ 500
    struct BulkReadInfo
    {
        int cnt;          // Number of variables in this read
        uint16_t amsPort; // The port this goes to!
        struct
        {
            uint32_t iGroup;
            uint32_t iOffset;
            uint32_t iSize;
        } sum[BULKSIZ];       // The actual request!
        int paramID[BULKSIZ]; // The asyn parameter handles
        int readSize;         // The total size of the read expected (including status).
        int refreshNeeded;
    } bulk[MAXBULK];
    int bulk_delay_us; // Rate to process bulk reads.
    uint8_t* bulkdata; // A read buffer of maximum size.
    int bulkdatasize;  // Size of the read buffer.
  public:
    int bulkOK;          // OK to process bulk reads!
    int bulk_elapsed_us; // Time of last bulk read loop.

    // member declarations
    std::unordered_map<std::string, AdsSymbolDictEntry> symbolDict_;
    asynStatus resolveSymbolInfo(uint16_t amsClientPort);
    asynStatus resolveSymbolHandles(uint16_t amsClientPort);
    bool symInfoResolved_ = false;

#ifdef ADS_UNIT_TEST
    // ── Test accessors — only compiled when ADS_UNIT_TEST is defined ──────────
    // Exposes private/protected state for Google Test without affecting
    // production builds.

    /** True if connected to the ADS server. */
    bool isConnected() const
    {
        return connectedAds_ != 0;
    }

    /** Local ADS port handle (> 0 when open). */
    long getAdsPort() const
    {
        return adsPort_;
    }

    /** True if the ADS route has been added (not deleted on disconnect). */
    bool isRouteAdded() const
    {
        return routeAdded_ != 0;
    }

    /** Number of entries in symbolDict_ (0 before resolveSymbolInfo). */
    size_t symbolDictSize() const
    {
        return symbolDict_.size();
    }

    /** Look up a symbol by name (case-insensitive). Returns nullptr if not found. */
    const AdsSymbolDictEntry* lookupSymbol(const char* name) const
    {
        std::string key(name);
        std::transform(
            key.begin(), key.end(), key.begin(), [](unsigned char c) { return std::tolower(c); });
        auto it = symbolDict_.find(key);
        return (it != symbolDict_.end()) ? &it->second : nullptr;
    }

    /** Current value of bulkOK flag (1 = bulk read thread active). */
    int getBulkOK() const
    {
        return bulkOK;
    }

    /** Access raw param info array for a given param index. */
    adsParamInfo* getAdsParamInfo(int index) const
    {
        if (index < 0 || index >= (int)adsParamArray_.size())
            return nullptr;
        return const_cast<adsParamInfo*>(&adsParamArray_[index]);
    }

    /** Set the global adsAsynPortObj pointer for initHook callbacks.
   *  Required when creating the driver directly (not via iocsh command)
   *  so that initHookAfterScanInit can find the driver and set bulkOK=1. */
    static void setGlobalInstance(adsAsynPortDriver* obj);

#endif /* ADS_UNIT_TEST */
};

class AdsClientPortGuard
{
  public:
    AdsClientPortGuard(adsAsynPortDriver& adsAsynPortDriver, long& adsClientPort);
    ~AdsClientPortGuard();
    long getAdsClientPort() const;

  private:
    adsAsynPortDriver& adsAsynPortDriver_;
    long adsClientPort_;
    epicsThreadId threadId_;
};


#endif /* ADSASYNPORTDRIVER_H_ */
