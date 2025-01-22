/*
    This file is part of twincat-ads.

    twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

    twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License along with twincat-ads. If not, see <https://www.gnu.org/licenses/>.

*/
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
#include <mutex>

/** Class derived of asynPortDriver for ads communication with TwinCAT plc:s */

class adsAsynPortDriver : public asynPortDriver {
public:
  adsAsynPortDriver(const char *portName,
                    const char *ipaddr,
                    const char *amsaddr,
                    unsigned int amsport,
                    int paramTableSize,
                    unsigned int priority,
                    int autoConnect,
                    int defaultSampleTimeMS,
                    int maxDelayTimeMS,
                    int adsTimeoutMS,
                    ADSTIMESOURCE defaultTimeSource);

  virtual ~adsAsynPortDriver();
  virtual void report(FILE *fp, int details);
  virtual asynStatus disconnect(asynUser *pasynUser);
  virtual asynStatus connect(asynUser *pasynUser);
  virtual asynStatus drvUserCreate(asynUser *pasynUser,
                                   const char *drvInfo,
                                   const char **pptypeName,
                                   size_t *psize);
  virtual asynStatus writeOctet(asynUser *pasynUser,
                                const char *value,
                                size_t maxChars,
                                size_t *nActual);
  virtual asynStatus readOctet(asynUser *pasynUser,
                               char *value,
                               size_t maxChars,
                               size_t *nActual,
                               int *eomReason);
  virtual asynStatus writeInt32(asynUser *pasynUser,
                                epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser,
                                  epicsFloat64 value);
  virtual asynStatus readInt8Array(asynUser *pasynUser,
                                   epicsInt8 *value,
                                   size_t nElements,
                                   size_t *nIn);
  virtual asynStatus writeInt8Array(asynUser *pasynUser,
                                    epicsInt8 *value,
                                    size_t nElements);
  virtual asynStatus readInt16Array(asynUser *pasynUser,
                                    epicsInt16 *value,
                                   size_t nElements,
                                    size_t *nIn);
  virtual asynStatus writeInt16Array(asynUser *pasynUser,
                                     epicsInt16 *value,
                                     size_t nElements);
  virtual asynStatus readInt32Array(asynUser *pasynUser,
                                    epicsInt32 *value,
                                    size_t nElements,
                                    size_t *nIn);
  virtual asynStatus writeInt32Array(asynUser *pasynUser,
                                     epicsInt32 *value,
                                     size_t nElements);
  virtual asynStatus readFloat32Array(asynUser *pasynUser,
                                      epicsFloat32 *value,
                                      size_t nElements,
                                      size_t *nIn);
  virtual asynStatus writeFloat32Array(asynUser *pasynUser,
                                       epicsFloat32 *value,
                                       size_t nElements);
  virtual asynStatus readFloat64Array(asynUser *pasynUser,
                                      epicsFloat64 *value,
                                      size_t nElements,
                                      size_t *nIn);
  virtual asynStatus writeFloat64Array(asynUser *pasynUser,
                                       epicsFloat64 *value,
                                       size_t nElements);
  asynStatus adsUpdateParameterLock(adsParamInfo* paramInfo,
                                    const void *data);
  asynStatus invalidateParamsLock(uint16_t amsPort);
  asynStatus refreshParamsLock(uint16_t amsPort);
  asynStatus adsDelRouteLock(int force);
  asynStatus adsAddRouteLock();
  asynStatus fireAllCallbacksLock();
  asynUser *getTraceAsynUser();
  int getParamTableSize();
  adsParamInfo *getAdsParamInfo(int index);
  int getAdsParamCount();
  bool isCallbackAllowed(adsParamInfo *paramInfo);
  bool isCallbackAllowed(uint16_t amsPort);

  void cyclicThread();
  void bulkReadThread();
  void poll_info(char *name);
protected:

private:
  //Asyn and EPICS methods
  asynStatus connectLock(asynUser *pasynUser);
  asynStatus disconnectLock(asynUser *pasynUser);

  asynStatus validateDrvInfo(const char *drvInfo);
  asynStatus getRecordInfoFromDrvInfo(const char *drvInfo,
                                      adsParamInfo *paramInfo);
  asynStatus parsePlcInfofromDrvInfo(const char* drvInfo,
                                     adsParamInfo *paramInfo);
  asynStatus refreshParams();
  asynStatus refreshParams(uint16_t amsPort);
  asynStatus invalidateParams(uint16_t amsPort);
  asynStatus adsUpdateParameter(adsParamInfo* paramInfo,
                                 const void *data);
  asynStatus adsUpdateParameter(adsParamInfo* paramInfo,
                                 const void *data,size_t dataSize);
  asynStatus adsUpdateParameterLock(adsParamInfo* paramInfo,
                                    const void *data,
                                    size_t dataSize);

  // ADS methods
  asynStatus adsAddDataCallback(adsParamInfo *paramInfo);

  asynStatus adsDelDataCallback(adsParamInfo *paramInfo);
  asynStatus adsDelDataCallback(adsParamInfo *paramInfo,
                                        bool blockErrorMsg);
  asynStatus adsAddSymbolsChangedCallback(amsPortInfo *port);
  asynStatus adsDelSymbolsChangedCallback(amsPortInfo *port);
  asynStatus adsGetSymInfoByName(adsParamInfo *paramInfo);
  asynStatus adsGetSymInfoByName(uint16_t amsPort,
                                 const char * varName,
                                 adsSymbolEntry * info);
  asynStatus adsGetSymInfoByName(uint16_t amsPort,
                                 const char *varName,
                                 adsSymbolEntry *info,
                                 long *errorCode);
  asynStatus adsGetSymHandleByName(adsParamInfo *paramInfo);
  asynStatus adsGetSymHandleByName(adsParamInfo *paramInfo,
                                   bool blockErrorMsg);
  asynStatus adsReleaseSymbolicHandle(adsParamInfo *paramInfo);
  asynStatus adsReleaseSymbolicHandle(adsParamInfo *paramInfo,
                                      bool blockErrorMsg);
  asynStatus adsConnect();
  asynStatus adsDisconnect();
  asynStatus adsWriteParam(adsParamInfo *paramInfo,
                      const void *binaryBuffer,
                      uint32_t bytesToWrite);
  asynStatus adsReadParam(adsParamInfo *paramInfo);
  asynStatus adsReadParam(adsParamInfo *paramInfo,
                          long *error,
                          int updateAsynPar);
  asynStatus adsReadState(uint16_t *adsState);
  asynStatus adsReadStateLock(uint16_t amsport,
                              uint16_t *adsState,
                              bool blockErrorMsg);
  asynStatus adsReadStateLock(uint16_t amsport,
                              uint16_t *adsState,
                              bool blockErrorMsg,
                              long *error);
  asynStatus adsReadState(uint16_t amsport,
                          uint16_t *adsState,
                          bool blockErrorMsg,
                          long *error);
  asynStatus adsWriteState(uint16_t amsport,
                          uint16_t adsState);

  asynStatus adsDelRoute(int force);
  asynStatus adsGenericArrayWrite(asynUser *pasynUser,
                                  long allowedType,
                                  const void *epicsDataBuffer,
                                  size_t nEpicsBufferBytes);
  asynStatus adsGenericArrayRead(asynUser *pasynUser,
                                 long allowedType,
                                 void *epicsDataBuffer,
                                 size_t nEpicsBufferBytes,
                                 size_t *nBytesRead);
  asynStatus adsReadVersion(amsPortInfo *port);
  asynStatus updateParamInfoWithPLCInfo(adsParamInfo *paramInfo);
  asynStatus refreshParamTime(adsParamInfo *paramInfo);
  asynStatus setAlarmPortLock(uint16_t amsPort,int alarm,int severity);
  asynStatus setAlarmPort(uint16_t amsPort,int alarm,int severity);
  asynStatus setAlarmParam(adsParamInfo *paramInfo,int alarm,int severity);
  asynStatus fireCallbacks(adsParamInfo* paramInfo);
  asynStatus addNewAmsPortToList(uint16_t amsPort);
  amsPortInfo* getAmsPortObject(uint16_t amsPort);
  void       adsLock();
  void       adsUnlock();
  asynStatus adsAddToBulkRead(adsParamInfo* paramInfo);
  int        adsFindBulkTimeStamp(uint16_t amsPort);

  //Octet interface methods (ascii command parser through readoctet() and writeoctet())
  int        octetCMDreadIt(char *outbuf,
                            size_t outlen);
  int        octetCMDwriteIt(const char *inbuf,
                             size_t inlen);
  int        octetCmdHandleInputLine(const char *input_line,
                                     adsOctetOutputBufferType *buffer);
  int        octetMotorHandleOneArg(const char *myarg_1,
                                    adsOctetOutputBufferType *buffer);
  int        octetMotorHandleADRCmd(const char *arg,
                                    uint16_t adsport,
                                    adsOctetOutputBufferType *buffer);
  int        octetAdsReadByName(uint16_t amsPort,
                                const char *variableAddr,
                                adsOctetOutputBufferType* outBuffer);
  int        octetAdsWriteByName(uint16_t amsPort,
                                 const char *variableAddr,
                                 const char *asciiValueToWrite,
                                 adsOctetOutputBufferType *outBuffer);
  int        octetAdsReadByGroupOffset(uint16_t amsPort,
                                       adsSymbolEntry *info,
                                       adsOctetOutputBufferType *outBuffer);
  int        octetAdsWriteByGroupOffset(uint16_t amsPort,
                                        uint32_t group,
                                        uint32_t offset,
                                        uint16_t dataType,
                                        uint32_t dataSize,
                                        const char *asciiValueToWrite,
                                        adsOctetOutputBufferType *asciiResponseBuffer);

  char                           *ipaddr_;
  char                           *amsaddr_;
  int                            autoConnect_;
  int                            adsParamArrayCount_;
  int                            paramTableSize_;
  int                            defaultSampleTimeMS_;
  int                            defaultMaxDelayTimeMS_;
  int                            adsTimeoutMS_;
  int                            connectedAds_;
  long                           adsPort_;
  int                            routeAdded_;
  int                            notConnectedCounter_;
  int                            oneAmsConnectionOKold_;
  uint16_t                       amsportDefault_;
  unsigned int                   priority_;
  AmsNetId                       remoteNetId_;
  adsParamInfo                   **pAdsParamArray_;
  std::vector<amsPortInfo*>      amsPortList_;
  ADSTIMESOURCE                  defaultTimeSource_;
  std::mutex                     adsMutex;

  //octet
  adsOctetOutputBufferType       octetAsciiBuffer_;
  uint8_t                        octetBinaryBuffer_[ADS_CMD_BUFFER_SIZE];
  int                            octetReturnVarName_;

  //bulk read
#define MAXTSENTRY 10
  struct tsentry {
      uint16_t amsPort;
      uint32_t iHandleH;
      uint32_t iHandleL;
      int refreshNeeded;
  } bulkTS[MAXTSENTRY];
  int bulkTScnt;
#define MAXBULK 2000
#define BULKSIZ 500
  struct {
      int cnt;               // Number of variables in this read
      uint16_t amsPort;      // The port this goes to!
      struct {
          uint32_t iGroup;
          uint32_t iOffset;
          uint32_t iSize;
      } sum[BULKSIZ];        // The actual request!
      int paramID[BULKSIZ];  // The asyn parameter handles
      int readSize;          // The total size of the read expected (including status).
      int refreshNeeded;
  } bulk[MAXBULK];
  int bulk_delay_us;         // Rate to process bulk reads.
  uint8_t *bulkdata;         // A read buffer of maximum size.
  int bulkdatasize;          // Size of the read buffer.
 public:
  int bulkOK;                // OK to process bulk reads!
  int bulk_elapsed_us;       // Time of last bulk read loop.
};

#endif /* ADSASYNPORTDRIVER_H_ */



