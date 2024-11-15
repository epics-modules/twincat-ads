/*
    This file is part of epics-twincat-ads.

    epics-twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

    epics-twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License along with epics-twincat-ads. If not, see <https://www.gnu.org/licenses/>.

*/
//#define MCB_DEBUG
/*
* adsAsynPortDriver.cpp
*
* Class derived of asynPortDriver for ADS communication with TwinCAT plcs.
* AdsLib written by Beckhoff is used for communication: https://github.com/Beckhoff/ADS
*
* Author: Anders Sandström
* Edited to add bulk reads: Michael Browne
*
* Created January 25, 2018
* Edited  December 6, 2019
*/

#define USE_TYPED_RSET    // Shut up about rset already!
#include "adsAsynPortDriver.h"

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
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

static const char *driverName="adsAsynPortDriver";
static adsAsynPortDriver *adsAsynPortObj;
static long oldTimeStamp=0;
static struct timeval oldTime={0};
static int allowCallbackEpicsState=0;
static initHookState currentEpicsState=initHookAtIocBuild;


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

  if(!adsAsynPortObj){
    printf("%s:%s: ERROR: adsAsynPortObj==NULL\n", driverName, functionName);
    return;
  }

  asynUser *asynTraceUser=adsAsynPortObj->getTraceAsynUser();

  switch(state) {
      break;
    case initHookAfterInitDevSup:
        gettimeofday(&start, NULL);
        break;
    case initHookAfterInitDatabase:
        gettimeofday(&now, NULL);
        timersub(&now, &start, &diff);
        printf("Database initialization took %ld.%05ld seconds.\n", diff.tv_sec, (long)diff.tv_usec);
        break;
    case initHookAfterScanInit:
      allowCallbackEpicsState=1;

      //make all callbacks if data arrived from callback before interrupts were registered (before allowCallbackEpicsState==1)
      if(!adsAsynPortObj){
        printf("%s:%s: ERROR: adsAsynPortObj==NULL\n", driverName, functionName);
        return;
      }
      adsAsynPortObj->fireAllCallbacksLock();
      adsAsynPortObj->bulkOK = 1;
      printf("Begin polling PLC!\n");
      break;
    default:
      break;
  }

  currentEpicsState=state;
  asynPrint(asynTraceUser, ASYN_TRACEIO_DRIVER , "%s:%s: EPICS state: %s (%d). Allow ADS callbacks: %s.\n", driverName, functionName,epicsStateToString((int)state),(int)state,allowCallbackEpicsState ? "true" : "false");
}

/** Register EPICS hook function
 * \return void
 */
int initHook(void)
{
  return(initHookRegister(getEpicsState));
}

/** Callback from ads lib for symbols changed in PLC.
 * \param[in] pAddr AmsAddr of the system generating the callback.
 * \param[in] pNotification Data structure containing the updated data and timestamp information.
 * \param[in] hUser Identification index of the callback parameter.
 * \return void
 * This function will be called by the ADS lib if the symbol version in the PLC is changed.
 */
static void adsSymbolsChangedCallback(const AmsAddr* pAddr, const AdsNotificationHeader* pNotification, uint32_t hUser)
{
  const char* functionName = "adsSymbolsChangedCallback";

  if(!adsAsynPortObj){
    printf("%s:%s: ERROR: adsAsynPortObj==NULL\n", driverName, functionName);
    return;
  }

  asynUser *asynTraceUser=adsAsynPortObj->getTraceAsynUser();
  asynPrint(asynTraceUser, ASYN_TRACE_INFO , "%s:%s: Symbols changed for Ams-port %u.\n", driverName, functionName,pAddr->port);

  adsAsynPortObj->invalidateParamsLock(pAddr->port);
  adsAsynPortObj->refreshParamsLock(pAddr->port);
}

/** Callback from ads lib for updated data.
 * \param[in] pAddr AmsAddr of the system generating the callback.
 * \param[in] pNotification Data structure containing the updated data and timestamp information.
 * \param[in] hUser Identification index of the callback parameter.
 * \return void
 * This function will be called by the ADS lib when a registered parameter is updated (changed in PLC).
 */
static void adsDataCallback(const AmsAddr* pAddr, const AdsNotificationHeader* pNotification, uint32_t hUser)
{
  const char* functionName = "adsDataCallback";

  if(!adsAsynPortObj){
    printf("%s:%s: ERROR: adsAsynPortObj==NULL\n", driverName, functionName);
    return;
  }

  asynUser *asynTraceUser=adsAsynPortObj->getTraceAsynUser();
  asynPrint(asynTraceUser, ASYN_TRACEIO_DRIVER , "%s:%s:\n", driverName, functionName);

  const uint8_t* data = reinterpret_cast<const uint8_t*>(pNotification + 1);
  struct timeval newTime;
  gettimeofday(&newTime, NULL);

  asynPrint(asynTraceUser, ASYN_TRACEIO_DRIVER,"TIME %ld.%06ld\n",(long) newTime.tv_sec, (long) newTime.tv_usec);

  long secs_used=(newTime.tv_sec - oldTime.tv_sec); //avoid overflow by subtracting first
  long micros_used= ((secs_used*1000000) + newTime.tv_usec) - (oldTime.tv_usec);
  oldTime=newTime;

  //Ensure hUser is within range
  if(hUser>(uint32_t)(adsAsynPortObj->getParamTableSize()-1)){
    asynPrint(asynTraceUser, ASYN_TRACE_ERROR, "%s:%s: hUser out of range: %u.\n", driverName, functionName,hUser);
    return;
  }

  //Get paramInfo
  adsParamInfo *paramInfo=adsAsynPortObj->getAdsParamInfo(hUser);
  if(!paramInfo){
    asynPrint(asynTraceUser, ASYN_TRACE_ERROR, "%s:%s: getAdsParamInfo() for hUser %u failed\n", driverName, functionName,hUser);
    return;
  }

  asynPrint(asynTraceUser, ASYN_TRACEIO_DRIVER,"Callback for parameter %s (%d).\n",paramInfo->drvInfo,paramInfo->paramIndex);
  asynPrint(asynTraceUser, ASYN_TRACEIO_DRIVER,"hUser 0x%x, data size[b]: %d.\n", hUser,pNotification->cbSampleSize);
  asynPrint(asynTraceUser, ASYN_TRACEIO_DRIVER,"time stamp [100ns]: %" PRIuMAX ", since last plc [ms]: %4.2lf, since last ioc [ms]: %4.2lf.\n",
            (uintmax_t)pNotification->nTimeStamp,
            ((double)(pNotification->nTimeStamp-oldTimeStamp))/10000.0,
            (((double)(micros_used))/1000.0));
  oldTimeStamp=pNotification->nTimeStamp;

  //Ensure hUser is equal to parameter index
  if(hUser!=(uint32_t)(paramInfo->paramIndex)){
    asynPrint(asynTraceUser, ASYN_TRACE_ERROR, "%s:%s: hUser not equal to parameter index (%u vs %d).\n", driverName, functionName,hUser,paramInfo->paramIndex);
    return;
  }

  paramInfo->plcTimeStampRaw=pNotification->nTimeStamp;
  paramInfo->lastCallbackSize=pNotification->cbSampleSize;

  adsAsynPortObj->adsUpdateParameterLock(paramInfo,data);
}

/** Start cyclic thread for supervision of connection.
 * \param[in] drvPvt adsAsynPortDriver object
 * \return void
 */
void cyclicThread(void *drvPvt)
{
  adsAsynPortDriver *pPvt = (adsAsynPortDriver *)drvPvt;
  pPvt->cyclicThread();
}

/** Start bulk read thread.
 * \param[in] drvPvt adsAsynPortDriver object
 * \return void
 */
void bulkReadThread(void *drvPvt)
{
  adsAsynPortDriver *pPvt = (adsAsynPortDriver *)drvPvt;
  pPvt->bulkReadThread();
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
 *            updated data arrives in the EPCIS client.\n

 * Initializes all variables and tries to connect to PLC system.
 */
adsAsynPortDriver::adsAsynPortDriver(const char *portName,
                                     const char *ipaddr,
                                     const char *amsaddr,
                                     unsigned int amsport,
                                     int paramTableSize,
                                     unsigned int priority,
                                     int autoConnect,
                                     int defaultSampleTimeMS,
                                     int maxDelayTimeMS,
                                     int adsTimeoutMS,
                                     ADSTIMESOURCE defaultTimeSource)
                     :asynPortDriver(portName,
                                     1, /* maxAddr */
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
                                     asynInt64Mask |
#endif
                                     asynInt32Mask | asynFloat64Mask | asynFloat32ArrayMask | asynFloat64ArrayMask | asynDrvUserMask | asynOctetMask | asynInt8ArrayMask | asynInt16ArrayMask | asynInt32ArrayMask, /* Interface mask */
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
                                     asynInt64Mask |
#endif
                                     asynInt32Mask | asynFloat64Mask | asynFloat32ArrayMask | asynFloat64ArrayMask | asynDrvUserMask | asynOctetMask | asynInt8ArrayMask | asynInt16ArrayMask | asynInt32ArrayMask,  /* Interrupt mask */
                                     ASYN_CANBLOCK, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
                                     autoConnect, /* Autoconnect */
                                     priority, /* Default priority */
                                     0) /* Default stack size*/
{
  const char* functionName = "adsAsynPortDriver";
  //Extra Debugging from the beginning: pasynTrace->setTraceMask(pasynUserSelf, 0x11);
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  pAdsParamArray_= new adsParamInfo*[paramTableSize];
  memset(pAdsParamArray_,0,sizeof(*pAdsParamArray_));
  adsParamArrayCount_=0;
  paramTableSize_=paramTableSize;
  ipaddr_=strdup(ipaddr);
  amsaddr_=strdup(amsaddr);
  amsportDefault_=amsport;
  priority_=priority;
  autoConnect_=autoConnect;
  defaultSampleTimeMS_=defaultSampleTimeMS;
  defaultMaxDelayTimeMS_=maxDelayTimeMS;
  adsTimeoutMS_=adsTimeoutMS;
  connectedAds_=0;
  defaultTimeSource_=defaultTimeSource;
  routeAdded_=0;
  notConnectedCounter_ = 0;
  oneAmsConnectionOKold_=0;

  //Octet interface
  octetAsciiBuffer_.bufferSize = ADS_CMD_BUFFER_SIZE;
  octetAsciiBuffer_.bytesUsed=0;
  memset(&octetBinaryBuffer_,0,ADS_CMD_BUFFER_SIZE);
  octetReturnVarName_=0;

  //ADS
  adsPort_=0; //handle
  remoteNetId_={0,0,0,0,0,0};
  amsPortList_.clear();

  if(amsportDefault_<=0){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Invalid default AMS port: %d\n", driverName, functionName,amsportDefault_);
    return;
  }
  addNewAmsPortToList(amsportDefault_);

  int nvals = sscanf(amsaddr_, "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                     &remoteNetId_.b[0],
                     &remoteNetId_.b[1],
                     &remoteNetId_.b[2],
                     &remoteNetId_.b[3],
                     &remoteNetId_.b[4],
                     &remoteNetId_.b[5]);
  if(nvals!=6){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: AMS address invalid %s.\n", driverName, functionName,amsaddr_);
    return;
  }

  if (nvals != 6) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Invalid AMS address: %s\n", driverName, functionName,amsaddr_);
    return;
  }

  if(paramTableSize_<1){  //If paramTableSize_==1 then only stream device or motor record can use the driver through the "default access" param below.
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Param table size to small: %d\n", driverName, functionName,paramTableSize_);
    return;
  }

  //Add first param for other access (like motor record or stream device).
  int index;
  asynStatus status=createParam("Default access",asynParamNotDefined,&index);
  if(status!=asynSuccess){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: createParam for default access failed.\n", driverName, functionName);
    return;
  }
  adsParamInfo *paramInfo=new adsParamInfo();
  memset(paramInfo,0,sizeof(adsParamInfo));
  paramInfo->recordName=strdup("Any record");
  paramInfo->recordType=strdup("No type");
  paramInfo->scan=strdup("No scan");
  paramInfo->dtyp=strdup("No dtyp");
  paramInfo->inp=strdup("No inp");
  paramInfo->out=strdup("No out");
  paramInfo->drvInfo=strdup("No drvinfo");
  paramInfo->asynType=asynParamNotDefined;
  paramInfo->paramIndex=index;  //also used as hUser for ads callback
  paramInfo->plcAdrStr=strdup("No adr str");
  pAdsParamArray_[0]=paramInfo;
  adsParamArrayCount_++;

  if(status!=asynSuccess){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: createParam for default access failed.\n", driverName, functionName);
    return;
  }

  //* Create the thread that computes the waveforms in the background */
  status = (asynStatus)(epicsThreadCreate("adsAsynPortDriverCyclicThread",
                                                     epicsThreadPriorityMedium,
                                                     epicsThreadGetStackSize(epicsThreadStackMedium),
                                                     (EPICSTHREADFUNC)::cyclicThread,this) == NULL);

  if(status){
    printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
    return;
  }

  for (int i = 0; i < MAXBULK; i++)
      bulk[i].cnt = 0;            // Entry is currently unused!!
  bulkTScnt = 0;
  if (defaultSampleTimeMS_ < 1000) {
      printf("Default Sample Time of %d ms is too small, defaulting to 1Hz.\n",
             defaultSampleTimeMS_);
      bulk_delay_us = 1000000;        // 1 Hz
  } else {
      printf("Default bulk read time: %d ms\n", defaultSampleTimeMS_);
      bulk_delay_us = defaultSampleTimeMS_ * 1000;
  }
  bulkdatasize = 4 * 1024 * 1024; // This is excessive!
  bulkdata = (uint8_t *) malloc(bulkdatasize);
  bulkOK = 0;
  bulk_elapsed_us = 0;

  //* Create the thread that does the bulk reads */
  status = (asynStatus)(epicsThreadCreate("adsAsynPortDriverBulkReadThread",
                                          epicsThreadPriorityMedium,
                                          epicsThreadGetStackSize(epicsThreadStackMedium),
                                          (EPICSTHREADFUNC)::bulkReadThread,this) == NULL);

  if(status){
    printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
    return;
  }
  //try to connect, and hang until we succeed!
  for (;;) {
      if (connect(pasynUserSelf) != asynSuccess) {
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: connect failed for port %s.\n",
                    driverName, functionName, portName);
          epicsThreadSleep(1.0);
          continue;
      }
      long error = 0;
      uint16_t adsState = 0;
      if (adsReadStateLock(amsport,&adsState,true,&error) != asynSuccess) {
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: adsReadStateLock failed for port '%s' error=0x%lx\n",
                    driverName, functionName, portName, error);
          disconnect(pasynUserSelf);
          epicsThreadSleep(10.0);
          continue;
      }
      if (adsState == ADSSTATE_RUN) {
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: connection established for port '%s'\n",
                    driverName, functionName, portName);
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
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  free(ipaddr_);
  free(amsaddr_);

  for(int i=0;i<adsParamArrayCount_;i++){
    if(!pAdsParamArray_[i]){
      continue;
    }
    adsDelDataCallback(pAdsParamArray_[i],true);  //Block error messages
    adsReleaseSymbolicHandle(pAdsParamArray_[i],true);    //Block error messages
    free(pAdsParamArray_[i]->recordName);
    free(pAdsParamArray_[i]->recordType);
    free(pAdsParamArray_[i]->scan);
    free(pAdsParamArray_[i]->dtyp);
    free(pAdsParamArray_[i]->inp);
    free(pAdsParamArray_[i]->out);
    free(pAdsParamArray_[i]->drvInfo);
    free(pAdsParamArray_[i]->plcAdrStr);
    if(pAdsParamArray_[i]->plcDataIsArray){
      free(pAdsParamArray_[i]->arrayDataBuffer);
    }
    delete pAdsParamArray_[i];
  }
  delete pAdsParamArray_;

  for(amsPortInfo *port : amsPortList_){
    delete port;
  }
}

/** Cyclic thread for supervision of connection.
 * \return void
 * Check ads state of all connected ams ports and reconnects if needed.
 * At reconnect all symbolic handles and callbacks will be reregistered.
 */
void adsAsynPortDriver::cyclicThread()
{
  const char* functionName = "cyclicThread";
  double sampleTime=0.5;
  while (1){
    asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Sample time [s]= %lf.\n",driverName,functionName,sampleTime);

    epicsThreadSleep(sampleTime);
    if(!allowCallbackEpicsState){
      continue; //Epics not started
    }

    uint16_t adsState=0;
    //Check state of all used ams ports
    bool oneAmsConnectionOK=false;
    if(connectedAds_){
      for(amsPortInfo *port : amsPortList_){
        long error=0;
        asynStatus stat=adsReadStateLock(port->amsPort,&adsState,true,&error);
        bool portConnected=(stat==asynSuccess && adsState == ADSSTATE_RUN);
        port->adsStateOld=port->adsState;
        if(stat==asynSuccess){
            port->adsState=(ADSSTATE)adsState;
        }
        else{
            port->adsState=ADSSTATE_INVALID;
        }

        port->connectedOld=port->connected;
        port->connected=portConnected;
        port->paramsOK=portConnected;

        oneAmsConnectionOK=oneAmsConnectionOK || portConnected;

        if(port->connected && port->refreshNeeded){
          refreshParamsLock(port->amsPort);
        }
        if(port->connectedOld && !port->connected){
          invalidateParamsLock(port->amsPort);
          port->refreshNeeded=true;
          setAlarmPortLock(port->amsPort,COMM_ALARM,INVALID_ALARM);
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: connection failed for port %s.\n",
                    driverName, functionName, portName);
          exit(-1);
        }
        if(!port->connectedOld && port->connected){
          adsReadVersion(port);
        }
      }
    }

    //Printout state status
    for(amsPortInfo *port : amsPortList_){
      if(port->connectedOld!=port->connected){
        asynPrint(pasynUserSelf, ASYN_TRACE_INFO,"%s:%s: Device \"%s\" %s (Ams-port %u, Ams router version %u.%u.%u).\n",driverName,functionName,port->devName,port->connected ? "connected" : "disconnected",port->amsPort,port->version.version,port->version.revision,port->version.build);
      }
      if(port->adsStateOld!=port->adsState){
        //If Ams-router is a asyn paramter then update
        if(port->paramInfo){
          if(port->paramInfo->dataSource==ADS_DATASOURCE_AMS_STATE){
            void * pData=(void *)&port->adsState;
            adsUpdateParameterLock(port->paramInfo,pData,2);
          }
        }
        asynPrint(pasynUserSelf, ASYN_TRACE_INFO,"%s:%s: Ams-port, %u, state change: \"%s\" -> \"%s\".\n",driverName,functionName,port->amsPort,adsStateToString(port->adsStateOld),adsStateToString(port->adsState));
      }
    }


    if(!oneAmsConnectionOK){
      notConnectedCounter_++;
      if (notConnectedCounter_ < 100 || notConnectedCounter_ % 100 == 0) {
        asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Not connected counter: %d.\n",driverName,functionName,notConnectedCounter_);
      }
    }
    if(oneAmsConnectionOK){
      notConnectedCounter_=0;
    }

    if(!oneAmsConnectionOK && autoConnect_){
      if(oneAmsConnectionOKold_){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: No connection! Try to reconnect...\n",driverName,functionName);
      }
      connectedAds_=0;
      if((notConnectedCounter_&2)==2){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "cyclicThread: forcing disconnect.\n");
        disconnectLock(pasynUserSelf);
      }
    }
    oneAmsConnectionOKold_=oneAmsConnectionOK;
  }
}

/* TBD - Poll at different rates depending on pollClass! */
void adsAsynPortDriver::bulkReadThread()
{
    const char* functionName = "bulkReadThread";
    struct timeval start, now;
    uint32_t bytesRead;
    long status;
    uint32_t cnt, readSize;
    asynUser *asynTraceUser=getTraceAsynUser();

    gettimeofday(&now, NULL);
    while (1) {
        start = now;
        adsLock();
        for (int i = 0; bulk[i].cnt; i++) {
            if (!bulkOK || !bulk[i].cnt) {
                break;
            }
#ifdef MCB_DEBUG
            static int first = 1;
            if (first) {
                printf("Starting to poll!\n");
                first = 0;
            }
#endif
            bytesRead = 0;
            cnt = bulk[i].cnt;
            readSize = bulk[i].readSize;
            AmsAddr amsServer={remoteNetId_,bulk[i].amsPort};
            status = AdsSyncReadWriteReqEx2(adsPort_, &amsServer,
                                            ADSIGRP_SUMUP_READ, cnt,
                                            readSize, bulkdata,
                                            sizeof(bulk[i].sum[0]) * cnt, &bulk[i].sum,
                                            &bytesRead);
            if (status) {
                printf("Sum read %d failed: status %ld\n", i, status);
                continue;
            }

            uint32_t *stat = (uint32_t *)bulkdata;
            uint8_t  *srd  = bulkdata + cnt * sizeof(uint32_t);
            uint64_t nTimeStamp = 0;
            /* The first *two* bulk parameters might be the timestamp! */
            if (!stat[0] && !stat[1] && bulk[i].sum[0].iGroup == ADSIGRP_SYM_VALBYHND) {
                nTimeStamp = ((uint32_t *)srd)[0];
                nTimeStamp = (nTimeStamp << 32) | ((uint32_t *)srd)[1];
            } else {
                /*
                 * Sigh.  now has the time since 1970-01-01 00:00:00 UTC, but
                 * we want 100ns increments since 1601-01-01!! So we grab the constant
                 * from adsAsynPortDriverUtils.cpp and convert.
                 */
#define SEC_TO_UNIX_EPOCH 11644473600LL
                nTimeStamp = now.tv_sec + SEC_TO_UNIX_EPOCH;
                nTimeStamp = (nTimeStamp * 1000000 + now.tv_usec) * 10;
            }
            if (!stat[0])
                srd += sizeof(uint32_t);
            if (!stat[1])
                srd += sizeof(uint32_t);
            stat += 2;
            for (uint32_t j = 2; j < cnt; j++) {
                adsParamInfo *paramInfo=getAdsParamInfo(bulk[i].paramID[j]);
                if (!paramInfo){
                    asynPrint(asynTraceUser, ASYN_TRACE_ERROR,
                              "%s:%s: getAdsParamInfo() for hUser %u failed\n",
                              driverName, functionName, bulk[i].paramID[j]);
                    continue;
                }
                if (*stat++) {
                    asynPrint(asynTraceUser, ASYN_TRACE_ERROR,
                              "%s:%s: bulk read for %s (%d) failed\n",
                              driverName, functionName, paramInfo->drvInfo, j);
                    continue;
                }
                paramInfo->plcTimeStampRaw=nTimeStamp;
                paramInfo->lastCallbackSize=paramInfo->plcSize;
                adsUpdateParameter(paramInfo, srd);
                srd += paramInfo->lastCallbackSize;
            }
        }
        adsUnlock();
        gettimeofday(&now, NULL);
        bulk_elapsed_us = (now.tv_sec - start.tv_sec) * 1000000 +
                          (now.tv_usec - start.tv_usec);
#ifdef MCB_DEBUG
        printf("ELAPSED: %g\n", bulk_elapsed_us / 1000000.0);
#endif
        if (bulk_elapsed_us < bulk_delay_us) {
            usleep(bulk_delay_us - bulk_elapsed_us);
            gettimeofday(&now, NULL);
        }
    }
}

/** Report of configured parameters.
 * \param[in] fp Output file.
 * \param[in] details Details of printout. A higher number results in more
 *            details.
 * \return void
 * Check ads state of all connected ams ports and reconnects if needed.
 */
void adsAsynPortDriver::report(FILE *fp, int details)
{
  const char* functionName = "report";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  if(!fp){
    fprintf(fp,"%s:%s: ERROR: File NULL.\n", driverName, functionName);
    return;
  }

  if (details >= 1) {
    fprintf(fp, "General information:\n");
    fprintf(fp, "  Port:                        %s\n",portName);
    fprintf(fp, "  Ip-address:                  %s\n",ipaddr_);
    fprintf(fp, "  Ams-address:                 %s\n",amsaddr_);
    fprintf(fp, "  Default Ams-port :           %d\n",amsportDefault_);
    fprintf(fp, "  Auto-connect:                %s\n",autoConnect_ ? "true" : "false");
    fprintf(fp, "  Priority:                    %d\n",priority_);
    fprintf(fp, "  Param. table size:           %d\n",paramTableSize_);
    fprintf(fp, "  Param. count:                %d\n",adsParamArrayCount_);
    fprintf(fp, "  ADS command timeout [ms]:    %d\n",adsTimeoutMS_);
    fprintf(fp, "  Default sample time [ms]     %d\n",defaultSampleTimeMS_);
    fprintf(fp, "  Default max delay time [ms]: %d\n",defaultMaxDelayTimeMS_);
    fprintf(fp, "  Default time source:         %s\n",(defaultTimeSource_==ADS_TIME_BASE_PLC) ? ADS_OPTION_TIMEBASE_PLC : ADS_OPTION_TIMEBASE_EPICS);
    fprintf(fp, "  NOTE: Several records can be linked to the same parameter.\n");
    fprintf(fp,"\n");
  }
  if(details>=2){
    //print all parameters
    fprintf(fp,"Parameter details:\n");
    for(int i=0; i<adsParamArrayCount_;i++){
      if(!pAdsParamArray_[i]){
        fprintf(fp,"%s:%s: ERROR: Parameter array null at index %d\n", driverName, functionName,i);
        return;
      }
      adsParamInfo *paramInfo=pAdsParamArray_[i];
      fprintf(fp,"  Parameter %d:\n",i);
      if(i==0){
        fprintf(fp,"    Parameter 0 (pasynUser->reason==0) is reserved for Asyn octet interface (Motor Record and Stream Device access).\n");
        fprintf(fp,"\n");
        continue;
      }
      fprintf(fp,"    Param name:                %s\n",paramInfo->drvInfo);
      fprintf(fp,"    Param index:               %d\n",paramInfo->paramIndex);
      fprintf(fp,"    Param type:                %s (%d)\n",asynTypeToString((long)paramInfo->asynType),paramInfo->asynType);
      fprintf(fp,"    Param sample time [ms]:    %lf\n",paramInfo->sampleTimeMS);
      fprintf(fp,"    Param max delay time [ms]: %lf\n",paramInfo->maxDelayTimeMS);
      fprintf(fp,"    Param isIOIntr:            %s\n",paramInfo->isIOIntr ? "true" : "false");
      fprintf(fp,"    Param asyn addr:           %d\n",paramInfo->asynAddr);
      fprintf(fp,"    Param time source:         %s\n",(paramInfo->timeBase==ADS_TIME_BASE_PLC) ? ADS_OPTION_TIMEBASE_PLC : ADS_OPTION_TIMEBASE_EPICS);
      fprintf(fp,"    Param plc time:            %us:%uns\n",paramInfo->plcTimeStamp.secPastEpoch,paramInfo->plcTimeStamp.nsec);
      fprintf(fp,"    Param epics time:          %us:%uns\n",paramInfo->epicsTimestamp.secPastEpoch,paramInfo->epicsTimestamp.nsec);
      fprintf(fp,"    Param array buffer alloc:  %s\n",paramInfo->arrayDataBuffer ? "true" : "false");
      fprintf(fp,"    Param array buffer size:   %lu\n",paramInfo->arrayDataBufferSize);
      fprintf(fp,"    Param alarm:               %d\n",paramInfo->alarmStatus);
      fprintf(fp,"    Param severity:            %d\n",paramInfo->alarmSeverity);
      fprintf(fp,"    Param data source:         %s\n",paramInfo->dataSource==ADS_DATASOURCE_PLC ? "PLC" : "DRIVER");
      fprintf(fp,"    Plc ams port:              %d\n",paramInfo->amsPort);
      fprintf(fp,"    Plc adr str:               %s\n",paramInfo->plcAdrStr);
      fprintf(fp,"    Plc adr str is ADR cmd:    %s\n",paramInfo->isAdrCommand ? "true" : "false");
      fprintf(fp,"    Plc abs adr valid:         %s\n",paramInfo->plcAbsAdrValid ? "true" : "false");
      fprintf(fp,"    Plc abs adr group:         16#%x\n",paramInfo->plcAbsAdrGroup);
      fprintf(fp,"    Plc abs adr offset:        16#%x\n",paramInfo->plcAbsAdrOffset);
      fprintf(fp,"    Plc data type:             %s\n",adsTypeToString(paramInfo->plcDataType));
      fprintf(fp,"    Plc data type size:        %zu\n",adsTypeSize(paramInfo->plcDataType));
      fprintf(fp,"    Plc data size:             %u\n",paramInfo->plcSize);
      fprintf(fp,"    Plc data is array:         %s\n",paramInfo->plcDataIsArray ? "true" : "false");
      fprintf(fp,"    Plc data type warning:     %s\n",paramInfo->plcDataTypeWarn ? "true" : "false");
      fprintf(fp,"    Ads hCallbackNotify:       %u\n",paramInfo->hCallbackNotify);
      fprintf(fp,"    Ads CallbackNotify valid:  %s\n",paramInfo->bCallbackNotifyValid ? "true" : "false");
      fprintf(fp,"    Ads hSymbHndle:            %u\n",paramInfo->hSymbolicHandle);
      fprintf(fp,"    Ads hSymbHndleValid:       %s\n",paramInfo->bSymbolicHandleValid ? "true" : "false");
      fprintf(fp,"    Record name:               %s\n",paramInfo->recordName);
      fprintf(fp,"    Record type:               %s\n",paramInfo->recordType);
      fprintf(fp,"    Record dtyp:               %s\n",paramInfo->dtyp);
      fprintf(fp,"\n");
    }
  }
}

/** Disconencts the PLC with asyn lock.
 * \param[in] pasynUser Asyn user.
 * \return asynSuccess or asynError.
 * Thread safe.
 */
asynStatus adsAsynPortDriver::disconnectLock(asynUser *pasynUser)
{
  lock();
  asynStatus stat=disconnect(pasynUser);
  unlock();
  return stat;
}

/** Disconencts the PLC.
 * \param[in] pasynUser Asyn user.
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::disconnect(asynUser *pasynUser)
{
  const char* functionName = "disconnect";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  asynStatus disconnectStatus=adsDisconnect();
  if (disconnectStatus){
    return asynError;
  }

  return asynPortDriver::disconnect(pasynUser);
}

/** Refreshes the parameters that need refresh after a reconnect or a
 * connection failure to a ams port.
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::refreshParams()
{
  return refreshParams(0);
}

/** Refreshes all parameters for a specific amsport (with asyn lock()).
 * \param[in] amsPort ams port.
 * \return asynSuccess or asynError.
 * Thread safe.
 */

asynStatus adsAsynPortDriver::refreshParamsLock(uint16_t amsPort)
{
  lock();
  asynStatus stat=refreshParams(amsPort);
  unlock();
  return stat;
}

/** Refreshes all parameters for a specific amsport.
 * \param[in] amsPort ams port.
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::refreshParams(uint16_t amsPort)
{
  const char* functionName = "refreshParams";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  if(connectedAds_){
    if(adsParamArrayCount_>1){
      //Renew data notification callbacks
      for(int i=1; i<adsParamArrayCount_;i++){  //Skip first param since used for motorrecord or stream device
        if(!pAdsParamArray_[i]){
          continue;
        }
        adsParamInfo *paramInfo=pAdsParamArray_[i];
        if((amsPort==0 || paramInfo->amsPort==amsPort) && paramInfo->refreshNeeded){
          updateParamInfoWithPLCInfo(paramInfo);
        }
      }
    }

    //Renew symbols changed notification callbacks
    for(amsPortInfo *port : amsPortList_){
      if(port->amsPort==amsPort && port->refreshNeeded){
        if(port->bCallbackNotifyValid){
          adsDelSymbolsChangedCallback(port);
        }
        adsAddSymbolsChangedCallback(port);
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
  asynStatus stat=invalidateParams(amsPort);
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
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  bulkOK = 0;
  if(adsParamArrayCount_>1){
    for(int i=1; i<adsParamArrayCount_;i++){  //Skip first param since used for motorrecord or stream device
      if(!pAdsParamArray_[i]){
        continue;
      }
      adsParamInfo *paramInfo=pAdsParamArray_[i];
      if(amsPort==0 || paramInfo->amsPort==amsPort){
        paramInfo->refreshNeeded=true;
      }
    }
  }
  for (int i = 0; i < bulkTScnt; i++) {
      if (amsPort == 0 || bulkTS[i].amsPort == amsPort)
          bulkTS[i].refreshNeeded = 1;
  }
  for (int i = 0; i < MAXBULK; i++) {
      if (bulk[i].cnt == 0)                        // Quit if unused!
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
asynStatus adsAsynPortDriver::connectLock(asynUser *pasynUser)
{
  lock();
  asynStatus stat=connect(pasynUser);
  unlock();
  return stat;
}

/** Connects to a PLC.
 * \param[in] pasynUser Asyn user
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::connect(asynUser *pasynUser)
{
  const char* functionName = "connect";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: %s\n", driverName, functionName, epicsThreadGetNameSelf());

  bool err=false;
  asynStatus stat = adsConnect();

  if (stat!= asynSuccess){
    return asynError;
  }

  if(asynPortDriver::connect(pasynUser)!=asynSuccess){
    return asynError;
  }

  connectedAds_=1;
  return err ? asynError : asynSuccess;
}

/** Validates drvInfo string
 * \param[in] drvInfo String containing information about the parameter.
 * \return asynSuccess or asynError.
 * The drvInfo string is what is after the asyn() in the "INP" or "OUT"
 * field of an record.
 */
asynStatus adsAsynPortDriver::validateDrvInfo(const char *drvInfo)
{
  const char* functionName = "validateDrvInfo";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName,drvInfo);

  if(strlen(drvInfo)==0){
    asynPrint(pasynUserSelf,ASYN_TRACE_ERROR,"Invalid drvInfo string: Length 0 (%s).\n",drvInfo);
    return asynError;
  }

  //Check '?' mark last or '=' last
  const char* read=strrchr(drvInfo,'?');
  if(read){
    if(strlen(read)==1){
      return asynSuccess;
    }
  }

  const char* write=strrchr(drvInfo,'=');
  if(write){
    if(strlen(write)==1){
      return asynSuccess;
    }
  }

  asynPrint(pasynUserSelf,ASYN_TRACE_ERROR,"Invalid drvInfo string (%s).\n",drvInfo);
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
asynStatus adsAsynPortDriver::drvUserCreate(asynUser *pasynUser,const char *drvInfo,const char **pptypeName,size_t *psize)
{
  const char* functionName = "drvUserCreate";
  static int vcnt = 0;

  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName,drvInfo);

  if(validateDrvInfo(drvInfo)!=asynSuccess){
    return asynError;
  }

  int index=0;
  asynStatus status=findParam(drvInfo,&index);
  if(status==asynSuccess){
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: Parameter index found at: %d for %s. \n", driverName, functionName,index,drvInfo);
    if(!pAdsParamArray_[index]){
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s:pAdsParamArray_[%d]==NULL (drvInfo=%s).", driverName, functionName,index,drvInfo);
      return asynError;
    }

    if(pAdsParamArray_[index]->dataSource==ADS_DATASOURCE_AMS_STATE){ //Local variable (not in PLC) like AMS port state.

      return asynPortDriver::drvUserCreate(pasynUser,drvInfo,pptypeName,psize);
    }

    if(!connectedAds_){
      //try to connect without error handling
      connect(pasynUser);
    }

    if(connectedAds_){
      status =adsReadParam(pAdsParamArray_[index]);
      if(status!=asynSuccess){
        return asynError;
      }
    }
    return asynPortDriver::drvUserCreate(pasynUser,drvInfo,pptypeName,psize);
  }

  if (!vcnt++)
      printf("Linking EPICS PVs to PLC variables...\n");
  if (vcnt % 1000 == 0)
      printf("%d...\n", vcnt);

  //Ensure space left in param table
  if(adsParamArrayCount_>=(paramTableSize_-1)){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Parameter table full. Parameter with drvInfo %s will be discarded.", driverName, functionName,drvInfo);
    return asynError;
  }

  // Collect data from drvInfo string and recordpasynUser->reason=index;
  adsParamInfo *paramInfo=new adsParamInfo();
  memset(paramInfo,0,sizeof(adsParamInfo));
  paramInfo->sampleTimeMS=defaultSampleTimeMS_;
  paramInfo->maxDelayTimeMS=defaultMaxDelayTimeMS_;
  paramInfo->refreshNeeded=1;
  paramInfo->bulkIndex = -1;
  paramInfo->bulkOffset = -1;

  status=getRecordInfoFromDrvInfo(drvInfo, paramInfo);
  if(status!=asynSuccess){
    return asynError;
  }

  status=createParam(drvInfo,paramInfo->asynType,&index);
  if(status!=asynSuccess){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: createParam() failed.",driverName, functionName);
    return asynError;
  }
  asynPrint(pasynUser, ASYN_TRACE_INFO, "%s:%s: Parameter created: \"%s\" (index %d).\n", driverName, functionName,drvInfo,index);

  //Set default value for basic types...
  switch(paramInfo->asynType){
    case asynParamInt32:
      setIntegerParam(index,0);
      break;
    case asynParamFloat64:
      setDoubleParam(index,0);
      break;
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
    case asynParamInt64:
      setInteger64Param(index,0);
      break;
#endif
    default:
      break;
  }

  paramInfo->paramIndex=index;

  int addr=0;
  status = getAddress(pasynUser, &addr);
  if (status != asynSuccess){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: getAddress() failed.",driverName, functionName);
    return(status);
  }

  paramInfo->asynAddr=addr;

  status=parsePlcInfofromDrvInfo(drvInfo,paramInfo);
  if(status!=asynSuccess){
    return asynError;
  }
  pasynUser->timeout=(paramInfo->maxDelayTimeMS*2)/1000;

  pAdsParamArray_[adsParamArrayCount_]=paramInfo;
  adsParamArrayCount_++;

  if(!connectedAds_){
    //try to connect without error handling
    connect(pasynUser);
  }

  if(connectedAds_ && !(paramInfo->dataSource==ADS_DATASOURCE_AMS_STATE)){  //Do not read info from PLC if local variable (like ams-port state)
    status=updateParamInfoWithPLCInfo(paramInfo);
    if(status!=asynSuccess){
      return asynError;
    }
  }
  return asynPortDriver::drvUserCreate(pasynUser,drvInfo,pptypeName,psize); //Assigns pasynUser->reason;
}

/** Update parameter with info from PLC (variable size, type and abs addr).
 * \param[in/out] paramInfo Parameter information structure.
 * \return asynSuccess or asynError.
 * If the PLC variable is an array then a buffer is allocated in the paramInfo to
 * hold the information.
 */
asynStatus adsAsynPortDriver::updateParamInfoWithPLCInfo(adsParamInfo *paramInfo)
{
  const char* functionName = "updateParamInfoWithPLCInfo";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: : %s\n", driverName, functionName,paramInfo->drvInfo);

  asynStatus status;

  //Do not read information from PLC if "variable" in driver (like ams router state)
  if(paramInfo->dataSource!=ADS_DATASOURCE_PLC){
    paramInfo->refreshNeeded=false;
    return asynSuccess;
  }

  // Read symbolic information if needed (to get paramInfo->plcSize)
  if(!paramInfo->isAdrCommand){
    status=adsGetSymInfoByName(paramInfo);
    if(status!=asynSuccess){
      return asynError;
    }
  }

  // Check if array
  bool isArray=false;
  switch (paramInfo->plcDataType) {
    case ADST_VOID:
      isArray=false;
      break;
    case ADST_STRING:
      isArray=true;  // Special case
      break;
    case ADST_WSTRING:
      isArray=true;  // Special case?
      break;
    case ADST_BIGTYPE:
      isArray=false;
      break;
    case ADST_MAXTYPES:
      isArray=false;
      break;
    default:
      isArray=paramInfo->plcSize>adsTypeSize(paramInfo->plcDataType);
      break;
  }
  paramInfo->plcDataIsArray=isArray;

  // Allocate memory for array
  if(isArray){
    if(paramInfo->plcSize!=paramInfo->arrayDataBufferSize && paramInfo->arrayDataBuffer){ //new size of array
      free(paramInfo->arrayDataBuffer);
      paramInfo->arrayDataBuffer=NULL;
    }
    if(!paramInfo->arrayDataBuffer){
      paramInfo->arrayDataBuffer = calloc(paramInfo->plcSize,1);
      paramInfo->arrayDataBufferSize=paramInfo->plcSize;
      if(!paramInfo->arrayDataBuffer){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to allocate memory for array data for %s.\n.", driverName, functionName,paramInfo->drvInfo);
        unlock();
        return asynError;
      }
      memset(paramInfo->arrayDataBuffer,0,paramInfo->plcSize);
    }
  }

  if (!paramInfo->isAdrCommand) {
      adsReleaseSymbolicHandle(paramInfo,true); //try to delete
      status=adsGetSymHandleByName(paramInfo);
      if(status!=asynSuccess){
          return asynError;
      }
  }

  if(paramInfo->isIOIntr){
      /* If it's not a bulk read or if it's really big, just subscribe to it! */
      if (!paramInfo->isBulkRead || paramInfo->plcSize > 1024*1024) {
          adsDelDataCallback(paramInfo,true);   //try to delete
          status=adsAddDataCallback(paramInfo);
          if(status!=asynSuccess){
              return asynError;
          }
      } else { /* Otherwise, put it in a bulk read! */
          status=adsAddToBulkRead(paramInfo);
          if(status!=asynSuccess){
              return asynError;
          }
      }
  }

  // Make first read
  long errorCode=0;
  status = adsReadParam(paramInfo,&errorCode,0);
  if(status!=asynSuccess){
    // Try read again
    asynStatus stat=adsReadParam(paramInfo,&errorCode,0);
    if(stat!=asynSuccess){
      paramInfo->refreshNeeded=true;
      return asynError;
    }
  }

  paramInfo->refreshNeeded=false;
  return asynSuccess;
}

void adsAsynPortDriver::poll_info(char *name)
{
    int i;
    printf("Bulk read loop: desired period = %gs, last loop time = %gs\n", bulk_delay_us / 1000000.0, bulk_elapsed_us / 1000000.0);
    for (i = 0; bulk[i].cnt && i < MAXBULK; i++);
    printf("Bulk read count = %d\n", i);
    if (name[0] == 0)
        name = 0;
    for (i = 0; bulk[i].cnt && i < MAXBULK; i++) {
      printf("Bulk Read #%d:\n", i);
      if (!name) {
          printf("    0: MAIN.fbSystemTime.timeLoDW (G=0x%x, O=0x%x, S=%d)\n",
                 bulk[i].sum[0].iGroup, bulk[i].sum[0].iOffset, bulk[i].sum[0].iSize);
          printf("    1: MAIN.fbSystemTime.timeHiDW (G=0x%x, O=0x%x, S=%d)\n",
                 bulk[i].sum[1].iGroup, bulk[i].sum[1].iOffset, bulk[i].sum[1].iSize);
      }
      for (int j = 2; j < bulk[i].cnt; j++) {
        adsParamInfo *paramInfo=getAdsParamInfo(bulk[i].paramID[j]);
        if (!paramInfo)
          continue;
        if (!name || strstr(paramInfo->plcAdrStr, name))
            printf("  %3d: %s (G=0x%x, O=0x%x, S=%d, TS=%d.%09d)\n", j, paramInfo->plcAdrStr,
                   bulk[i].sum[j].iGroup, bulk[i].sum[j].iOffset, bulk[i].sum[j].iSize,
                   paramInfo->epicsTimestamp.secPastEpoch, paramInfo->epicsTimestamp.nsec);
      }
    }
}

/* TBD - Use paramInfo->pollClass to separate into different poll rates!! */
asynStatus adsAsynPortDriver::adsAddToBulkRead(adsParamInfo* paramInfo)
{
    adsLock(); // Prevent reads while we change this!
    if (paramInfo->bulkIndex < 0) { /* Not assigned yet, find one! */
        int i;
        for (i = 0; i < MAXBULK; i++) {
            /* Look for an unused entry or a non-full entry for this port. */
            if (bulk[i].cnt == 0 || (bulk[i].cnt != BULKSIZ &&
                                     bulk[i].amsPort == paramInfo->amsPort))
                break;
        }
        if (i == MAXBULK) {
            adsUnlock();
            return asynError; // No room at the inn.
        }
        if (!bulk[i].cnt) { // First variable in this bulk request!
            bulk[i].amsPort = paramInfo->amsPort;
            int j = adsFindBulkTimeStamp(paramInfo->amsPort);
            if (bulkTS[j].refreshNeeded) { /* No TS variables!! */
                bulk[i].sum[0].iGroup  = 0x4020; // %M
                bulk[i].sum[0].iOffset = 0;
                bulk[i].sum[0].iSize   = sizeof(uint32_t);
                bulk[i].sum[1].iGroup  = 0x4020; // %M
                bulk[i].sum[1].iOffset = 0;
                bulk[i].sum[1].iSize   = sizeof(uint32_t);
            } else {
                bulk[i].sum[0].iGroup  = ADSIGRP_SYM_VALBYHND;
                bulk[i].sum[0].iOffset = bulkTS[j].iHandleH;
                bulk[i].sum[0].iSize   = sizeof(uint32_t);
                bulk[i].sum[1].iGroup  = ADSIGRP_SYM_VALBYHND;
                bulk[i].sum[1].iOffset = bulkTS[j].iHandleL;
                bulk[i].sum[1].iSize   = sizeof(uint32_t);
            }
            bulk[i].cnt = 2;
            bulk[i].readSize = 4 * sizeof(uint32_t);
        }
        paramInfo->bulkIndex  = i;
        paramInfo->bulkOffset = bulk[i].cnt;
        bulk[i].paramID[bulk[i].cnt++] = paramInfo->paramIndex;
    }
    /* Update the information for a previously allocated element.  Possibly *just*
       allocated, but that's still previously! */
    uint32_t group, offset;
    if (paramInfo->isAdrCommand) {
        group = paramInfo->plcAbsAdrGroup;
        offset = paramInfo->plcAbsAdrOffset;
    } else {
        group = ADSIGRP_SYM_VALBYHND;
        offset = paramInfo->hSymbolicHandle;
    }
    bulk[paramInfo->bulkIndex].sum[paramInfo->bulkOffset].iGroup  = group;
    bulk[paramInfo->bulkIndex].sum[paramInfo->bulkOffset].iOffset = offset;
    bulk[paramInfo->bulkIndex].sum[paramInfo->bulkOffset].iSize   = paramInfo->plcSize;
    bulk[paramInfo->bulkIndex].readSize += paramInfo->plcSize + sizeof(uint32_t);
    adsUnlock();
    return asynSuccess;
}

// Assume locked!!
int adsAsynPortDriver::adsFindBulkTimeStamp(uint16_t amsPort)
{
    int i;
    for (i = 0; i < bulkTScnt; i++) {
        if (amsPort == bulkTS[i].amsPort)
            break;
    }
    if (i == bulkTScnt) {
        bulkTScnt++;
        bulkTS[i].amsPort = amsPort;
        bulkTS[i].refreshNeeded = 1;
    }
#define TSLO "MAIN.fbSystemTime.timeLoDW"
#define TSHI "MAIN.fbSystemTime.timeHiDW"
    if (bulkTS[i].refreshNeeded) {
        long statL, statH;
        AmsAddr amsServer = {remoteNetId_, amsPort};
        statH = AdsSyncReadWriteReqEx2(adsPort_,
                                       &amsServer,
                                       ADSIGRP_SYM_HNDBYNAME,
                                       0,
                                       sizeof(uint32_t), &bulkTS[i].iHandleH,
                                       strlen(TSHI), TSHI,
                                       nullptr);
        statL = AdsSyncReadWriteReqEx2(adsPort_,
                                       &amsServer,
                                       ADSIGRP_SYM_HNDBYNAME,
                                       0,
                                       sizeof(uint32_t), &bulkTS[i].iHandleL,
                                       strlen(TSLO), TSLO,
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
asynStatus adsAsynPortDriver::getRecordInfoFromDrvInfo(const char *drvInfo,adsParamInfo *paramInfo)
{
  const char* functionName = "getRecordInfoFromDrvInfo";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName,drvInfo);

  bool isInput=false;
  bool isOutput=false;
  paramInfo->amsPort=amsportDefault_;
  DBENTRY *pdbentry;
  pdbentry = dbAllocEntry(pdbbase);
  long status = dbFirstRecordType(pdbentry);
  bool recordFound=false;
  if(status) {
    dbFreeEntry(pdbentry);
    return asynError;
  }
  while(!status) {
    paramInfo->recordType=strdup(dbGetRecordTypeName(pdbentry));
    status = dbFirstRecord(pdbentry);
    while(!status) {
      paramInfo->recordName=strdup(dbGetRecordName(pdbentry));
      if(!dbIsAlias(pdbentry)){
        status=dbFindField(pdbentry,"INP");
        if(!status){
          paramInfo->inp=strdup(dbGetString(pdbentry));
          isInput=true;
          char port[ADS_MAX_FIELD_CHAR_LENGTH];
          int adr;
          int timeout;
          char currdrvInfo[ADS_MAX_FIELD_CHAR_LENGTH];
          int nvals=sscanf(paramInfo->inp,"@asyn(%[^,],%d,%d)%s",port,&adr,&timeout,currdrvInfo);
          if(nvals==4){
            // Ensure correct port and drvinfo
            if(strcmp(port,portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
              recordFound=true;  // Correct port and drvinfo!\n");
            }
          }
        }
        else{
          isInput=false;
        }
        status=dbFindField(pdbentry,"OUT");
        if(!status){
          paramInfo->out=strdup(dbGetString(pdbentry));
          isOutput=true;
          char port[ADS_MAX_FIELD_CHAR_LENGTH];
          int adr;
          int timeout;
          char currdrvInfo[ADS_MAX_FIELD_CHAR_LENGTH];
          int nvals=sscanf(paramInfo->out,"@asyn(%[^,],%d,%d)%s",port,&adr,&timeout,currdrvInfo);
          if(nvals==4){
            // Ensure correct port and drvinfo
            if(strcmp(port,portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
              recordFound=true;  // Correct port and drvinfo!\n");
            }
          }
        }
        else{
          isOutput=false;
        }

        if(recordFound){
          // Correct record found. Collect data from fields
          //DTYP
          status=dbFindField(pdbentry,"DTYP");
          if(!status){
            paramInfo->dtyp=strdup(dbGetString(pdbentry));
            paramInfo->asynType=dtypStringToAsynType(dbGetString(pdbentry));
          }
          else{
            paramInfo->dtyp=0;
            paramInfo->asynType=asynParamNotDefined;
          }

          //drvInput (not a field)
          paramInfo->drvInfo=strdup(drvInfo);
          dbFreeEntry(pdbentry);
          return asynSuccess;  // The correct record was found and the paramInfo structure is filled
        }
        else{
          //Not correct record. Do cleanup.
          if(isInput){
            free(paramInfo->inp);
            paramInfo->inp=0;
          }
          if(isOutput){
            free(paramInfo->out);
            paramInfo->out=0;
          }
          paramInfo->drvInfo=0;
          paramInfo->scan=0;
          paramInfo->dtyp=0;
          isInput=false;
          isOutput=false;
        }
      }
      status = dbNextRecord(pdbentry);
      free(paramInfo->recordName);
      paramInfo->recordName=0;
    }
    status = dbNextRecordType(pdbentry);
    free(paramInfo->recordType);
    paramInfo->recordType=0;
  }
  dbFreeEntry(pdbentry);
  return asynError;
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
asynStatus adsAsynPortDriver::parsePlcInfofromDrvInfo(const char* drvInfo,adsParamInfo *paramInfo)
{
  const char* functionName = "parsePlcInfofromDrvInfo";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName,drvInfo);

  //Check if input or output
  paramInfo->isIOIntr=false;
  const char* temp=strrchr(drvInfo,'?');
  if(temp){
    if(strlen(temp)==1){
      paramInfo->isIOIntr=true; //All inputs will be created I/O intr
    }
  }

  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: drvInfo %s is %s\n", driverName, functionName,drvInfo,paramInfo->isIOIntr ? "I/O Intr (end with ?)": " not I/O Intr (end with =)");

  //take part after last "/" if option or complete string..
  char buffer[ADS_MAX_FIELD_CHAR_LENGTH];
  //See if option (find last '/')
  const char *drvInfoEnd=strrchr(drvInfo,'/');
  if(drvInfoEnd){ // found '/'
    int nvals=sscanf(drvInfoEnd,"/%s",buffer);
    if(nvals==1){
      paramInfo->plcAdrStr=strdup(buffer);
      paramInfo->plcAdrStr[strlen(paramInfo->plcAdrStr)-1]=0; //Strip ? or = from end
    }
    else{
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse PLC address string from drvInfo (%s)\n", driverName, functionName,drvInfo);
      return asynError;
    }
  }
  else{  //No options
    paramInfo->plcAdrStr=strdup(drvInfo);  //Symbolic or .ADR.
    paramInfo->plcAdrStr[strlen(paramInfo->plcAdrStr)-1]=0; //Strip ? or = from end
  }

  //Check if .ADR. command
  const char *option=ADS_ADR_COMMAND_PREFIX;
  paramInfo->plcAbsAdrValid=false;
  paramInfo->isAdrCommand=false;
  const char *isThere=strstr(drvInfo,option);
  if(isThere){
    if(strlen(isThere)<(strlen(option)+strlen("16#%x,16#%x,%u,%u"))){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s command from drvInfo (%s). String to short.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
    paramInfo->isAdrCommand=true;

    int nvals = sscanf(isThere+strlen(option),"16#%x,16#%x,%u,%u",
             &paramInfo->plcAbsAdrGroup,
             &paramInfo->plcAbsAdrOffset,
             &paramInfo->plcSize,
             &paramInfo->plcDataType);

    if(nvals==4){
      paramInfo->plcAbsAdrValid=true;
    }
    else{
      paramInfo->plcAbsAdrValid=false;
      paramInfo->plcAbsAdrGroup=-1;
      paramInfo->plcAbsAdrOffset=-1;
      paramInfo->plcSize=-1;
      paramInfo->plcDataType=-1;
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s command from drvInfo (%s). Wrong format.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
  }

  //Check if ADS_OPTION_T_MAX_DLY_MS option
  option=ADS_OPTION_T_MAX_DLY_MS;
  isThere=strstr(drvInfo,option);
  if(isThere){
    if(strlen(isThere)<(strlen(option)+strlen("=0/"))){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }

    int nvals = sscanf(isThere+strlen(option),"=%lf/",&paramInfo->maxDelayTimeMS);

    if(nvals!=1){
      paramInfo->maxDelayTimeMS=defaultMaxDelayTimeMS_;
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
  }

  //Check if ADS_OPTION_T_SAMPLE_RATE_MS option
  option=ADS_OPTION_T_SAMPLE_RATE_MS;
  paramInfo->sampleTimeMS=defaultSampleTimeMS_;
  isThere=strstr(drvInfo,option);
  if(isThere){
    if(strlen(isThere)<(strlen(option)+strlen("=0/"))){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }

    int nvals = sscanf(isThere+strlen(option),"=%lf/",&paramInfo->sampleTimeMS);

    if(nvals!=1){
      paramInfo->sampleTimeMS=defaultSampleTimeMS_;
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
  }

  //Check if ADS_OPTION_POLLRATE option
  option=ADS_OPTION_POLLRATE;
  paramInfo->isBulkRead = false;
  paramInfo->pollClass = 1.0;
  isThere=strstr(drvInfo,option);
  if(isThere){
    if(strlen(isThere)<(strlen(option)+strlen("=0/"))){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
    paramInfo->isBulkRead = true;

    int nvals = sscanf(isThere+strlen(option),"=%lf/",&paramInfo->pollClass);

    if(nvals!=1){
      paramInfo->pollClass = 1.0;
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
  }

  //Check if ADS_OPTION_TIMEBASE option
  option=ADS_OPTION_TIMEBASE;
  paramInfo->timeBase=defaultTimeSource_;
  isThere=strstr(drvInfo,option);
  if(isThere){
    int minLen=strlen(ADS_OPTION_TIMEBASE_PLC);
    int epicsLen=strlen(ADS_OPTION_TIMEBASE_EPICS);
    if(epicsLen<minLen){
      minLen=epicsLen;
    }
    if(strlen(isThere)<(strlen(option)+strlen("=/")+minLen)){ //Allowed "PLC" or "EPICS"
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }

    int nvals = sscanf(isThere+strlen(option),"=%[^/]/",buffer);
    if(nvals!=1){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }

    if(strcmp(ADS_OPTION_TIMEBASE_PLC,buffer)==0){
      paramInfo->timeBase=ADS_TIME_BASE_PLC;
    }

    if(strcmp(ADS_OPTION_TIMEBASE_EPICS,buffer)==0){
      paramInfo->timeBase=ADS_TIME_BASE_EPICS;
    }
  }

  //Check if ADS_OPTION_ADSPORT option
  option=ADS_OPTION_ADSPORT;
  paramInfo->amsPort=amsportDefault_;
  isThere=strstr(drvInfo,option);
  if(isThere){
    if(strlen(isThere)<(strlen(option)+strlen("=0/"))){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
    int nvals;
    int val;
    nvals = sscanf(isThere+strlen(option),"=%d/",&val);
    if(nvals==1){
      paramInfo->amsPort=(uint16_t)val;
    }
    else{
      paramInfo->amsPort=amsportDefault_;
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
  }

  //Check if ADS_AMS_STATE_COMMAND option Local variable/parameter (not in PLC)
  option=ADS_AMS_STATE_COMMAND;
  paramInfo->dataSource=ADS_DATASOURCE_PLC;
  isThere=strstr(drvInfo,option);
  if(isThere){
    addNewAmsPortToList(paramInfo->amsPort);//Only add if not already there
    amsPortInfo* port=getAmsPortObject(paramInfo->amsPort);
    if(!port){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
    paramInfo->dataSource=ADS_DATASOURCE_AMS_STATE;  //This information is accessible in driver (not PLC)
    paramInfo->plcDataType=ADST_UINT16;
    paramInfo->plcSize=2;
    paramInfo->plcDataIsArray=false;
    paramInfo->timeBase=ADS_TIME_BASE_EPICS;
    port->paramInfo=paramInfo;
  }

  return addNewAmsPortToList(paramInfo->amsPort);//Only add if not already there
}

/** Get ams port information object from ams-port list.
 * \param[in] amsPort ams-port
 *
 * \return amsPortInfo object.
 */
amsPortInfo* adsAsynPortDriver::getAmsPortObject(uint16_t amsPort)
{
  const char* functionName = "getAmsPortObject";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: amsPort:%u\n", driverName, functionName,amsPort);

  for(amsPortInfo *port : amsPortList_){
    if(port->amsPort==amsPort){
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
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: amsPort:%u\n", driverName, functionName,amsPort);

  //See if new amsPort, then update list
  bool newAmsPort=true;
  for(amsPortInfo *port : amsPortList_){
    if(port->amsPort==amsPort){
      newAmsPort=false;
    }
  }

  if(!newAmsPort){
    return asynSuccess;
  }

  try{
    amsPortInfo *newPort=new amsPortInfo();
    memset(newPort,0,sizeof(amsPortInfo));
    newPort->amsPort=amsPort;
    newPort->adsState=(ADSSTATE)(ADSSTATE_MAXSTATES+1); //Set unknown state..
    newPort->adsStateOld=newPort->adsState;
    newPort->refreshNeeded=false;     // This is actually all initialized!!
    amsPortList_.push_back(newPort);
  }
  catch(std::exception &e)
  {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to add new amsPort to list. Exception: %s.\n", driverName, functionName,e.what());
    return asynError;
  }
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Added new amsPort to amsPortList: %d .\n", driverName, functionName,amsPort);

  return asynSuccess;
}

/** Checks if callback is allowed for a certain parameter.
 * \param[in] paramInfo Parameter info structure.
 *
 * \return true if parameter information and ams-port connection is OK
 *  otherwise false.
 */
bool adsAsynPortDriver::isCallbackAllowed(adsParamInfo *paramInfo)
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
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: amsPort:%u\n", driverName, functionName,amsPort);

  for(amsPortInfo *port : amsPortList_){
    if(port->amsPort==amsPort)
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
asynStatus adsAsynPortDriver::readOctet(asynUser *pasynUser, char *value, size_t maxChars,size_t *nActual, int *eomReason)
{
  const char* functionName = "readOctet";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  size_t thisRead = 0;
  int reason = 0;
  asynStatus status = asynSuccess;

  *value = '\0';
  lock();
  int error=octetCMDreadIt(value, maxChars);
  if (error) {
    status = asynError;
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, CMDreadIt failed (0x%x).\n",
              driverName, functionName, error);
    unlock();
    return asynError;
  }

  thisRead = strlen(value);
  *nActual = thisRead;

  /* May be not enough space ? */
  if (thisRead > maxChars-1) {
    reason |= ASYN_EOM_CNT;
  }
  else{
    reason |= ASYN_EOM_EOS;
  }

  if (thisRead == 0 && pasynUser->timeout == 0){
    status = asynTimeout;
  }

  if (eomReason){
    *eomReason = reason;
  }

  *nActual = thisRead;
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
             "%s thisRead=%lu data=\"%s\"\n",
             portName,
             (unsigned long)thisRead, value);
  unlock();
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:%s\n", driverName, functionName,value);

  return status;
}

/** Implements part of the asyn-octet ASCII command parser.
 * (see readOctet() and writeOctet for more info).
 * \param[in] outbuf Buffer for read data.
 * \param[in] outlen Size of value buffer.
 *
 * \return 0 for success or error code.
 */
int adsAsynPortDriver::octetCMDreadIt(char *outbuf, size_t outlen)
{
  const char* functionName = "octetCMDreadIt";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Buffer: %s, size: %d\n", driverName, functionName,outbuf,(int)outlen);

  int ret;
  if (!outbuf || !outlen){
    return -1;
  }
  ret = snprintf(outbuf, outlen+1, "%s",octetAsciiBuffer_.buffer);

  if (ret < 0){
    octetClearBuffer(&octetAsciiBuffer_);
    return ret;
  }

  if(ret>=(int)outlen+1){
    ret=outlen;
  }
  octetRemoveFromBuffer(&octetAsciiBuffer_,ret);

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
asynStatus adsAsynPortDriver::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars,size_t *nActual)
{
  const char* functionName = "writeOctet";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: %s\n", driverName, functionName,value);

  size_t thisWrite = 0;
  asynStatus status = asynError;

  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s write.\n",portName);
  asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, value, maxChars,
              "%s write %lu\n",
              portName,
              (unsigned long)maxChars);
  *nActual = 0;

  if (maxChars == 0){
    return asynSuccess;
  }
  //lock();
  int errorCode=octetCMDwriteIt(value, maxChars);
  if (errorCode){
    /*Return asyn error if communication is down (all client errors) otherwise asynSuccess
     * but error message in buffer*/
    if (errorCode>=ADSERR_CLIENT_ERROR){
      return asynError;
    }
  }
  status = asynSuccess;
  thisWrite = maxChars;
  *nActual = thisWrite;

  //unlock();
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
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
int adsAsynPortDriver::octetCMDwriteIt(const char *inbuf, size_t inlen)
{
  const char* functionName = "octetCMDwriteIt";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Write command: %s, length: %d\n", driverName, functionName,inbuf,(int) inlen);

  int had_cr = 0;
  int had_lf = 0;
  int errorCode;
  char *new_buf = (char *)inbuf;
  if (!inbuf || !inlen) return -1;

  new_buf = (char *)malloc(inlen + 1);
  memcpy(new_buf, inbuf, inlen);
  new_buf[inlen] = 0;

  if (inlen > 1 && new_buf[inlen-1] == '\n') {
    had_lf = 1;
    new_buf[inlen-1] = '\0';
    inlen--;
    if (inlen > 1 && new_buf[inlen-1] == '\r') {
      had_cr = 1;
      new_buf[inlen-1] = '\0';
      inlen--;
    }
  }

  errorCode = octetCmdHandleInputLine(new_buf,&octetAsciiBuffer_);
  free(new_buf);

  octetCmdBuf_printf(&octetAsciiBuffer_,"%s%s",had_cr ? "\r" : "", had_lf ? "\n" : "");

  return errorCode;
}

int adsAsynPortDriver::octetCmdHandleInputLine(const char *input_line, adsOctetOutputBufferType *buffer)
{
  const char* functionName = "octetCmdHandleInputLine";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Input line: %s\n", driverName, functionName,input_line);

  const char **my_argv = NULL;
  char **my_sepv = NULL;
  int argc = octetCreateArgvSepv(input_line,
                              (const char*** )&my_argv,
                              (char*** )&my_sepv);

  int errorCodeLatch =0;
  for (int i = 1; i <= argc; i++) {
    int errorCode = octetMotorHandleOneArg(my_argv[i],buffer);  //Continue with next cmd even if error
    if(errorCode && !errorCodeLatch){ //latch first error code for stacked commands
      errorCodeLatch=errorCode;
    }
    octetCmdBuf_printf(buffer,"%s", my_sepv[i]);
  }

  for (int i=0; i <= argc; i++)
  {
    free((void *)my_argv[i]);
    free((void *)my_sepv[i]);
  }
  free(my_argv);
  free(my_sepv);

  return errorCodeLatch;  //First encountered error code
}

/** Parse one ascii command.\
 * Implements part of the asyn-octet ASCII command parser.
 * (see readOctet() and writeOctet for more info).
 * \param[in] myarg_1 Command to parse.
 * \param[out] buffer Output buffer.
 *
 * \return 0 for success or error code.
 */
int adsAsynPortDriver::octetMotorHandleOneArg(const char *myarg_1,adsOctetOutputBufferType *buffer)
{
  const char* functionName = "octetMotorHandleOneArg";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Command: %s\n", driverName, functionName,myarg_1);

  //const char *myarg = myarg_1;
  int err_code=0;

  uint16_t amsPort=amsportDefault_; //should actually be called amsport ( 851 for first plc as default) ...

  /* ADSPORT= */
  if (!strncmp(myarg_1, ADS_OPTION_ADSPORT, strlen(ADS_OPTION_ADSPORT))) {
    myarg_1 += strlen(ADS_OPTION_ADSPORT)+1; //+1 Because equal sign
    int nvals=sscanf(myarg_1,"%" SCNu16,&amsPort);
    if(nvals!=1){
      err_code=ADS_COM_ERROR_OCTET_ADSPORT_OPTION_FAIL;
      OCTET_RETURN_ERROR(buffer,err_code,"%s","ADS_COM_ERROR_OCTET_ADSPORT_OPTION_FAIL");
    }
    myarg_1=strchr(myarg_1, '/');
    myarg_1++;
  }

  /* .THIS.sFeatures? */
  if (0 == strcmp(myarg_1,ADS_OCTET_FEATURES_COMMAND)) {
#ifdef DUT_AXIS_STATUS
    const char *feature_str = "ads;stv1";
#else
    const char *feature_str = "ads";
#endif
    octetCmdBuf_printf(buffer, "%s", feature_str);
    return 0;
  }

  /*.ADR.*/
  const char *adr=strstr(myarg_1,ADS_ADR_COMMAND_PREFIX);
  if(adr) {
    myarg_1 = adr;

    err_code = octetMotorHandleADRCmd(myarg_1,amsPort,buffer);
    if (err_code == -1 || err_code == 0) {
      return 0;
    }
    OCTET_RETURN_ERROR(buffer,err_code,"%s\n",adsErrorToString(err_code));
  }

  char variableName[255];
  memset(&variableName,0,sizeof(variableName));

  //symbolic write
  adr=strchr(myarg_1,'=');
  if(adr)
  {
    //Copy variable name
    strncpy(variableName,myarg_1,adr-myarg_1);
    adr++; //Jump over '='
    err_code = octetAdsWriteByName(amsPort,variableName,adr,buffer);
    if (err_code) {
      OCTET_RETURN_ERROR(buffer,err_code,"%s",adsErrorToString(err_code));
    }
    octetCmdBuf_printf(buffer,"OK");
    return 0;
  }

  //symbolic read
  adr = strchr(myarg_1, '?');
  if (adr)
  {
    //Copy variable name
    strncpy(variableName,myarg_1,adr-myarg_1);
    variableName[adr-myarg_1]=0;
    err_code = octetAdsReadByName(amsPort,variableName,buffer);
    if (err_code) {
      OCTET_RETURN_ERROR(buffer,err_code,"%s",adsErrorToString(err_code));
    }
    return 0;
  }
  /*  if we come here, it is a bad command */
  octetCmdBuf_printf(buffer,"Error: Bad command");
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
int adsAsynPortDriver::octetMotorHandleADRCmd(const char *arg, uint16_t amsport,adsOctetOutputBufferType *buffer)
{
  const char* functionName = "octetMotorHandleADRCmd";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Command: %s, amsPort: %d\n",driverName,functionName,arg,(int)amsport);

  const char *myarg_1 = NULL;
  unsigned group_no = 0;
  unsigned offset_in_group = 0;
  unsigned len_in_PLC = 0;
  unsigned type_in_PLC = 0;
  int nvals;
  nvals = sscanf(arg, ".ADR.16#%x,16#%x,%u,%u=",
                 &group_no,
                 &offset_in_group,
                 &len_in_PLC,
                 &type_in_PLC);

  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: nvals=%d amsport=%u group_no=0x%x offset_in_group=0x%x len_in_PLC=%u type_in_PLC=%u\n",
            driverName,
            functionName,
            nvals,
            amsport,
            group_no,
            offset_in_group,
            len_in_PLC,
            type_in_PLC);

  if (nvals != 4){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse .ADR. command.\n", driverName, functionName);
    return __LINE__;
  }

  //WRITE
  myarg_1 = strchr(arg, '=');
  if (myarg_1) {
    myarg_1++; /* Jump over '=' */

    int error=octetAdsWriteByGroupOffset(amsport,(uint32_t)group_no,(uint32_t) offset_in_group,(uint16_t)type_in_PLC,(uint32_t)len_in_PLC,myarg_1,buffer);
    if (error){
      OCTET_RETURN_ERROR(buffer,error,"%s",adsErrorToString(error));
    }
    octetCmdBuf_printf(buffer,"OK");
    return 0;
  }

  //READ
  myarg_1 = strchr(arg, '?');
  if (myarg_1) {
    myarg_1++; /* Jump over '?' */
    adsSymbolEntry info;
    memset(&info,0,sizeof(info));
    info.dataType=type_in_PLC;
    info.size=len_in_PLC;
    info.iGroup=group_no;
    info.iOffset=offset_in_group;

    int error=octetAdsReadByGroupOffset(amsport,&info,buffer);
    if (error){
      OCTET_RETURN_ERROR(buffer,error,"%s",adsErrorToString(error));
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
int adsAsynPortDriver::octetAdsReadByName(uint16_t amsPort,const char *variableAddr,adsOctetOutputBufferType* outBuffer)
{
  const char* functionName = "octetAdsReadByName";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Variable:%s, amsPort %u\n", driverName, functionName,variableAddr,amsPort);

  adsSymbolEntry infoStruct;
  memset(&infoStruct,0,sizeof(infoStruct));

  long errorCode=0;
  asynStatus stat=adsGetSymInfoByName(amsPort,variableAddr,&infoStruct,&errorCode);
  if (stat!=asynSuccess) {
    return errorCode;
  }

  return octetAdsReadByGroupOffset(amsPort,&infoStruct,outBuffer);
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
int adsAsynPortDriver::octetAdsWriteByName(uint16_t amsPort,const char *variableAddr,const char *asciiValueToWrite,adsOctetOutputBufferType *outBuffer)
{
  const char* functionName = "octetAdsWriteByName";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Variable: %s, value: %s.\n", driverName, functionName,variableAddr,asciiValueToWrite);

  adsSymbolEntry infoStruct;
  memset(&infoStruct,0,sizeof(infoStruct));

  long errorCode=0;
  asynStatus stat=adsGetSymInfoByName(amsPort,variableAddr,&infoStruct,&errorCode);
  if (stat!=asynSuccess) {
    return errorCode;
  }

 return octetAdsWriteByGroupOffset(amsPort,infoStruct.iGroup,infoStruct.iOffset,infoStruct.dataType,infoStruct.size,asciiValueToWrite,outBuffer);
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
int adsAsynPortDriver::octetAdsReadByGroupOffset(uint16_t amsPort,adsSymbolEntry *info, adsOctetOutputBufferType *outBuffer)
{
  const char* functionName = "octetAdsReadByGroupOffset";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: amsPort: %d, group: %d, offset: %d, dataType: %s (%d), dataSize: %d.\n", driverName, functionName,(int)amsPort,(int)info->iGroup,(int)info->iOffset,adsTypeToString(info->dataType),(int)info->dataType,(int)info->size);

  uint32_t bytesRead=0;
  AmsAddr amsServer={remoteNetId_,amsPort};

  int dataSize=info->size;
  if(info->size>ADS_CMD_BUFFER_SIZE){
    dataSize=ADS_CMD_BUFFER_SIZE;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s:%s: Read buffer size smaller than size in plc.\n", driverName, functionName);
  }

  adsLock();
  memset(&octetBinaryBuffer_,0,ADS_CMD_BUFFER_SIZE);

  int error = AdsSyncReadReqEx2(adsPort_, &amsServer, info->iGroup,info->iOffset,dataSize, &octetBinaryBuffer_, &bytesRead);

  if (error) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: ADS read failed with: %s (0x%x).\n", driverName, functionName,adsErrorToString(error),error);
    adsUnlock();
    return error;
  }

  error=octetBinary2ascii(octetReturnVarName_,&octetBinaryBuffer_,ADS_CMD_BUFFER_SIZE,info,outBuffer);
  if (error) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Binary to ASCII conversion failed with: %d\n", driverName, functionName,error);
    adsUnlock();
    return error;
  }
  adsUnlock();
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
int adsAsynPortDriver::octetAdsWriteByGroupOffset(uint16_t amsPort,uint32_t group, uint32_t offset,uint16_t dataType,uint32_t dataSize, const char *asciiValueToWrite,adsOctetOutputBufferType *asciiResponseBuffer)
{
  const char* functionName = "octetAdsWriteByGroupOffset";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: amsPort: %d, group: %d, offset: %d, dataType: %s (%d), dataSize: %d.\n", driverName, functionName,(int)amsPort,(int)group,(int)offset,adsTypeToString(dataType),(int)dataType,(int)dataSize);

  uint32_t bytesToWrite=0;
  AmsAddr amsServer={remoteNetId_,amsPort};

  adsLock();
  memset(&octetBinaryBuffer_,0,ADS_CMD_BUFFER_SIZE);

  int error=octetAscii2binary(asciiValueToWrite,dataType,&octetBinaryBuffer_,ADS_CMD_BUFFER_SIZE,&bytesToWrite);
  if(error){
    adsUnlock();
    octetCmdBuf_printf(asciiResponseBuffer,"Error: %x", error);
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: ASCII to binary conversion failed with: %d.\n", driverName, functionName,error);
    return error;
  }

  if(bytesToWrite>dataSize){
    bytesToWrite=dataSize;
  }

  error = AdsSyncWriteReqEx(adsPort_, &amsServer, group, offset, bytesToWrite, &octetBinaryBuffer_);

  if (error) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: ADS write failed with: %s (0x%x).\n", driverName, functionName,adsErrorToString(error),error);
    adsUnlock();
    return error;
  }

  adsUnlock();
  return 0;
}

/** Overrides asynPortDriver::writeInt32.
 * Writes int32 to PLC
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Value to write.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  const char* functionName = "writeInt32";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);
  adsParamInfo *paramInfo;
  int paramIndex = pasynUser->reason;

  if(!pAdsParamArray_[paramIndex]){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: pAdsParamArray NULL\n", driverName, functionName);
    pasynUser->alarmStatus=WRITE_ALARM;
    callParamCallbacks();
    return asynError;
  }

  paramInfo=pAdsParamArray_[paramIndex];

  //Special case. Check if write ams port state
  if(paramInfo->dataSource==ADS_DATASOURCE_AMS_STATE){
    if(adsWriteState(paramInfo->amsPort,(uint16_t)value)!=asynSuccess){
      return setAlarmParam(paramInfo,WRITE_ALARM,INVALID_ALARM);
    }
    // Write OK -> reset write alarm
    if(paramInfo->alarmStatus==WRITE_ALARM){
      return setAlarmParam(paramInfo,NO_ALARM,NO_ALARM);
    }
    return asynSuccess;
  }

  uint8_t buffer[8]; //largest datatype is 8bytes
  uint32_t maxBytesToWrite=0;
  // Convert epicsInt32 to plctype if possible..
  switch(paramInfo->plcDataType){
    case ADST_INT8:
      int8_t *ADST_INT8Var;
      ADST_INT8Var=((int8_t*)buffer);
      *ADST_INT8Var=(int8_t)value;
      maxBytesToWrite=1;
      break;
    case ADST_INT16:
      int16_t *ADST_INT16Var;
      ADST_INT16Var=((int16_t*)buffer);
      *ADST_INT16Var=(int16_t)value;
      maxBytesToWrite=2;
      break;
    case ADST_INT32:
      int32_t *ADST_INT32Var;
      ADST_INT32Var=((int32_t*)buffer);
      *ADST_INT32Var=(int32_t)value;
      maxBytesToWrite=4;
      break;
    case ADST_INT64:
      int64_t *ADST_INT64Var;
      ADST_INT64Var=((int64_t*)buffer);
      *ADST_INT64Var=(int64_t)value;
      maxBytesToWrite=8;
      break;
    case ADST_UINT8:
      uint8_t *ADST_UINT8Var;
      ADST_UINT8Var=((uint8_t*)buffer);
      *ADST_UINT8Var=(uint8_t)value;
      maxBytesToWrite=1;
      break;
    case ADST_UINT16:
      uint16_t *ADST_UINT16Var;
      ADST_UINT16Var=((uint16_t*)buffer);
      *ADST_UINT16Var=(uint16_t)value;
      maxBytesToWrite=2;
      break;
    case ADST_UINT32:
      uint32_t *ADST_UINT32Var;
      ADST_UINT32Var=((uint32_t*)buffer);
      *ADST_UINT32Var=(uint32_t)value;
      maxBytesToWrite=4;
      break;
    case ADST_UINT64:
      uint64_t *ADST_UINT64Var;
      ADST_UINT64Var=((uint64_t*)buffer);
      *ADST_UINT64Var=(uint64_t)value;
      maxBytesToWrite=8;
      break;
    case ADST_REAL32:
      float *ADST_REAL32Var;
      ADST_REAL32Var=((float*)buffer);
      *ADST_REAL32Var=(float)value;
      maxBytesToWrite=4;
      break;
    case ADST_REAL64:
      double *ADST_REAL64Var;
      ADST_REAL64Var=((double*)buffer);
      *ADST_REAL64Var=(double)value;
      maxBytesToWrite=8;
      break;
    case ADST_BIT:
      buffer[0]=value>0;
      maxBytesToWrite=1;
      break;
    default:
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Data types not compatible (epicsInt32 and %s). Write canceled.\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType));
      return asynError;
      break;
  }

  // Warning. Risk of loss of data..
  if(sizeof(value)>maxBytesToWrite || sizeof(value)>paramInfo->plcSize){
    asynPrint(pasynUser, ASYN_TRACE_WARNING, "%s:%s: WARNING. EPICS datatype size larger than PLC datatype size (%ld vs %d bytes).\n", driverName,functionName,sizeof(value),paramInfo->plcDataType);
    paramInfo->plcDataTypeWarn=true;
  }

  //Ensure that PLC datatype and number of bytes to write match
  if(maxBytesToWrite!=paramInfo->plcSize || maxBytesToWrite==0){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Data types size missmatch (%s and %d bytes). Write canceled.\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),maxBytesToWrite);
    setAlarmParam(paramInfo,WRITE_ALARM,INVALID_ALARM);
    callParamCallbacks();
    return asynError;
  }

  //Do the write
  if(adsWriteParam(paramInfo,(const void *)buffer,maxBytesToWrite)!=asynSuccess){
    setAlarmParam(paramInfo,WRITE_ALARM,INVALID_ALARM);
    callParamCallbacks();
    return asynError;
  }
  //Only reset if write alarm
  if(paramInfo->alarmStatus==WRITE_ALARM){
    setAlarmParam(paramInfo,NO_ALARM,NO_ALARM);
  }

  return asynPortDriver::writeInt32(pasynUser, value);
}

/** Overrides asynPortDriver::writeFloat64.
 * Writes float64 to PLC
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Value to write.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  const char* functionName = "writeFloat64";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

   adsParamInfo *paramInfo;
  int paramIndex = pasynUser->reason;

  if(!pAdsParamArray_[paramIndex]){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: pAdsParamArray NULL\n", driverName, functionName);
    pasynUser->alarmStatus=WRITE_ALARM;
    pasynUser->alarmSeverity=INVALID_ALARM;
    callParamCallbacks();
    return asynError;
  }
  paramInfo=pAdsParamArray_[paramIndex];

  //Special case. Check if write ams port state
  if(paramInfo->dataSource==ADS_DATASOURCE_AMS_STATE){
    if(adsWriteState(paramInfo->amsPort,(uint16_t)value)!=asynSuccess){
      return setAlarmParam(paramInfo,WRITE_ALARM,INVALID_ALARM);
    }
    // Write OK -> reset write alarm
    if(paramInfo->alarmStatus==WRITE_ALARM){
      return setAlarmParam(paramInfo,NO_ALARM,NO_ALARM);
    }
    return asynSuccess;
  }

  uint8_t buffer[8]; //largest datatype is 8bytes
  uint32_t maxBytesToWrite=0;
  // Convert epicsFloat64 to plctype if possible..
  switch(paramInfo->plcDataType){
    case ADST_INT8:
      int8_t *ADST_INT8Var;
      ADST_INT8Var=((int8_t*)buffer);
      *ADST_INT8Var=(int8_t)value;
      maxBytesToWrite=1;
      break;
    case ADST_INT16:
      int16_t *ADST_INT16Var;
      ADST_INT16Var=((int16_t*)buffer);
      *ADST_INT16Var=(int16_t)value;
      maxBytesToWrite=2;
      break;
    case ADST_INT32:
      int32_t *ADST_INT32Var;
      ADST_INT32Var=((int32_t*)buffer);
      *ADST_INT32Var=(int32_t)value;
      maxBytesToWrite=4;
      break;
    case ADST_INT64:
      int64_t *ADST_INT64Var;
      ADST_INT64Var=((int64_t*)buffer);
      *ADST_INT64Var=(int64_t)value;
      maxBytesToWrite=8;
      break;
    case ADST_UINT8:
      uint8_t *ADST_UINT8Var;
      ADST_UINT8Var=((uint8_t*)buffer);
      *ADST_UINT8Var=(uint8_t)value;
      maxBytesToWrite=1;
      break;
    case ADST_UINT16:
      uint16_t *ADST_UINT16Var;
      ADST_UINT16Var=((uint16_t*)buffer);
      *ADST_UINT16Var=(uint16_t)value;
      maxBytesToWrite=2;
      break;
    case ADST_UINT32:
      uint32_t *ADST_UINT32Var;
      ADST_UINT32Var=((uint32_t*)buffer);
      *ADST_UINT32Var=(uint32_t)value;
      maxBytesToWrite=4;
      break;
    case ADST_UINT64:
      uint64_t *ADST_UINT64Var;
      ADST_UINT64Var=((uint64_t*)buffer);
      *ADST_UINT64Var=(uint64_t)value;
      maxBytesToWrite=8;
      break;
    case ADST_REAL32:
      float *ADST_REAL32Var;
      ADST_REAL32Var=((float*)buffer);
      *ADST_REAL32Var=(float)value;
      maxBytesToWrite=4;
      break;
    case ADST_REAL64:
      double *ADST_REAL64Var;
      ADST_REAL64Var=((double*)buffer);
      *ADST_REAL64Var=(double)value;
      maxBytesToWrite=8;
      break;
    case ADST_BIT:
      buffer[0]=value>0;
      maxBytesToWrite=1;
      break;
    default:
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Data types not compatible (epicsInt32 and %s). Write canceled.\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType));
      return asynError;
      break;
  }

  // Warning. Risk of loss of data..
  if(sizeof(value)>maxBytesToWrite || sizeof(value)>paramInfo->plcSize){
    asynPrint(pasynUser, ASYN_TRACE_WARNING, "%s:%s: WARNING. EPICS datatype size larger than PLC datatype size (%ld vs %d bytes).\n", driverName,functionName,sizeof(value),paramInfo->plcDataType);
    paramInfo->plcDataTypeWarn=true;
  }

  //Ensure that PLC datatype and number of bytes to write match
  if(maxBytesToWrite!=paramInfo->plcSize || maxBytesToWrite==0){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Data types size mismatch (%s and %d bytes). Write canceled.\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),maxBytesToWrite);
    setAlarmParam(paramInfo,WRITE_ALARM,INVALID_ALARM);
    callParamCallbacks();
    return asynError;
  }

  //Do the write
  if(adsWriteParam(paramInfo,(const void *)buffer,maxBytesToWrite)!=asynSuccess){
    setAlarmParam(paramInfo,WRITE_ALARM,INVALID_ALARM);
    callParamCallbacks();
    return asynError;
  }

  //Only reset if write alarm
  if(paramInfo->alarmStatus==WRITE_ALARM){
    setAlarmParam(paramInfo,NO_ALARM,NO_ALARM);
  }

  return asynPortDriver::writeFloat64(pasynUser,value);
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
asynStatus adsAsynPortDriver::adsGenericArrayRead(asynUser *pasynUser,long allowedType,void *epicsDataBuffer,size_t nEpicsBufferBytes,size_t *nBytesRead)
{
  const char* functionName = "adsGenericArrayRead";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  int paramIndex=pasynUser->reason;

  if(!pAdsParamArray_[paramIndex] || paramIndex>=paramTableSize_){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: pAdsParamArray NULL or index (pasynUser->reason) our of range\n", driverName, functionName);
    pasynUser->alarmStatus=READ_ALARM;
    pasynUser->alarmSeverity=INVALID_ALARM;
    return asynError;
  }

  adsParamInfo *paramInfo=pAdsParamArray_[paramIndex];

  //Only support same datatype as in PLC
  if(paramInfo->plcDataType!=allowedType){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Data types not compatible (%s vs %s). Read canceled.\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),adsTypeToString(allowedType));
    setAlarmParam(paramInfo,READ_ALARM,INVALID_ALARM);
    return asynError;
  }

  size_t bytesToWrite=nEpicsBufferBytes;
  if(paramInfo->plcSize<nEpicsBufferBytes){
    bytesToWrite=paramInfo->plcSize;
  }

  if(!paramInfo->arrayDataBuffer || !epicsDataBuffer){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Buffer(s) NULL. Read canceled.\n", driverName, functionName);
    setAlarmParam(paramInfo,READ_ALARM,INVALID_ALARM);
    return asynError;
  }

  memcpy(epicsDataBuffer,paramInfo->arrayDataBuffer,bytesToWrite);
  *nBytesRead=bytesToWrite;

  //Only reset if read alarm
  if(paramInfo->alarmStatus==READ_ALARM){
    setAlarmParam(paramInfo,NO_ALARM,NO_ALARM);
  }

  //update timestamp
  pasynUser->timestamp=paramInfo->epicsTimestamp;

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
asynStatus adsAsynPortDriver::adsGenericArrayWrite(asynUser *pasynUser,long allowedType,const void *data,size_t nEpicsBufferBytes)
{
  const char* functionName = "adsGenericArrayWrite";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  int paramIndex=pasynUser->reason;

  if(!pAdsParamArray_[paramIndex] || paramIndex>=paramTableSize_){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: pAdsParamArray NULL or index (pasynUser->reason) our of range\n", driverName, functionName);
    pasynUser->alarmStatus=WRITE_ALARM;
    pasynUser->alarmSeverity=INVALID_ALARM;
    return asynError;
  }

  adsParamInfo *paramInfo=pAdsParamArray_[paramIndex];

  //Only support same datatype as in PLC
  if(paramInfo->plcDataType!=allowedType){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Data types not compatible (%s vs %s). Write canceled.\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),adsTypeToString(allowedType));
    setAlarmParam(paramInfo,WRITE_ALARM,INVALID_ALARM);
    return asynError;
  }

  size_t bytesToWrite=nEpicsBufferBytes;
  if(paramInfo->plcSize<nEpicsBufferBytes){
    bytesToWrite=paramInfo->plcSize;
  }

  //Write to ADS
  asynStatus stat=adsWriteParam(paramInfo,data,bytesToWrite);
  if(stat!=asynSuccess){
    setAlarmParam(paramInfo,WRITE_ALARM,INVALID_ALARM);
    return asynError;
  }

  //copy data to buffer;
  if(paramInfo->arrayDataBuffer){
    memcpy(paramInfo->arrayDataBuffer,data,bytesToWrite);
  }

  //Only reset if write alarm
  if(paramInfo->alarmStatus==WRITE_ALARM){
    setAlarmParam(paramInfo,NO_ALARM,NO_ALARM);
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
asynStatus adsAsynPortDriver::readInt8Array(asynUser *pasynUser,epicsInt8 *value,size_t nElements,size_t *nIn)
{
  const char* functionName = "readInt8Array";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  if(!pAdsParamArray_[pasynUser->reason]){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: pAdsParamArray NULL\n", driverName, functionName);
    return asynError;
  }

  long allowedType=ADST_INT8;

  //Also allow string and bool array as int8array (special case)
  if(pAdsParamArray_[pasynUser->reason]->plcDataType==ADST_STRING){
    allowedType=ADST_STRING;
  }
  else if(pAdsParamArray_[pasynUser->reason]->plcDataType==ADST_BIT){
    allowedType=ADST_BIT;
  }


  size_t nBytesRead=0;
  asynStatus stat=adsGenericArrayRead(pasynUser, allowedType,(void *)value,nElements*sizeof(epicsInt8),&nBytesRead);
  if(stat!=asynSuccess){
    return asynError;
  }
  *nIn=nBytesRead/sizeof(epicsInt8);
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
asynStatus adsAsynPortDriver::writeInt8Array(asynUser *pasynUser, epicsInt8 *value,size_t nElements)
{
  const char* functionName = "writeInt8Array";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);


  long allowedType=ADST_INT8;

  //Also allow string and bool array as int8array (special case)
  if(pAdsParamArray_[pasynUser->reason]->plcDataType==ADST_STRING){
    allowedType=ADST_STRING;
  }
  else if(pAdsParamArray_[pasynUser->reason]->plcDataType==ADST_BIT){
    allowedType=ADST_BIT;
  }

  return adsGenericArrayWrite(pasynUser,allowedType,(const void *)value,nElements*sizeof(epicsInt8));
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
asynStatus adsAsynPortDriver::readInt16Array(asynUser *pasynUser,epicsInt16 *value,size_t nElements,size_t *nIn)
{
  const char* functionName = "readInt16Array";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  long allowedType=ADST_INT16;

  size_t nBytesRead=0;
  asynStatus stat=adsGenericArrayRead(pasynUser, allowedType,(void *)value,nElements*sizeof(epicsInt16),&nBytesRead);
  if(stat!=asynSuccess){
    return asynError;
  }
  *nIn=nBytesRead/sizeof(epicsInt16);
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
asynStatus adsAsynPortDriver::writeInt16Array(asynUser *pasynUser, epicsInt16 *value,size_t nElements)
{
  const char* functionName = "writeInt16Array";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  long allowedType=ADST_INT16;

  return adsGenericArrayWrite(pasynUser,allowedType,(const void *)value,nElements*sizeof(epicsInt16));
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
asynStatus adsAsynPortDriver::readInt32Array(asynUser *pasynUser,epicsInt32 *value,size_t nElements,size_t *nIn)
{
  const char* functionName = "readInt32Array";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  long allowedType=ADST_INT32;

  size_t nBytesRead=0;
  asynStatus stat=adsGenericArrayRead(pasynUser, allowedType,(void *)value,nElements*sizeof(epicsInt32),&nBytesRead);
  if(stat!=asynSuccess){
    return asynError;
  }
  *nIn=nBytesRead/sizeof(epicsInt32);
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
asynStatus adsAsynPortDriver::writeInt32Array(asynUser *pasynUser, epicsInt32 *value,size_t nElements)
{
  const char* functionName = "writeInt32Array";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  long allowedType=ADST_INT32;

  return adsGenericArrayWrite(pasynUser,allowedType,(const void *)value,nElements*sizeof(epicsInt32));
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
asynStatus adsAsynPortDriver::readFloat32Array(asynUser *pasynUser,epicsFloat32 *value,size_t nElements,size_t *nIn)
{
  const char* functionName = "readFloat32Array";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  long allowedType=ADST_REAL32;

  size_t nBytesRead=0;
  asynStatus stat=adsGenericArrayRead(pasynUser, allowedType,(void *)value,nElements*sizeof(epicsFloat32),&nBytesRead);
  if(stat!=asynSuccess){
    return asynError;
  }
  *nIn=nBytesRead/sizeof(epicsFloat32);
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
asynStatus adsAsynPortDriver::writeFloat32Array(asynUser *pasynUser,epicsFloat32 *value,size_t nElements)
{
  const char* functionName = "writeFloat32Array";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  long allowedType=ADST_REAL32;
  return adsGenericArrayWrite(pasynUser,allowedType,(const void *)value,nElements*sizeof(epicsFloat32));
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
asynStatus adsAsynPortDriver::readFloat64Array(asynUser *pasynUser,epicsFloat64 *value,size_t nElements,size_t *nIn)
{
  const char* functionName = "readFloat64Array";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  long allowedType=ADST_REAL64;

  size_t nBytesRead=0;
  asynStatus stat=adsGenericArrayRead(pasynUser, allowedType,(void *)value,nElements*sizeof(epicsFloat64),&nBytesRead);
  if(stat!=asynSuccess){
    return asynError;
  }
  *nIn=nBytesRead/sizeof(epicsFloat64);
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
asynStatus adsAsynPortDriver::writeFloat64Array(asynUser *pasynUser,epicsFloat64 *value,size_t nElements)
{
  const char* functionName = "writeFloat64Array";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  long allowedType=ADST_REAL64;
  return adsGenericArrayWrite(pasynUser,allowedType,(const void *)value,nElements*nElements*sizeof(epicsFloat64));
}

/** Returns pasynUserSelf for use in asynPrint().
 *
 * \return pasynUserSelf
 */
asynUser *adsAsynPortDriver::getTraceAsynUser()
{
  const char* functionName = "getTraceAsynUser";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  return pasynUserSelf;
}

/** Get handle to symbolic plc variable.
 *
 * \param[in/out] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsGetSymHandleByName(adsParamInfo *paramInfo)
{
  return adsGetSymHandleByName(paramInfo,false);
}

/** Get handle to symbolic plc variable.
 *
 * \param[in/out] paramInfo Parameter information.
 * \param[in] blockErrorMsg Suppress error messages
 *            (used while trying to reconnect to avoid alot of error messages).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsGetSymHandleByName(adsParamInfo *paramInfo,bool blockErrorMsg)
{
  const char* functionName = "adsGetSymHandleByName";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  AmsAddr amsServer;
  amsServer={remoteNetId_,paramInfo->amsPort};

  uint32_t symbolHandle=0;
  adsLock();
  const long handleStatus = AdsSyncReadWriteReqEx2(adsPort_,
                                                   &amsServer,
                                                   ADSIGRP_SYM_HNDBYNAME,
                                                   0,
                                                   sizeof(paramInfo->hSymbolicHandle),
                                                   &symbolHandle,
                                                   strlen(paramInfo->plcAdrStr),
                                                   paramInfo->plcAdrStr,
                                                   nullptr);
  adsUnlock();
  if (handleStatus) {
    if(!blockErrorMsg){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Create handle for %s failed with: %s (0x%lx)\n", driverName, functionName,paramInfo->plcAdrStr,adsErrorToString(handleStatus),handleStatus);
    }
    return asynError;
  }

  //Add handle succeded
  paramInfo->hSymbolicHandle=symbolHandle;
  paramInfo->bSymbolicHandleValid=true;

  return asynSuccess;
}

/** Register on-change callback for symbols version
 *
 * \param[in] port Structure containig Ams-port information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsAddSymbolsChangedCallback(amsPortInfo *port)
{
  const char* functionName = "adsAddSymbolsChangedCallback";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Ams-port %u.\n", driverName, functionName,port->amsPort);

  AmsAddr amsServer;
  amsServer={remoteNetId_,port->amsPort};

  AdsNotificationAttrib attrib;
  attrib.cbLength=1;
  attrib.nTransMode=ADSTRANS_SERVERONCHA;  //Add option
  attrib.nMaxDelay=(uint32_t)(defaultMaxDelayTimeMS_*10000); // 100ms
  attrib.nCycleTime=(uint32_t)(defaultSampleTimeMS_*10000);

  uint32_t hNotify=0;
  adsLock();
  long addStatus = AdsSyncAddDeviceNotificationReqEx(adsPort_,
                                                     &amsServer,
                                                     ADSIGRP_SYM_VERSION,
                                                     0,
                                                     &attrib,
                                                     &adsSymbolsChangedCallback,
                                                     (uint32_t)port->amsPort,  //Use amsPort as hUser
                                                     &hNotify);
  adsUnlock();
  if (addStatus){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Add device notification failed with: %s (0x%lx)\n", driverName, functionName,adsErrorToString(addStatus),addStatus);
    return asynError;
  }

  //Add was successful
  port->hCallbackNotify=hNotify;
  port->bCallbackNotifyValid=true;
  port->refreshNeeded=false;

  return asynSuccess;
}

/** Unregister on-change callback for symbols version
 *
 * \param[in] port Structure containig Ams-port information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsDelSymbolsChangedCallback(amsPortInfo *port)
{
  const char* functionName = "adsDelSymbolsChangedCallback";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  AmsAddr amsServer;
  amsServer={remoteNetId_,port->amsPort};

  adsLock();
  const long delStatus = AdsSyncDelDeviceNotificationReqEx(adsPort_, &amsServer,port->hCallbackNotify);
  adsUnlock();
  port->bCallbackNotifyValid=false;
  port->hCallbackNotify=-1;

  if (delStatus){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Delete device notification failed with: %s (0x%lx)\n", driverName, functionName,adsErrorToString(delStatus),delStatus);
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
asynStatus adsAsynPortDriver::adsAddDataCallback(adsParamInfo *paramInfo)
{
  const char* functionName = "adsAddDataCallback";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  uint32_t group=0;
  uint32_t offset=0;

  paramInfo->bCallbackNotifyValid=false;

  AmsAddr amsServer;
  amsServer={remoteNetId_,paramInfo->amsPort};

  if(paramInfo->isAdrCommand){// Abs access (ADR command)
    if(!paramInfo->plcAbsAdrValid){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Absolute address in paramInfo not valid.\n", driverName, functionName);
      return asynError;
    }

    group=paramInfo->plcAbsAdrGroup;
    offset=paramInfo->plcAbsAdrOffset;
  }
  else{ // Symbolic access

    // Read symbolic information if needed (to get paramInfo->plcSize)
    if(!paramInfo->plcAbsAdrValid){
      asynStatus statusInfo=adsGetSymInfoByName(paramInfo);
      if(statusInfo!=asynSuccess){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: adsGetSymInfoByName failed.\n", driverName, functionName);
        return asynError;
      }
    }

    // Get symbolic handle if needed
    if(!paramInfo->bSymbolicHandleValid){
      asynStatus statusHandle=adsGetSymHandleByName(paramInfo);
      if(statusHandle!=asynSuccess){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: adsGetSymHandleByName failed.\n", driverName, functionName);
        return asynError;
      }
    }

    group=ADSIGRP_SYM_VALBYHND;  //Access via symbolic handle stored in paramInfo->hSymbolicHandle
    offset=paramInfo->hSymbolicHandle;
  }

  AdsNotificationAttrib attrib;
  /** Length of the data that is to be passed to the callback function. */
  attrib.cbLength=paramInfo->plcSize;
  /**
  * ADSTRANS_SERVERCYCLE: The notification's callback function is invoked cyclically.
  * ADSTRANS_SERVERONCHA: The notification's callback function is only invoked when the value changes.
  */
  attrib.nTransMode=ADSTRANS_SERVERONCHA;  //Add option
  /** The notification's callback function is invoked at the latest when this time has elapsed. The unit is 100 ns. */
  attrib.nMaxDelay=(uint32_t)(paramInfo->maxDelayTimeMS*10000); // 100ms
  /** The ADS server checks whether the variable has changed after this time interval. The unit is 100 ns. */
  attrib.nCycleTime=(uint32_t)(paramInfo->sampleTimeMS*10000);

  uint32_t hNotify=0;
  adsLock();
  long addStatus = AdsSyncAddDeviceNotificationReqEx(adsPort_,
                                                     &amsServer,
                                                     group,
                                                     offset,
                                                     &attrib,
                                                     &adsDataCallback,
                                                     (uint32_t)paramInfo->paramIndex,
                                                     &hNotify);
  adsUnlock();
  if (addStatus){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Add device notification failed with: %s (0x%lx)\n", driverName, functionName,adsErrorToString(addStatus),addStatus);
    return asynError;
  }

  //Add was successful
  paramInfo->hCallbackNotify=hNotify;
  paramInfo->bCallbackNotifyValid=true;

  return asynSuccess;
}

/** Unregister on-change callback for parameter (plc-variable).
 *
 * \param[in/out] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsDelDataCallback(adsParamInfo *paramInfo)
{
  return adsDelDataCallback(paramInfo,false);
}

/** Unregister on-change callback for parameter (plc-variable).
 *
 * \param[in/out] paramInfo Parameter information.
 * \param[in] blockErrorMsg Suppress error messages
 *            (used while trying to reconnect to avoid alot of error messages).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsDelDataCallback(adsParamInfo *paramInfo,bool blockErrorMsg)
{
  const char* functionName = "adsDelDataCallback";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  paramInfo->bCallbackNotifyValid=false;

  AmsAddr amsServer;
  amsServer={remoteNetId_,paramInfo->amsPort};

  adsLock();
  const long delStatus = AdsSyncDelDeviceNotificationReqEx(adsPort_, &amsServer,paramInfo->hCallbackNotify);
  paramInfo->hCallbackNotify=-1;
  adsUnlock();
  if (delStatus){
    if(!blockErrorMsg){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Delete device notification failed with: %s (0x%lx)\n", driverName, functionName,adsErrorToString(delStatus),delStatus);
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
asynStatus adsAsynPortDriver::adsGetSymInfoByName(uint16_t amsPort,const char *varName, adsSymbolEntry *info)
{
  long errorCode=0;
  return adsGetSymInfoByName(amsPort,varName,info,&errorCode);
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
asynStatus adsAsynPortDriver::adsGetSymInfoByName(uint16_t amsPort,const char *varName, adsSymbolEntry *info,long *errorCode)
{
  const char* functionName = "adsGetSymInfoByName";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Variable name: %s, amsPort: %d.\n", driverName, functionName,varName,(int)amsPort);

  if(!info || !varName){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Info struct or varName NULL.\n", driverName, functionName);
    return asynError;
  }

  uint32_t bytesRead=0;
  AmsAddr amsServer;

  amsServer={remoteNetId_,amsPort};
  adsLock();
  const long infoStatus = AdsSyncReadWriteReqEx2(adsPort_,
                                                 &amsServer,
                                                 ADSIGRP_SYM_INFOBYNAMEEX,
                                                 0,
                                                 sizeof(adsSymbolEntry),
                                                 info,
                                                 strlen(varName),
                                                 varName,
                                                 &bytesRead);
  adsUnlock();
  *errorCode=infoStatus;

  if (infoStatus) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Get symbolic information failed for %s with: %s (0x%lx)\n", driverName, functionName,varName,adsErrorToString(infoStatus),infoStatus);
    if (infoStatus == GLOBALERR_TARGET_PORT) {
        /* Sigh.  It was up, now it's down.  Let's go home. */
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Port error, giving up!\n",
                  driverName, functionName);
        exit(1);
    }
    return asynError;
  }

  info->variableName = info->buffer;

  if(info->nameLength>=sizeof(info->buffer)-1){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Missalignment of type in AdsSyncReadWriteReqEx2 return struct for %s\n", driverName, functionName,varName);
    return asynError;
  }
  info->symDataType = info->buffer+info->nameLength+1;

  if(info->nameLength + info->typeLength+2>=(uint16_t)(sizeof(info->buffer)-1)){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Missalignment of comment in AdsSyncReadWriteReqEx2 return struct for %s\n", driverName, functionName,varName);
  }
  info->symComment= info->symDataType+info->typeLength+1;

  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"Symbolic information\n");
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"SymEntrylength: %d\n",info->entryLen);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"idxGroup: 0x%x\n",info->iGroup);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"idxOffset: 0x%x\n",info->iOffset);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"ByteSize: %d\n",info->size);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"adsDataType: %d\n",info->dataType);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"Flags: %d\n",info->flags);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"Name length: %d\n",info->nameLength);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"Type length: %d\n",info->typeLength);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"Type length: %d\n",info->commentLength);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"Variable name: %s\n",info->variableName);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"Data type: %s\n",info->symDataType);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER ,"Comment: %s\n",info->symComment);

  return asynSuccess;
}

/** Get symbolic information for a plc variable.
 *
 * \param[in/out] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsGetSymInfoByName(adsParamInfo *paramInfo)
{
  const char* functionName = "adsGetSymInfoByName";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  adsSymbolEntry infoStruct;
  memset(&infoStruct,0,sizeof(infoStruct));

  asynStatus stat=adsGetSymInfoByName(paramInfo->amsPort,paramInfo->plcAdrStr,&infoStruct);
  if (stat) {
    return asynError;
  }

  //fill paramInfo data structure
  paramInfo->plcAbsAdrGroup=infoStruct.iGroup;
  paramInfo->plcAbsAdrOffset=infoStruct.iOffset;
  paramInfo->plcSize=infoStruct.size;
  paramInfo->plcDataType=infoStruct.dataType;
  paramInfo->plcAbsAdrValid=true;

  return asynSuccess;
}

/** Connect to ads router (TwinCAT system).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsConnect()
{
  const char* functionName = "adsConnect";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  adsDelRoute(1);
  // add local route to your ADS Master
  if(!routeAdded_){
    asynStatus stat=adsAddRouteLock();

    if (stat!=asynSuccess) {
      adsDelRoute(1);
      if (adsPort_) {
          adsLock();
          AdsPortCloseEx(adsPort_);
          adsPort_=0;
          adsUnlock();
          return asynError;
      }
    }
  }

  // open a new ADS port
  adsLock();
  if (!adsPort_)
      adsPort_ = AdsPortOpenEx();
  adsUnlock();
  if (!adsPort_) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s:Open ADS port failed.\n", driverName, functionName);
    return asynError;
  }
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s:Open ADS port = %ld.\n", driverName, functionName, adsPort_);
  // Update timeout
  uint32_t defaultTimeout=0;
  adsLock();
  long status=AdsSyncGetTimeoutEx(adsPort_,&defaultTimeout);
  adsUnlock();
  if(status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: AdsSyncGetTimeoutEx failed with: %s (0x%lx).\n", driverName, functionName,adsErrorToString(status),status);
    return asynError;
  }
  adsLock();
  status=AdsSyncSetTimeoutEx(adsPort_,(uint32_t)adsTimeoutMS_);
  adsUnlock();
  if(status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: AdsSyncSetTimeoutEx failed with: %s (0x%lx).\n", driverName, functionName,adsErrorToString(status),status);
    return asynError;
  }

  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"%s:%s: Update ADS sync time out from %u to %u.\n", driverName, functionName,defaultTimeout,(uint32_t)adsTimeoutMS_);

  return asynSuccess;
}

/** Read Ams port version information
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReadVersion(amsPortInfo *port)
{
  const char* functionName = "adsReadVersion";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Ams-port %u\n", driverName, functionName,port->amsPort);

  AmsAddr amsServer;
  AdsVersion version;
  char devName[255];
  amsServer={remoteNetId_,port->amsPort};

  adsLock();
  long status=AdsSyncReadDeviceInfoReqEx(adsPort_,&amsServer,devName,&version);
  adsUnlock();
  if(status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: AdsSyncReadDeviceInfoReqEx failed with: %s (0x%lx).\n", driverName, functionName,adsErrorToString(status),status);
    return asynError;
  }

  port->version=version;
  strncpy(port->devName,devName,sizeof(port->devName));
  return asynSuccess;
}

/** Disconnect ads router (TwinCAT system).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsDisconnect()
{
  const char* functionName = "adsDisconnect";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: adsPort_=%ld\n", driverName, functionName, adsPort_);

  adsLock();
  const long closeStatus = AdsPortCloseEx(adsPort_);
  adsPort_ = 0;
  adsUnlock();
  if (closeStatus) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Close ADS port failed with: %s (0x%lx)\n", driverName, functionName,adsErrorToString(closeStatus),closeStatus);
    return asynError;
  }

  return asynSuccess;
}

/** Release handle to symbolic variable (in TwinCAT plc)
 *
 * \param[in/out] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReleaseSymbolicHandle(adsParamInfo *paramInfo)
{
  return adsReleaseSymbolicHandle(paramInfo, false);
}

/** Release handle to symbolic variable (in TwinCAT plc)
 *
 * \param[in/out] paramInfo Parameter information.
 * \param[in] blockErrorMsg Suppress error messages
 *            (used while trying to reconnect to avoid alot of error messages).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReleaseSymbolicHandle(adsParamInfo *paramInfo, bool blockErrorMsg)
{
  const char* functionName = "adsReleaseHandle";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  if (paramInfo->bSymbolicHandleValid) {
    AmsAddr amsServer;
    amsServer={remoteNetId_,paramInfo->amsPort};

    adsLock();
    const long releaseStatus = AdsSyncWriteReqEx(adsPort_, &amsServer, ADSIGRP_SYM_RELEASEHND, 0, sizeof(paramInfo->hSymbolicHandle), &paramInfo->hSymbolicHandle);
    adsUnlock();
    paramInfo->hSymbolicHandle=-1;
    paramInfo->bSymbolicHandleValid=false;
    if (releaseStatus && !blockErrorMsg) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Release of handle 0x%x failed with: %s (0x%lx)\n", driverName, functionName,paramInfo->hSymbolicHandle,adsErrorToString(releaseStatus),releaseStatus);
      return asynError;
    }
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
asynStatus adsAsynPortDriver::adsWriteParam(adsParamInfo *paramInfo,const void *binaryBuffer,uint32_t bytesToWrite)
{
  const char* functionName = "adsWriteParam";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  // Calculate consumed time by this method
  struct timeval start, end;
  long secs_used,micros_used;
  gettimeofday(&start, NULL);

  uint32_t group=0;
  uint32_t offset=0;

  AmsAddr amsServer;
  amsServer={remoteNetId_,paramInfo->amsPort};

  if(paramInfo->isAdrCommand){// Abs access (ADR command)
    if(!paramInfo->plcAbsAdrValid){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Absolute address in paramInfo not valid.\n", driverName, functionName);
      return asynError;
    }

    group=paramInfo->plcAbsAdrGroup;
    offset=paramInfo->plcAbsAdrOffset;
  }
  else{ // Symbolic access

    // Read symbolic information if needed (to get paramInfo->plcSize)
    if(!paramInfo->plcAbsAdrValid){
      asynStatus statusInfo=adsGetSymInfoByName(paramInfo);
      if(statusInfo==asynError){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: adsGetSymInfoByName failed.\n", driverName, functionName);
        return asynError;
      }
    }

    // Get symbolic handle if needed
    if(!paramInfo->bSymbolicHandleValid){
      asynStatus statusHandle=adsGetSymHandleByName(paramInfo);
      if(statusHandle!=asynSuccess){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: adsGetSymHandleByName failed.\n", driverName, functionName);
        return asynError;
      }
    }

    group=ADSIGRP_SYM_VALBYHND;  //Access via symbolic handle stored in paramInfo->hSymbolicHandle
    offset=paramInfo->hSymbolicHandle;
  }
  adsLock();
  long writeStatus= AdsSyncWriteReqEx(adsPort_,
                                      &amsServer,
                                      group,
                                      offset,
                                      paramInfo->plcSize,
                                      binaryBuffer);
  adsUnlock();
  if (writeStatus) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: ADS write failed with: %s (0x%lx)\n", driverName, functionName,adsErrorToString(writeStatus),writeStatus);
    return asynError;
  }

  gettimeofday(&end, NULL);
  secs_used=(end.tv_sec - start.tv_sec); //avoid overflow by subtracting first
  micros_used= ((secs_used*1000000) + end.tv_usec) - (start.tv_usec);
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER , "%s:%s: ADS write: micros used: 0x%lx\n", driverName, functionName,micros_used);

  return asynSuccess;
}

/** Read value of variable in TwinCAT.
 *
 * \param[in] paramInfo Parameter information.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReadParam(adsParamInfo *paramInfo)
{
  long notused=0;
  return adsReadParam(paramInfo,&notused,1);
}

/** Read value of variable in TwinCAT.
 *
 * \param[in] paramInfo Parameter information.
 * \param[out] error Error code.
 * \param[in] updateAsynPar Update asyn parameter.
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReadParam(adsParamInfo *paramInfo,long *error,int updateAsynPar)
{
  const char* functionName = "adsReadParam";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  uint32_t group=0;
  uint32_t offset=0;
  *error=0;

  AmsAddr amsServer;
  amsServer={remoteNetId_,paramInfo->amsPort};

  if(paramInfo->isAdrCommand){// Abs access (ADR command)
    if(!paramInfo->plcAbsAdrValid){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Absolute address in paramInfo not valid.\n", driverName, functionName);
      return asynError;
    }

    group=paramInfo->plcAbsAdrGroup;
    offset=paramInfo->plcAbsAdrOffset;
  }
  else{ // Symbolic access

    // Read symbolic information if needed (to get paramInfo->plcSize)
    if(!paramInfo->plcAbsAdrValid){
      asynStatus statusInfo=adsGetSymInfoByName(paramInfo);
      if(statusInfo==asynError){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: adsGetSymInfoByName failed.\n", driverName, functionName);
        return asynError;
      }
    }

    // Get symbolic handle if needed
    if(!paramInfo->bSymbolicHandleValid){
      asynStatus statusHandle=adsGetSymHandleByName(paramInfo);
      if(statusHandle!=asynSuccess){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: adsGetSymHandleByName failed.\n", driverName, functionName);
        return asynError;
      }
    }

    group=ADSIGRP_SYM_VALBYHND;  //Access via symbolic handle stored in paramInfo->hSymbolicHandle
    offset=paramInfo->hSymbolicHandle;
  }

  char *data=new char[paramInfo->plcSize];
  uint32_t bytesRead=0;
  adsLock();
  *error = AdsSyncReadReqEx2(adsPort_,
                                     &amsServer,
                                     group,
                                     offset,
                                     paramInfo->plcSize,
                                     (void *)data,
                                     &bytesRead);
  adsUnlock();
  if(*error){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: AdsSyncReadReqEx2 failed: %s (%lu).\n", driverName, functionName,adsErrorToString(*error),*error);
    return asynError;
  }

  if(bytesRead!=paramInfo->plcSize){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Read bytes differ from parameter plc size (%u vs %u).\n", driverName, functionName,bytesRead,paramInfo->plcSize);
    return asynError;
  }

  //No timestamp available
  paramInfo->plcTimeStampRaw=0;
  paramInfo->firstReadDone=true;

  asynStatus stat =asynSuccess;
  if(updateAsynPar){
    stat=adsUpdateParameterLock(paramInfo,(const void *)data,bytesRead);
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
asynStatus adsAsynPortDriver::adsReadStateLock(uint16_t amsport,uint16_t *adsState,bool blockErrorMsg)
{
  asynStatus stat;
  lock();
  long error=0;
  stat=adsReadState(amsport,adsState,blockErrorMsg,&error);
  unlock();
  return stat;
}

/** Read state of amsport in TwinCAT
 *
 * \param[in] amsport Ams-prot.
 * \param[out] adsState State of ams-port (running, invalid, config..).
 * \param[in] blockErrorMsg Suppress error messages
 *            (used while trying to reconnect to avoid alot of error messages).
 * \param[out] error Error code.
 *
 * Thread safe.
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReadStateLock(uint16_t amsport,uint16_t *adsState,bool blockErrorMsg,long *error)
{
  asynStatus stat;
  lock();
  stat=adsReadState(amsport,adsState,blockErrorMsg,error);
  unlock();
  return stat;
}

/** Read state of default amsport in TwinCAT
 *
 * \param[out] adsState State of ams-port (running, invalid, config..).
 *
 * \return asynSuccess or asynError.
 */
asynStatus adsAsynPortDriver::adsReadState(uint16_t *adsState)
{
  long error=0;
  return adsReadState(amsportDefault_,adsState,false,&error);
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
asynStatus adsAsynPortDriver::adsReadState(uint16_t amsport,uint16_t *adsState,bool blockErrorMsg,long *error)
{

  const char* functionName = "adsReadState";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  AmsAddr amsServer={remoteNetId_,amsport};

  uint16_t devState;
  adsLock();
  const long status = AdsSyncReadStateReqEx(adsPort_, &amsServer, adsState, &devState);
  *error=status;
  adsUnlock();
  if (status) {
    if(!blockErrorMsg){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: ADS read state failed with: %s (0x%lx)\n",driverName, functionName,adsErrorToString(status),status);
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
asynStatus adsAsynPortDriver::adsWriteState(uint16_t amsport,uint16_t adsState)
{
  const char* functionName = "adsWriteState";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: adsState = %s (%u)\n", driverName, functionName,adsStateToString(adsState),adsState);

  void *pData=NULL;
  AmsAddr amsServer={remoteNetId_,amsport};
  const long status = AdsSyncWriteControlReqEx (adsPort_,&amsServer, adsState, 0, 0, pData);
  if(status){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: ADS write state failed with: %s (0x%lx)\n",driverName, functionName,adsErrorToString(status),status);
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
adsParamInfo *adsAsynPortDriver::getAdsParamInfo(int index)
{
  const char* functionName = "getAdsParamInfo";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: Get paramInfo for index: %d\n", driverName, functionName,index);

  if(index<adsParamArrayCount_){
    return pAdsParamArray_[index];
  }
  else{
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
asynStatus adsAsynPortDriver::refreshParamTime(adsParamInfo *paramInfo)
{
  const char* functionName = "refreshParamTime";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: plcTime %" PRIuMAX ".\n", driverName, functionName,
            (uintmax_t)paramInfo->plcTimeStampRaw);

  //Convert plc timeStamp (windows format) to epicsTimeStamp
  if(windowsToEpicsTimeStamp(paramInfo->plcTimeStampRaw,&paramInfo->plcTimeStamp)){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: windowsToEpicsTimeStamp() failed.\n", driverName, functionName);
    return asynError;
  }

  epicsTimeStamp ts;

  //Update time stamp
  if(paramInfo->timeBase==ADS_TIME_BASE_EPICS || paramInfo->plcTimeStampRaw==0){
    if(updateTimeStamp()!=asynSuccess){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: updateTimeStamp() failed.\n", driverName, functionName);
      return asynError;
    }
  }
  else{ //ADS_TIME_BASE_PLC
    if(setTimeStamp(&paramInfo->plcTimeStamp)!=asynSuccess){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: updateTimeStamp() failed.\n", driverName, functionName);
      return asynError;
    }
  }

  if(getTimeStamp(&ts)!=asynSuccess){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: getTimeStamp() failed.\n", driverName, functionName);
    return asynError;
  }

  paramInfo->epicsTimestamp=ts;

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
asynStatus adsAsynPortDriver::adsUpdateParameterLock(adsParamInfo* paramInfo,const void *data)
{
  lock();
  asynStatus stat=adsUpdateParameter(paramInfo,data);
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
asynStatus adsAsynPortDriver::adsUpdateParameterLock(adsParamInfo* paramInfo,const void *data,size_t dataSize)
{
  lock();
  asynStatus stat=adsUpdateParameter(paramInfo,data,dataSize);
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
asynStatus adsAsynPortDriver::adsUpdateParameter(adsParamInfo* paramInfo,const void *data)
{
  return adsUpdateParameter(paramInfo,data,paramInfo->lastCallbackSize);
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
asynStatus adsAsynPortDriver::adsUpdateParameter(adsParamInfo* paramInfo,const void *data,size_t dataSize)
{
  const char* functionName = "adsUpdateParameter";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  if(!paramInfo){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: paramInfo NULL.\n", driverName, functionName);
    return asynError;
  }

  if(!data){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: data NULL.\n", driverName, functionName);
    return asynError;
  }

  if(refreshParamTime(paramInfo)!=asynSuccess){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: refreshParamTime() failed.\n", driverName, functionName);
    return asynError;
  }

  asynStatus ret=asynError;

  //Ensure check if array
  if(paramInfo->plcDataIsArray){
    if(!paramInfo->arrayDataBuffer){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Array but buffer is NULL.\n", driverName, functionName);
      return asynError;
    }
    //Copy data to param buffer
    memcpy(paramInfo->arrayDataBuffer,data,paramInfo->lastCallbackSize);
  }

  switch(paramInfo->plcDataType){
    case ADST_INT8:
      int8_t *ADST_INT8Var;
      ADST_INT8Var=((int8_t*)data);
      //Asyn types
      switch(paramInfo->asynType){
        case asynParamInt32:
          ret=setIntegerParam(paramInfo->paramIndex,(int)(*ADST_INT8Var));
          break;
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
        case asynParamInt64:
          ret=setInteger64Param(paramInfo->paramIndex,(epicsInt64)(*ADST_INT8Var));
          break;
#endif
        case asynParamFloat64:
          ret=setDoubleParam(paramInfo->paramIndex,(double)(*ADST_INT8Var));
          break;
        case asynParamInt8Array:
          // handled in fireCallbacks()
          ret=asynSuccess;
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;

    case ADST_INT16:
      int16_t *ADST_INT16Var;
      ADST_INT16Var=((int16_t*)data);
      //Asyn types
      switch(paramInfo->asynType){
        case asynParamInt32:
          ret=setIntegerParam(paramInfo->paramIndex,(int)(*ADST_INT16Var));
          break;
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
        case asynParamInt64:
          ret=setInteger64Param(paramInfo->paramIndex,(int)(*ADST_INT16Var));
          break;
#endif
        case asynParamFloat64:
          ret=setDoubleParam(paramInfo->paramIndex,(double)(*ADST_INT16Var));
          break;
        case asynParamInt16Array:
          // handled in fireCallbacks()
          ret=asynSuccess;
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;
    case ADST_INT32:
      int32_t *ADST_INT32Var;
      ADST_INT32Var=((int32_t*)data);
      //Asyn types
      switch(paramInfo->asynType){
        case asynParamInt32:
          ret=setIntegerParam(paramInfo->paramIndex,(int)(*ADST_INT32Var));
          break;
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
        case asynParamInt64:
          ret=setInteger64Param(paramInfo->paramIndex,(int)(*ADST_INT32Var));
          break;
#endif
        case asynParamFloat64:
          ret=setDoubleParam(paramInfo->paramIndex,(double)(*ADST_INT32Var));
          break;
        case asynParamInt32Array:
          // handled in fireCallbacks()
          ret=asynSuccess;
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;
    case ADST_INT64:
      int64_t *ADST_INT64Var;
      ADST_INT64Var=((int64_t*)data);
      //Asyn types
      switch(paramInfo->asynType){
        case asynParamInt32:
          ret=setIntegerParam(paramInfo->paramIndex,(int)(*ADST_INT64Var));
          break;
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
        case asynParamInt64:
          ret=setInteger64Param(paramInfo->paramIndex,(epicsInt64)(*ADST_INT64Var));
          break;
#endif
        case asynParamFloat64:
          ret=setDoubleParam(paramInfo->paramIndex,(double)(*ADST_INT64Var));
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;
    case ADST_UINT8:
      uint8_t *ADST_UINT8Var;
      ADST_UINT8Var=((uint8_t*)data);
      //Asyn types
      switch(paramInfo->asynType){
        case asynParamInt32:
          ret=setIntegerParam(paramInfo->paramIndex,(int)(*ADST_UINT8Var));
          break;
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
        case asynParamInt64:
          ret=setInteger64Param(paramInfo->paramIndex,(epicsInt64)(*ADST_UINT8Var));
          break;
#endif
        case asynParamFloat64:
          ret=setDoubleParam(paramInfo->paramIndex,(double)(*ADST_UINT8Var));
          break;
        // Arrays of unsigned not supported
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;
    case ADST_UINT16:
      uint16_t *ADST_UINT16Var;
      ADST_UINT16Var=((uint16_t*)data);
      //Asyn types
      switch(paramInfo->asynType){
        case asynParamInt32:
          ret=setIntegerParam(paramInfo->paramIndex,(int)(*ADST_UINT16Var));
          break;
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
        case asynParamInt64:
          ret=setInteger64Param(paramInfo->paramIndex,(epicsInt64)(*ADST_UINT16Var));
          break;
#endif
        case asynParamFloat64:
          ret=setDoubleParam(paramInfo->paramIndex,(double)(*ADST_UINT16Var));
          break;
        // Arrays of unsigned not supported
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;
    case ADST_UINT32:
      uint32_t *ADST_UINT32Var;
      ADST_UINT32Var=((uint32_t*)data);
      //Asyn types
      switch(paramInfo->asynType){
        case asynParamInt32:
          ret=setIntegerParam(paramInfo->paramIndex,(int)(*ADST_UINT32Var));
          break;
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
        case asynParamInt64:
          ret=setInteger64Param(paramInfo->paramIndex,(epicsInt64)(*ADST_UINT32Var));
          break;
#endif
        case asynParamFloat64:
          ret=setDoubleParam(paramInfo->paramIndex,(double)(*ADST_UINT32Var));
          break;
        // Arrays of unsigned not supported
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;
    case ADST_UINT64:
      uint64_t *ADST_UINT64Var;
      ADST_UINT64Var=((uint64_t*)data);
      switch(paramInfo->asynType){
        case asynParamInt32:
          ret=setIntegerParam(paramInfo->paramIndex,(int)(*ADST_UINT64Var));
          break;
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
        case asynParamInt64:
          ret=setInteger64Param(paramInfo->paramIndex,(epicsInt64)(*ADST_UINT64Var));
          break;
#endif
        case asynParamFloat64:
          ret=setDoubleParam(paramInfo->paramIndex,(double)(*ADST_UINT64Var));
          break;
        // Arrays of unsigned not supported
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;
    case ADST_REAL32:
      float *ADST_REAL32Var;
      ADST_REAL32Var=((float*)data);
      switch(paramInfo->asynType){
        case asynParamInt32:
          ret=setIntegerParam(paramInfo->paramIndex,(int)(*ADST_REAL32Var));
          break;
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
        case asynParamInt64:
          ret=setInteger64Param(paramInfo->paramIndex,(epicsInt64)(*ADST_REAL32Var));
          break;
#endif
        case asynParamFloat64:
          ret=setDoubleParam(paramInfo->paramIndex,(double)(*ADST_REAL32Var));
          break;
        case asynParamFloat32Array:
          // handled in fireCallbacks()
          ret=asynSuccess;
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;
    case ADST_REAL64:
      double *ADST_REAL64Var;
      ADST_REAL64Var=((double*)data);
      switch(paramInfo->asynType){
        case asynParamInt32:
          ret=setIntegerParam(paramInfo->paramIndex,(int)(*ADST_REAL64Var));
          break;
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
        case asynParamInt64:
          ret=setInteger64Param(paramInfo->paramIndex,(epicsInt64)(*ADST_REAL64Var));
          break;
#endif
        case asynParamFloat64:
          ret=setDoubleParam(paramInfo->paramIndex,(double)(*ADST_REAL64Var));
          break;
        case asynParamFloat64Array:
          // handled in fireCallbacks()
          ret=asynSuccess;
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;

    case ADST_BIT:
      int8_t *ADST_BitVar;
      ADST_BitVar=((int8_t*)data);
       switch(paramInfo->asynType){
        case asynParamInt32:
          ret=setIntegerParam(paramInfo->paramIndex,(int)(*ADST_BitVar));
          break;
        case asynParamFloat64:
          ret=setDoubleParam(paramInfo->paramIndex,(double)(*ADST_BitVar));
          break;
        case asynParamInt8Array:
          // handled in fireCallbacks()
          ret=asynSuccess;
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;
     case ADST_STRING:
      switch(paramInfo->asynType){
        case asynParamInt8Array:
          // handled in fireCallbacks()
          ret=asynSuccess;
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
        }
      break;
    default:
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
      return asynError;
      break;
  }

  if(ret!=asynSuccess){
    return ret;
  }

  ret=setAlarmParam(paramInfo,NO_ALARM, NO_ALARM);
  if(ret!=asynSuccess){
    return ret;
  }

  if(allowCallbackEpicsState){
    return fireCallbacks(paramInfo);
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
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  lock();
  for(int i=0;i<adsParamArrayCount_;i++){
    if(pAdsParamArray_[i]){
      fireCallbacks(pAdsParamArray_[i]);
    }
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
asynStatus adsAsynPortDriver::fireCallbacks(adsParamInfo* paramInfo)
{
  const char* functionName = "fireCallbacks";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  if(!paramInfo->plcDataIsArray){
    return callParamCallbacks();
  }

  if(paramInfo->lastCallbackSize<=0){
    return asynSuccess;
  }

  asynStatus ret=asynError;

  //Array
  switch(paramInfo->plcDataType){
    case ADST_INT8:
      switch(paramInfo->asynType){
        case asynParamInt8Array:
          ret=doCallbacksInt8Array((epicsInt8 *)paramInfo->arrayDataBuffer,paramInfo->lastCallbackSize, paramInfo->paramIndex,paramInfo->asynAddr);
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;

    case ADST_INT16:
      switch(paramInfo->asynType){
        case asynParamInt16Array:
          ret=doCallbacksInt16Array((epicsInt16 *)paramInfo->arrayDataBuffer,paramInfo->lastCallbackSize, paramInfo->paramIndex,paramInfo->asynAddr);
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;
    case ADST_INT32:
      switch(paramInfo->asynType){
        case asynParamInt32Array:
          ret=doCallbacksInt32Array((epicsInt32 *)paramInfo->arrayDataBuffer,paramInfo->lastCallbackSize, paramInfo->paramIndex,paramInfo->asynAddr);
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;

    case ADST_REAL32:
      switch(paramInfo->asynType){
        case asynParamFloat32Array:
          ret=doCallbacksFloat32Array((epicsFloat32 *)paramInfo->arrayDataBuffer,paramInfo->lastCallbackSize, paramInfo->paramIndex,paramInfo->asynAddr);
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;

    case ADST_REAL64:
      switch(paramInfo->asynType){
        case asynParamFloat64Array:
          ret=doCallbacksFloat64Array((epicsFloat64 *)paramInfo->arrayDataBuffer,paramInfo->lastCallbackSize, paramInfo->paramIndex,paramInfo->asynAddr);
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;

    case ADST_BIT:
      switch(paramInfo->asynType){
        case asynParamInt8Array:
          ret=doCallbacksInt8Array((epicsInt8 *) paramInfo->arrayDataBuffer,paramInfo->lastCallbackSize, paramInfo->paramIndex,paramInfo->asynAddr);
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
      }
      break;
    case ADST_STRING:
      switch(paramInfo->asynType){
        case asynParamInt8Array:
          ret=doCallbacksInt8Array((epicsInt8 *) paramInfo->arrayDataBuffer,paramInfo->lastCallbackSize, paramInfo->paramIndex,paramInfo->asynAddr);
          break;
        default:
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
          return asynError;
          break;
        }
      break;

    default:
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Type combination not supported. PLC type = %s, ASYN type= %s\n", driverName, functionName,adsTypeToString(paramInfo->plcDataType),asynTypeToString(paramInfo->asynType));
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
asynStatus adsAsynPortDriver::setAlarmParam(adsParamInfo *paramInfo,int alarm,int severity)
{
  const char* functionName = "setAlarmParam";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  if(!paramInfo){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: paramInfo==NULL.\n", driverName, functionName);
    return asynError;
  }

  asynStatus stat;
  int oldAlarmStatus=0;
  stat=getParamAlarmStatus(paramInfo->paramIndex,&oldAlarmStatus);
  if(stat!=asynSuccess){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: getParamAlarmStatus failed for parameter %s (%d).\n", driverName, functionName,paramInfo->drvInfo,paramInfo->paramIndex);
    return asynError;
  }

  bool doCallbacks=false;

  if(oldAlarmStatus!=alarm){
    stat=setParamAlarmStatus(paramInfo->paramIndex,alarm);
    if(stat!=asynSuccess){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed set alarm status for parameter %s (%d).\n", driverName, functionName,paramInfo->drvInfo,paramInfo->paramIndex);
      return asynError;
    }
    paramInfo->alarmStatus=alarm;
    doCallbacks=true;
  }

  int oldAlarmSeverity=0;
  stat=getParamAlarmSeverity(paramInfo->paramIndex,&oldAlarmSeverity);
  if(stat!=asynSuccess){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: getParamAlarmStatus failed for parameter %s (%d).\n", driverName, functionName,paramInfo->drvInfo,paramInfo->paramIndex);
    return asynError;
  }

  if(oldAlarmSeverity!=severity){
    stat=setParamAlarmSeverity(paramInfo->paramIndex,severity);
    if(stat!=asynSuccess){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed set alarm severity for parameter %s (%d).\n", driverName, functionName,paramInfo->drvInfo,paramInfo->paramIndex);
      return asynError;
    }
    paramInfo->alarmSeverity=severity;
    doCallbacks=true;
  }

  if(!doCallbacks || !allowCallbackEpicsState){
    return asynSuccess;
  }
  //Write size used for arrays
  size_t writeSize=paramInfo->arrayDataBufferSize;
  if(paramInfo->plcSize<writeSize){
    writeSize=paramInfo->plcSize;
  }

  //Alarm status or severity changed=>Do callbacks with old buffered data (if nElemnts==0 then no data in record...)
  if(paramInfo->plcDataIsArray && paramInfo->arrayDataBuffer and paramInfo->arrayDataBufferSize>0 && allowCallbackEpicsState){
    switch(paramInfo->asynType){
      case asynParamInt8Array:
        stat=doCallbacksInt8Array((epicsInt8 *)paramInfo->arrayDataBuffer,writeSize/sizeof(epicsInt8), paramInfo->paramIndex,paramInfo->asynAddr);
        break;
      case asynParamInt16Array:
        stat=doCallbacksInt16Array((epicsInt16 *)paramInfo->arrayDataBuffer,writeSize/sizeof(epicsInt16), paramInfo->paramIndex,paramInfo->asynAddr);
        break;
      case asynParamInt32Array:
        stat=doCallbacksInt32Array((epicsInt32 *)paramInfo->arrayDataBuffer,writeSize/sizeof(epicsInt32), paramInfo->paramIndex,paramInfo->asynAddr);
        break;
      case asynParamFloat32Array:
        stat=doCallbacksFloat32Array((epicsFloat32 *)paramInfo->arrayDataBuffer,writeSize/sizeof(epicsFloat32), paramInfo->paramIndex,paramInfo->asynAddr);
        break;
      case asynParamFloat64Array:
        stat=doCallbacksFloat64Array((epicsFloat64 *)paramInfo->arrayDataBuffer,writeSize/sizeof(epicsFloat64), paramInfo->paramIndex,paramInfo->asynAddr);
        break;
      default:
        stat=callParamCallbacks();
        break;
    }
  }
  else{
      stat=callParamCallbacks();
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
asynStatus adsAsynPortDriver::setAlarmPortLock(uint16_t amsPort,int alarm,int severity)
{
  asynStatus stat;
  lock();
  stat=setAlarmPort(amsPort,alarm,severity);
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
asynStatus adsAsynPortDriver::setAlarmPort(uint16_t amsPort,int alarm,int severity)
{
  const char* functionName = "setAlarmPort";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  for(int i=0;i<adsParamArrayCount_;i++){
    if(!pAdsParamArray_[i]){
      continue;
    }
    if(pAdsParamArray_[i]->amsPort==amsPort){
      if(setAlarmParam(pAdsParamArray_[i],alarm,severity)!=asynSuccess){
        return asynError;
      }
    }
  }
  return asynSuccess;
}

/** Take adsLib lock.
 */
void adsAsynPortDriver::adsLock()
{
  adsMutex.lock();
}

/** Release adsLib lock.
 */
void adsAsynPortDriver::adsUnlock()
{
  adsMutex.unlock();
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
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: force = %s\n", driverName, functionName,force ? "true" : "false");
  if(routeAdded_ || force){
    routeAdded_=0;
    AdsDelRoute(remoteNetId_);
  }
  return asynSuccess;
}

/** Delete ads route
 *
 * \param[in] force Force delete.
 *
 * \return asynSuccess or asynError.
 *
 * Thread safe.
 */
asynStatus adsAsynPortDriver::adsDelRouteLock(int force)
{
  adsLock();
  asynStatus stat=adsDelRoute(force);
  adsUnlock();
  return stat;
}

/** Add ads route
 *
 * \return asynSuccess or asynError.
 *
 * Thread safe.
 */
asynStatus adsAsynPortDriver::adsAddRouteLock()
{
  const char* functionName = "adsAddRouteLock";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  // add local route to your ADS Master
  adsLock();
  const long addRouteStatus = AdsAddRoute(remoteNetId_, ipaddr_);
  adsUnlock();
  if(addRouteStatus){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Adding ADS route failed with: %s (0x%lx).\n", driverName, functionName,adsErrorToString(addRouteStatus),addRouteStatus);
    return asynError;
  }
  routeAdded_=1;
  return asynSuccess;
}


/* Configuration routine.  Called directly, or from the iocsh function below */

extern "C" {

  asynUser *pPrintOutAsynUser;

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
    printf(" 7. default sample time ms                     : 500 (check if variable changed each 500ms)\n");
    printf(" 8. max delay time ms (buffer time in plc)     : 1000 (if changed, send data atleast each 1000ms or faster if send buffer is full)\n");
    printf(" 9. ADS command timeout in ms                 : 1000 (timeout for adsLib commands)\n");
    printf(" 10. default time source (PLC=0,EPICS=1).      : 0 (PLC) NOTE: record TSE field need to be set to -2 for timestamp in asyn (field(TSE, -2))\n");
    printf("\n");
    printf(" Resulting adsAsynPortDriverConfigure() command: \n");
    printf(" adsAsynPortDriverConfigure(\"ADS_1\",\"192.168.88.44\",\"192.168.88.44.1.1\",851,1000,0,0,50,100,1000,0)\n");
    printf("\n");
    printf("\n");
    printf(" NOTE: An ADS route needs to be added to the TwinCAT router of the controller/PLC:\n");
    printf("       1. \"TwinCAT->System->Routes->Static Routes\": Press \"Add\" button.\n");
    printf("       2. \"Route Name (Target)\": Enter name of EPICS machine.\n");
    printf("       3. \"AMSNetId\": Enter IP of EPICS machine. Add \".1.1\" in the end (x.x.x.x.1.1).\n");
    printf("       4. \"Address Info\": Enter IP of EPICS machine (x.x.x.x).\n");
    printf("       5. Choose \"IP Address\" checkbox.\n");
    printf("       6. Choose \"Remote Route\"->\"None\" checkbox.\n");
    printf("       7. Press \"Add Route\" button.\n");
    printf("       8. Close \"Add Route Dialog\".\n");
    printf("       9. Ensure that the route was successfully added in the \"Static Routes\" list.\n");
    printf("\n");

    return;
  }
  /*
   * Configure and register
   */
  epicsShareFunc int
  adsAsynPortDriverConfigure(const char *portName,
                             const char *ipaddr,
                             const char *amsaddr,
                             unsigned int amsport,
                             unsigned int asynParamTableSize,
                             unsigned int priority,
                             int noAutoConnect,
                             int defaultSampleTimeMS,
                             int maxDelayTimeMS,
                             int adsTimeoutMS,
                             int defaultTimeSource)
  {

    if (!portName) {
      printHelp();
      return -1;
    }

    if(strlen(portName)==0 || strcmp(portName,"-h")==0){
      printHelp();
      return -1;
    }

    if (!ipaddr) {
      printf("adsAsynPortDriverConfigure bad ipaddr: %s\n",ipaddr ? ipaddr : "");
      return -1;
    }
    if (!amsaddr) {
      printf("adsAsynPortDriverConfigure bad amsaddr: %s\n",amsaddr ? amsaddr : "");
      return -1;
    }
    if (defaultSampleTimeMS<0) {
      printf("adsAsynPortDriverConfigure bad defaultSampleTimeMS: %dms. Standard value of 100ms will be used.\n",defaultSampleTimeMS);
      defaultSampleTimeMS=100;
    }

    if (maxDelayTimeMS <= 0) {
      printf("adsAsynPortDriverConfigure bad maxDelayTimeMS: %dms. Standard value of 500ms will be used.\n",maxDelayTimeMS);
      maxDelayTimeMS=500;
    }

    if (adsTimeoutMS <= 0) {
      printf("adsAsynPortDriverConfigure bad adsTimeoutMS: %dms. Standard value of 2000ms will be used.\n",adsTimeoutMS);
      adsTimeoutMS=2000;
    }

    if(defaultTimeSource<0 || defaultTimeSource>=ADS_TIME_BASE_MAX){
      printf("adsAsynPortDriverConfigure bad default time source: %d. PLC time stamps will be used. Valid options are: PLC=%d and EPICS=%d.\n",defaultTimeSource,(int)ADS_TIME_BASE_PLC,(int)ADS_TIME_BASE_EPICS);
      defaultTimeSource=ADS_TIME_BASE_PLC;
    }

    adsAsynPortObj=new adsAsynPortDriver(portName,
                                         ipaddr,
                                         amsaddr,
                                         amsport,
                                         asynParamTableSize,
                                         priority,
                                         noAutoConnect==0,
                                         defaultSampleTimeMS,
                                         maxDelayTimeMS,
                                         adsTimeoutMS,
                                         (ADSTIMESOURCE)defaultTimeSource);
    if(adsAsynPortObj){
      asynUser *traceUser= adsAsynPortObj->getTraceAsynUser();
      if(!traceUser){
        printf("adsAsynPortDriverConfigure: ERROR: Failed to retrieve asynUser for trace. \n");
        return (asynError);
      }
      pPrintOutAsynUser=traceUser;
    }

    initHook();

    return asynSuccess;
  }

  /*
   * IOC shell command registration
   */
  static const iocshArg adsAsynPortDriverConfigureArg0 = { "port name",iocshArgString};
  static const iocshArg adsAsynPortDriverConfigureArg1 = { "ip-addr",iocshArgString};
  static const iocshArg adsAsynPortDriverConfigureArg2 = { "ams-addr",iocshArgString};
  static const iocshArg adsAsynPortDriverConfigureArg3 = { "default-ams-port",iocshArgInt};
  static const iocshArg adsAsynPortDriverConfigureArg4 = { "asyn param table size",iocshArgInt};
  static const iocshArg adsAsynPortDriverConfigureArg5 = { "priority",iocshArgInt};
  static const iocshArg adsAsynPortDriverConfigureArg6 = { "disable auto-connect",iocshArgInt};
  static const iocshArg adsAsynPortDriverConfigureArg7 = { "default sample time ms",iocshArgInt};
  static const iocshArg adsAsynPortDriverConfigureArg8 = { "max delay time ms",iocshArgInt};
  static const iocshArg adsAsynPortDriverConfigureArg9 = { "ADS communication timeout ms",iocshArgInt};
  static const iocshArg adsAsynPortDriverConfigureArg10 = { "default time source (EPCIS=0,PLC=1)",iocshArgInt};
  static const iocshArg *adsAsynPortDriverConfigureArgs[] = {
    &adsAsynPortDriverConfigureArg0, &adsAsynPortDriverConfigureArg1,
    &adsAsynPortDriverConfigureArg2, &adsAsynPortDriverConfigureArg3,
    &adsAsynPortDriverConfigureArg4, &adsAsynPortDriverConfigureArg5,
    &adsAsynPortDriverConfigureArg6, &adsAsynPortDriverConfigureArg7,
    &adsAsynPortDriverConfigureArg8,&adsAsynPortDriverConfigureArg9,
    &adsAsynPortDriverConfigureArg10};

  static const iocshFuncDef adsAsynPortDriverConfigureFuncDef =
    {"adsAsynPortDriverConfigure",11,adsAsynPortDriverConfigureArgs};

  static void adsAsynPortDriverConfigureCallFunc(const iocshArgBuf *args)
  {
    adsAsynPortDriverConfigure(args[0].sval,args[1].sval,args[2].sval,args[3].ival, args[4].ival, args[5].ival,args[6].ival,args[7].ival,args[8].ival,args[9].ival,args[10].ival);
  }

  /*
   * adsSetLocalAddress("ams_net_id")
   */
  static const iocshArg adsSetLocalAddressArg0 = {"local_ams_id", iocshArgString};
  static const iocshArg *adsSetLocalAddressArgs[] = {&adsSetLocalAddressArg0};
  static const iocshFuncDef adsSetLocalAddressFuncDef = {"adsSetLocalAddress",1,adsSetLocalAddressArgs};

  static void adsSetLocalAddressCallFunc(const iocshArgBuf *args)
  {
    const char* functionName = "adsSetLocalAddress";
    if (!args[0].sval || strlen(args[0].sval) < 11) {
        printf("%s:%s: local_ams_id parameter required (of the form A.B.C.D.E.F)\n", driverName, functionName);
        return;
    }
    printf("%s:%s: Setting local AMS Net ID to: %s\n", driverName, functionName, args[0].sval);
    AdsSetLocalAddress(std::string(args[0].sval));
  }

  /*
   * adsPollInfo("name")
   */
  static const iocshArg adsPollInfoArg0 = {"name", iocshArgString};
  static const iocshArg *adsPollInfoArgs[] = {&adsPollInfoArg0};
  static const iocshFuncDef adsPollInfoFuncDef = {"adsPollInfo",1,adsPollInfoArgs};

  static void adsPollInfoCallFunc(const iocshArgBuf *args)
  {
    adsAsynPortObj->poll_info(args[0].sval);
  }

  /*
   * This routine is called before multitasking has started, so there's
   * no race condition in the test/set of firstTime.
   */

  static void adsAsynPortDriverRegister(void)
  {
    iocshRegister(&adsAsynPortDriverConfigureFuncDef,adsAsynPortDriverConfigureCallFunc);
    iocshRegister(&adsSetLocalAddressFuncDef,adsSetLocalAddressCallFunc);
    iocshRegister(&adsPollInfoFuncDef, adsPollInfoCallFunc);
  }

  epicsExportRegistrar(adsAsynPortDriverRegister);
}
