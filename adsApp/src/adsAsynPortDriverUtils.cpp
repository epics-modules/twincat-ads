/*
    This file is part of epics-twincat-ads.

    epics-twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

    epics-twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License along with epics-twincat-ads. If not, see <https://www.gnu.org/licenses/>.

*/
/*
* adsAsynPortDriverUtils.cpp
*
* Utilities and definitions used by adsAsynPortDriver-class.
*
* Author: Anders Sandström
*
* Created January 30, 2018
*/

#include "adsAsynPortDriverUtils.h"
#include <string.h>
#include <stdlib.h>
#include <initHooks.h>
#include "epicsTime.h"

typedef struct {
    char bEnable;
    char bReset;
    char bExecute;
    uint16_t nCommand;
    uint16_t nCmdData;
    double fVelocity;
    double fPosition;
    double fAcceleration;
    double fDeceleration;
    char bJogFwd;
    char bJogBwd;
    char bLimitFwd;
    char bLimitBwd;
    double fOverride;
    char bHomeSensor;
    char bEnabled;
    char bError;
    uint32_t nErrorId;
    double fActVelocity;
    double fActPosition;
    double fActDiff;
    char bHomed;
    char bBusy;
  } adsOctetSTAXISSTATUSSTRUCT;

#define RETURN_VAR_NAME_IF_NEEDED                              \
  do {                                                         \
    if(returnVarName){                                         \
      octetCmdBuf_printf(asciiBuffer,"%s=",info->variableName);    \
    }                                                          \
  }                                                            \
  while(0)

/** Convert ADS error code to string.
 *
 * \param[in] error Ads error code (from adsLib https://github.com/Beckhoff/ADS)
 *
 * \return Error string.
 */
const char *adsErrorToString(long error)
{
  switch (error) {
    case GLOBALERR_TARGET_PORT:
      return "GLOBALERR_TARGET_PORT";
    case GLOBALERR_MISSING_ROUTE:
      return "GLOBALERR_MISSING_ROUTE";
    case GLOBALERR_NO_MEMORY:
      return "GLOBALERR_NO_MEMORY";
    case GLOBALERR_TCP_SEND:
      return "GLOBALERR_TCP_SEND";
    case ADSERR_DEVICE_ERROR:
      return "ADSERR_DEVICE_ERROR";
    case ADSERR_DEVICE_SRVNOTSUPP:
      return "ADSERR_DEVICE_SRVNOTSUPP";
    case ADSERR_DEVICE_INVALIDGRP:
      return "ADSERR_DEVICE_INVALIDGRP";
    case ADSERR_DEVICE_INVALIDOFFSET:
      return "ADSERR_DEVICE_INVALIDOFFSET";
    case ADSERR_DEVICE_INVALIDACCESS:
      return "ADSERR_DEVICE_INVALIDACCESS";
    case ADSERR_DEVICE_INVALIDSIZE:
      return "ADSERR_DEVICE_INVALIDSIZE";
    case ADSERR_DEVICE_INVALIDDATA:
      return "ADSERR_DEVICE_INVALIDDATA";
    case ADSERR_DEVICE_NOTREADY:
      return "ADSERR_DEVICE_NOTREADY";
    case ADSERR_DEVICE_BUSY:
      return "ADSERR_DEVICE_BUSY";
    case ADSERR_DEVICE_INVALIDCONTEXT:
      return "ADSERR_DEVICE_INVALIDCONTEXT";
    case ADSERR_DEVICE_NOMEMORY:
      return "ADSERR_DEVICE_NOMEMORY";
    case ADSERR_DEVICE_INVALIDPARM:
      return "ADSERR_DEVICE_INVALIDPARM";
    case ADSERR_DEVICE_NOTFOUND:
      return "ADSERR_DEVICE_NOTFOUND";
    case ADSERR_DEVICE_SYNTAX:
      return "ADSERR_DEVICE_SYNTAX";
    case ADSERR_DEVICE_INCOMPATIBLE:
      return "ADSERR_DEVICE_INCOMPATIBLE";
    case ADSERR_DEVICE_EXISTS:
      return "ADSERR_DEVICE_EXISTS";
    case ADSERR_DEVICE_SYMBOLNOTFOUND:
      return "ADSERR_DEVICE_SYMBOLNOTFOUND";
    case ADSERR_DEVICE_SYMBOLVERSIONINVALID:
      return "ADSERR_DEVICE_SYMBOLVERSIONINVALID";
    case ADSERR_DEVICE_INVALIDSTATE:
      return "ADSERR_DEVICE_INVALIDSTATE";
    case ADSERR_DEVICE_TRANSMODENOTSUPP:
      return "ADSERR_DEVICE_TRANSMODENOTSUPP";
    case ADSERR_DEVICE_NOTIFYHNDINVALID:
      return "ADSERR_DEVICE_NOTIFYHNDINVALID";
    case ADSERR_DEVICE_CLIENTUNKNOWN:
      return "ADSERR_DEVICE_CLIENTUNKNOWN";
    case ADSERR_DEVICE_NOMOREHDLS:
      return "ADSERR_DEVICE_NOMOREHDLS";
    case ADSERR_DEVICE_INVALIDWATCHSIZE:
      return "ADSERR_DEVICE_INVALIDWATCHSIZE";
    case ADSERR_DEVICE_NOTINIT:
      return "ADSERR_DEVICE_NOTINIT";
    case ADSERR_DEVICE_TIMEOUT:
      return "ADSERR_DEVICE_TIMEOUT";
    case ADSERR_DEVICE_NOINTERFACE:
      return "ADSERR_DEVICE_NOINTERFACE";
    case ADSERR_DEVICE_INVALIDINTERFACE:
      return "ADSERR_DEVICE_INVALIDINTERFACE";
    case ADSERR_DEVICE_INVALIDCLSID:
      return "ADSERR_DEVICE_INVALIDCLSID";
    case ADSERR_DEVICE_INVALIDOBJID:
      return "ADSERR_DEVICE_INVALIDOBJID";
    case ADSERR_DEVICE_PENDING:
      return "ADSERR_DEVICE_PENDING";
    case ADSERR_DEVICE_ABORTED:
      return "ADSERR_DEVICE_ABORTED";
    case ADSERR_DEVICE_WARNING:
      return "ADSERR_DEVICE_WARNING";
    case ADSERR_DEVICE_INVALIDARRAYIDX:
      return "ADSERR_DEVICE_INVALIDARRAYIDX";
    case ADSERR_DEVICE_SYMBOLNOTACTIVE:
      return "ADSERR_DEVICE_SYMBOLNOTACTIVE";
    case ADSERR_DEVICE_ACCESSDENIED:
      return "ADSERR_DEVICE_ACCESSDENIED";
    case ADSERR_DEVICE_LICENSENOTFOUND:
      return "ADSERR_DEVICE_LICENSENOTFOUND";
    case ADSERR_DEVICE_LICENSEEXPIRED:
      return "ADSERR_DEVICE_LICENSEEXPIRED";
    case ADSERR_DEVICE_LICENSEEXCEEDED:
      return "ADSERR_DEVICE_LICENSEEXCEEDED";
    case ADSERR_DEVICE_LICENSEINVALID:
      return "ADSERR_DEVICE_LICENSEINVALID";
    case ADSERR_DEVICE_LICENSESYSTEMID:
      return "ADSERR_DEVICE_LICENSESYSTEMID";
    case ADSERR_DEVICE_LICENSENOTIMELIMIT:
      return "ADSERR_DEVICE_LICENSENOTIMELIMIT";
    case ADSERR_DEVICE_LICENSEFUTUREISSUE:
      return "ADSERR_DEVICE_LICENSEFUTUREISSUE";
    case ADSERR_DEVICE_LICENSETIMETOLONG:
      return "ADSERR_DEVICE_LICENSETIMETOLONG";
    case ADSERR_DEVICE_EXCEPTION:
      return "ADSERR_DEVICE_EXCEPTION";
    case ADSERR_DEVICE_LICENSEDUPLICATED:
      return "ADSERR_DEVICE_LICENSEDUPLICATED";
    case ADSERR_DEVICE_SIGNATUREINVALID:
      return "ADSERR_DEVICE_SIGNATUREINVALID";
    case ADSERR_DEVICE_CERTIFICATEINVALID:
      return "ADSERR_DEVICE_CERTIFICATEINVALID";
    case ADSERR_CLIENT_ERROR:
      return "ADSERR_CLIENT_ERROR";
    case ADSERR_CLIENT_INVALIDPARM:
      return "ADSERR_CLIENT_INVALIDPARM";
    case ADSERR_CLIENT_LISTEMPTY:
      return "ADSERR_CLIENT_LISTEMPTY";
    case ADSERR_CLIENT_VARUSED:
      return "ADSERR_CLIENT_VARUSED";
    case ADSERR_CLIENT_DUPLINVOKEID:
      return "ADSERR_CLIENT_DUPLINVOKEID";
    case ADSERR_CLIENT_SYNCTIMEOUT:
      return "ADSERR_CLIENT_SYNCTIMEOUT";
    case ADSERR_CLIENT_W32ERROR:
      return "ADSERR_CLIENT_W32ERROR";
    case ADSERR_CLIENT_TIMEOUTINVALID:
      return "ADSERR_CLIENT_TIMEOUTINVALID";
    case ADSERR_CLIENT_PORTNOTOPEN:
      return "ADSERR_CLIENT_PORTNOTOPEN";
    case ADSERR_CLIENT_NOAMSADDR:
      return "ADSERR_CLIENT_NOAMSADDR";
    case ADSERR_CLIENT_SYNCINTERNAL:
      return "ADSERR_CLIENT_SYNCINTERNAL";
    case ADSERR_CLIENT_ADDHASH:
      return "ADSERR_CLIENT_ADDHASH";
    case ADSERR_CLIENT_REMOVEHASH:
      return "ADSERR_CLIENT_REMOVEHASH";
    case ADSERR_CLIENT_NOMORESYM:
      return "ADSERR_CLIENT_NOMORESYM";
    case ADSERR_CLIENT_SYNCRESINVALID:
      return "ADSERR_CLIENT_SYNCRESINVALID";
    case ADSERR_CLIENT_SYNCPORTLOCKED:
      return "ADSERR_CLIENT_SYNCPORTLOCKED";
    default:
      return "ADSERR_ERROR_UNKNOWN";
  }
}

/** Convert ADS type enum to string.
 *
 * \param[in] type Ads type (from adsLib https://github.com/Beckhoff/ADS)
 *
 * \return Ads type string.
 */
const char *adsTypeToString(long type)
{
  switch (type) {
    case ADST_VOID:
      return "ADST_VOID";
    case ADST_INT16:
      return "ADST_INT16";
    case ADST_INT32:
      return "ADST_INT32";
    case ADST_REAL32:
      return "ADST_REAL32";
    case ADST_REAL64:
      return "ADST_REAL64";
    case ADST_INT8:
      return "ADST_INT8";
    case ADST_UINT8:
      return "ADST_UINT8";
    case ADST_UINT16:
      return "ADST_UINT16";
    case ADST_UINT32:
      return "ADST_UINT32";
    case ADST_INT64:
      return "ADST_INT64";
    case ADST_UINT64:
      return "ADST_UINT64";
    case ADST_STRING:
      return "ADST_STRING";
    case ADST_WSTRING:
      return "ADST_WSTRING";
    case ADST_REAL80:
      return "ADST_REAL80";
    case ADST_BIT:
      return "ADST_BIT";
    case ADST_BIGTYPE:
      return "ADST_BIGTYPE";
    case ADST_MAXTYPES:
      return "ADST_MAXTYPES";
    default:
      return "ADS_UNKNOWN_DATATYPE";
  }
}

/** Convert asyn type enum to string.
 *
 * \param[in] type Asyn type (from asynDriver https://github.com/epics-modules/asyn)
 *
 * \return Asyn type string.
 */
const char *asynTypeToString(long type)
{
  switch (type) {
    case asynParamInt32:
      return "asynParamInt32";
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
    case asynParamInt64:
      return "asynParamInt64";
#endif
    case asynParamFloat64:
      return "asynParamFloat64";
    case asynParamUInt32Digital:
      return "asynParamUInt32Digital";
    case asynParamOctet:
      return "asynParamOctet";
    case asynParamInt8Array:
      return "asynParamInt8Array";
    case asynParamInt16Array:
      return "asynParamInt16Array";
    case asynParamInt32Array:
      return "asynParamInt32Array";
    case asynParamFloat32Array:
      return "asynParamFloat32Array";
    case asynParamFloat64Array:
      return "asynParamFloat64Array";
    case asynParamGenericPointer:
      return "asynParamGenericPointer";
    default:
      return "asynUnknownType";
  }
}

/** Convert ADS  ams port state enum to string.
 *
 * \param[in] state Ads state (from adsLib https://github.com/Beckhoff/ADS)
 *
 * \return Ads ams port state string.
 */
const char *adsStateToString(long state)
{
  switch (state) {
     case ADSSTATE_INVALID:
       return "ADSSTATE_INVALID";
       break;
     case ADSSTATE_IDLE:
       return "ADSSTATE_IDLE";
       break;
     case ADSSTATE_RESET:
       return "ADSSTATE_RESET";
       break;
     case ADSSTATE_INIT:
       return "ADSSTATE_INIT";
       break;
     case ADSSTATE_START:
       return "ADSSTATE_START";
       break;
     case ADSSTATE_RUN:
       return "ADSSTATE_RUN";
       break;
     case ADSSTATE_STOP:
       return "ADSSTATE_STOP";
       break;
     case ADSSTATE_SAVECFG:
       return "ADSSTATE_SAVECFG";
       break;
     case ADSSTATE_LOADCFG:
       return "ADSSTATE_LOADCFG";
       break;
     case ADSSTATE_POWERFAILURE:
       return "ADSSTATE_POWERFAILURE";
       break;
     case ADSSTATE_POWERGOOD:
       return "ADSSTATE_POWERGOOD";
       break;
     case ADSSTATE_ERROR:
       return "ADSSTATE_ERROR";
       break;
     case ADSSTATE_SHUTDOWN:
       return "ADSSTATE_SHUTDOWN";
       break;
     case ADSSTATE_SUSPEND:
       return "ADSSTATE_SUSPEND";
       break;
     case ADSSTATE_RESUME:
       return "ADSSTATE_RESUME";
       break;
     case ADSSTATE_CONFIG:
       return "ADSSTATE_CONFIG";
       break;
     case ADSSTATE_RECONFIG:
       return "ADSSTATE_RECONFIG";
       break;
     case ADSSTATE_STOPPING:
       return "ADSSTATE_STOPPING";
       break;
     case ADSSTATE_INCOMPATIBLE:
       return "ADSSTATE_INCOMPATIBLE";
       break;
     case ADSSTATE_EXCEPTION:
       return "ADSSTATE_EXCEPTION";
       break;
     case ADSSTATE_MAXSTATES:
       return "ADSSTATE_MAXSTATES";
       break;

  }
  return "UNKNOWN ADSSTATE";
}

/** Get size of ADS data type.
 *
 * \param[in] type Ads data type (from adsLib https://github.com/Beckhoff/ADS)
 *
 * \return Size of one element of type.
 */
size_t adsTypeSize(long type)
{
  switch (type) {
    case ADST_VOID:
      return 0;
    case ADST_INT16:
      return 2;
    case ADST_INT32:
      return 4;
    case ADST_REAL32:
      return 4;
    case ADST_REAL64:
      return 8;
    case ADST_INT8:
      return 1;
    case ADST_UINT8:
      return 1;
    case ADST_UINT16:
      return 2;
    case ADST_UINT32:
      return 4;
    case ADST_INT64:
      return 8;
    case ADST_UINT64:
      return 8;
    case ADST_STRING:
      return 1;  //Array of char
    case ADST_WSTRING:
      return -1;
    case ADST_REAL80:
      return 10;
    case ADST_BIT:
      return 1;
    case ADST_BIGTYPE:
      return -1;
    case ADST_MAXTYPES:
      return -1;
    default:
      return -1;
  }
}

/** Convert EPICS dtyp field to asyn type.
 *
 * \param[in] dtype field
 *
 * \return asyn param type enum.
 */
asynParamType dtypStringToAsynType(char *dtype)
{
  if(strcmp("asynFloat64",dtype)==0){
    return asynParamFloat64;
  }
  if(strcmp("asynInt32",dtype)==0){
    return asynParamInt32;
  }
#ifndef NO_ADS_ASYN_ASYNPARAMINT64
  if(strcmp("asynInt64",dtype)==0){
    return asynParamInt64;
  }
#endif
  if(strcmp("asynInt8ArrayIn",dtype)==0 || strcmp("asynInt8ArrayOut",dtype)==0){
    return asynParamInt8Array;
  }
  if(strcmp("asynInt16ArrayIn",dtype)==0 || strcmp("asynInt16ArrayOut",dtype)==0){
    return asynParamInt16Array;
  }
  if(strcmp("asynInt32ArrayIn",dtype)==0 || strcmp("asynInt32ArrayOut",dtype)==0){
    return asynParamInt32Array;
  }
  if(strcmp("asynFloat32ArrayIn",dtype)==0 || strcmp("asynFloat32ArrayOut",dtype)==0){
    return asynParamFloat32Array;
  }
  if(strcmp("asynFloat64ArrayIn",dtype)==0 || strcmp("asynFloat64ArrayOut",dtype)==0){
    return asynParamFloat64Array;
  }
  //  asynParamUInt32Digital,
  //  asynParamOctet,
  //  asynParamGenericPointer

  return asynParamNotDefined;
}

/** Convert EPICS state to string.
 *
 * \param[in] state EPICS state enum.
 *
 * \return EPICS state string.
 */
const char* epicsStateToString(int state)
{
  switch(state) {
    case initHookAtIocBuild:
      return "initHookAtIocBuild";
      break;
    case initHookAtBeginning:
      return "initHookAtBeginning";
      break;
    case initHookAfterCallbackInit:
      return "initHookAfterCallbackInit";
      break;
    case initHookAfterCaLinkInit:
      return "initHookAfterCaLinkInit";
      break;
    case initHookAfterInitDrvSup:
      return "initHookAfterInitDrvSup";
      break;
    case initHookAfterInitRecSup:
      return "initHookAfterInitRecSup";
      break;
    case initHookAfterInitDevSup:
      return "initHookAfterInitDevSup";
      break;
    case initHookAfterInitDatabase:
      return "initHookAfterInitDatabase";
      break;
    case initHookAfterFinishDevSup:
      return "initHookAfterFinishDevSup";
      break;
    case initHookAfterScanInit:
      return "initHookAfterScanInit";
      break;
    case initHookAfterInitialProcess:
      return "initHookAfterInitialProcess";
      break;
    case initHookAfterIocBuilt:
      return "initHookAfterIocBuilt";
      break;
    case initHookAtIocRun:
      return "initHookAtIocRun";
      break;
    case initHookAfterDatabaseRunning:
      return "initHookAfterDatabaseRunning";
      break;
    case initHookAfterCaServerRunning:
      return "initHookAfterCaServerRunning";
      break;
    case initHookAfterIocRunning:
      return "initHookAfterIocRunning";
      break;
    case initHookAtIocPause:
      return "initHookAtIocPause";
      break;
    case initHookAfterCaServerPaused:
      return "initHookAfterCaServerPaused";
      break;
    case initHookAfterDatabasePaused:
      return "initHookAfterDatabasePaused";
      break;
    case initHookAfterIocPaused:
      return "initHookAfterIocPaused";
      break;
     case initHookAfterInterruptAccept:
      return "initHookAfterInterruptAccept";
      break;
    default:
      return "Unknown state";
      break;
  }
  return "Unknown state";
}

#define WINDOWS_TICK_PER_SEC 10000000
#define SEC_TO_UNIX_EPOCH 11644473600LL

/** Convert Windows timestamp to EPICS timestamp
 *
 * \param[in] plcTime Timestamp from ams router (Windows format, epoch start jan
 * 1 1601).
 * \param[out] ts Epics timestamp (Epoch start jan 1 1990).
 *
 * \return 0 or error code.
 */
int windowsToEpicsTimeStamp(uint64_t plcTime, epicsTimeStamp *ts)
{
  if(!ts){
    return 1;
  }
  //move from WindowsTime to Epics time 1601 jan 1 to 1990 jan 1 (POSIX_TIME_AT_EPICS_EPOCH defined in epicsTime.h)
  plcTime=plcTime-(POSIX_TIME_AT_EPICS_EPOCH + SEC_TO_UNIX_EPOCH)*WINDOWS_TICK_PER_SEC;

  ts->secPastEpoch=(uint32_t)(plcTime/WINDOWS_TICK_PER_SEC);
  ts->nsec=(uint32_t)((plcTime-(ts->secPastEpoch*WINDOWS_TICK_PER_SEC))*100);

  return 0;
}

/** Octet interface: Add data to buffer.
 *
 * \param[in] buffer Output data buffer.
 * \param[in] addText Data to add to output buffer.
 * \param[in] addLength Length of data to add.
 *
 * \return 0 or error code.
 */
int addToBuffer(adsOctetOutputBufferType *buffer,const char *addText, size_t addLength)
{
  if(buffer==NULL){
    return __LINE__;
  }

  if(addLength>=buffer->bufferSize-buffer->bytesUsed-1){
    return __LINE__;
  }

  memcpy(&buffer->buffer[buffer->bytesUsed], addText, addLength);
  buffer->bytesUsed+=addLength;
  buffer->buffer[buffer->bytesUsed] = '\0';
  return 0;
}

/** Octet interface: Print data to buffer.
 *
 * \param[in] buffer Output data buffer.
 * \param[in] format Format of data to add to output buffer.
 * \param[in] arg va_list of data
 *
 * \return 0 or error code.
 */
static int cmd_buf_vprintf(adsOctetOutputBufferType *buffer,  const char* format, va_list arg)
{
  const static size_t len = 4096;

  char *buf = (char *)calloc(len, 1);
  int res = vsnprintf(buf, len-1, format, arg);
  if (res >= 0) {
    addToBuffer(buffer, buf, res);
  }
  free(buf);
  return res;
}

/** Octet interface: Print data to buffer.
 *
 * \param[in] buffer Output data buffer.
 * \param[in] format Format of data to add to output buffer.
 * \param[in] ... List of data
 *
 * \return 0 or error code.
 */
int octetCmdBuf_printf(adsOctetOutputBufferType *buffer,const char *format, ...)
{
  if(buffer==NULL){
    return __LINE__;
  }
  va_list ap;
  va_start(ap, format);
  (void)cmd_buf_vprintf(buffer, format, ap);
  va_end(ap);
  return 0;
}

/** Octet interface: Remove data from buffer.
 *
 * \param[in] buffer Output data buffer.
 * \param[in] len Bytes to remove from buffer.
 *
 * \return 0 or error code.
 */
int octetRemoveFromBuffer(adsOctetOutputBufferType *buffer,size_t len)
{
  if(buffer==NULL){
    return __LINE__;
  }

  int bytesToMove= buffer->bytesUsed-len;
  if(bytesToMove<0){
    return __LINE__;
  }

  memmove(&buffer->buffer[0],&buffer->buffer[len],bytesToMove);
  buffer->bytesUsed=bytesToMove;
  buffer->buffer[buffer->bytesUsed] = '\0';
  return 0;
}

/** Octet interface: Clear buffer.
 *
 * \param[in] buffer Output data buffer.
 *
 * \return 0 or error code.
 */
int octetClearBuffer(adsOctetOutputBufferType *buffer)
{
  if(buffer==NULL){
    return __LINE__;
  }
  buffer->bytesUsed=0;
  buffer->buffer[0]='\0';
  return 0;
}

/** Octet interface: Divide line into commands.
 *
 * \param[in] line Line of ASCII commands.
 * \param[out] argv_p Array of commands.
 * \param[out] sepv_p Array of separators.
 *
 * \return 0 or error code.
 */
int octetCreateArgvSepv(const char *line,
                        const char*** argv_p,
                        char*** sepv_p)
{
  char *input_line = strdup(line);
  size_t calloc_len = 2 + strlen(input_line);
  char *separator = NULL;
  static const size_t MAX_SEPARATORS = 4;
  int argc = 0;
  /* Allocate an array big enough, could be max strlen/2
     space <-> non-space transitions */
  const char **argv; /* May be more */
  char **sepv;

  argv = (const char **) (void *)calloc(calloc_len, sizeof(char *));
  *argv_p = argv;
  if (argv  == NULL)
  {
    return 0;
  }
  sepv = (char **) (void *)calloc(calloc_len, sizeof(char *));
  *sepv_p = sepv;
  if (sepv  == NULL)
  {
    return 0;
  }
  /* argv[0] is the whole line */
  {
    argv[argc] = strdup(input_line);
    sepv[argc] = (char*)calloc(1, MAX_SEPARATORS);
  }
  if (!strlen(input_line)) {
    return argc;
  }
  if (strchr(input_line, ';') != NULL) {
    separator = (char *) ";";
  } else if (strchr(input_line, ' ') != NULL) {
    separator = (char *)" ";
  }
  if (separator) {
    argc++;
    /* Start the loop */
    char *arg_begin = input_line;
    char *next_sep = strchr(input_line, separator[0]);
    char *arg_end = next_sep ? next_sep : input_line + strlen(input_line);

    while (arg_begin) {
      size_t sepi = 0;
      char *sep = NULL;
      size_t arg_len = arg_end - arg_begin;

      argv[argc] = (const char *)calloc(1, arg_len+1);
      memcpy((char *)argv[argc], arg_begin, arg_len);
      sepv[argc] = (char *)calloc(1, MAX_SEPARATORS);
      sep = sepv[argc];
      if (next_sep) {
        /* There is another separator */
        sep[sepi++] = separator[0];
      }
      arg_begin = arg_end;
      if (arg_begin[0] == separator[0]) {
        arg_begin++; /* Jump over, if any */
      }
      next_sep = strchr(arg_begin, separator[0]);
      arg_end = next_sep ? next_sep : input_line + strlen(input_line);

      if (!strlen(arg_begin)) {
        break;
      } else {
        argc++;
      }
    }
  } else {
    /* argv[1] is the whole line */
    argc = 1;
    argv[argc] = strdup(input_line);
    sepv[argc] = (char *)calloc(1, MAX_SEPARATORS);
  }

  free(input_line);

  return argc;
}

/** Octet interface: Convert binary data to ASCII.
 *
 * \param[in] returnVarName Print variable name in output buffer.
 * \param[in] binaryBuffer Binary data buffer.
 * \param[in] info Information of data type
 * \param[out] asciiBuffer Output buffer (ASCII).
 *
 * \return 0 or error code.
 */
int octetBinary2ascii(bool returnVarName,
                      void *binaryBuffer,
                      uint32_t binaryBufferSize,
                      adsSymbolEntry *info,
                      adsOctetOutputBufferType *asciiBuffer)
{
  uint32_t bytesProcessed=0;
  int cycles=0;
  int error=0;
  int bytesPerDataPoint=0;

  while(bytesProcessed<info->size && !error){
    //write comma for arrays
    if(bytesProcessed!=0){
      octetCmdBuf_printf(asciiBuffer,",");
    }
    switch(info->dataType){
      case ADST_INT8:
        RETURN_VAR_NAME_IF_NEEDED;
        int8_t *ADST_INT8Var;
        ADST_INT8Var=((int8_t*)binaryBuffer)+cycles;
        octetCmdBuf_printf(asciiBuffer,"%hhd",*ADST_INT8Var);
        //printf("Binary 2 ASCII ADST_INT8, value: %d\n", *ADST_INT8Var);
        bytesPerDataPoint=1;
        bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_INT16:
        RETURN_VAR_NAME_IF_NEEDED;
        int16_t *ADST_INT16Var;
        ADST_INT16Var=((int16_t*)binaryBuffer)+cycles;
        octetCmdBuf_printf(asciiBuffer,"%d",*ADST_INT16Var);
        //printf("Binary 2 ASCII ADST_INT16, value: %d\n", *ADST_INT16Var);
        bytesPerDataPoint=2;
        bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_INT32:
        RETURN_VAR_NAME_IF_NEEDED;
        int32_t *ADST_INT32Var;
        ADST_INT32Var=((int32_t*)binaryBuffer)+cycles;
        octetCmdBuf_printf(asciiBuffer,"%d",*ADST_INT32Var);
        //printf("Binary 2 ASCII ADST_INT32, value: %d\n", *ADST_INT32Var);
        bytesPerDataPoint=4;
        bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_INT64:
        RETURN_VAR_NAME_IF_NEEDED;
        int64_t *ADST_INT64Var;
        ADST_INT64Var=((int64_t*)binaryBuffer)+cycles;
        octetCmdBuf_printf(asciiBuffer,"% PRId64",*ADST_INT64Var);
        //printf("Binary 2 ASCII ADST_INT64, value: %" PRId64 "\n", *ADST_INT64Var);
        bytesPerDataPoint=8;
        bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_UINT8:
        RETURN_VAR_NAME_IF_NEEDED;
        uint8_t *ADST_UINT8Var;
        ADST_UINT8Var=((uint8_t*)binaryBuffer)+cycles;
        octetCmdBuf_printf(asciiBuffer,"%hhu",*ADST_UINT8Var);
        //printf("Binary 2 ASCII ADST_UINT8, value: %d\n", *ADST_UINT8Var);
        bytesPerDataPoint=1;
        bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_UINT16:
        RETURN_VAR_NAME_IF_NEEDED;
        uint16_t *ADST_UINT16Var;
        ADST_UINT16Var=((uint16_t*)binaryBuffer)+cycles;
        octetCmdBuf_printf(asciiBuffer,"%d",*ADST_UINT16Var);
        //printf("Binary 2 ASCII ADST_UINT16, value: %d\n", *ADST_UINT16Var);
        bytesPerDataPoint=2;
        bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_UINT32:
        RETURN_VAR_NAME_IF_NEEDED;
        uint32_t *ADST_UINT32Var;
        ADST_UINT32Var=((uint32_t*)binaryBuffer)+cycles;
        octetCmdBuf_printf(asciiBuffer,"%d",*ADST_UINT32Var);
        //printf("Binary 2 ASCII ADST_UINT32, value: %d\n", *ADST_UINT32Var);
        bytesPerDataPoint=4;
        bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_UINT64:
        RETURN_VAR_NAME_IF_NEEDED;
        uint64_t *ADST_UINT64Var;
        ADST_UINT64Var=((uint64_t*)binaryBuffer)+cycles;
        octetCmdBuf_printf(asciiBuffer,"% PRIu64",*ADST_UINT64Var);
        //printf("Binary 2 ASCII ADST_UINT64, value: %" PRIu64 "\n", *ADST_UINT64Var);
        bytesPerDataPoint=8;
        bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_REAL32:
        RETURN_VAR_NAME_IF_NEEDED;
        float *ADST_REAL32Var;
        ADST_REAL32Var=((float*)binaryBuffer)+cycles;
        octetCmdBuf_printf(asciiBuffer,"%f",*ADST_REAL32Var);
        //printf("Binary 2 ASCII ADST_REAL32, value: %lf\n", *ADST_REAL32Var);
        bytesPerDataPoint=4;
        bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_REAL64:
        RETURN_VAR_NAME_IF_NEEDED;
        double *ADST_REAL64Var;
        ADST_REAL64Var=((double*)binaryBuffer)+cycles;
        octetCmdBuf_printf(asciiBuffer,"%lf",*ADST_REAL64Var);
        //printf("Binary 2 ASCII ADST_REAL64, value: %lf\n", *ADST_REAL64Var);
        bytesPerDataPoint=8;
        bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_BIT:
        RETURN_VAR_NAME_IF_NEEDED;
        char *charVar;
        charVar=((char*)binaryBuffer)+cycles;
        if(*charVar==1){
            octetCmdBuf_printf(asciiBuffer,"1");
        }
        else{
            octetCmdBuf_printf(asciiBuffer,"0");
        }
        bytesPerDataPoint=1;//TODO: Check if each bit takes one byte or actually only one bit?!
        bytesProcessed+=bytesPerDataPoint;
        //printf("Binary 2 ASCII ADST_BIT, value: %c\n", *charVar);
        break;
      case ADST_STRING:
        RETURN_VAR_NAME_IF_NEEDED;
        char *ADST_STRINGVar;
        ADST_STRINGVar = (char*)binaryBuffer;
        octetCmdBuf_printf(asciiBuffer,"%s",ADST_STRINGVar);
        //printf("Binary 2 ASCII ADST_STRING, value: %s\n", ADST_STRINGVar);
        bytesProcessed=info->size;
        break;
      case ADST_BIGTYPE:
        if(strstr(info->symDataType,DUT_AXIS_STATUS)!=NULL){
          //RETURN_VAR_NAME_IF_NEEDED;
            octetCmdBuf_printf(asciiBuffer,"%s=",info->variableName); //Always output variable name for stAxisStatus
          adsOctetSTAXISSTATUSSTRUCT * stAxisData;
          stAxisData=(adsOctetSTAXISSTATUSSTRUCT*)binaryBuffer;

          if(stAxisData->bEnable){
            octetCmdBuf_printf(asciiBuffer,"1,");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0,");
          }
          if(stAxisData->bReset){
            octetCmdBuf_printf(asciiBuffer,"1,");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0,");
          }
          if(stAxisData->bExecute){
            octetCmdBuf_printf(asciiBuffer,"1,");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0,");
          }
          octetCmdBuf_printf(asciiBuffer,"%d,", stAxisData->nCommand);
          octetCmdBuf_printf(asciiBuffer,"%d,", stAxisData->nCmdData);
          octetCmdBuf_printf(asciiBuffer,"%lf,", stAxisData->fVelocity);
          octetCmdBuf_printf(asciiBuffer,"%lf,", stAxisData->fPosition);
          octetCmdBuf_printf(asciiBuffer,"%lf,", stAxisData->fAcceleration);
          octetCmdBuf_printf(asciiBuffer,"%lf,", stAxisData->fDeceleration);
          if(stAxisData->bJogFwd){
            octetCmdBuf_printf(asciiBuffer,"1,");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0,");
          }
          if(stAxisData->bJogBwd){
            octetCmdBuf_printf(asciiBuffer,"1,");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0,");
          }
          if(stAxisData->bLimitFwd){
            octetCmdBuf_printf(asciiBuffer,"1,");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0,");
          }
          if(stAxisData->bLimitBwd){
            octetCmdBuf_printf(asciiBuffer,"1,");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0,");
          }
          octetCmdBuf_printf(asciiBuffer,"%lf,", stAxisData->fOverride);
          if(stAxisData->bHomeSensor){
            octetCmdBuf_printf(asciiBuffer,"1,");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0,");
          }
          if(stAxisData->bEnabled){
            octetCmdBuf_printf(asciiBuffer,"1,");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0,");
          }
          if(stAxisData->bError){
            octetCmdBuf_printf(asciiBuffer,"1,");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0,");
          }
          octetCmdBuf_printf(asciiBuffer,"%d,", stAxisData->nErrorId);
          octetCmdBuf_printf(asciiBuffer,"%lf,", stAxisData->fActVelocity);
          octetCmdBuf_printf(asciiBuffer,"%lf,", stAxisData->fActPosition);
          octetCmdBuf_printf(asciiBuffer,"%lf,", stAxisData->fActDiff);
          if(stAxisData->bHomed){
            octetCmdBuf_printf(asciiBuffer,"1,");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0,");
          }
          if(stAxisData->bBusy){
            octetCmdBuf_printf(asciiBuffer,"1;");
          }
          else{
            octetCmdBuf_printf(asciiBuffer,"0;");
          }
          //printf("Binary 2 ASCII ADST_BIGTYPE, type: %s\n", info->symDataType);
          bytesProcessed=info->size;
          break; //end DUT_AXIS_STATUS
        }
        break;
      default:
        error=ADS_COM_ERROR_INVALID_DATA_TYPE;
        //printf("Data type %s (%d) not implemented. Error: %d\n", info->symDataType, info->dataType,error);
        bytesPerDataPoint=0;
        break;
    }
    cycles++;

    if(binaryBufferSize<bytesProcessed+bytesPerDataPoint){
      error=ADS_COM_ERROR_ADS_READ_BUFFER_INDEX_EXCEEDED_SIZE;
      //printf("Buffer size exceeded. Error: %d\n",error);
    }
    if((asciiBuffer->bufferSize-asciiBuffer->bytesUsed)<20){
      error=ADS_COM_ERROR_BUFFER_TO_EPICS_FULL;
      //printf("Buffer size exceeded. Error: %d\n",error);
    }
  }
  return error;
}

/** Octet interface: Convert ASCII data to binary.
 *
 * \param[in] asciiBuffer ASCII buffer.
 * \param[in] dataType Data type.
 * \param[out] binaryBuffer Output buffer (binary)
 * \param[in] binaryBufferSize Binary buffer size.
 * \param[out] Bytes written to binary buffer.
 *
 * \return 0 or error code.
 */
int octetAscii2binary(const char *asciiBuffer,uint16_t dataType,void *binaryBuffer, uint32_t binaryBufferSize, uint32_t *bytesProcessed)
{
  int cycles=0;
  int error=0;
  int bytesPerDataPoint=0;
  int converted = 0 ;

  do
  {
    switch(dataType){
      case ADST_INT8:
        int8_t *ADST_INT8Var;
        ADST_INT8Var=((int8_t*)binaryBuffer)+cycles;
        converted = sscanf( asciiBuffer, "%" SCNd8, (ADST_INT8Var));
        bytesPerDataPoint=1;
        *bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_INT16:
        int16_t *ADST_INT16Var;
        ADST_INT16Var=((int16_t*)binaryBuffer)+cycles;
        converted = sscanf( asciiBuffer, "%" SCNd16, ADST_INT16Var);
        bytesPerDataPoint=2;
        *bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_INT32:
        int32_t *ADST_INT32Var;
        ADST_INT32Var=((int32_t*)binaryBuffer)+cycles;
        converted = sscanf( asciiBuffer, "%" SCNd32, (ADST_INT32Var));
        bytesPerDataPoint=4;
        *bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_INT64:
        int64_t *ADST_INT64Var;
        ADST_INT64Var=((int64_t*)binaryBuffer)+cycles;
        converted = sscanf( asciiBuffer, "%" SCNd64, (ADST_INT64Var));
        bytesPerDataPoint=8;
        *bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_UINT8:
        uint8_t *ADST_UINT8Var;
        ADST_UINT8Var=((uint8_t*)binaryBuffer)+cycles;
        converted = sscanf( asciiBuffer, "%" SCNu8, (ADST_UINT8Var));
        bytesPerDataPoint=1;
        *bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_UINT16:
        uint16_t *ADST_UINT16Var;
        ADST_UINT16Var=((uint16_t*)binaryBuffer)+cycles;
        converted = sscanf( asciiBuffer, "%" SCNu16, (ADST_UINT16Var));
        bytesPerDataPoint=2;
        *bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_UINT32:
        uint32_t *ADST_UINT32Var;
        ADST_UINT32Var=((uint32_t*)binaryBuffer)+cycles;
        converted = sscanf( asciiBuffer, "%" SCNu32, (ADST_UINT32Var));
        bytesPerDataPoint=4;
        *bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_UINT64:
        uint64_t *ADST_UINT64Var;
        ADST_UINT64Var=((uint64_t*)binaryBuffer)+cycles;
        converted = sscanf( asciiBuffer, "%" SCNu64, (ADST_UINT64Var));
        bytesPerDataPoint=8;
        *bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_REAL32:
        float *ADST_REAL32Var;
        ADST_REAL32Var=((float*)binaryBuffer)+cycles;
        converted = sscanf( asciiBuffer, "%f", (ADST_REAL32Var));
        bytesPerDataPoint=4;
        *bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_REAL64:
        double *ADST_REAL64Var;
        ADST_REAL64Var=((double*)binaryBuffer)+cycles;
        converted = sscanf( asciiBuffer, "%lf", (ADST_REAL64Var));
        bytesPerDataPoint=8;
        *bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_BIT:
        char *charVar;
        charVar=((char*)binaryBuffer)+cycles;
        converted = sscanf( asciiBuffer, "%hhu", (charVar));
        bytesPerDataPoint=1; //TODO: Check if each bit takes one byte or actually only one bit?!
        *bytesProcessed+=bytesPerDataPoint;
        break;
      case ADST_STRING:
        char *stringVar;
        stringVar=((char*)binaryBuffer);
        converted = sscanf( asciiBuffer, "%s", stringVar);
        bytesPerDataPoint=1; //TODO: Check if each bit takes one byte or actually only one bit?!
        *bytesProcessed=binaryBufferSize;
        break;
      default:
        //printf("ERROR: Data type: %d not implemented.\n",dataType);
        error=ADS_COM_ERROR_INVALID_DATA_TYPE;
        bytesPerDataPoint=0;
        break;
    }
    if(binaryBufferSize<*bytesProcessed+bytesPerDataPoint){
      error=ADS_COM_ERROR_ADS_READ_BUFFER_INDEX_EXCEEDED_SIZE;
    }

    cycles++;
    asciiBuffer= strchr( asciiBuffer, ',' ) ;
    if(asciiBuffer){
        asciiBuffer++;
    }

  } while(asciiBuffer !=NULL  && converted != 0 && !error);

  return error;
}
