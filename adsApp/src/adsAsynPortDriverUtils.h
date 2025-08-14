/*
    This file is part of epics-twincat-ads.

    epics-twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

    epics-twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License along with epics-twincat-ads. If not, see <https://www.gnu.org/licenses/>.

*/
/*
* adsAsynPortDriverUtils.h
*
* Utilities and definitions used by adsAsynPortDriver-class.
*
* Author: Anders Sandström
*
* Created January 30, 2018
*/

#ifndef ADSASYNPORTDRIVERUTILS_H_
#define ADSASYNPORTDRIVERUTILS_H_

#include "asynPortDriver.h"  //data types
#include "AdsLib.h"          //error codes
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

//Error codes
#define ADS_COM_ERROR_INVALID_DATA_TYPE 1004
#define ADS_COM_ERROR_ADS_READ_BUFFER_INDEX_EXCEEDED_SIZE 1005
#define ADS_COM_ERROR_BUFFER_TO_EPICS_FULL 1006
#define ADS_COM_ERROR_OCTET_ADSPORT_OPTION_FAIL 1007

#define ADS_MAX_FIELD_CHAR_LENGTH 128
#define ADS_ADR_COMMAND_PREFIX ".ADR."
#define ADS_OPTION_T_MAX_DLY_MS "T_DLY_MS"
#define ADS_OPTION_T_SAMPLE_RATE_MS "TS_MS"
#define ADS_OPTION_TIMEBASE "TIMEBASE"  //PLC or EPICS
#define ADS_OPTION_POLLRATE "POLL_RATE"
#define ADS_OPTION_TIMEBASE_EPICS "EPICS"
#define ADS_OPTION_TIMEBASE_PLC "PLC"
#define ADS_OPTION_ADSPORT "ADSPORT"
#define ADS_OCTET_FEATURES_COMMAND ".THIS.sFeatures?"
#define ADS_AMS_STATE_COMMAND ".AMSPORTSTATE."

#ifndef ASYN_TRACE_INFO
  #define ASYN_TRACE_INFO      0x0040
#endif

typedef enum{
  ADS_TIME_BASE_PLC=0,
  ADS_TIME_BASE_EPICS=1,
  ADS_TIME_BASE_MAX
} ADSTIMESOURCE;

typedef enum{
  ADS_DATASOURCE_PLC=0,       //Data in PLC (Normal/default)
  ADS_DATASOURCE_AMS_STATE=1, //Special case parameter linked to ads status (not plc "data")
  ADS_DATASOURCE_MAX=2,
} ADSDATASOURCE;

typedef struct adsParamInfo{
  char           *recordName;
  char           *recordType;
  char           *scan;
  char           *dtyp;
  char           *inp;
  char           *out;
  char           *drvInfo;
  asynParamType  asynType;
  int            asynAddr;
  bool           hasInput;
  double         sampleTimeMS;  //milli seconds
  double         maxDelayTimeMS;  //milli seconds
  uint16_t       amsPort;
  int            paramIndex;  //also used as hUser for ads callback
  bool           plcAbsAdrValid;  //Symbolic address converted to abs address or .ADR. command parsed
  bool           isAdrCommand;
  bool           isBulkRead;
  double         pollClass;
  char           *plcAdrStr;
  uint32_t       plcAbsAdrGroup;
  uint32_t       plcAbsAdrOffset;
  uint32_t       plcSize;
  uint32_t       plcDataType;
  bool           plcDataTypeWarn;
  bool           plcDataIsArray;
  uint32_t       hCallbackNotify;
  bool           bCallbackNotifyValid;
  uint32_t       hSymbolicHandle;
  bool           bSymbolicHandleValid;
  size_t         lastCallbackSize;
  size_t         arrayDataBufferSize;
  void*          arrayDataBuffer;
  bool           refreshNeeded;  //Communication broken update handles and callbacks
  ADSDATASOURCE  dataSource;          //Variable in PLC or in driver (not in PLC)
  //timing
  ADSTIMESOURCE  timeBase;
  uint64_t       plcTimeStampRaw;
  epicsTimeStamp plcTimeStamp;
  epicsTimeStamp epicsTimestamp;
  int            alarmStatus;
  int            alarmSeverity;
  bool           firstReadDone;
  int            bulkIndex;
  int            bulkOffset;
}adsParamInfo;

typedef struct amsPortInfo{
  uint16_t amsPort;
  int connectedOld;
  int connected;
  int paramsOK;
  AdsVersion version;
  char devName[255];
  ADSSTATE adsStateOld;
  ADSSTATE adsState;
  adsParamInfo* paramInfo;
  uint32_t      hCallbackNotify;
  bool          bCallbackNotifyValid;
  bool          refreshNeeded;  //Communication broken update handles and callbacks
}amsPortInfo;

//For info from symbolic name Actually this data type should be in the adslib (but missing)..
typedef struct {
  uint32_t entryLen;
  uint32_t iGroup;
  uint32_t iOffset;
  uint32_t size;
  uint32_t dataType;
  uint32_t flags;
  uint16_t nameLength;
  uint16_t typeLength;
  uint16_t commentLength;
  char  buffer[768]; //256*3, 256 is string size in TwinCAT then 768 is max
  char* variableName;
  char* symDataType;
  char* symComment;
} adsSymbolEntry;

typedef enum{
  ADST_VOID     = 0,
  ADST_INT8     = 16,
  ADST_UINT8    = 17,
  ADST_INT16    = 2,
  ADST_UINT16   = 18,
  ADST_INT32    = 3,
  ADST_UINT32   = 19,
  ADST_INT64    = 20,
  ADST_UINT64   = 21,
  ADST_REAL32   = 4,
  ADST_REAL64   = 5,
  ADST_BIGTYPE  = 65,
  ADST_STRING   = 30,
  ADST_WSTRING  = 31,
  ADST_REAL80   = 32,
  ADST_BIT      = 33,
  ADST_MAXTYPES
} ADSDATATYPEID;

const char *adsErrorToString(long error);
const char *adsTypeToString(long type);
const char *asynTypeToString(long type);
const char *adsStateToString(long state);
const char *epicsStateToString(int state);
size_t adsTypeSize(long type);
asynParamType dtypStringToAsynType(char *dtype);
int windowsToEpicsTimeStamp(uint64_t plcTime, epicsTimeStamp *ts);


/**
 * Octet interface functions and definitions
 */

#define DUT_AXIS_STATUS "DUT_AxisStatus_v0_01"
#define ADS_CMD_BUFFER_SIZE 65536

#define OCTET_RETURN_ERROR(buffer,errcode,fmt, ...)   \
  do {                                            \
    octetCmdBuf_printf(buffer,"Error: ");             \
    octetCmdBuf_printf(buffer,fmt, ##__VA_ARGS__);    \
    return errcode;                               \
  }                                               \
  while(0)

typedef struct {
  size_t   bufferSize;
  size_t   bytesUsed;
  char  buffer[ADS_CMD_BUFFER_SIZE];
} adsOctetOutputBufferType;
int octetCmdBuf_printf(adsOctetOutputBufferType *buffer,
                   const char *format, ...);
int octetRemoveFromBuffer(adsOctetOutputBufferType *buffer,
                     size_t len);
int octetClearBuffer(adsOctetOutputBufferType *buffer);
int octetCreateArgvSepv(const char *line,
                        const char*** argv_p,
                        char*** sepv_p);
int octetBinary2ascii(bool returnVarName,
                      void *binaryBuffer,
                      uint32_t binaryBufferSize,
                      adsSymbolEntry *info,
                      adsOctetOutputBufferType *asciiBuffer);
int octetAscii2binary(const char *asciiBuffer,
                      uint16_t dataType,
                      void *binaryBuffer,
                      uint32_t binaryBufferSize,
                      uint32_t *bytesProcessed);

#endif /* ADSASYNPORTDRIVERUTILS_H_ */



