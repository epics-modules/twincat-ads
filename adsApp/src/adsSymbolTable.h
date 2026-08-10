/**
 * adsSymbolTable.h
 *
 * Symbol-table cache for adsAsynPortDriver.
 *
 * Rationale
 * ---------
 * The existing per-record startup path issues two serial ADS round-trips per
 * record:
 *   RTT ①  ADSIGRP_SYM_INFOBYNAMEEX  (0xF009) – fill plcSize / plcDataType
 *   RTT ②  ADSIGRP_SYM_HNDBYNAME     (0xF003) – obtain hSymbolicHandle
 *
 * For N=10 000 records at 3 ms RTT this gives ~60 s startup time.
 *
 * By downloading the complete PLC symbol table once
 * (ADSIGRP_SYM_UPLOADINFO 0xF00C + ADSIGRP_SYM_UPLOAD 0xF00B, 2 RTTs total)
 * and building an unordered_map keyed on the lower-cased symbol name, every
 * adsGetSymInfoByName() call becomes an O(1) hash-map lookup that fills the
 * same fields – eliminating all N×RTT① round-trips.
 *
 * Data-structure choice
 * ---------------------
 * std::unordered_map<string, AdsSymbolInfo> is the best fit:
 *   • Average O(1) lookup vs O(log N) for std::map.
 *   • Keys are std::string (owned, lower-cased copy – no lifetime dependency
 *     on the raw blob buffer).
 *   • Values are AdsSymbolInfo (small POD, ~20 bytes), copied out of the blob
 *     so the blob itself can be freed immediately after the map is built.
 *   • reserve(symbolCount) pre-allocates buckets and avoids rehashing during
 *     the blob walk.
 *   • One SymbolMap per AMS port stored in SymbolCache so multi-port drivers
 *     (e.g. port 851 PLC + port 501 NC) work correctly.
 *
 * Thread-safety
 * -------------
 * adsLoadSymbolTable() is always called while the asyn port lock is held
 * (either inside connect() before iocInit, or inside refreshParams() which
 * is called under refreshParamsLock() -> adsLock()).  No additional locking
 * is needed for the map itself.
 */

#ifndef ADS_SYMBOL_TABLE_H
#define ADS_SYMBOL_TABLE_H

#include <cstdint>
#include <string>
#include <unordered_map>

/**
 * AdsSymbolUploadInfo2 – returned by ADSIGRP_SYM_UPLOADINFO2 (0xF00F).
 * Provides symbolCount (for reserve()) and uploadLength (for the blob
 * allocation) before the full SYM_UPLOAD download.
 *
 * Note: the older ADSIGRP_SYM_UPLOADINFO (0xF00C) returns only the first two
 * fields as AdsSymbolUploadInfo2 {nSymbols, nSymSize}.  Use 0xF00C if the
 * target may be a TC2 runtime; use 0xF00F for TC3-only deployments.
 */
struct AdsSymbolUploadInfo2
{
    uint32_t symbolCount;  /**< total number of symbols in the PLC        */
    uint32_t uploadLength; /**< byte-size of the full SYM_UPLOAD blob     */
    uint32_t dataTypeCount;
    uint32_t dataTypeLength;
    uint32_t extraCount;
    uint32_t extraLength;
};

/**
 * AdsSymbolEntryWire - the on-wire layout of each entry in the SYM_UPLOAD
 * blob, using explicit uint32_t/uint16_t to avoid the LP64 problem where
 * the TC2-era AdsSymbolEntry uses unsigned long (8 bytes on 64-bit Linux)
 * instead of the 4-byte wire format the PLC actually sends.
 * sizeof(AdsSymbolEntryWire) == 26 bytes on all platforms.
 */
#pragma pack(push, 1)
struct AdsSymbolEntryWire
{
    uint32_t entryLength;
    uint32_t iGroup;
    uint32_t iOffs;
    uint32_t size;
    uint32_t dataType;
    uint32_t flags;
    uint16_t nameLength;
    uint16_t typeLength;
    uint16_t commentLength;
    /* char name   [nameLength+1]  follows immediately */
    /* char type   [typeLength+1]  follows             */
    /* char comment[commentLength+1] follows           */
};
#pragma pack(pop)

/**
 * AdsSymbolInfo – the subset of AdsSymbolEntry fields we actually need.
 *
 * During the blob walk we cast blob bytes to const AdsSymbolEntry* (the SDK
 * struct from TcAdsDef.h / AdsLib.h) to read the wire format, then copy only
 * these five scalar fields into AdsSymbolInfo.  This means:
 *   • The blob buffer can be freed immediately after the walk.
 *   • No pointer into the blob survives into the map.
 *   • Each map value is ~20 bytes – keeps the hash table cache-friendly.
 *
 * Field names match the SDK's AdsSymbolEntry (iGroup, iOffs) so assignments
 * are self-documenting.  Note the driver's own adsSymbolEntry typedef uses
 * iOffset (no 's') – that difference only matters in the ADS fallback path.
 */
struct AdsSymbolInfo
{
    uint32_t iGroup;   /**< abs ADS index group  → paramInfo->plcAbsAdrGroup  */
    uint32_t iOffs;    /**< abs ADS index offset → paramInfo->plcAbsAdrOffset */
    uint32_t size;     /**< symbol byte-size     → paramInfo->plcSize         */
    uint32_t dataType; /**< ADST_* type code     → paramInfo->plcDataType     */
    uint32_t flags;    /**< ADSSYMBOLFLAG_* bits                               */
};

/**
 * SymbolMap – per-port lookup table.
 * Key   = lower-cased symbol name (TwinCAT names are case-insensitive).
 * Value = AdsSymbolInfo (POD copy, ~20 bytes).
 */
using SymbolMap = std::unordered_map<std::string, AdsSymbolInfo>;

/**
 * SymbolCache – one SymbolMap per AMS port.
 * Stored as private member  SymbolCache symbolCache_  of adsAsynPortDriver.
 * Key   = AMS port number (e.g. 851, 501).
 * Value = SymbolMap for that port.
 */
using SymbolCache = std::unordered_map<uint16_t, SymbolMap>;
/**
 * One entry in the JSON-derived symbol dictionary.
 * Populated at construction from ads_symbol_dict.json.
 * handle filled by resolveSymbolHandles() via SUMUP.
 */
struct AdsSymbolDictEntry
{
    std::string symbol;      // PLC path as-is  e.g. "GVL_Logger.bTrickleTripped"
    std::string symbolLower; // lowercase key for case-insensitive lookup
    std::string datatype;    // "BOOL", "LREAL", "DINT" etc.
    std::string pvName;      // EPICS PV name
    uint32_t size   = 0;     // byte size
    uint32_t adst   = 0;     // ADST_* type code
    uint32_t handle = 0;     // ADS variable handle — filled by SUMUP
    bool resolved   = false; // true after handle obtained
};

/** key = lowercase symbol path */
using SymbolDict = std::unordered_map<std::string, AdsSymbolDictEntry>;


#endif /* ADS_SYMBOL_TABLE_H */
