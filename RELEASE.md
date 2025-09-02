# twincat-ads


## Release v2.1.1 (2025-09-02)
- Bugfix: Remove invalid adsUnlock()
- Add licence information for LGPL v3
- Improvements after a code review at ESS:
    Lots of cleanup. Improved documentation
## Release v2.1.0 (2020-01-23)
- Integrate changes from SLAC, be "on par" with the source code, more or less
  - New features:
    Make it possible to use polling instead of "on change subscription"
    (1Hz fixed) on a Record-by-Records configuration
   Support for 64 bit EPICS, asynInt64
 - Bugfixes:
   Various small improvements and bug fixes both from SLAC and ESS

 - Known limitations:
   Whenever the connection to the PLC is lost, exit() is called
   and the IOC is terminated. The IOC needs to be restarted,
   may be as a system service.

## Release v2.0.2 (2020-01-23)
- First Version that had up-to-date examples after moving from EEE to e3,
  both specific for ESS

## Older releases:
- Not worth to be documented here. PRs are welcome.
