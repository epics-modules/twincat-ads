#Makefile at top of the test IOC tree
TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure
DIRS += $(wildcard *App)
DIRS += $(wildcard iocBoot)

$(foreach dir, $(filter-out configure, $(DIRS)), \
    $(eval $(dir)_DEPEND_DIRS += configure))
iocBoot_DEPEND_DIRS += $(filter %App,$(DIRS))

include $(TOP)/configure/RULES_TOP
