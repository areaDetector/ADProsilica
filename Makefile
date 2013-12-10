#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) configure
DIRS := $(DIRS) prosilicaSupport
DIRS := $(DIRS) prosilicaApp
prosilicaApp_DEPEND_DIRS += prosilicaSupport
ifeq ($(BUILD_APPS), YES)
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard iocs))
iocs_DEPEND_DIRS += prosilicaApp
endif
include $(TOP)/configure/RULES_TOP

uninstall: uninstall_iocs
uninstall_iocs:
	$(MAKE) -C iocs uninstall
.PHONY: uninstall uninstall_iocs

realuninstall: realuninstall_iocs
realuninstall_iocs:
	$(MAKE) -C iocs realuninstall
.PHONY: realuninstall realuninstall_iocs

