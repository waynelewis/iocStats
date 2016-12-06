TOP=..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# For DTYP="IOC stats"
DBD += devIocStats.dbd
DBD += devIocStatsNTP.dbd

# For DTYP="VX stats"
DBD += devVxStats.dbd

INC += devIocStatsOSD.h

#=============================

LIBRARY_IOC = devIocStats

devIocStats_LIBS = $(EPICS_BASE_IOC_LIBS)
devIocStats_SYS_LIBS_solaris = kstat
devIocStats_SYS_LIBS_WIN32 += psapi

# OSI parts
devIocStats_SRCS += devIocStatsAnalog.c
devIocStats_SRCS += devIocStatsString.c
devIocStats_SRCS += devIocStatsWaveform.c
devIocStats_SRCS += devIocStatsSub.c
devIocStats_SRCS += devIocStatsRec.c
devIocStats_SRCS += devIocStatsTest.c
ifeq ($(USE_NTP), YES)
  devIocStats_SRCS += devIocStatsNTP.cpp
  devIocStats_SRCS += ntphelper.cpp
endif

# OSD parts
# Base 3.14 does not correctly define POSIX=NO for mingw
ifeq (mingw,$(findstring mingw, $(T_A)))
  POSIX=NO
endif
devIocStats_SRCS += osdCpuUsage.c
devIocStats_SRCS += osdCpuUtilization.c
devIocStats_SRCS += osdFdUsage.c
devIocStats_SRCS += osdMemUsage.c
devIocStats_SRCS += osdWorkspaceUsage.c
devIocStats_SRCS += osdClustInfo.c
devIocStats_SRCS += osdSuspTasks.c
devIocStats_SRCS += osdIFErrors.c
devIocStats_SRCS += osdBootInfo.c
devIocStats_SRCS += osdSystemInfo.c
devIocStats_SRCS += osdHostInfo.c
devIocStats_SRCS += osdPIDInfo.c

OBJS_vxWorks += osdCpuUsageTest.o

TESTPROD_HOST += testntphelper
TESTS += testntphelper
testntphelper_SRCS += testntphelper.cpp
testntphelper_SRCS += ntphelper.cpp
testntphelper_LIBS += Com

TESTSCRIPTS_HOST += $(TESTS:%=%.t)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

