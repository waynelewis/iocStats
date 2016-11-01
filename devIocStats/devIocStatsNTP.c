/*************************************************************************\
* Copyright (c) 2009-2010 Helmholtz-Zentrum Berlin
*     fuer Materialien und Energie GmbH.
* Copyright (c) 2002 The University of Chicago, as Operator of Argonne
*     National Laboratory.
* Copyright (c) 2002 The Regents of the University of California, as
*     Operator of Los Alamos National Laboratory.
* EPICS BASE Versions 3.13.7
* and higher are distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
\*************************************************************************/

/* devIocStatsNTP.c - device support routines for NTP statistics - based on */
/* devIocStatsAnalog.c - Analog Device Support Routines for IOC statistics - based on */
/* devVXStats.c - Device Support Routines for vxWorks statistics */
/*
 *	Author: Jim Kowalkowski
 *	Date:  2/1/96
 *
 * Modifications at LBNL:
 * -----------------
 * 	97-11-21	SRJ	Added reports of max free mem block,
 *				Channel Access connections and CA clients.
 *				Repaired "artificial load" function.
 *	98-01-28	SRJ	Changes per M. Kraimer's devVXStats of 97-11-19:
 *				explicitly reports file descriptors used;
 *				uses Kraimer's method for CPU load average;
 *				some code simplification and name changes.
 *
 * Modifications for SNS at ORNL:
 * -----------------
 *	03-01-29	CAL 	Add stringin device support.
 *	03-05-08	CAL	Add minMBuf
 *
 * Modifications for LCLS/SPEAR at SLAC:
 * ----------------
 *  08-09-29    Stephanie Allison - moved os-specific parts to
 *              os/<os>/devIocStatsOSD.h and devIocStatsOSD.c.
 *              Split into devIocStatsAnalog, devIocStatsString,
 *              devIocStatTest.
 *  2009-05-15  Ralph Lange (HZB/BESSY)
 *              Restructured OSD parts
 *  2010-07-14  Ralph Lange (HZB/BESSY)
 *              Added CPU Utilization (IOC load), number of CPUs
 *  2010-08-12  Stephanie Allison (SLAC):
 *              Added RAM workspace support developed by
 *              Charlie Xu.
 *  2015-04-27  Stephanie Allison (SLAC):
 *              Added process ID and parent process ID.
 *              Perform statistics in a task separate from the low priority
 *              callback task.
 *  2016-10-31  Wayne Lewis (Osprey DCS)
 *              Created new file from devIOCStatsAnalog.c to support NTP
 *              statistics.
 */

/*
	--------------------------------------------------------------------
	Note that the valid values for the parm field of the link
	information are:

	ai (DTYP="IOC stats NTP"):
        ntp_version         - NTP version number
        ntp_leap_second     - NTP leap second status
        ntp_stratum         - NTP server stratum
        ntp_precision       - NTP precision
        ntp_root_delay      - NTP root delay
        ntp_root_dispersion - NTP root dispersion
        ntp_tc              - NTP time constant
        ntp_min_tc          - NTP minimum time constant
        ntp_offset          - NTP offset
        ntp_frequency       - NTP frequency
        ntp_system_jitter   - NTP system jitter
        ntp_clock_jitter    - NTP clock jitter
        ntp_clock_wander    - NTP clock wander
        ntp_num_peers       - NTP number of potential peers 
        ntp_num_good_peers  - NTP number of candidate peers
        ntp_max_peer_offset - NTP maximum peer offset
        ntp_max_peer_jitter - NTP maximum peer jitter
        ntp_min_peer_stratum- NTP minimum peer jitter
        ntp_sync_status     - NTP daemon sync status
        ntp_peer_selections i - NTP daemon sync status
        ntp_peer_stratum i  - NTP daemon sync status
        ntp_peer_poll i     - NTP daemon sync status
        ntp_peer_reach i    - NTP daemon sync status
        ntp_peer_delay i    - NTP daemon sync status
        ntp_peer_offset i   - NTP daemon sync status
        ntp_peer_jitter i   - NTP daemon sync status

 * scan rates are all in seconds

 default rates:
 20 - daemon scan rate
*/

#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <epicsThread.h>
#include <epicsTimer.h>
#include <epicsMutex.h>

#include <rsrv.h>
#include <dbAccess.h>
#include <dbStaticLib.h>
#include <dbScan.h>
#include <devSup.h>
#include <menuConvert.h>
#include <aiRecord.h>
#include <aoRecord.h>
#include <recGbl.h>
#include <epicsExport.h>

#include "devIocStats.h"

#define INT_ZERO 0
#define DOUBLE_ZERO 0.0
#define INT_ARRAY_ZERO {0,0,0,0,0,0,0,0,0,0}
#define DOUBLE_ARRAY_ZERO {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}

struct aStatsNTP
{
    long		number;
    DEVSUPFUN	report;
    DEVSUPFUN	init;
    DEVSUPFUN	init_record;
    DEVSUPFUN	get_ioint_info;
    DEVSUPFUN	read_write;
    DEVSUPFUN	special_linconv;
};
typedef struct aStatsNTP aStatsNTP;

struct aStatsNTPPeer
{
    long		number;
    DEVSUPFUN	report;
    DEVSUPFUN	init;
    DEVSUPFUN	init_record;
    DEVSUPFUN	get_ioint_info;
    DEVSUPFUN	read_write;
    DEVSUPFUN	special_linconv;
};
typedef struct aStatsNTPPeer aStatsNTPPeer;

struct pvtNTPArea
{
    int index;
    int type;
    int peer;
};
typedef struct pvtNTPArea pvtNTPArea;

typedef void (*statNTPGetFunc)(double*);
typedef void (*statNTPPeerGetFunc)(double*, int);

struct validNTPGetParms
{
    char* name;
    statNTPGetFunc func;
    int type;
};
typedef struct validNTPGetParms validNTPGetParms;

struct validNTPPeerGetParms
{
    char* name;
    statNTPPeerGetFunc func;
    int type;
};
typedef struct validNTPPeerGetParms validNTPPeerGetParms;

struct scanNTPInfo
{
    IOSCANPVT ioscan;
    epicsTimerId  wd;
    volatile int total;			/* total users connected */
    volatile int on;			/* watch dog on? */
    double rate_sec;	/* seconds */
};
typedef struct scanNTPInfo scanNTPInfo;

static long ai_ntp_init(int pass);
static long ai_ntp_init_record(aiRecord*);
static long ai_ntp_read(aiRecord*);
static long ai_ntp_ioint_info(int cmd,aiRecord* pr,IOSCANPVT* iopvt);

static long ai_ntp_peer_init_record(aiRecord*);
static long ai_ntp_peer_read(aiRecord*);

static void statsNTPVersion(double *);
static void statsNTPLeapSecond(double *);
static void statsNTPStratum(double *);
static void statsNTPPrecision(double *);
static void statsNTPRootDelay(double *);
static void statsNTPRootDispersion(double *);
static void statsNTPTC(double *);
static void statsNTPMinTC(double *);
static void statsNTPOffset(double *);
static void statsNTPFrequency(double *);
static void statsNTPSystemJitter(double *);
static void statsNTPClockJitter(double *);
static void statsNTPClockWander(double *);
static void statsNTPNumPeers(double *);
static void statsNTPNumGoodPeers(double *);
static void statsNTPMaxPeerOffset(double *);
static void statsNTPMaxPeerJitter (double *);
static void statsNTPMinPeerStratum(double *);
static void statsNTPPeerSelection(double *, int);
static void statsNTPPeerStratum(double *, int);
static void statsNTPPeerPoll(double *, int);
static void statsNTPPeerReach(double *, int);
static void statsNTPPeerDelay(double *, int);
static void statsNTPPeerOffset(double *, int);
static void statsNTPPeerJitter(double *, int);

struct {
	char *name;
	double scan_rate;
} parmNTPTypes[] = {
    { "memory_scan_rate",   10.0 },
    { "cpu_scan_rate",  20.0 },
    { "fd_scan_rate",   10.0 },
    { "ca_scan_rate",   15.0 },
    { "static_scan_rate",   0.0 },
    { "daemon_scan_rate",   20.0 },
	{ NULL,			0.0  },
};

static validNTPGetParms statsGetNTPParms[]={
    { "ntp_version",        statsNTPVersion,        DAEMON_TYPE },
    { "ntp_leap_second",    statsNTPLeapSecond,     DAEMON_TYPE },
    { "ntp_stratum",        statsNTPStratum,        DAEMON_TYPE },
    { "ntp_precision",      statsNTPPrecision,      DAEMON_TYPE },
    { "ntp_root_delay",     statsNTPRootDelay,      DAEMON_TYPE },
    { "ntp_root_dispersion",statsNTPRootDispersion, DAEMON_TYPE },
    { "ntp_tc",             statsNTPTC,             DAEMON_TYPE },
    { "ntp_min_tc",         statsNTPMinTC,          DAEMON_TYPE },
    { "ntp_offset",         statsNTPOffset,         DAEMON_TYPE },
    { "ntp_frequency",      statsNTPFrequency,      DAEMON_TYPE },
    { "ntp_system_jitter",  statsNTPSystemJitter,   DAEMON_TYPE },
    { "ntp_clock_jitter",   statsNTPClockJitter,    DAEMON_TYPE },
    { "ntp_clock_wander",   statsNTPClockWander,    DAEMON_TYPE },
    { "ntp_num_peers",      statsNTPNumPeers,       DAEMON_TYPE },
    { "ntp_num_good_peers", statsNTPNumGoodPeers,   DAEMON_TYPE },
    { "ntp_max_peer_offset",statsNTPMaxPeerOffset,  DAEMON_TYPE },
    { "ntp_max_peer_jitter",statsNTPMaxPeerJitter,  DAEMON_TYPE },
    { "ntp_min_peer_stratum",statsNTPMinPeerStratum,DAEMON_TYPE },
	{ NULL,NULL,0 }
};

static validNTPPeerGetParms statsGetNTPPeerParms[]={
    { "ntp_peer_selection", statsNTPPeerSelection, DAEMON_TYPE },
    { "ntp_peer_stratum",   statsNTPPeerStratum,   DAEMON_TYPE },
    { "ntp_peer_poll",      statsNTPPeerPoll,      DAEMON_TYPE },
    { "ntp_peer_reach",     statsNTPPeerReach,     DAEMON_TYPE },
    { "ntp_peer_delay",     statsNTPPeerDelay,     DAEMON_TYPE },
    { "ntp_peer_offset",    statsNTPPeerOffset,    DAEMON_TYPE },
    { "ntp_peer_jitter",    statsNTPPeerJitter,    DAEMON_TYPE },
	{ NULL,NULL,0 }
};

aStatsNTP devAiNTPStats={ 6,NULL,ai_ntp_init,ai_ntp_init_record,ai_ntp_ioint_info,ai_ntp_read,NULL };

epicsExportAddress(dset,devAiNTPStats);

aStatsNTP devAiNTPPeerStats={ 6,NULL,ai_ntp_init,ai_ntp_peer_init_record,ai_ntp_ioint_info,ai_ntp_peer_read,NULL };

epicsExportAddress(dset,devAiNTPPeerStats);

static ntpStatus ntpstatus = 
{
    INT_ZERO,
    INT_ZERO,
    INT_ZERO,
    INT_ZERO,
    DOUBLE_ZERO,
    DOUBLE_ZERO,
    INT_ZERO,
    INT_ZERO,
    DOUBLE_ZERO,
    DOUBLE_ZERO,
    DOUBLE_ZERO,
    DOUBLE_ZERO,
    DOUBLE_ZERO,
    INT_ZERO,
    INT_ZERO,
    DOUBLE_ZERO,
    DOUBLE_ZERO,
    DOUBLE_ZERO,
    INT_ZERO,
    INT_ZERO,
    INT_ARRAY_ZERO,
    INT_ARRAY_ZERO,
    INT_ARRAY_ZERO,
    INT_ARRAY_ZERO,
    DOUBLE_ARRAY_ZERO,
    DOUBLE_ARRAY_ZERO,
    DOUBLE_ARRAY_ZERO
};

static scanNTPInfo scanNTP[TOTAL_TYPES] = {{0}};
static epicsTimerQueueId ntpTimerQ = 0;
static epicsMutexId ntp_scan_mutex;


/* ---------------------------------------------------------------------- */

/* 
 * Run timer task just below the low priority callback task and higher
 * than the default for the channel access tasks.  Also, set okToShare to 0
 * so that the task is dedicated to devIocStats.
 */
static void ntpTimerQCreate(void*unused)
{
	ntpTimerQ = epicsTimerQueueAllocate(0, epicsThreadPriorityScanLow - 2);
}

static epicsTimerId
wdogCreate(void (*fn)(int), long arg)
{
	static epicsThreadOnceId inited = EPICS_THREAD_ONCE_INIT;

	/* lazy init of timer queue */
	if ( EPICS_THREAD_ONCE_INIT == inited )
		epicsThreadOnce( &inited, ntpTimerQCreate, 0);

	return epicsTimerQueueCreateTimer(ntpTimerQ, (void (*)(void*))fn, (void*)arg);
}

static void scan_ntp_time(int type)
{
    switch(type) {
      case DAEMON_TYPE:
      {
          ntpStatus ntpstatus_local = 
          {
              INT_ZERO,
              INT_ZERO,
              INT_ZERO,
              INT_ZERO,
              DOUBLE_ZERO,
              DOUBLE_ZERO,
              INT_ZERO,
              INT_ZERO,
              DOUBLE_ZERO,
              DOUBLE_ZERO,
              DOUBLE_ZERO,
              DOUBLE_ZERO,
              DOUBLE_ZERO,
              INT_ZERO,
              INT_ZERO,
              DOUBLE_ZERO,
              DOUBLE_ZERO,
              DOUBLE_ZERO,
              INT_ZERO,
              INT_ZERO,
              INT_ARRAY_ZERO,
              INT_ARRAY_ZERO,
              INT_ARRAY_ZERO,
              INT_ARRAY_ZERO,
              DOUBLE_ARRAY_ZERO,
              DOUBLE_ARRAY_ZERO,
              DOUBLE_ARRAY_ZERO
          };
          devIocStatsGetNtpStats(&ntpstatus_local);
          epicsMutexLock(ntp_scan_mutex);
          ntpstatus = ntpstatus_local;
          epicsMutexUnlock(ntp_scan_mutex);
          break;
      }
      default:
      break;
    }
    scanIoRequest(scanNTP[type].ioscan);
    if(scanNTP[type].on)
		epicsTimerStartDelay(scanNTP[type].wd, scanNTP[type].rate_sec);
}

/* -------------------------------------------------------------------- */

static long ai_ntp_init(int pass)
{
    long i;

    if (pass) return 0;

    /* Create timers */
    for (i = 0; i < TOTAL_TYPES; i++) {
        scanIoInit(&scanNTP[i].ioscan);
        scanNTP[i].wd = wdogCreate(scan_ntp_time, i);
        scanNTP[i].total = 0;
        scanNTP[i].on = 0;
        scanNTP[i].rate_sec = parmNTPTypes[i].scan_rate;
    }

    /* Init OSD stuff */
    ntp_scan_mutex = epicsMutexMustCreate();
    return 0;
}

static long ai_ntp_init_record(aiRecord* pr)
{
    int		i;
    char	*parm;
    pvtNTPArea	*pvtNTP = NULL;

    if(pr->inp.type!=INST_IO)
    {
        recGblRecordError(S_db_badField,(void*)pr,
                "devAiNTPStats (init_record) Illegal INP field");
        return S_db_badField;
    }
    parm = pr->inp.value.instio.string;
    for(i=0;statsGetNTPParms[i].name && pvtNTP==NULL;i++)
    {
        if(strcmp(parm,statsGetNTPParms[i].name)==0)
        {
            pvtNTP=(pvtNTPArea*)malloc(sizeof(pvtNTPArea));
            pvtNTP->index=i;
            pvtNTP->type=statsGetNTPParms[i].type;
            pvtNTP->peer = 0;
        }
    }

    if(pvtNTP==NULL)
    {
        recGblRecordError(S_db_badField,(void*)pr,
                "devAiNTPStats (init_record) Illegal INP parm field");
        return S_db_badField;
    }

    /* Make sure record processing routine does not perform any conversion*/
    pr->linr=menuConvertNO_CONVERSION;
    pr->dpvt=pvtNTP;
    return 0;
}

static long ai_ntp_peer_init_record(aiRecord* pr)
{
    int		i;
    char	*parm;
    char    *parameter;
    char    pattern[] = "%ms %d";
    int     peer;
    int     elements;
    pvtNTPArea	*pvtNTP = NULL;

    if(pr->inp.type!=INST_IO)
    {
        recGblRecordError(S_db_badField,(void*)pr,
                "devAiNTPPeerStats (init_record) Illegal INP field");
        return S_db_badField;
    }
    parm = pr->inp.value.instio.string;
    for(i=0;statsGetNTPPeerParms[i].name && pvtNTP==NULL;i++)
    {
        // Test how many elements are in the string
        // If there is one, then the parameter is non-array type
        // If there are two, then the integer indicates the peer index
        //
        elements = sscanf(parm, pattern, &parameter, &peer);
        if(strcmp(parameter,statsGetNTPPeerParms[i].name)==0)
            if (elements == 2) 
            {
                pvtNTP=(pvtNTPArea*)malloc(sizeof(pvtNTPArea));
                pvtNTP->index=i;
                pvtNTP->type=statsGetNTPPeerParms[i].type;
                pvtNTP->peer = peer;
            }
    }

    free(parameter);

    if(pvtNTP==NULL)
    {
        recGblRecordError(S_db_badField,(void*)pr,
                "devAiNTPPeerStats (init_record) Illegal INP parm field");
        return S_db_badField;
    }

    /* Make sure record processing routine does not perform any conversion*/
    pr->linr=menuConvertNO_CONVERSION;
    pr->dpvt=pvtNTP;
    return 0;
}

static long ai_ntp_ioint_info(int cmd,aiRecord* pr,IOSCANPVT* iopvt)
{
    pvtNTPArea* pvtNTP=(pvtNTPArea*)pr->dpvt;

    if (!pvtNTP) return S_dev_badInpType;

    if(cmd==0) /* added */
    {
        if(scanNTP[pvtNTP->type].total++ == 0)
        {
            /* start a watchdog */
            epicsTimerStartDelay(scanNTP[pvtNTP->type].wd, scanNTP[pvtNTP->type].rate_sec);
            scanNTP[pvtNTP->type].on=1;
        }
    }
    else /* deleted */
    {
        if(--scanNTP[pvtNTP->type].total == 0)
            scanNTP[pvtNTP->type].on=0; /* stop the watchdog */
    }

    *iopvt=scanNTP[pvtNTP->type].ioscan;
    return 0;
}

/* Generic read - calling function from table */
static long ai_ntp_read(aiRecord* pr)
{
    double val;
    pvtNTPArea* pvtNTP=(pvtNTPArea*)pr->dpvt;

    if (!pvtNTP) return S_dev_badInpType;

    epicsMutexLock(ntp_scan_mutex);
    statsGetNTPParms[pvtNTP->index].func(&val);
    epicsMutexUnlock(ntp_scan_mutex);
    pr->val = val;
    pr->udf = 0;
    return 2; /* don't convert */
}

/* Generic read - calling function from table */
static long ai_ntp_peer_read(aiRecord* pr)
{
    double val;
    pvtNTPArea* pvtNTP=(pvtNTPArea*)pr->dpvt;

    if (!pvtNTP) return S_dev_badInpType;

    epicsMutexLock(ntp_scan_mutex);
    statsGetNTPPeerParms[pvtNTP->index].func(&val, pvtNTP->peer);
    epicsMutexUnlock(ntp_scan_mutex);
    pr->val = val;
    pr->udf = 0;
    return 2; /* don't convert */
}

/* -------------------------------------------------------------------- */

static void statsNTPVersion(double* val)
{
    *val = (double)ntpstatus.ntpVersionNumber;
}
static void statsNTPLeapSecond(double* val)
{
    *val = (double)ntpstatus.ntpLeapSecond;
}
static void statsNTPStratum(double* val)
{
    *val = (double)ntpstatus.ntpStratum;
}
static void statsNTPPrecision(double* val)
{
    *val = (double)ntpstatus.ntpPrecision;
}
static void statsNTPRootDelay(double* val)
{
    *val = (double)ntpstatus.ntpRootDelay;
}
static void statsNTPRootDispersion(double* val)
{
    *val = (double)ntpstatus.ntpRootDispersion;
}
static void statsNTPTC(double* val)
{
    *val = (double)ntpstatus.ntpTC;
}
static void statsNTPMinTC(double* val)
{
    *val = (double)ntpstatus.ntpMinTC;
}
static void statsNTPOffset(double* val)
{
    *val = (double)ntpstatus.ntpOffset;
}
static void statsNTPFrequency(double* val)
{
    *val = (double)ntpstatus.ntpFrequency;
}
static void statsNTPSystemJitter(double* val)
{
    *val = (double)ntpstatus.ntpSystemJitter;
}
static void statsNTPClockJitter(double* val)
{
    *val = (double)ntpstatus.ntpClockJitter;
}
static void statsNTPClockWander(double* val)
{
    *val = (double)ntpstatus.ntpClockWander;
}
static void statsNTPNumPeers(double* val)
{
    *val = (double)ntpstatus.ntpNumPeers;
}
static void statsNTPNumGoodPeers(double* val)
{
    *val = (double)ntpstatus.ntpNumGoodPeers;
}
static void statsNTPMaxPeerOffset(double* val)
{
    *val = (double)ntpstatus.ntpMaxPeerOffset;
}
static void statsNTPMaxPeerJitter(double* val)
{
    *val = (double)ntpstatus.ntpMaxPeerJitter;
}
static void statsNTPMinPeerStratum(double* val)
{
    *val = (double)ntpstatus.ntpMinPeerStratum;
}
/*
static void statsNTPSyncStatus(double* val)
{
    *val = (double)ntpstatus.ntpSyncStatus;
}
*/
static void statsNTPPeerSelection(double* val, int peer)
{
    *val = (double)ntpstatus.ntpPeerSelectionStatus[peer];
}
static void statsNTPPeerStratum(double* val, int peer)
{
    *val = (double)ntpstatus.ntpPeerStratums[peer];
}
static void statsNTPPeerPoll(double* val, int peer)
{
    *val = (double)ntpstatus.ntpPeerPolls[peer];
}
static void statsNTPPeerReach(double* val, int peer)
{
    *val = (double)ntpstatus.ntpPeerReaches[peer];
}
static void statsNTPPeerDelay(double* val, int peer)
{
    *val = (double)ntpstatus.ntpPeerDelays[peer];
}
static void statsNTPPeerOffset(double* val, int peer)
{
    *val = (double)ntpstatus.ntpPeerOffsets[peer];
}
static void statsNTPPeerJitter(double* val, int peer)
{
    *val = (double)ntpstatus.ntpPeerJitters[peer];
}
