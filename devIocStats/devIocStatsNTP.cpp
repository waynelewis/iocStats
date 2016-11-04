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
#include <string>
#include <iostream>

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
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>

//#include "devIocStats.h"
#include "epicsTypes.h"
#include "devIocStatsNTP.h"

using namespace std;

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

static IOSCANPVT ioscanpvt;

struct pvtNTPArea
{
    int index;
    int type;
    int peer;
};
typedef struct pvtNTPArea pvtNTPArea;

typedef void (*statNTPGetFunc)(double*, int);

struct validNTPGetParms
{
    string name;
    statNTPGetFunc func;
};
typedef struct validNTPGetParms validNTPGetParms;

static long ai_ntp_init(int pass);
static long ai_ntp_init_record(aiRecord*);
static long ai_ntp_read(aiRecord*);
static long ai_ntp_ioint_info(int cmd,aiRecord* pr,IOSCANPVT* iopvt);

static void statsNTPVersion(double *, int peer=-1);
static void statsNTPLeapSecond(double *, int peer=-1);
static void statsNTPStratum(double *, int peer=-1);
static void statsNTPPrecision(double *, int peer=-1);
static void statsNTPRootDelay(double *, int peer=-1);
static void statsNTPRootDispersion(double *, int peer=-1);
static void statsNTPTC(double *, int peer=-1);
static void statsNTPMinTC(double *, int peer=-1);
static void statsNTPOffset(double *, int peer=-1);
static void statsNTPFrequency(double *, int peer=-1);
static void statsNTPSystemJitter(double *, int peer=-1);
static void statsNTPClockJitter(double *, int peer=-1);
static void statsNTPClockWander(double *, int peer=-1);
static void statsNTPNumPeers(double *, int peer=-1);
static void statsNTPNumGoodPeers(double *, int peer=-1);
static void statsNTPMaxPeerOffset(double *, int peer=-1);
static void statsNTPMaxPeerJitter (double *, int peer=-1);
static void statsNTPMinPeerStratum(double *, int peer=-1);
static void statsNTPPeerSelection(double *, int);
static void statsNTPPeerStratum(double *, int);
static void statsNTPPeerPoll(double *, int);
static void statsNTPPeerReach(double *, int);
static void statsNTPPeerDelay(double *, int);
static void statsNTPPeerOffset(double *, int);
static void statsNTPPeerJitter(double *, int);

static validNTPGetParms statsGetNTPParms[]={
    { "ntp_version",        statsNTPVersion },
    { "ntp_leap_second",    statsNTPLeapSecond },
    { "ntp_stratum",        statsNTPStratum },
    { "ntp_precision",      statsNTPPrecision },
    { "ntp_root_delay",     statsNTPRootDelay },
    { "ntp_root_dispersion",statsNTPRootDispersion },
    { "ntp_tc",             statsNTPTC },
    { "ntp_min_tc",         statsNTPMinTC },
    { "ntp_offset",         statsNTPOffset },
    { "ntp_frequency",      statsNTPFrequency },
    { "ntp_system_jitter",  statsNTPSystemJitter },
    { "ntp_clock_jitter",   statsNTPClockJitter },
    { "ntp_clock_wander",   statsNTPClockWander },
    { "ntp_num_peers",      statsNTPNumPeers },
    { "ntp_num_good_peers", statsNTPNumGoodPeers },
    { "ntp_max_peer_offset",statsNTPMaxPeerOffset },
    { "ntp_max_peer_jitter",statsNTPMaxPeerJitter },
    { "ntp_min_peer_stratum",statsNTPMinPeerStratum },
    { "ntp_peer_selection", statsNTPPeerSelection },
    { "ntp_peer_stratum",   statsNTPPeerStratum },
    { "ntp_peer_poll",      statsNTPPeerPoll },
    { "ntp_peer_reach",     statsNTPPeerReach },
    { "ntp_peer_delay",     statsNTPPeerDelay },
    { "ntp_peer_offset",    statsNTPPeerOffset },
    { "ntp_peer_jitter",    statsNTPPeerJitter },
	{ "",NULL }
};

aStatsNTP devAiNTPStats={ 
    6,
    NULL,
    (DEVSUPFUN)ai_ntp_init,
    (DEVSUPFUN)ai_ntp_init_record,
    (DEVSUPFUN)ai_ntp_ioint_info,
    (DEVSUPFUN)ai_ntp_read,
    NULL };

epicsExportAddress(dset,devAiNTPStats);

static ntpStatus ntpstatus;
static epicsMutexId ntp_scan_mutex;

static void poll_ntp_daemon(void)
{

    while(1)
    {

        devIocStatsGetNtpStats(&ntpstatus);

        scanIoRequest(ioscanpvt);
        epicsThreadSleep(20);
    }
}


static long ai_ntp_init(int pass)
{

    if (pass) return 0;

    // Create a thread to poll the NTP daemon and populate the data structure
    if (epicsThreadCreate(
                "NTP_poller_thread",
                epicsThreadPriorityLow,
                epicsThreadGetStackSize(epicsThreadStackSmall),
                (EPICSTHREADFUNC)poll_ntp_daemon,
                NULL) == NULL)
    {
        fprintf(stderr, "epicsThreadCreate failure\n");
        return -1;
    }

    scanIoInit(&ioscanpvt);

    ntp_scan_mutex = epicsMutexMustCreate();

    return 0;
}

static long ai_ntp_init_record(aiRecord* pr)
{
    int		i;
    string	parm;
    string  parameter;
    size_t  index;
    int     peer;
    pvtNTPArea	*pvtNTP = NULL;

    // Check the record INP type
    if(pr->inp.type!=INST_IO)
    {
        recGblRecordError(S_db_badField,(void*)pr,
                "devAiNTPStats (init_record) Illegal INP field");
        return S_db_badField;
    }

    parm = (string)pr->inp.value.instio.string;
    for(i=0; pvtNTP==NULL; i++)
    {
        // Test if there is a space in the INP string.
        // If there is, then the peer number will follow.
        index = parm.find(" ");

        if (index == string::npos)
        {
            // System variable
            parameter = parm;
            peer = -1;
        }
        else
        {
            // Peer variable
            parameter = parm.substr(0, index);
            peer = atoi(parm.substr(index+1).c_str());
        }

        // Find the correct function in the list
        if(parameter.compare(statsGetNTPParms[i].name)==0)
        {
            pvtNTP=(pvtNTPArea*)malloc(sizeof(pvtNTPArea));
            pvtNTP->index=i;
            pvtNTP->peer = peer;
        }
    }

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

// I/O interrupt initialization
static long ai_ntp_ioint_info(int cmd, aiRecord* pr, IOSCANPVT* iopvt)
{
    pvtNTPArea* pvtNTP=(pvtNTPArea*)pr->dpvt;

    if (!pvtNTP) return S_dev_badInpType;

    *iopvt = ioscanpvt;

    return 0;
}

/* Generic read - calling function from table */
static long ai_ntp_read(aiRecord* pr)
{
    double val;
    pvtNTPArea* pvtNTP=(pvtNTPArea*)pr->dpvt;

    if (!pvtNTP) return S_dev_badInpType;

    epicsMutexLock(ntp_scan_mutex);
    statsGetNTPParms[pvtNTP->index].func(&val,pvtNTP->peer);
    epicsMutexUnlock(ntp_scan_mutex);
    pr->val = val;
    pr->udf = 0;
    return 2; /* don't convert */
}

/* -------------------------------------------------------------------- */

static void statsNTPVersion(double* val, int)
{
    *val = (double)ntpstatus.ntpVersionNumber;
}
static void statsNTPLeapSecond(double* val, int)
{
    *val = (double)ntpstatus.ntpLeapSecond;
}
static void statsNTPStratum(double* val, int)
{
    *val = (double)ntpstatus.ntpStratum;
}
static void statsNTPPrecision(double* val, int)
{
    *val = (double)ntpstatus.ntpPrecision;
}
static void statsNTPRootDelay(double* val, int)
{
    *val = (double)ntpstatus.ntpRootDelay;
}
static void statsNTPRootDispersion(double* val, int)
{
    *val = (double)ntpstatus.ntpRootDispersion;
}
static void statsNTPTC(double* val, int)
{
    *val = (double)ntpstatus.ntpTC;
}
static void statsNTPMinTC(double* val, int)
{
    *val = (double)ntpstatus.ntpMinTC;
}
static void statsNTPOffset(double* val, int)
{
    *val = (double)ntpstatus.ntpOffset;
}
static void statsNTPFrequency(double* val, int)
{
    *val = (double)ntpstatus.ntpFrequency;
}
static void statsNTPSystemJitter(double* val, int)
{
    *val = (double)ntpstatus.ntpSystemJitter;
}
static void statsNTPClockJitter(double* val, int)
{
    *val = (double)ntpstatus.ntpClockJitter;
}
static void statsNTPClockWander(double* val, int)
{
    *val = (double)ntpstatus.ntpClockWander;
}
static void statsNTPNumPeers(double* val, int)
{
    *val = (double)ntpstatus.ntpNumPeers;
}
static void statsNTPNumGoodPeers(double* val, int)
{
    *val = (double)ntpstatus.ntpNumGoodPeers;
}
static void statsNTPMaxPeerOffset(double* val, int)
{
    *val = (double)ntpstatus.ntpMaxPeerOffset;
}
static void statsNTPMaxPeerJitter(double* val, int)
{
    *val = (double)ntpstatus.ntpMaxPeerJitter;
}
static void statsNTPMinPeerStratum(double* val, int)
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
    *val = (double)ntpstatus.ntp_peer_data[peer].ntpPeerSelectionStatus;
}
static void statsNTPPeerStratum(double* val, int peer)
{
    *val = (double)ntpstatus.ntp_peer_data[peer].ntpPeerStratum;
}
static void statsNTPPeerPoll(double* val, int peer)
{
    *val = (double)ntpstatus.ntp_peer_data[peer].ntpPeerPoll;
}
static void statsNTPPeerReach(double* val, int peer)
{
    *val = (double)ntpstatus.ntp_peer_data[peer].ntpPeerReach;
}
static void statsNTPPeerDelay(double* val, int peer)
{
    *val = (double)ntpstatus.ntp_peer_data[peer].ntpPeerDelay;
}
static void statsNTPPeerOffset(double* val, int peer)
{
    *val = (double)ntpstatus.ntp_peer_data[peer].ntpPeerOffset;
}
static void statsNTPPeerJitter(double* val, int peer)
{
    *val = (double)ntpstatus.ntp_peer_data[peer].ntpPeerJitter;
}


int devIocStatsInitNtpStats (void) {
    return 0;
}

int devIocStatsGetNtpStats (ntpStatus *pval)
{
    struct ntp_control ntp_message;
    int ret;

    unsigned short association_ids[NTP_MAX_PEERS];
    unsigned short peer_selections[NTP_MAX_PEERS];
    int num_associations;

    // Perform an NTP variable query to get the system level status
    if (( ret = do_ntp_query(
                    NTP_OP_READ_VAR, 
                    NTP_SYS_ASSOCIATION_ID, 
                    &ntp_message) != 0))
        return ret;

    parse_ntp_sys_vars(&ntp_message, pval);

    // Perform an NTP status query to get the association IDs
    if ((ret = get_association_ids(
                    association_ids,
                    peer_selections,
                    &num_associations,
                    NTP_MAX_PEERS) != 0))
        return ret;

    parse_ntp_associations(
            association_ids,
            peer_selections,
            num_associations,
            pval);

    // Get stats from the peers

    ret = get_peer_stats(
            association_ids,
            num_associations,
            pval);

    if (ret < NTP_NO_ERROR)
        return ret;

    return NTP_NO_ERROR;

}

void parse_ntp_associations(
        unsigned short *association_ids,
        unsigned short *peer_selections,
        int num_associations,
        ntpStatus *pval)
{
    int i;
    int num_good_peers;
    int reference_peer;

    reference_peer = FALSE;

    pval->ntpNumPeers = num_associations;

    // Count the number of peers at candidate level or above
    num_good_peers = 0;

    for (i = 0; i < num_associations; i++)
    {
        if (peer_selections[i] >= NTP_PEER_SEL_CANDIDATE)
            num_good_peers++;

        if (peer_selections[i] >= NTP_PEER_SEL_SYSPEER)
            reference_peer = TRUE;
    }

    pval->ntpNumGoodPeers = num_good_peers;

    // If we have at least one good peer, set the sync status to good
    if (reference_peer == TRUE)
        pval->ntpSyncStatus = NTP_SYNC_STATUS_NTP;
    else
        pval->ntpSyncStatus = NTP_SYNC_STATUS_UNSYNC;

    // Store the selection statuses
    //
    for (i = 0; i < num_associations; i++)
        pval->ntp_peer_data[i].ntpPeerSelectionStatus = peer_selections[i];

}

void parse_ntp_sys_vars(
        struct ntp_control *ntp_message,
        ntpStatus *pval)
{
    int ntp_version;
    //int ntp_mode;
    //int ntp_more_bit;
    //int ntp_error_bit;
    //int ntp_response;

    //const char NTP_VERSION[] = "version=";
    //const char NTP_PROCESSOR[] = "processor=";
    //const char NTP_SYSTEM[] = "system=";
    const char NTP_LEAP[] = "leap=";
    const char NTP_STRATUM[] = "stratum=";
    const char NTP_PRECISION[] = "precision=";
    const char NTP_ROOT_DELAY[] = "rootdelay=";
    const char NTP_ROOT_DISPERSION[] = "rootdisp=";
    //const char NTP_REF_ID[] = "refid=";
    //const char NTP_REF_TIME[] = "reftime=";
    //const char NTP_CLOCK[] = "clock=";
    //const char NTP_PEER[] = "peer=";
    const char NTP_TC[] = "tc=";
    const char NTP_MINTC[] = "mintc=";
    const char NTP_OFFSET[] = "offset=";
    const char NTP_FREQUENCY[] = "frequency=";
    const char NTP_SYS_JITTER[] = "sys_jitter=";
    const char NTP_CLOCK_JITTER[] = "clk_jitter=";
    const char NTP_CLOCK_WANDER[] = "clk_wander=";
    
    /* Variables used to parse the NTP response string */
    char buffer[DATA_SIZE];
    char *substr;
    char *ntp_param_value;

    /* Extract the NTP version number */
    ntp_version = (ntp_message->ver_mode & VER_MASK) >> VER_SHIFT;
    pval->ntpVersionNumber = ntp_version;

    /* Leap second status */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_LEAP)))
    {
        substr += sizeof(NTP_LEAP) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpLeapSecond = (int)(atoi(ntp_param_value));
    }

    /* Stratum */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_STRATUM)))
    {
        substr += sizeof(NTP_STRATUM) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpStratum = (double)(atof(ntp_param_value));
    }

    /* Precision */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_PRECISION)))
    {
        substr += sizeof(NTP_PRECISION) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpPrecision = (int)(atoi(ntp_param_value));
    }

    /* Root delay */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_ROOT_DELAY)))
    {
        substr += sizeof(NTP_ROOT_DELAY) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpRootDelay = (double)(atof(ntp_param_value));
    }

    /* Root dispersion */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_ROOT_DISPERSION)))
    {
        substr += sizeof(NTP_ROOT_DISPERSION) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpRootDispersion = (double)(atof(ntp_param_value));
    }

    /* Time constant */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_TC)))
    {
        substr += sizeof(NTP_TC) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpTC = (int)(atoi(ntp_param_value));
    }

    /* Minimum time constant */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_MINTC)))
    {
        substr += sizeof(NTP_MINTC) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpMinTC = (int)(atoi(ntp_param_value));
    }

    /* Offset */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_OFFSET)))
    {
        substr += sizeof(NTP_OFFSET) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpOffset = (double)(atof(ntp_param_value));
    }

    /* Frequency */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_FREQUENCY)))
    {
        substr += sizeof(NTP_FREQUENCY) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpFrequency = (double)(atof(ntp_param_value));
    }

    /* System jitter */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_SYS_JITTER)))
    {
        substr += sizeof(NTP_SYS_JITTER) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpSystemJitter = (double)(atof(ntp_param_value));
    }

    /* Clock jitter */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_CLOCK_JITTER)))
    {
        substr += sizeof(NTP_CLOCK_JITTER) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpClockJitter = (double)(atof(ntp_param_value));
    }

    /* Clock wander */
    strncpy(buffer, ntp_message->data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_CLOCK_WANDER)))
    {
        substr += sizeof(NTP_CLOCK_WANDER) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpClockWander = (double)(atof(ntp_param_value));
    }
}

int do_ntp_query(
        unsigned char op_code,
        unsigned short association_id,
        struct ntp_control *ntp_message
        )
{
    struct sockaddr_in  ntp_socket;
    struct in_addr address;
    int sd;
    int ret;
    fd_set fds;
    struct timeval timeout_val;
    FD_ZERO(&fds);

    // Set up the socket to the local NTP daemon
    inet_aton("127.0.0.1", &address);
    ntp_socket.sin_family = AF_INET;
    ntp_socket.sin_addr = address;
    ntp_socket.sin_port = htons(NTP_PORT);

    /* Create the socket */
    if ((sd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
        return NTP_SOCKET_OPEN_ERROR;

    if (connect(sd, (struct sockaddr *)&ntp_socket, sizeof(ntp_socket)) < 0)
        return NTP_SOCKET_CONNECT_ERROR;

    FD_SET(sd, &fds);

    // Initialise the NTP control packet
    memset(ntp_message, 0, sizeof(struct ntp_control));

    // Populate the NTP control packet
    ntp_message->ver_mode = NTP_VER_MODE;
    ntp_message->op_code = op_code;
    ntp_message->association_id = association_id;

    timeout_val.tv_sec = 1;
    timeout_val.tv_usec = 0;

    // Send the request to the NTP daemon
    if (send(sd, ntp_message, sizeof(*ntp_message), 0) < 0)
        return NTP_COMMAND_SEND_ERROR;

    // Wait for a response
    ret = select(sd+1, &fds, (fd_set *)0, (fd_set *)0, &timeout_val);

    if (ret == 0)
        return NTP_TIMEOUT_ERROR;

    if (ret == -1)
        return NTP_SELECT_ERROR;

    // TODO; Make sure the full response is read from the daemon
    // TODO: Allow for UDP packet ordering
    //
    // Read the response
    if ((ret = recv(sd, ntp_message, sizeof(*ntp_message), 0)) < 0)
        return NTP_DAEMON_COMMS_ERROR;

    return 0;

}

// Get the following statistics from the peers:
// - largest offset
// - largest jitter
// - minimum stratum
//
// Get the following information for each peer:
// - stratum
// - polling rate
// - reach value
// - delay
// - offset
// - jitter
int get_peer_stats(
        unsigned short *association_ids,
        int num_peers,
        ntpStatus *pval
        )
{
    int i;
    int ret;

    string ntp_data;
    string substr;
    string ntp_param_value;

    string NTP_PEER_STRATUM ("stratum=");
    string NTP_PEER_POLL ("ppoll=");
    string NTP_PEER_REACH ("reach=");
    string NTP_ROOT_DELAY ("rootdelay=");
    string NTP_PEER_DELAY ("delay=");
    string NTP_PEER_OFFSET ("offset=");
    string NTP_PEER_JITTER ("jitter=");
    string SEPARATOR (",");

    int stratums[NTP_MAX_PEERS];
    int polls[NTP_MAX_PEERS];
    int reaches[NTP_MAX_PEERS];
    double delays[NTP_MAX_PEERS];
    double offsets[NTP_MAX_PEERS];
    double jitters[NTP_MAX_PEERS];

    double max_delay;
    double max_jitter;
    double max_offset;
    double min_stratum;

    size_t separator;
    size_t found;
    size_t length;
    size_t start;

    struct ntp_control ntp_message;

    // Iterate through the associated peers and gather the required data
    for (i = 0; i < num_peers; i++)
    {
        ret = do_ntp_query(NTP_OP_READ_STS, association_ids[i], &ntp_message);
        if (ret < 0)
            return ret;

        /* Peer stratum */
        //strncpy(buffer, ntp_message.data, sizeof(buffer));
        ntp_data = string(ntp_message.data);

        if (((found= ntp_data.find(NTP_PEER_STRATUM)) != string::npos))
        {
            separator = ntp_data.find(SEPARATOR, found);
            start = found + NTP_PEER_STRATUM.length();
            length = separator - start;
            ntp_param_value = ntp_data.substr(start, length);
            stratums[i] = atoi(ntp_param_value.c_str());
        }

        /* Peer poll */
        //strncpy(buffer, ntp_message.data, sizeof(buffer));
        //if ((substr = strstr(buffer, NTP_PEER_POLL)))
        //{
            ////substr += sizeof(NTP_PEER_POLL) - 1;
            //ntp_param_value = strtok(substr, ",");

            //polls[i] = (int)(atoi(ntp_param_value));
        //}

        /* Peer reach */
        //strncpy(buffer, ntp_message.data, sizeof(buffer));
        //if ((substr = strstr(buffer, NTP_PEER_REACH)))
        //{
            //substr += sizeof(NTP_PEER_REACH) - 1;
            //ntp_param_value = strtok(substr, ",");
//
            //reaches[i] = (int)(atoi(ntp_param_value));
        //}

        /* Peer delay */
        //strncpy(buffer, ntp_message.data, sizeof(buffer));
        /* First go past the root delay */
        //if ((substr = strstr(buffer, NTP_ROOT_DELAY)))
        //{
            //substr += sizeof(NTP_ROOT_DELAY);
            //strncpy(buffer, substr, sizeof(buffer));
            //if ((substr = strstr(buffer, NTP_PEER_DELAY)))
            //{
                //substr += sizeof(NTP_PEER_DELAY) - 1;
                //ntp_param_value = strtok(substr, ",");
//
                //delays[i] = (double)(atof(ntp_param_value));
            //}
        //}
//
        /* Peer offset */
        //strncpy(buffer, ntp_message.data, sizeof(buffer));
        //if ((substr = strstr(buffer, NTP_PEER_OFFSET)))
        //{
            //substr += sizeof(NTP_PEER_OFFSET) - 1;
            //ntp_param_value = strtok(substr, ",");
//
            //offsets[i] = (double)(atof(ntp_param_value));
        //}

        /* Peer jitter */
        //strncpy(buffer, ntp_message.data, sizeof(buffer));
        //if ((substr = strstr(buffer, NTP_PEER_JITTER)))
        ////{
            ////substr += sizeof(NTP_PEER_JITTER) - 1;
            //ntp_param_value = strtok(substr, ",");

            //jitters[i] = (double)(atof(ntp_param_value));
        //}

    }

    // Iterate through the gathered data and extract the required values

    max_delay = delays[0];
    max_offset = offsets[0];
    max_jitter = jitters[0];
    min_stratum = stratums[0];

    for (i = 1; i < num_peers; i++)
    {
        if (abs(offsets[i]) > abs(max_offset))
            max_offset = offsets[i];

        if (abs(delays[i]) > abs(max_delay))
            max_delay = delays[i];

        if (jitters[i] > max_jitter)
            max_jitter = jitters[i];

        if (stratums[i] < min_stratum)
            min_stratum = stratums[i];
    }

    // Transfer the peer data
    for (i = 0; i < num_peers; i++)
    {
        pval->ntp_peer_data[i].ntpPeerStratum = stratums[i];
        pval->ntp_peer_data[i].ntpPeerPoll = polls[i];
        pval->ntp_peer_data[i].ntpPeerReach = reaches[i];
        pval->ntp_peer_data[i].ntpPeerDelay = delays[i];
        pval->ntp_peer_data[i].ntpPeerOffset = offsets[i];
        pval->ntp_peer_data[i].ntpPeerJitter = jitters[i];
    }

    // Transfer the values 
    pval->ntpMaxPeerDelay = max_delay;
    pval->ntpMaxPeerOffset = max_offset;
    pval->ntpMaxPeerJitter = max_jitter;
    pval->ntpMinPeerStratum = min_stratum;

    return NTP_NO_ERROR;
}

int get_association_ids(
        unsigned short *association_ids, 
        unsigned short *peer_selections, 
        int *num_associations,
        int max_association_ids)
{
    struct ntp_control ntp_message;
    int i;
    int ret;
    int association_count;
    unsigned short association_id;
    int peer_sel;

    // Send a system level read status query
    ret = do_ntp_query(NTP_OP_READ_STS, NTP_SYS_ASSOCIATION_ID, &ntp_message);
    if (ret != 0)
        return ret;

    // Extract the association IDs from the response
    association_count = 0;
    for (i = 0; i < DATA_SIZE; i += 4)
    {
        // Decode the association ID
        association_id = 0x100 * (unsigned char) ntp_message.data[i+1];
        association_id += (unsigned char) ntp_message.data[i];

        // Get the peer selection status
        peer_sel = (ntp_message.data[i+2] & PEER_SEL_MASK) >> PEER_SEL_SHIFT;

        // Check if we have reached the end of the IDs
        if (association_id == 0)
            break;

        // Return the values to the calling function
        association_ids[association_count] = association_id;
        peer_selections[association_count] = peer_sel;
        association_count++;

        // Check if we have reached the limit of the associations allowed for
        if (association_count >= max_association_ids)
            break;
    }

    *num_associations = association_count;

    return 0;
}


