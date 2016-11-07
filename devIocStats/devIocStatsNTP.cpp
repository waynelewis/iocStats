/*************************************************************************\
* Copyright (c) 2016 Osprey DCS
*
* EPICS BASE Versions 3.13.7
* and higher are distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
\*************************************************************************/

/* devIocStatsNTP.c - device support routines for NTP statistics - based on */
/*
 *	Author: Wayne Lewis
 *	Date:  2016-11-06
 *
 * 
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
        */


#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <cstring>
#include <cstdlib>

#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <rsrv.h>

#include <epicsThread.h>
#include <epicsMutex.h>
#include <dbAccess.h>
#include <dbStaticLib.h>
#include <dbScan.h>
#include <devSup.h>
#include <menuConvert.h>
#include <aiRecord.h>
#include <recGbl.h>
#include <epicsExport.h>
#include <epicsTypes.h>

#define epicsExportSharedSymbols
#include <shareLib.h>
#include <iocsh.h>

#include "devIocStatsNTP.h"

using namespace std;

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

aStatsNTP devAiNTPStats={ 
    6,
    NULL,
    (DEVSUPFUN)ai_ntp_init,
    (DEVSUPFUN)ai_ntp_init_record,
    (DEVSUPFUN)ai_ntp_ioint_info,
    (DEVSUPFUN)ai_ntp_read,
    NULL };

epicsExportAddress(dset,devAiNTPStats);

// Default the daemon poll rate to 20 seconds
volatile int ntp_daemon_poll_rate = 20;
volatile bool ntp_daemon_disable = FALSE;

static IOSCANPVT ioscanpvt;
static ntpStatus ntpstatus;
static epicsMutexId ntp_scan_mutex;
static epicsThreadId ntp_poll_thread_id;
static unsigned short ntp_message_sequence_id;

static void poll_ntp_daemon(void)
{
    // Polling function to get data from the NTP daemon
    while(1)
    {
        if (ntp_daemon_disable == TRUE)
            epicsThreadSuspendSelf();

        devIocStatsGetNtpStats(&ntpstatus);

        scanIoRequest(ioscanpvt);
        epicsThreadSleep(ntp_daemon_poll_rate);
    }
}

static long ai_ntp_init(int pass)
{

    if (pass) return 0;

    // Create a thread to poll the NTP daemon and populate the data structure
    ntp_poll_thread_id = epicsThreadCreate(
                "NTP_poller_thread",
                epicsThreadPriorityLow,
                epicsThreadGetStackSize(epicsThreadStackSmall),
                (EPICSTHREADFUNC)poll_ntp_daemon,
                NULL);

    if (ntp_poll_thread_id == NULL)
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


int devIocStatsGetNtpStats (ntpStatus *pval)
{
    int ret;

    unsigned short association_ids[NTP_MAX_PEERS];
    unsigned short peer_selections[NTP_MAX_PEERS];
    int num_associations;
    string ntp_data;

    // Perform an NTP variable query to get the system level status
    if (( ret = do_ntp_query(
                    NTP_OP_READ_VAR, 
                    NTP_SYS_ASSOCIATION_ID, 
                    &ntp_data) != 0))
        return ret;

    parse_ntp_sys_vars(pval, ntp_data);

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
    bool reference_peer;

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
        ntpStatus *pval,
        string ntp_data)
{
    //string NTP_VERSION[] = "version=";
    //string NTP_PROCESSOR[] = "processor=";
    //string NTP_SYSTEM[] = "system=";
    string NTP_LEAP ("leap=");
    string NTP_STRATUM ("stratum=");
    string NTP_PRECISION ("precision=");
    string NTP_ROOT_DELAY ("rootdelay=");
    string NTP_ROOT_DISPERSION ("rootdisp=");
    //string NTP_REF_ID ("refid=");
    //string NTP_REF_TIME ("reftime=");
    //string NTP_CLOCK ("clock=");
    //string NTP_PEER ("peer=");
    string NTP_TC ("tc=");
    string NTP_MINTC ("mintc=");
    string NTP_OFFSET ("offset=");
    string NTP_FREQUENCY ("frequency=");
    string NTP_SYS_JITTER ("sys_jitter=");
    string NTP_CLOCK_JITTER ("clk_jitter=");
    string NTP_CLOCK_WANDER ("clk_wander=");
    
    string ntp_param_value;

    /* Leap second status */
    if (find_substring(ntp_data, NTP_LEAP, &ntp_param_value))
        pval->ntpLeapSecond = (int)(atoi(ntp_param_value.c_str()));

    /* Stratum */
    if (find_substring(ntp_data, NTP_STRATUM, &ntp_param_value))
        pval->ntpStratum = (double)(atof(ntp_param_value.c_str()));

    /* Precision */
    if (find_substring(ntp_data, NTP_PRECISION, &ntp_param_value))
        pval->ntpPrecision = (int)(atoi(ntp_param_value.c_str()));

    /* Root delay */
    if (find_substring(ntp_data, NTP_ROOT_DELAY, &ntp_param_value))
        pval->ntpRootDelay = (double)(atof(ntp_param_value.c_str()));

    /* Root dispersion */
    if (find_substring(ntp_data, NTP_ROOT_DISPERSION, &ntp_param_value))
        pval->ntpRootDispersion = (double)(atof(ntp_param_value.c_str()));

    /* Time constant */
    if (find_substring(ntp_data, NTP_TC, &ntp_param_value))
        pval->ntpTC = (int)(atoi(ntp_param_value.c_str()));

    /* Minimum time constant */
    if (find_substring(ntp_data, NTP_MINTC, &ntp_param_value))
        pval->ntpMinTC = (int)(atoi(ntp_param_value.c_str()));

    /* Offset */
    if (find_substring(ntp_data, NTP_OFFSET, &ntp_param_value))
        pval->ntpOffset = (double)(atof(ntp_param_value.c_str()));

    /* Frequency */
    if (find_substring(ntp_data, NTP_FREQUENCY, &ntp_param_value))
        pval->ntpFrequency = (double)(atof(ntp_param_value.c_str()));

    /* System jitter */
    if (find_substring(ntp_data, NTP_SYS_JITTER, &ntp_param_value))
        pval->ntpSystemJitter = (double)(atof(ntp_param_value.c_str()));

    /* Clock jitter */
    if (find_substring(ntp_data, NTP_CLOCK_JITTER, &ntp_param_value))
        pval->ntpClockJitter = (double)(atof(ntp_param_value.c_str()));

    /* Clock wander */
    if (find_substring(ntp_data, NTP_CLOCK_WANDER, &ntp_param_value))
        pval->ntpClockWander = (double)(atof(ntp_param_value.c_str()));
}

int do_ntp_query(
        unsigned char op_code,
        unsigned short association_id,
        string *ntp_data
        )
{
    struct sockaddr_in ntp_socket;
    struct in_addr address;
    int sd;
    int ret;
    fd_set fds;
    struct timeval timeout_val;
    FD_ZERO(&fds);
    unsigned int more_bit = 1;
    unsigned int i;
    int next_offset;

    string ntp_data_result;

    struct ntp_control *ntp_message;
    ntp_message = (struct ntp_control *)malloc(sizeof(struct ntp_control));

    vector <ntpDataFragment> ntp_data_fragments;

    ntpDataFragment ntp_data_fragment = ntpDataFragment();


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
    if (ntp_message_sequence_id == 0)
        ntp_message_sequence_id++;
    ntp_message->sequence = ntp_message_sequence_id++;

    timeout_val.tv_sec = 1;
    timeout_val.tv_usec = 0;

    // Send the request to the NTP daemon
    if (send(sd, ntp_message, sizeof(*ntp_message), 0) < 0)
        return NTP_COMMAND_SEND_ERROR;

    while (more_bit == 1)
    {
        // Clear the message structure contents prior to receiving the new
        // message
        memset(ntp_message, 0, sizeof(struct ntp_control));

        // Wait for a response
        ret = select(sd+1, &fds, (fd_set *)0, (fd_set *)0, &timeout_val);

        if (ret == 0)
            return NTP_TIMEOUT_ERROR;

        if (ret == -1)
            return NTP_SELECT_ERROR;

        // Read the response
        if ((ret = recv(sd, ntp_message, sizeof(*ntp_message), 0)) < 0)
            return NTP_DAEMON_COMMS_ERROR;

        // Extract the status bit that tells us if we have more data waiting
        more_bit = (ntp_message->op_code & MORE_MASK) >> MORE_SHIFT;

        ntp_data_fragment.offset = reverse(ntp_message->offset);
        ntp_data_fragment.count = reverse(ntp_message->count);
        ntp_data_fragment.data = ntp_message->data;

        ntp_data_fragments.push_back(ntp_data_fragment);

    }

    // Assemble the returned data into a single string
    next_offset = 0;
    for (i = 0; i < ntp_data_fragments.size(); i++)
        for (vector<ntpDataFragment>::iterator it = ntp_data_fragments.begin(); 
                it != ntp_data_fragments.end(); 
                ++it)
            if (it->offset == next_offset)
            {
                ntp_data_result += it->data;
                next_offset = it->count;
                break;
            }

    *ntp_data = ntp_data_result;

    free(ntp_message);

    // Close the socket now that we've received the message
    close(sd);

    return 0;

}

unsigned short reverse(unsigned short v)
{
    unsigned short val = v;         // reverse the bits in this
    unsigned short t = 0;     // t will have the reversed bits of v
    unsigned int i;

    for (i = 0; i < sizeof(v) * 8; i++)
    {
        t <<= 1;
        t |= (val & 1);
        val >>= 1;
    }

    return t;
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

    // Iterate through the associated peers and gather the required data
    for (i = 0; i < num_peers; i++)
    {
        ret = do_ntp_query(
                NTP_OP_READ_STS, 
                association_ids[i], 
                &ntp_data);
        if (ret < 0)
            return ret;

        if (find_substring(ntp_data, NTP_PEER_STRATUM, &ntp_param_value))
            stratums[i] = atoi(ntp_param_value.c_str());

        if (find_substring(ntp_data, NTP_PEER_POLL, &ntp_param_value))
            polls[i] = atoi(ntp_param_value.c_str());

        if (find_substring(ntp_data, NTP_PEER_REACH, &ntp_param_value))
            reaches[i] = atoi(ntp_param_value.c_str());

        if (find_substring(ntp_data, NTP_PEER_DELAY, 2, &ntp_param_value))
            delays[i] = atof(ntp_param_value.c_str());

        if (find_substring(ntp_data, NTP_PEER_OFFSET, &ntp_param_value))
            offsets[i] = atof(ntp_param_value.c_str());

        if (find_substring(ntp_data, NTP_PEER_JITTER, &ntp_param_value))
            jitters[i] = atof(ntp_param_value.c_str());
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

bool find_substring(const string data, const string pattern, string *value)
{
    // Call to this version of the function will give the first instance of the
    // search string.

    return find_substring(data, pattern, 1, value);
}

bool find_substring(
        const string data, 
        const string pattern, 
        int occurrence,
        string *value)
{
    // General version of the substring function, with an option to find a
    // specific occurrence of the search patttern.

    size_t found;
    size_t start;
    size_t separator;
    size_t length;
    bool result = FALSE;

    int num_found = 0;
    start = 0;

    const string SEPARATOR (",");

    // Search for the required occurrence number
    while (num_found < occurrence)
    {
        found = data.find(pattern, start);
        num_found++;
        if (found != string::npos)
            start = found + pattern.length();
    }

    // Extract the remaining string up until the next separator
    if (found != string::npos)
    {
        separator = data.find(SEPARATOR, found);
        length = separator - start;
        *value = data.substr(start, length);
        result = TRUE;
    }
    return result;
}


int get_association_ids(
        unsigned short *association_ids, 
        unsigned short *peer_selections, 
        int *num_associations,
        int max_association_ids)
{
    unsigned int i;
    int ret;
    int association_count;
    unsigned short association_id;
    int peer_sel;
    string ntp_data;

    // Finds the integer values used to identify the NTP peer servers
    //
    // Send a system level read status query
    ret = do_ntp_query(
            NTP_OP_READ_STS, 
            NTP_SYS_ASSOCIATION_ID, 
            &ntp_data);
    if (ret != 0)
        return ret;

    // Extract the association IDs from the response
    association_count = 0;
    for (i = 0; i < ntp_data.length(); i += 4)
    {
        association_id = 0x100 * (unsigned char) ntp_data.at(i+1);
        association_id += (unsigned char) ntp_data.at(i);

        // Get the peer selection status
        peer_sel = (ntp_data.at(i+2) & PEER_SEL_MASK) >> PEER_SEL_SHIFT;

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

static const iocshArg devIocStatsNTPSetPollRateArg0 = {"rate", iocshArgInt};

static const iocshArg *const devIocStatsNTPSetPollRateArgs[] = {
    &devIocStatsNTPSetPollRateArg0};

static const iocshFuncDef devIocStatsNTPSetPollRateDef = {
    "devIocStatsNTPSetPollRate",
    1,
    devIocStatsNTPSetPollRateArgs};

epicsShareFunc void devIocStatsNTPSetPollRate(const int rate)
{
    ntp_daemon_poll_rate = rate;
}

static void devIocStatsNTPSetPollRateCall(const iocshArgBuf * args)
{
    devIocStatsNTPSetPollRate(args[0].ival);
}

static const iocshFuncDef devIocStatsNTPDisableDef = {
    "devIocStatsNTPDisable",
    0,
    NULL};

epicsShareFunc void devIocStatsNTPDisable(void)
{
    ntp_daemon_disable = TRUE;
}

static void devIocStatsNTPDisableCall(const iocshArgBuf * args)
{
    devIocStatsNTPDisable();
}

static const iocshFuncDef devIocStatsNTPEnableDef = {
    "devIocStatsNTPEnable",
    0,
    NULL};

epicsShareFunc void devIocStatsNTPEnable(void)
{
    ntp_daemon_disable = FALSE;
    epicsThreadResume(ntp_poll_thread_id);
}

static void devIocStatsNTPEnableCall(const iocshArgBuf * args)
{
    devIocStatsNTPEnable();
}

static void devIocStatsNTPRegister(void)
{
    iocshRegister(&devIocStatsNTPSetPollRateDef, devIocStatsNTPSetPollRateCall);
    iocshRegister(&devIocStatsNTPEnableDef, devIocStatsNTPEnableCall);
    iocshRegister(&devIocStatsNTPDisableDef, devIocStatsNTPDisableCall);
}

epicsExportRegistrar(devIocStatsNTPRegister);

