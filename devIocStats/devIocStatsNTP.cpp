/*************************************************************************\
* Copyright (c) 2016 Osprey DCS
* TODO: Change this to the FRIB copyright
*
* EPICS BASE Versions 3.13.7
* and higher are distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
\*************************************************************************/

/* devIocStatsNTP.c - device support routines for NTP statistics
 *
 * Query an NTP server for status information.
 * Uses NTP protocol mode 6
 * See RFC1305 Appendix B  "NTP Control Messages"
 */
/*
 * TODO: Add Michael to author list
 *	Author: Wayne Lewis
 *	Date:  2016-11-06
 */

/*
	--------------------------------------------------------------------
	Note that the valid values for the parm field of the link
	information are:

	ai (DTYP="IOC stats NTP"):
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
        ntp_peer_reach i    - NTP eaemon sync status
        ntp_peer_delay i    - NTP daemon sync status
        ntp_peer_offset i   - NTP daemon sync status
        ntp_peer_jitter i   - NTP daemon sync status
        */


#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <sstream>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errlog.h>

#include <epicsMath.h>
#include <epicsExit.h>
#include <epicsThread.h>
#include <epicsGuard.h>
#include <epicsEvent.h>
#include <dbAccess.h>
#include <dbStaticLib.h>
#include <dbScan.h>
#include <devSup.h>
#include <drvSup.h>
#include <menuConvert.h>
#include <aiRecord.h>
#include <stringinRecord.h>
#include <alarm.h>
#include <recGbl.h>
#include <osiSock.h>
#include <initHooks.h>
#include <epicsExport.h>

#define epicsExportSharedSymbols
#include <shareLib.h>
#include <iocsh.h>

#include "devIocStats.h"
#include "devIocStatsNTP.h"
#include "ntphelper.h"

static long ntp_init(int pass);
static long ntp_init_record(dbCommon*);
static long ntp_read_ai(aiRecord*);
static long ntp_read_si(stringinRecord*);
static long ntp_ioint_info(int cmd, dbCommon *pr, IOSCANPVT* iopvt);

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

aStatsNTP devNTPStats={ 
    6,
    NULL,
    (DEVSUPFUN)ntp_init,
    (DEVSUPFUN)ntp_init_record,
    (DEVSUPFUN)ntp_ioint_info,
    (DEVSUPFUN)ntp_read_ai,
    NULL };
epicsExportAddress(dset,devNTPStats);

aStatsNTP devNTPStatsName={
    6,
    NULL,
    NULL,
    (DEVSUPFUN)ntp_init_record,
    (DEVSUPFUN)ntp_ioint_info,
    (DEVSUPFUN)ntp_read_si,
    NULL };
epicsExportAddress(dset,devNTPStatsName);


// Default the daemon poll rate to 20 seconds
volatile int ntp_daemon_poll_rate = 20;
epicsExportAddress(int, ntp_daemon_poll_rate);

typedef epicsGuard<epicsMutex> Guard;
typedef epicsGuardRelease<epicsMutex> UnGuard;

static volatile int ntp_verb = 1;
epicsExportAddress(int, ntp_verb);

static
struct ntp_poller_t : public epicsThreadRunable
{
    epicsMutex lock;
    bool running;
    IOSCANPVT scan;

    std::auto_ptr<ntpStatus> data;

    epicsEvent event;
    epicsThread thread;

    ntp_poller_t()
        :running(false)
        ,data(new ntpStatus)
        ,thread(*this,
                "ntpdMonitor",
                epicsThreadGetStackSize(epicsThreadStackSmall),
                epicsThreadPriorityMedium-1)
    {
        scanIoInit(&scan);
    }

    virtual void run()
    {
        if(ntp_verb>1)
            errlogPrintf("NTPd monitor thread starts\n");
        std::auto_ptr<ntpStatus> next(new ntpStatus);

        Guard G(lock);

        while(running) {
            if(ntp_verb>2)
                errlogPrintf("NTPd monitor thread runs\n");
            try {
                UnGuard U(G);

                next->ntpDaemonOk = devIocStatsGetNtpStats(next.get());
            }catch(std::exception& e){
                next->ntpDaemonOk = false;
                errlogPrintf("Uncaught exception in NTP daemon thread: %s\n", e.what());
            }
            if(ntp_verb>1)
                errlogPrintf("NTPd monitor thread complete %s\n",
                             next->ntpDaemonOk ? "OK" : "Error");
            next->updateTime = epicsTime::getCurrent();

            std::swap(data, next);

            scanIoRequest(scan);
            {
                UnGuard U(G);
                event.wait(ntp_daemon_poll_rate);
            }
        }

        if(ntp_verb>1)
            errlogPrintf("NTPd monitor thread stops\n");
    }
} ntp_poller; // singleton

static unsigned short ntp_message_sequence_id;

// dbior("ntp_monitor_drv", #)
static
long ntp_monitor_report(int level)
{
    printf("iocStats NTPd monitor\n");
    try {
        ntpStatus data;

        {
            Guard G(ntp_poller.lock);
            data = *ntp_poller.data; // struct copy
        }

        printf(" Comm. w/ NTPd: %s\n", data.ntpDaemonOk ? "OK" : "Error");
        {
            char buf[100];
            data.updateTime.strftime(buf, sizeof(buf), "%a %b %d %Y %H:%M:%S.%09f");
            printf(" Last Update: %s\n", buf);
        }
        printf(" Stratum: %d\n", data.ntpStratum);

        if(!data.ntpDaemonOk || level<1)
            return 0;

        printf(" Min Ref. Stratum: %d\n Max delay: %f\n Max Offset: %f\n Max Jitter: %f\n",
               data.ntpMinPeerStratum, data.ntpMaxPeerDelay,
               data.ntpMaxPeerOffset, data.ntpMaxPeerJitter);

        if(level<2)
            return 0;

        for(size_t i=0; i<data.ntp_peer_data.size(); i++) {
            //ntpPeerData& peer = data.ntp_peer_data[i];
            ntp_peer_data_t ntp_peer_data = data.ntp_peer_data[i];
            //printf(" Peer %19s statum=%d delay=%f offset=%f jitter=%f\n",
                   //peer.src.c_str(), peer.ntpPeerStratum, peer.ntpPeerDelay,
                   //peer.ntpPeerOffset, peer.ntpPeerJitter);
            printf(" Peer %19s statum=%s delay=%s offset=%s jitter=%s\n",
                   ntp_peer_data["srcadr"].c_str(), 
                   ntp_peer_data["stratum"].c_str(), 
                   ntp_peer_data["delay"].c_str(),
                   ntp_peer_data["offset"].c_str(), 
                   ntp_peer_data["jitter"].c_str());

        }

        return 0;
    } catch(std::exception& e) {
        printf("  Error: %s\n", e.what());
        return 1;
    }
}

static drvet ntp_monitor_drv = {
    2,
    (DRVSUPFUN)&ntp_monitor_report,
    NULL,
};
epicsExportAddress(drvet, ntp_monitor_drv);

static
void ntp_exit(void *)
{
    try {
        Guard G(ntp_poller.lock);
        if(ntp_poller.running) {
            ntp_poller.running = false;
            ntp_poller.event.signal();
            UnGuard U(G);
            ntp_poller.thread.exitWait();
        }
    }catch(std::exception& e) {
        std::cerr<<"Error stopping NTP daemon monitor thread: "<<e.what()<<"\n";
    }
}

static
void ntp_hook(initHookState state)
{
    // auto-start daemon monitoring
#if defined(__rtems__) || defined(vxWorks) || defined(_WIN32)
    if(ntp_verb>1)
        printf("NTP monitor auto-start disabled for this target\n"
               " run devIocStatsNTPEnable() for manual start\n");
#else
    if(state!=initHookAfterIocRunning)
        return;

    Guard G(ntp_poller.lock);

    if(!ntp_poller.running) {
        try {
            ntp_poller.thread.start();
            ntp_poller.running = true;
        } catch(std::exception& e) {
            std::cerr<<"Error starting NTP daemon monitor thread: "<<e.what()<<"\n";
        }
    }

#endif
    epicsAtExit(&ntp_exit, NULL);
}

static long ntp_init(int pass)
{
    if (pass==1)
        initHookRegister(&ntp_hook);

    return 0;
}

static long ntp_init_record(dbCommon *prec)
{
    int		i;
    std::string	parm;
    std::string  parameter;
    size_t  index;
    int     peer;
    pvtNTPArea	*pvtNTP = NULL;

    DBLINK *plink = devIocStatsGetDevLink(prec);

    // Check the record INP type
    if(plink->type!=INST_IO) {
    
        recGblRecordError(S_db_badField,(void*)prec,
                "devAiNTPStats (init_record) Illegal INP field");
        return S_db_badField;
    }

    parm = plink->value.instio.string;
    for(i=0; pvtNTP==NULL; i++)
    {
        // Test if there is a space in the INP string.
        // If there is, then the peer number will follow.
        index = parm.find(" ");

        if (index == std::string::npos)
        {
            // System variable
            parameter = parm;
            peer = -1;
        }
        else
        {
            // Peer variable
            parameter = parm.substr(0, index);
            peer = (int)strtoul(parm.substr(index+1).c_str(), NULL, 10);
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
        recGblRecordError(S_db_badField,(void*)prec,
                "devAiNTPPeerStats (init_record) Illegal INP parm field");
        return S_db_badField;
    }

    prec->dpvt=pvtNTP;
    return 0;
}

// I/O interrupt initialization
static long ntp_ioint_info(int cmd, dbCommon* prec, IOSCANPVT* iopvt)
{
    if (!prec->dpvt) return S_dev_badInpType;

    *iopvt = ntp_poller.scan;

    return 0;
}

/* Generic read - calling function from table */
static long ntp_read_ai(aiRecord* prec)
{
    if(!prec->dpvt) {
        (void)recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
        return S_dev_badInpType;
    }
    pvtNTPArea* pvtNTP=(pvtNTPArea*)prec->dpvt;

    Guard G(ntp_poller.lock);

    if(pvtNTP->peer<0 || (unsigned)pvtNTP->peer < ntp_poller.data->ntp_peer_data.size()) {
        double val;
        statsGetNTPParms[pvtNTP->index].func(&val,pvtNTP->peer);
        prec->val = val;
    } else {
        (void)recGblSetSevr(prec, READ_ALARM, INVALID_ALARM);
    }

    if (!ntp_poller.data->ntpDaemonOk)
        (void)recGblSetSevr(prec, READ_ALARM, INVALID_ALARM);

    prec->udf = 0;
    return 2; /* don't convert */
}

static long ntp_read_si(stringinRecord* prec)
{
    if(!prec->dpvt) {
        (void)recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
        return S_dev_badInpType;
    }
    pvtNTPArea* pvtNTP=(pvtNTPArea*)prec->dpvt;

    Guard G(ntp_poller.lock);

    if(pvtNTP->peer<0 || (unsigned)pvtNTP->peer < ntp_poller.data->ntp_peer_data.size()) {
        strncpy(prec->val,
                ntp_poller.data->ntp_peer_data[pvtNTP->peer]["src"].c_str(),
                sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    } else {
        (void)recGblSetSevr(prec, READ_ALARM, INVALID_ALARM);
        strcpy(prec->val, "<no peer>");
    }

    if (!ntp_poller.data->ntpDaemonOk)
        (void)recGblSetSevr(prec, READ_ALARM, INVALID_ALARM);

    return 0;
}

/* -------------------------------------------------------------------- */
// ntp_poller.lock mutex must be locked when the following are called

static void statsNTPLeapSecond(double* val, int)
{
    *val = ntp_poller.data->ntpLeapSecond;
}
static void statsNTPStratum(double* val, int)
{
    *val = ntp_poller.data->ntpStratum;
}
static void statsNTPPrecision(double* val, int)
{
    *val = ntp_poller.data->ntpPrecision;
}
static void statsNTPRootDelay(double* val, int)
{
    *val = ntp_poller.data->ntpRootDelay;
}
static void statsNTPRootDispersion(double* val, int)
{
    *val = ntp_poller.data->ntpRootDispersion;
}
static void statsNTPTC(double* val, int)
{
    *val = ntp_poller.data->ntpTC;
}
static void statsNTPMinTC(double* val, int)
{
    *val = ntp_poller.data->ntpMinTC;
}
static void statsNTPOffset(double* val, int)
{
    *val = ntp_poller.data->ntpOffset;
}
static void statsNTPFrequency(double* val, int)
{
    *val = ntp_poller.data->ntpFrequency;
}
static void statsNTPSystemJitter(double* val, int)
{
    *val = ntp_poller.data->ntpSystemJitter;
}
static void statsNTPClockJitter(double* val, int)
{
    *val = ntp_poller.data->ntpClockJitter;
}
static void statsNTPClockWander(double* val, int)
{
    *val = ntp_poller.data->ntpClockWander;
}
static void statsNTPNumPeers(double* val, int)
{
    *val = ntp_poller.data->ntpNumPeers;
}
static void statsNTPNumGoodPeers(double* val, int)
{
    *val = ntp_poller.data->ntpNumGoodPeers;
}
static void statsNTPMaxPeerOffset(double* val, int)
{
    *val = ntp_poller.data->ntpMaxPeerOffset;
}
static void statsNTPMaxPeerJitter(double* val, int)
{
    *val = ntp_poller.data->ntpMaxPeerJitter;
}
static void statsNTPMinPeerStratum(double* val, int)
{
    *val = ntp_poller.data->ntpMinPeerStratum;
}
static void statsNTPPeerSelection(double* val, int peer)
{
    //*val = ntp_poller.data->ntp_peer_data[peer].ntpPeerSelectionStatus;
    *val = strtod(ntp_poller.data->ntp_peer_data[peer]["selection"].c_str(), NULL);
}
static void statsNTPPeerStratum(double* val, int peer)
{
    //*val = ntp_poller.data->ntp_peer_data[peer].ntpPeerStratum;
    *val = strtod(ntp_poller.data->ntp_peer_data[peer]["stratum"].c_str(), NULL);
}
static void statsNTPPeerPoll(double* val, int peer)
{
    //*val = ntp_poller.data->ntp_peer_data[peer].ntpPeerPoll;
    *val = strtod(ntp_poller.data->ntp_peer_data[peer]["ppoll"].c_str(), NULL);
}
static void statsNTPPeerReach(double* val, int peer)
{
    //*val = ntp_poller.data->ntp_peer_data[peer].ntpPeerReach;
    *val = strtod(ntp_poller.data->ntp_peer_data[peer]["reach"].c_str(), NULL);
}
static void statsNTPPeerDelay(double* val, int peer)
{
    //*val = ntp_poller.data->ntp_peer_data[peer].ntpPeerDelay;
    *val = strtod(ntp_poller.data->ntp_peer_data[peer]["delay"].c_str(), NULL);
}
static void statsNTPPeerOffset(double* val, int peer)
{
    //*val = ntp_poller.data->ntp_peer_data[peer].ntpPeerOffset;
    *val = strtod(ntp_poller.data->ntp_peer_data[peer]["offset"].c_str(), NULL);
}
static void statsNTPPeerJitter(double* val, int peer)
{
    //*val = ntp_poller.data->ntp_peer_data[peer].ntpPeerJitter;
    *val = strtod(ntp_poller.data->ntp_peer_data[peer]["jitter"].c_str(), NULL);
}

/*
ntpPeerData::ntpPeerData()
    :ntpPeerSelectionStatus(0), ntpPeerStratum(16)
    ,ntpPeerPoll(-1), ntpPeerReach(-1)
    ,ntpPeerDelay(std::numeric_limits<double>::quiet_NaN())
    ,ntpPeerOffset(ntpPeerDelay)
    ,ntpPeerJitter(ntpPeerDelay)
{}
*/

bool devIocStatsGetNtpStats (ntpStatus *pval)
{
    std::vector<epicsUInt16> association_ids;
    std::vector<epicsUInt16> peer_selections;
    std::string ntp_data;

    // Perform an NTP variable query to get the system level status

    if(!do_ntp_query( NTP_OP_READ_VAR, NTP_SYS_ASSOCIATION_ID, &ntp_data)) {
        if(ntp_verb>0)
            errlogPrintf("Failed to get system status\n");
        // continue and try to fetch associations
    } else {
        parse_ntp_sys_vars(pval, ntp_data);
    }

    // Perform an NTP status query to get the association IDs
    if (!get_association_ids(
                    association_ids,
                    peer_selections))
    {
        if(ntp_verb>0)
            errlogPrintf("Failed to read association status\n");
        return false;
    }
    if(ntp_verb>1)
        errlogPrintf(" Found %u associations\n", (unsigned)association_ids.size());

    // Clear the existing vector elements and resize
    pval->ntp_peer_data.resize(0);
    pval->ntp_peer_data.resize(association_ids.size());

    parse_ntp_associations(
            association_ids,
            peer_selections,
            pval);

    // Get stats from the peers
    return get_peer_stats(
            association_ids,
            pval);
}

void parse_ntp_associations(const std::vector<epicsUInt16>& association_ids,
        const std::vector<epicsUInt16>& peer_selections,
        ntpStatus *pval)
{

    assert(association_ids.size()==peer_selections.size());

    bool reference_peer = FALSE;

    pval->ntpNumPeers = association_ids.size();

    // Count the number of peers at candidate level or above
    unsigned num_good_peers = 0;

    for (unsigned i = 0; i < association_ids.size(); i++)
    {
        if (peer_selections[i] >= NTP_PEER_SEL_CANDIDATE)
            num_good_peers++;

        if (peer_selections[i] >= NTP_PEER_SEL_SYSPEER)
            reference_peer = TRUE;

        std::ostringstream s;
        s << peer_selections[i];
        pval->ntp_peer_data[i]["selection"] = s.str();

    }

    pval->ntpNumGoodPeers = num_good_peers;

    // If we have at least one good peer, set the sync status to good
    if (reference_peer == TRUE)
        pval->ntpSyncStatus = NTP_SYNC_STATUS_NTP;
    else
        pval->ntpSyncStatus = NTP_SYNC_STATUS_UNSYNC;

    if(ntp_verb>1)
        errlogPrintf(" Peers %u/%u %s\n",
                     (unsigned)pval->ntpNumGoodPeers,
                     (unsigned)pval->ntpNumPeers,
                     reference_peer ? "found ref. peer" : "no ref peer");
}

void parse_ntp_sys_vars(ntpStatus *pval,
        const std::string &ntp_data)
{
    ntp_peer_data_t ntp_peer_data(ntp_parse_peer_data(ntp_data));

    pval->ntpLeapSecond = ntp_peer_as<int>(ntp_peer_data, "leap", -1);
    pval->ntpStratum = ntp_peer_as<int>(ntp_peer_data, "stratum", 16);
    pval->ntpPrecision = ntp_peer_as<int>(ntp_peer_data, "precision", 0);
    pval->ntpRootDelay = ntp_peer_as<double>(ntp_peer_data, "rootdelay", -1);
    pval->ntpRootDispersion = ntp_peer_as<double>(ntp_peer_data, "rootdisp", -1);
    pval->ntpRootDelay = ntp_peer_as<double>(ntp_peer_data, "rootdelay", 0);
    pval->ntpTC = ntp_peer_as<int>(ntp_peer_data, "tc", -1);
    pval->ntpMinTC = ntp_peer_as<int>(ntp_peer_data, "mintc", -1);
    pval->ntpOffset = ntp_peer_as<double>(ntp_peer_data, "offset", 0);
    pval->ntpFrequency = ntp_peer_as<double>(ntp_peer_data, "frequency", -1);
    pval->ntpSystemJitter = ntp_peer_as<double>(ntp_peer_data, "sys_jitter", -1);
    pval->ntpClockJitter = ntp_peer_as<double>(ntp_peer_data, "clk_jitter", -1);
    pval->ntpClockWander = ntp_peer_as<double>(ntp_peer_data, "clk_wander", -1);
}

namespace {
struct Socket {
    SOCKET sd;
    Socket() :sd(socket(PF_INET, SOCK_DGRAM, 0))
    {
        if(sd<0)
            throw std::runtime_error("Failed to create DGRAM socket");
    }
    ~Socket() {
        close(sd);
    }
    operator int() { return sd; }
};
}

bool do_ntp_query(unsigned char op_code,
        unsigned short association_id,
        std::string *ntp_data
        )
{
    struct sockaddr_in ntp_socket;
    struct in_addr address;
    int ret;
    fd_set fds;
    struct timeval timeout_val;

    std::vector<unsigned char> ntp_message;

    // Set up the socket to the local NTP daemon
    inet_aton("127.0.0.1", &address);
    ntp_socket.sin_family = AF_INET;
    ntp_socket.sin_addr = address;
    ntp_socket.sin_port = htons(NTP_PORT);

    /* Create the socket */
    Socket sd;

    if (connect(sd, (struct sockaddr *)&ntp_socket, sizeof(ntp_socket)) < 0)
        throw std::runtime_error("Error \"connecting\" UDP socket");

    FD_ZERO(&fds);
    FD_SET(sd, &fds);

    // Use a different sequence ID each time
    ntp_message_sequence_id++;

    // Don't use zero as sequence ID
    if (ntp_message_sequence_id == 0)
        ntp_message_sequence_id++;

    // Populate the NTP control packet
    /*      0     1
     * 0 | MODE | OP |
     * 2 |    SEQ    |
     * 4 |    STS    |
     * 6 |    AID    |
     * 8 |    OFF    |
     * A |    LEN    |
     * C |  body...
     */
    ntp_message.push_back(NTP_VER_MODE);
    ntp_message.push_back(op_code);
    ntp_message.push_back(ntp_message_sequence_id>>8);
    ntp_message.push_back(ntp_message_sequence_id);
    ntp_message.push_back(0); // status
    ntp_message.push_back(0);
    ntp_message.push_back(association_id>>8);
    ntp_message.push_back(association_id);
    ntp_message.push_back(0); // offset
    ntp_message.push_back(0);
    ntp_message.push_back(0); // length
    ntp_message.push_back(0);
    // no body in request

    timeout_val.tv_sec = 0;
    timeout_val.tv_usec = 500000;

    // Send the request to the NTP daemon
    if (send(sd, &ntp_message[0], ntp_message.size(), 0) < 0)
        throw std::runtime_error("NTP send() error");

    NTPAssembler ntp_assembler;

    while (!ntp_assembler.done())
    {
        // Clear the message structure contents prior to receiving the new
        // message
        ntp_message.clear();
        ntp_message.resize(1024*16);

        // Wait for a response
        ret = select(sd+1, &fds, (fd_set *)0, (fd_set *)0, &timeout_val);

        if (ret == 0) {
            return false;

        } else if (ret == -1) {
            int err = SOCKERRNO;
            if(err==SOCK_EINTR) {
                continue;
            } else {
                throw std::runtime_error("NTP select() error");
            }
        }

        // Read the response


        ret = recv(sd, &ntp_message[0], ntp_message.size(), 0);

        if (ret < 0) {
            int err = SOCKERRNO;
            if(err==SOCK_EINTR) {
                continue;
            } else {
                throw std::runtime_error("NTP recv() error");
            }
        }
        else if (ret<6*2) {
            // truncated packet?
            throw std::runtime_error("NTP recv() truncated packet");
        }

        ntp_message.resize((size_t)ret);

        // Check that the sequence ID and association IDs match
        bool reply = ntp_message[1]&RESPONSE_MASK;
        bool more  = ntp_message[1]&MORE_MASK;
        epicsUInt16 seq = ntohs(*(epicsUInt16*)&ntp_message[2]);
        epicsUInt16 aid = ntohs(*(epicsUInt16*)&ntp_message[6]);
        epicsUInt16 off = ntohs(*(epicsUInt16*)&ntp_message[8]);
        epicsUInt16 len = ntohs(*(epicsUInt16*)&ntp_message[10]);

        if(!reply || seq!=ntp_message_sequence_id || aid!=association_id) {
            continue; // not a reply to our request, ignore

        } else if(len+12u>ntp_message.size()) {
            throw std::runtime_error("Ignore truncated UDP");
        }

        ntp_assembler.add(
                off,
                len,
                more,
                &ntp_message[12]);
    }

    *ntp_data = ntp_assembler.tostring();
    return true;
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
bool get_peer_stats(
        const std::vector<epicsUInt16>& association_ids,
        ntpStatus *pval
        )
{
    std::string ntp_data;

    assert(pval->ntp_peer_data.size()==association_ids.size());


    pval->ntpMaxPeerDelay = 0.0;
    pval->ntpMaxPeerOffset = 0.0;
    pval->ntpMaxPeerJitter = 0.0;
    pval->ntpMinPeerStratum = 16;

    // Iterate through the associated peers and gather the required data
    for (unsigned i = 0; i < association_ids.size(); i++)
    {
        //ntpPeerData& peer = pval->ntp_peer_data[i];
        //ntp_peer_data_t ntp_peer_data = pval->ntp_peer_data[i];

        if(!do_ntp_query(
                NTP_OP_READ_STS, 
                association_ids[i], 
                &ntp_data))
            return false;

        //ntp_peer_data = ntp_parse_peer_data(ntp_data);
        // Get the map of the data from the query
        ntp_peer_data_t ntp_peer_data (ntp_parse_peer_data(ntp_data));
        
        // Append additional peer info to the existing data 
        pval->ntp_peer_data[i].insert(
                ntp_peer_data.begin(), 
                ntp_peer_data.end());

        ntp_peer_data_t::const_iterator it;

        std::ostringstream name;
        it = ntp_peer_data.find("srcadr");
        if(it==ntp_peer_data.end())
            name<<"<unknown>";
        else
            name<<it->second;
        name<<":";
        it = ntp_peer_data.find("srcport");
        if(it==ntp_peer_data.end())
            name<<"<unknown>";
        else
            name<<it->second;

        //peer.src = name.str();
        ntp_peer_data["src"] = name.str();

        //peer.ntpPeerStratum = ntp_peer_as<int>(ntp_peer_data, "stratum", 16);
        //pval->ntpMinPeerStratum = std::min(pval->ntpMinPeerStratum,
        //peer.ntpPeerStratum);

        //peer.ntpPeerPoll = ntp_peer_as<int>(ntp_peer_data, "ppoll", -1);
        //peer.ntpPeerReach = ntp_peer_as<int>(ntp_peer_data, "reach", -1);

        //peer.ntpPeerDelay = ntp_peer_as<double>(ntp_peer_data, "delay",
        //std::numeric_limits<double>::quiet_NaN());
        //if(fabs(pval->ntpMaxPeerDelay)<fabs(peer.ntpPeerDelay))
        //pval->ntpMaxPeerDelay = peer.ntpPeerDelay;
        double delay = strtod(ntp_peer_data["delay"].c_str(), NULL);
        if(fabs(pval->ntpMaxPeerDelay)<delay)
            pval->ntpMaxPeerDelay = delay;

        //peer.ntpPeerOffset = ntp_peer_as<double>(ntp_peer_data, "offset",
        //std::numeric_limits<double>::quiet_NaN());
        //if(fabs(pval->ntpMaxPeerOffset)<fabs(peer.ntpPeerOffset))
        //pval->ntpMaxPeerOffset = peer.ntpPeerOffset;
        double offset = strtod(ntp_peer_data["offset"].c_str(), NULL);
        if(fabs(pval->ntpMaxPeerOffset)<fabs(offset))
            pval->ntpMaxPeerOffset = offset;

        //peer.ntpPeerJitter = ntp_peer_as<double>(ntp_peer_data, "jitter",
        //std::numeric_limits<double>::quiet_NaN());
        //pval->ntpMaxPeerJitter = std::max(pval->ntpMaxPeerJitter, peer.ntpPeerJitter);
        double jitter = strtod(ntp_peer_data["jitter"].c_str(), NULL);
        pval->ntpMaxPeerJitter = std::max(
                pval->ntpMaxPeerJitter, jitter);

        if(ntp_verb>3)
            //errlogPrintf(" Peer %19s statum=%d delay=%f offset=%f jitter=%f\n",
            //peer.src.c_str(), peer.ntpPeerStratum, peer.ntpPeerDelay,
            //peer.ntpPeerOffset, peer.ntpPeerJitter);
            errlogPrintf(" Peer %19s statum=%s delay=%s offset=%s jitter=%s\n",
                    ntp_peer_data["src"].c_str(), 
                    ntp_peer_data["stratum"].c_str(), 
                    ntp_peer_data["delay"].c_str(),
                    ntp_peer_data["offset"].c_str(), 
                    ntp_peer_data["jitter"].c_str());
    }

    if(ntp_verb>2)
        errlogPrintf(" Min Ref. Stratum: %d\n Max delay: %f\n Max Offset: %f\n Max Jitter: %f\n",
                pval->ntpMinPeerStratum, pval->ntpMaxPeerDelay,
                pval->ntpMaxPeerOffset, pval->ntpMaxPeerJitter);

    return true;
}

bool find_substring(const std::string &data,
        const std::string &pattern,
        std::string *value)
{
    // Call to this version of the function will give the first instance of the
    // search string.

    return find_substring(data, pattern, 1, value);
}

bool find_substring(const std::string &data,
        const std::string &pattern,
        int occurrence,
        std::string *value)
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

    const std::string SEPARATOR (",");

    // Search for the required occurrence number
    while (num_found < occurrence)
    {
        found = data.find(pattern, start);
        num_found++;
        if (found != std::string::npos)
            start = found + pattern.length();
    }

    // Extract the remaining string up until the next separator
    if (found != std::string::npos)
    {
        separator = data.find(SEPARATOR, found);
        length = separator - start;
        *value = data.substr(start, length);
        result = TRUE;
    }
    return result;
}


bool get_association_ids(std::vector<unsigned short>& association_ids,
        std::vector<unsigned short>& peer_selections
        )
{
    unsigned int i;
    std::string ntp_data;

    // Finds the integer values used to identify the NTP peer servers
    //
    // Send a system level read status query
    if(!do_ntp_query(
                NTP_OP_READ_STS, 
                NTP_SYS_ASSOCIATION_ID, 
                &ntp_data))
        return false;

    // Extract the association IDs from the response
    for (i = 0; i < ntp_data.length(); i += 4)
    {
        unsigned short *ppeer = (unsigned short*)&ntp_data[i];
        unsigned short aid = ntohs(ppeer[0]),
                       sts = ntohs(ppeer[1]);

        if(aid==0)
            break;

        association_ids.push_back(aid);
        peer_selections.push_back((sts&PEER_SEL_MASK)>>PEER_SEL_SHIFT);
    }

    return true;
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
    Guard G(ntp_poller.lock);

    if(ntp_poller.running) {
        try {
            ntp_poller.running = false;
            ntp_poller.event.signal();
            UnGuard U(G);
            ntp_poller.thread.exitWait();
        } catch(std::exception& e) {
            std::cerr<<"Error stopping NTP daemon monitor thread: "<<e.what()<<"\n";
        }
    }
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
    Guard G(ntp_poller.lock);

    if(!ntp_poller.running) {
        try {
            ntp_poller.thread.start();
            ntp_poller.running = true;
        } catch(std::exception& e) {
            std::cerr<<"Error starting NTP daemon monitor thread: "<<e.what()<<"\n";
        }
    }
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

