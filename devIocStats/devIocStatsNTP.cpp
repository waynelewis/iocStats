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
	Note that the values for the parm field of the link must match the
    response from the NTP daemon.

    Values associated with a peer must have the peer number following the
    parameter, separated by a space.

    Other custom values that are valid are:
    ntp_max_peer_jitter
    ntp_max_peer_offset
    ntp_min_peer_stratum
    ntp_num_good_peers
    ntp_num_peers
    ntp_sync_status
    ntp_peer_selection i

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
        printf(" Stratum: %s\n", data.ntp_sys_data["stratum"].c_str());

        if(!data.ntpDaemonOk || level<1)
            return 0;

        printf(" Min Ref. Stratum: %s\n Max delay: %s\n Max Offset: %s\n Max Jitter: %s\n",
                data.ntp_sys_data["ntp_min_peer_stratum"].c_str(), 
                data.ntp_sys_data["ntp_max_peer_delay"].c_str(), 
                data.ntp_sys_data["ntp_max_peer_offset"].c_str(), 
                data.ntp_sys_data["ntp_max_peer_jitter"].c_str());

        if(level<2)
            return 0;

        for(size_t i=0; i<data.ntp_peer_data.size(); i++) {
            ntp_peer_data_t ntp_peer_data = data.ntp_peer_data[i];
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
    std::string	parm;
    std::auto_ptr<std::string> parameter (new std::string);
    size_t  index;
    int     peer;
    pvtNTPArea* pvtNTP;

    DBLINK *plink = devIocStatsGetDevLink(prec);

    // Check the record INP type
    if(plink->type!=INST_IO) {
    
        recGblRecordError(S_db_badField,(void*)prec,
                "devAiNTPStats (init_record) Illegal INP field");
        return S_db_badField;
    }

    parm = plink->value.instio.string;
        // Test if there is a space in the INP string.
        // If there is, then the peer number will follow.
        index = parm.find(" ");

        if (index == std::string::npos)
        {
            // System variable
            *parameter = parm;
            peer = -1;
        }
        else
        {
            // Peer variable
            *parameter = parm.substr(0, index);
            peer = (int)strtoul(parm.substr(index+1).c_str(), NULL, 10);
        }

        // Find the correct function in the list
            pvtNTP=(pvtNTPArea*)malloc(sizeof(pvtNTPArea));
            pvtNTP->parameter = parameter;
            pvtNTP->peer = peer;

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
        if (ntp_get_ai_value(&val, *(pvtNTP->parameter).get(), pvtNTP->peer) == NTP_PARAMETER_OK) {
            prec->val = val;
        }
        else  {
            recGblRecordError(S_db_badField,(void*)prec,
                    "devAiNTPStats (ntp_read_ai) Unknown parameter");
        }
    } 
    else {
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

int ntp_get_ai_value(
        double *val, 
        const string& parameter, 
        const int peer)
{
    if (peer < 0) {
        // System variable
        // Test if the parameter exists
        if (ntp_poller.data->ntp_sys_data.find(parameter) !=
                ntp_poller.data->ntp_sys_data.end()) {
            *val = strtod(ntp_poller.data->ntp_sys_data[parameter].c_str(), NULL);
            return NTP_PARAMETER_OK;
        }
        else
            return NTP_PARAMETER_ERROR;
    }
    else {
        // Peer variable
        if (ntp_poller.data->ntp_peer_data[peer].find(parameter) !=
                ntp_poller.data->ntp_peer_data[peer].end()) {
            *val = strtod(ntp_poller.data->ntp_peer_data[peer][parameter].c_str(), NULL);
            return NTP_PARAMETER_OK;
        }
        else
            return NTP_PARAMETER_ERROR;
    }
}

// TODO: Work out how to handle these default values. 
// Maybe create in initial map that has the values.
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
        pval->ntp_sys_data = ntp_parse_peer_data(ntp_data);
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

    std::ostringstream s;
    s << association_ids.size();
    pval->ntp_sys_data["ntp_num_peers"] = s.str();

    unsigned num_good_peers = 0;

    for (unsigned i = 0; i < association_ids.size(); i++)
    {
        if (peer_selections[i] >= NTP_PEER_SEL_CANDIDATE)
            num_good_peers++;

        if (peer_selections[i] >= NTP_PEER_SEL_SYSPEER)
            reference_peer = TRUE;

        s.str("");
        s << peer_selections[i];
        pval->ntp_peer_data[i]["ntp_peer_selection"] = s.str();

    }

    s.str("");
    s << association_ids.size();
    pval->ntp_sys_data["ntp_num_good_peers"] = s.str();

    // If we have at least one good peer, set the sync status to good
    if (reference_peer == TRUE)
        pval->ntpSyncStatus = NTP_SYNC_STATUS_NTP;
    else
        pval->ntpSyncStatus = NTP_SYNC_STATUS_UNSYNC;

    if(ntp_verb>1)
        errlogPrintf(" Peers %s/%s %s\n",
                     pval->ntp_sys_data["ntp_num_good_peers"].c_str(),
                     pval->ntp_sys_data["ntp_num_peers"].c_str(),
                     reference_peer ? "found ref. peer" : "no ref peer");
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

    double max_peer_delay = 0.0;
    double max_peer_offset = 0.0;
    double max_peer_jitter = 0.0;
    int min_peer_stratum = 16;

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

        ntp_peer_data["src"] = name.str();

        double delay = strtod(ntp_peer_data["delay"].c_str(), NULL);
        if(fabs(max_peer_delay)<delay)
            max_peer_delay = delay;

        double offset = strtod(ntp_peer_data["offset"].c_str(), NULL);
        if(fabs(max_peer_offset)<fabs(offset))
            max_peer_offset = offset;

        double jitter = strtod(ntp_peer_data["jitter"].c_str(), NULL);
        max_peer_jitter = std::max(
                max_peer_jitter, jitter);

        int stratum = strtol(ntp_peer_data["stratum"].c_str(), NULL, 10);
        min_peer_stratum = std::min(min_peer_stratum, stratum);

        if(ntp_verb>3)
            errlogPrintf(" Peer %19s statum=%s delay=%s offset=%s jitter=%s\n",
                    ntp_peer_data["src"].c_str(), 
                    ntp_peer_data["stratum"].c_str(), 
                    ntp_peer_data["delay"].c_str(),
                    ntp_peer_data["offset"].c_str(), 
                    ntp_peer_data["jitter"].c_str());
    }

    std::ostringstream s;
    s << max_peer_delay;
    pval->ntp_sys_data["ntp_max_peer_delay"] = s.str();
    s.str("");
    s << max_peer_offset;
    pval->ntp_sys_data["ntp_max_peer_offset"] = s.str();
    s.str("");
    s << max_peer_jitter;
    pval->ntp_sys_data["ntp_max_peer_jitter"] = s.str();
    s.str("");
    s << min_peer_stratum;
    pval->ntp_sys_data["ntp_min_peer_stratum"] = s.str();

    if(ntp_verb>2)
        errlogPrintf(" Min Ref. Stratum: %s\n Max delay: %s\n Max Offset: %s\n Max Jitter: %s\n",
                pval->ntp_sys_data["ntp_min_peer_stratum"].c_str(), 
                pval->ntp_sys_data["ntp_max_peer_delay"].c_str(), 
                pval->ntp_sys_data["ntp_max_peer_offset"].c_str(), 
                pval->ntp_sys_data["ntp_max_peer_jitter"].c_str());

    return true;
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

