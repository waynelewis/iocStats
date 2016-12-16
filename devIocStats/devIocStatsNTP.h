#ifndef DEV_IOC_STATS_NTP_H 
#define DEV_IOC_STATS_NTP_H

/*************************************************************************\
* Copyright (c) 2016 Osprey DCS
*
* EPICS BASE Versions 3.13.7
* and higher are distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
\*************************************************************************/

/* devIocStatsNTP.cpp - device support routines for NTP statistics - based on */
/*
 *  Author: Wayne Lewis
 *  Date:  2016-11-06
 */

/* NTP definitions */

#include <string>
#include <vector>
#include <memory>
#include <map>

#include <epicsTime.h>

using std::string;

// Record interface structures 
struct pvtNTPArea
{
    //unsigned index;
    std::auto_ptr<string> parameter;
    int peer; // -1 for data not associated with a peer
};
typedef struct pvtNTPArea pvtNTPArea;

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

//TODO: Fix the name of ntp_peer_data_t as it is used for both 
//      peer and system data
//      Or create ntp_sys_data_t of the same type?
typedef std::map<std::string, std::string> ntp_peer_data_t;
typedef std::map<std::string, std::string> ntp_sys_data_t;

// Data containing structures

struct ntpStatus {
        epicsTime updateTime;
        bool ntpDaemonOk;
        int ntpNumPeers;
        int ntpNumGoodPeers;
        double ntpMaxPeerDelay;
        double ntpMaxPeerOffset;
        double ntpMaxPeerJitter;
        int ntpMinPeerStratum;
        int ntpSyncStatus;
        std::vector<ntp_peer_data_t> ntp_peer_data;
        // TODO: Work out method of creating default value
        //ntpStatus() :ntpDaemonOk(false), ntpStratum(16) {}
        ntp_sys_data_t ntp_sys_data;
        ntpStatus() :ntpDaemonOk(false) {}
};


/* NTP status functions */
//extern int devIocStatsInitNtpStats (void);
bool devIocStatsGetNtpStats(ntpStatus *pval);

#define NTP_PORT    123

#define FALSE       0
#define TRUE        1

#define NTP_NO_ERROR                0
#define NTP_SOCKET_OPEN_ERROR       -1
#define NTP_SOCKET_CONNECT_ERROR    -2
#define NTP_COMMAND_SEND_ERROR      -3
#define NTP_TIMEOUT_ERROR           -4
#define NTP_SELECT_ERROR            -5
#define NTP_DAEMON_COMMS_ERROR      -6
#define NTP_SEQ_AID_ERROR           -7
#define NTP_PARSE_PEER_ERROR        -8

// Use a NTP mode 6 control message
#define NTP_VER_MODE                0x16
#define NTP_OP_READ_STS             0x01
#define NTP_OP_READ_VAR             0x02
#define NTP_OP_READ_CLK             0x04

#define NTP_SYS_ASSOCIATION_ID      0

#define NTP_PEER_SEL_INVALID        0
#define NTP_PEER_SEL_FALSETICKER    1
#define NTP_PEER_SEL_EXCESS         2
#define NTP_PEER_SEL_OUTLIER        3
#define NTP_PEER_SEL_CANDIDATE      4
#define NTP_PEER_SEL_SELECTED       5
#define NTP_PEER_SEL_SYSPEER        6
#define NTP_PEER_SEL_PPSPEER        7

#define NTP_SYNC_STATUS_UNSYNC      0
#define NTP_SYNC_STATUS_NTP         1

#define NTP_DAEMON_OK               0
#define NTP_DAEMON_ERROR            -1

#define NTP_PARAMETER_OK            0
#define NTP_PARAMETER_ERROR         -1

#define VER_MASK                    0x38
#define VER_SHIFT                   2
#define MODE_MASK                   0x07
#define MODE_SHIFT                  0
#define OP_CODE_MASK                0x1f
#define OP_CODE_SHIFT               0
#define MORE_MASK                   0x20
#define MORE_SHIFT                  5
#define ERROR_MASK                  0x40
#define ERROR_SHIFT                 6
#define RESPONSE_MASK               0x80
#define RESPONSE_SHIFT              7
#define PEER_SEL_MASK               0x0700
#define PEER_SEL_SHIFT              8

#define DATA_SIZE 468
#define AUTH_SIZE 96

struct ntp_control {
	unsigned char ver_mode;		    /* leap, version, mode */
	unsigned char op_code;		    /* response, more, error, opcode */
	unsigned char sequence0;		/* sequence number of request */
	unsigned char sequence1;		/* sequence number of request */
	unsigned char status0;			/* status word for association */
	unsigned char status1;			/* status word for association */
	unsigned char association_id0;	/* association ID */
	unsigned char association_id1;	/* association ID */
	unsigned char offset0;			/* offset of this batch of data */
	unsigned char offset1;			/* offset of this batch of data */
	unsigned char count0;			/* count of data in this packet */
	unsigned char count1;			/* count of data in this packet */
    char data[DATA_SIZE];  /* string data returned with packet */
	unsigned int authenticator[AUTH_SIZE];
};

// Function prototypes

int ntp_get_ai_value(
        double *val,
        const string& parameter,
        const int peer);

bool do_ntp_query(
        unsigned char op_code, 
        unsigned short association_id,
        std::string *ntp_data
        );

bool get_association_ids(std::vector<unsigned short> &association_ids,
                         std::vector<unsigned short> &peer_selections
                         );

bool get_peer_stats(const std::vector<epicsUInt16> &association_ids,
        ntpStatus *pval
        );

void parse_ntp_associations(const std::vector<epicsUInt16>& association_ids,
        const std::vector<epicsUInt16>& peer_selections,
        ntpStatus *pval);

unsigned short reverse(unsigned short);

epicsShareFunc void devIocStatsNTPSetPollRate(const int rate);
epicsShareFunc void devIocStatsNTPDisable(void);
epicsShareFunc void devIocStatsNTPEnable(void);


#endif /*DEV_IOC_STATS_NTP_H*/

