#ifndef DEV_IOC_STATS_NTP_H 
#define DEV_IOC_STATS_NTP_H

/*************************************************************************\
* Copyright (c) 2016 Osprey DCS
*
* EPICS BASE Versions 3.13.7
* and higher are distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
\*************************************************************************/

/* devIocStatsNTP.c - device support routines for NTP statistics - based on */
/*
 *  Author: Wayne Lewis
 *  Date:  2016-11-06
 */

/* NTP definitions */
/* Maximum number of peers that we report information for */
#define NTP_MAX_PEERS 10

#include <string>
using namespace std;

// Record interface structures 
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

// Data containing structures
typedef struct _ntpPeerData {
        int ntpPeerSelectionStatus;
        int ntpPeerStratum;
        int ntpPeerPoll;
        int ntpPeerReach;
        double ntpPeerDelay;
        double ntpPeerOffset;
        double ntpPeerJitter;
} ntpPeerData;

typedef struct _ntpStatus {
        int ntpLeapSecond;
        int ntpStratum;
        int ntpPrecision;
        double ntpRootDelay;
        double ntpRootDispersion;
        int ntpTC;
        int ntpMinTC;
        double ntpOffset;
        double ntpFrequency;
        double ntpSystemJitter;
        double ntpClockJitter;
        double ntpClockWander;
        int ntpNumPeers;
        int ntpNumGoodPeers;
        double ntpMaxPeerDelay;
        double ntpMaxPeerOffset;
        double ntpMaxPeerJitter;
        int ntpMinPeerStratum;
        int ntpSyncStatus;
        ntpPeerData ntp_peer_data[NTP_MAX_PEERS];
} ntpStatus;

class ntpDataFragment {
    public:
        int offset;
        int count;
        string data;
};



/* NTP status functions */
//extern int devIocStatsInitNtpStats (void);
int devIocStatsGetNtpStats (ntpStatus *pval);

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
#define PEER_SEL_MASK               0x07
#define PEER_SEL_SHIFT              0

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

static void poll_ntp_daemon(void);

bool find_substring(
        const string data,
        const string pattern,
        string *result);

bool find_substring(
        const string data,
        const string pattern,
        int occurrence,
        string *result);

int do_ntp_query(
        unsigned char op_code, 
        unsigned short association_id,
        string *ntp_data
        );

int get_association_ids(
        unsigned short *association_ids,
        unsigned short *peer_selections,
        int *num_associations,
        int max_association_ids
        );

int get_peer_stats(
        unsigned short *association_ids,
        int num_peers,
        ntpStatus *pval
        );

void parse_ntp_associations(
        unsigned short *association_ids,
        unsigned short *peer_selections,
        int num_associations,
        ntpStatus *pval);

void parse_ntp_sys_vars(
        ntpStatus *pval, 
        string ntp_data);

unsigned short reverse(unsigned short);

epicsShareFunc void devIocStatsNTPSetPollRate(const int rate);
epicsShareFunc void devIocStatsNTPDisable(void);
epicsShareFunc void devIocStatsNTPEnable(void);


#endif /*DEV_IOC_STATS_NTP_H*/

