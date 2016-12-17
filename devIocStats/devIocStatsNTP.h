#ifndef DEV_IOC_STATS_NTP_H 
#define DEV_IOC_STATS_NTP_H

/*
 * This software is Copyright by the Board of Trustees of Michigan
 * State University (c) Copyright 2015.
 *   
 */

/* devIocStatsNTP.h - device support routines for NTP statistics 
 *
 *  Authors: Wayne Lewis and Michael Davidsaver
 *  Date:  2016-11-06
 */

/* NTP definitions */

#include <string>
#include <vector>
#include <map>

#include <epicsTime.h>

#include <ntphelper.h>

using std::string;

// Record interface structures 
struct pvtNTPArea
{
    std::string parameter;
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

// Data containing structures

struct ntpStatus {
        epicsTime updateTime;
        bool ntpDaemonOk;
        ntp_data_t ntp_sys_data;
        std::vector<ntp_data_t> ntp_peer_data;

        // Initialize some of the values
        ntpStatus() :ntpDaemonOk(false) {}
};

/* Definitions */
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

#define MORE_MASK                   0x20
#define RESPONSE_MASK               0x80
#define PEER_SEL_MASK               0x0700
#define PEER_SEL_SHIFT              8

// Function prototypes
bool devIocStatsGetNtpStats(ntpStatus *pval);

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

string make_string(double);

epicsShareFunc void devIocStatsNTPSetPollRate(const int rate);
epicsShareFunc void devIocStatsNTPDisable(void);
epicsShareFunc void devIocStatsNTPEnable(void);


#endif /*DEV_IOC_STATS_NTP_H*/

