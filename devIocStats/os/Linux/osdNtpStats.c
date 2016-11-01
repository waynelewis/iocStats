/*************************************************************************\
* Copyright (c) 2016 Osprey Distributed Control Systems
* Copyright (c) 2009 Helmholtz-Zentrum Berlin fuer Materialien und Energie.
* Copyright (c) 2002 The University of Chicago, as Operator of Argonne
*     National Laboratory.
* Copyright (c) 2002 The Regents of the University of California, as
*     Operator of Los Alamos National Laboratory.
* EPICS BASE Versions 3.13.7
* and higher are distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
\*************************************************************************/

/* osdNtpStats.c - NTP service status */

/*
 *  Author: Wayne Lewis (OspreyDCS)
 *
 *  Modification History
 *  2016-10-18 Wayne Lewis (OspreyDCS)
 *     Original implementation
 *     With thanks to G.Richard Keech for information in ntpstat.c at:
 *     https://github.com/darkhelmet/ntpstat/blob/master/ntpstat.c
 *
 */

#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <devIocStats.h>
#include "epicsTypes.h"
#include "osdNtpStats.h"

int devIocStatsInitNtpStats (void) {
    return 0;
}

int devIocStatsGetNtpStats (ntpStatus *pval)
{
    struct ntp_control ntp_message;
    int ret;

    unsigned short association_ids[MAX_ASSOCIATION_IDS];
    unsigned short peer_selections[MAX_ASSOCIATION_IDS];
    int num_associations;

    // Perform an NTP variable query to get the system level status
    if (( ret = do_ntp_query(
                    NTP_OP_READ_VAR, 
                    SYS_ASSOCIATION_ID, 
                    &ntp_message) != 0))
        return ret;

    parse_ntp_sys_vars(&ntp_message, pval);

    // Perform an NTP status query to get the association IDs
    if ((ret = get_association_ids(
                    association_ids,
                    peer_selections,
                    &num_associations,
                    MAX_ASSOCIATION_IDS) != 0))
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
        if (peer_selections[i] >= NTP_PEER_SEL_NOT_OUTLYER)
            num_good_peers++;

        if (peer_selections[i] >= NTP_PEER_SEL_SYNC_OVER_MAX)
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
        pval->ntpPeerSelectionStatus[i] = peer_selections[i];

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

    char buffer[DATA_SIZE];
    char *substr;
    char *ntp_param_value;

    const char NTP_PEER_STRATUM[] = "stratum=";
    const char NTP_PEER_POLL[] = "ppoll=";
    const char NTP_PEER_REACH[] = "reach=";
    const char NTP_ROOT_DELAY[] = "rootdelay=";
    const char NTP_PEER_DELAY[] = "delay=";
    const char NTP_PEER_OFFSET[] = "offset=";
    const char NTP_PEER_JITTER[] = "jitter=";

    int stratums[num_peers];
    int polls[num_peers];
    int reaches[num_peers];
    double delays[num_peers];
    double offsets[num_peers];
    double jitters[num_peers];

    double max_delay;
    double max_jitter;
    double max_offset;
    double min_stratum;

    struct ntp_control ntp_message;

    // Iterate through the associated peers and gather the required data
    for (i = 0; i < num_peers; i++)
    {
        ret = do_ntp_query(NTP_OP_READ_STS, association_ids[i], &ntp_message);
        if (ret < 0)
            return ret;

        /* Peer stratum */
        strncpy(buffer, ntp_message.data, sizeof(buffer));
        if ((substr = strstr(buffer, NTP_PEER_STRATUM)))
        {
            substr += sizeof(NTP_PEER_STRATUM) - 1;
            ntp_param_value = strtok(substr, ",");

            stratums[i] = (int)(atoi(ntp_param_value));
        }

        /* Peer poll */
        strncpy(buffer, ntp_message.data, sizeof(buffer));
        if ((substr = strstr(buffer, NTP_PEER_POLL)))
        {
            substr += sizeof(NTP_PEER_POLL) - 1;
            ntp_param_value = strtok(substr, ",");

            polls[i] = (int)(atoi(ntp_param_value));
        }

        /* Peer reach */
        strncpy(buffer, ntp_message.data, sizeof(buffer));
        if ((substr = strstr(buffer, NTP_PEER_REACH)))
        {
            substr += sizeof(NTP_PEER_REACH) - 1;
            ntp_param_value = strtok(substr, ",");

            reaches[i] = (int)(atoi(ntp_param_value));
        }

        /* Peer delay */
        strncpy(buffer, ntp_message.data, sizeof(buffer));
        /* First go past the root delay */
        if ((substr = strstr(buffer, NTP_ROOT_DELAY)))
        {
            substr += sizeof(NTP_ROOT_DELAY);
            strncpy(buffer, substr, sizeof(buffer));
            if ((substr = strstr(buffer, NTP_PEER_DELAY)))
            {
                substr += sizeof(NTP_PEER_DELAY) - 1;
                ntp_param_value = strtok(substr, ",");

                delays[i] = (double)(atof(ntp_param_value));
            }
        }

        /* Peer offset */
        strncpy(buffer, ntp_message.data, sizeof(buffer));
        if ((substr = strstr(buffer, NTP_PEER_OFFSET)))
        {
            substr += sizeof(NTP_PEER_OFFSET) - 1;
            ntp_param_value = strtok(substr, ",");

            offsets[i] = (double)(atof(ntp_param_value));
        }

        /* Peer jitter */
        strncpy(buffer, ntp_message.data, sizeof(buffer));
        if ((substr = strstr(buffer, NTP_PEER_JITTER)))
        {
            substr += sizeof(NTP_PEER_JITTER) - 1;
            ntp_param_value = strtok(substr, ",");

            jitters[i] = (double)(atof(ntp_param_value));
        }

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
        pval->ntpPeerStratums[i] = stratums[i];
        pval->ntpPeerPolls[i] = polls[i];
        pval->ntpPeerReaches[i] = reaches[i];
        pval->ntpPeerDelays[i] = delays[i];
        pval->ntpPeerOffsets[i] = offsets[i];
        pval->ntpPeerJitters[i] = jitters[i];
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
    ret = do_ntp_query(NTP_OP_READ_STS, SYS_ASSOCIATION_ID, &ntp_message);
    if (ret != 0)
        return ret;

    // Extract the association IDs from the response
    association_count = 0;
    for (i = 0; i < DATA_SIZE; i += 4)
    {
        // Decode the association ID
        association_id = 0x100 * ntp_message.data[i+1];
        association_id += ntp_message.data[i];

        printf("association_id = %d\n", association_id);

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


