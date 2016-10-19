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

#define NTP_PORT 123

#define NTP_ERROR -1

/* 
 * This uses an NTP mode 6 control message. 
 */

#define VER_MODE 0x16
#define OP_CODE 0x02

#define VER_MASK 0x38
#define VER_SHIFT 2

#define MORE_MASK 0x20
#define ERROR_MASK 0x40
#define RESPONSE_MASK 0x80

#define DATA_SIZE 468

int devIocStatsInitNtpStats (void) {
    return 0;
}

int devIocStatsGetNtpStats (ntpStatus *pval)
{
    struct sockaddr_in ntp_socket;
    struct in_addr address;
    int sd;
    int ret;
    fd_set fds;
    struct timeval timeout_val;
    FD_ZERO(&fds);
    
    /* NTP response message structure */
    struct {
        unsigned char ver_mode;
        unsigned char op_code;
        unsigned short sequence;
        unsigned char status1;
        unsigned char status2;
        unsigned short association_id;
        unsigned short offset;
        unsigned short count;
        char data[DATA_SIZE];
        char authenticator[96];
    } ntp_message;

    char buffer[DATA_SIZE];

    unsigned int ntp_version;

    /* 
     * Define the character strings used to parse the NTP 
     * daemon response string 
     */
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
    char *substr;
    char *ntp_param_value;

    /* Define the socket connection */
    inet_aton("127.0.0.1", &address);
    ntp_socket.sin_family = AF_INET;
    ntp_socket.sin_addr = address;
    ntp_socket.sin_port = htons(NTP_PORT);

    /* Construct the NTP control message */
    memset(&ntp_message, 0, sizeof(ntp_message));
    ntp_message.ver_mode = VER_MODE;
    ntp_message.op_code = OP_CODE;

    /* Define the timeout */
    timeout_val.tv_sec = 1;
    timeout_val.tv_usec = 0;

    /* Create the socket and send the message */
    if ((sd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
    {
        printf("Socket create error\n");
        return NTP_ERROR;
    }

    if (connect(sd, (struct sockaddr *)&ntp_socket, sizeof(ntp_socket)) < 0)
    {
        printf("Socket connect error\n");
        return NTP_ERROR;
    }

    FD_SET(sd, &fds);

    if (send(sd, &ntp_message, sizeof(ntp_message), 0) < 0)
    {
        printf("Send error\n");
        return NTP_ERROR;
    }

    /* Wait for the response from the NTP daemon */
    ret = select(sd+1, &fds, (fd_set *)0, (fd_set *)0, &timeout_val);
    
    if (ret == 0)
    {
        printf("Select 0 error\n");
        return NTP_ERROR;
    }

    /* Read the response from the NTP daemon */
    if ((ret = recv(sd, &ntp_message, sizeof(ntp_message), 0)) < 0)
    {
        printf("Select -1 error\n");
        return NTP_ERROR;
    }
    
    /* 
     * If we got this far, we have a valid response packet from the 
     * NTP daemon. 
     */

    /* Extract the NTP version number */
    ntp_version = (ntp_message.ver_mode & VER_MASK) >> VER_SHIFT;
    pval->ntpVersionNumber = ntp_version;

    /* Leap second status */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_LEAP)))
    {
        substr += sizeof(NTP_LEAP) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpLeapSecond = (int)(atoi(ntp_param_value));
    }

    /* Stratum */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_STRATUM)))
    {
        substr += sizeof(NTP_STRATUM) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpStratum = (double)(atof(ntp_param_value));
    }

    /* Precision */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_PRECISION)))
    {
        substr += sizeof(NTP_PRECISION) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpPrecision = (int)(atoi(ntp_param_value));
    }

    /* Root delay */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_ROOT_DELAY)))
    {
        substr += sizeof(NTP_ROOT_DELAY) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpRootDelay = (double)(atof(ntp_param_value));
    }

    /* Root dispersion */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_ROOT_DISPERSION)))
    {
        substr += sizeof(NTP_ROOT_DISPERSION) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpRootDispersion = (double)(atof(ntp_param_value));
    }

    /* Time constant */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_TC)))
    {
        substr += sizeof(NTP_TC) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpTC = (int)(atoi(ntp_param_value));
    }

    /* Minimum time constant */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_MINTC)))
    {
        substr += sizeof(NTP_MINTC) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpMinTC = (int)(atoi(ntp_param_value));
    }

    /* Offset */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_OFFSET)))
    {
        substr += sizeof(NTP_OFFSET) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpOffset = (double)(atof(ntp_param_value));
    }

    /* Frequency */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_FREQUENCY)))
    {
        substr += sizeof(NTP_FREQUENCY) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpFrequency = (double)(atof(ntp_param_value));
    }

    /* System jitter */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_SYS_JITTER)))
    {
        substr += sizeof(NTP_SYS_JITTER) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpSystemJitter = (double)(atof(ntp_param_value));
    }

    /* Clock jitter */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_CLOCK_JITTER)))
    {
        substr += sizeof(NTP_CLOCK_JITTER) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpClockJitter = (double)(atof(ntp_param_value));
    }

    /* Clock wander */
    strncpy(buffer, ntp_message.data, sizeof(buffer));
    if ((substr = strstr(buffer, NTP_CLOCK_WANDER)))
    {
        substr += sizeof(NTP_CLOCK_WANDER) - 1;
        ntp_param_value = strtok(substr, ",");

        pval->ntpClockWander = (double)(atof(ntp_param_value));
    }

    return 0;
}
