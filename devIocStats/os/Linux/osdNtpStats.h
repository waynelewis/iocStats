#ifndef OSD_NTP_STATS_H
#define OSD_NTP_STATS_H

#define MAX_MAC_LEN (6 + sizeof(unsigned int))

#define NTP_PORT    123

#define NTP_NO_ERROR                0
#define NTP_SOCKET_OPEN_ERROR       -1
#define NTP_SOCKET_CONNECT_ERROR    -2
#define NTP_COMMAND_SEND_ERROR      -3
#define NTP_TIMEOUT_ERROR           -4
#define NTP_SELECT_ERROR            -5
#define NTP_DAEMON_COMMS_ERROR      -6


// Use a NTP mode 6 control message
#define NTP_VER_MODE 0x16
#define NTP_OP_READ_STS 0x01
#define NTP_OP_READ_VAR 0x02
#define NTP_OP_READ_CLK 0x04
#define SYS_ASSOCIATION_ID 0
#define NTP_PEER_SEL_REJECT             0
#define NTP_PEER_SEL_SANE               1
#define NTP_PEER_SEL_CORRECT            2
#define NTP_PEER_SEL_CANDIDATE          3
#define NTP_PEER_SEL_NOT_OUTLYER        4
#define NTP_PEER_SEL_SYNC_OVER_MAX      5
#define NTP_PEER_SEL_SYNC_UNDER_MAX     6


#define VER_MASK 0x38
#define VER_SHIFT 2

#define MODE_MASK 0x07
#define MODE_SHIFT 0

#define OP_CODE_MASK 0x1f
#define OP_CODE_SHIFT 0
#define MORE_MASK 0x20
#define MORE_SHIFT 5
#define ERROR_MASK 0x40
#define ERROR_SHIFT 6
#define RESPONSE_MASK 0x80
#define RESPONSE_SHIFT 7

#define PEER_SEL_MASK 0x07
#define PEER_SEL_SHIFT 0

#define DATA_SIZE 486
#define AUTH_SIZE 96

#define MAX_ASSOCIATION_IDS 10

struct ntp_control {
	unsigned char ver_mode;		/* leap, version, mode */
	unsigned char op_code;		/* response, more, error, opcode */
	unsigned short sequence;		/* sequence number of request */
	unsigned char status1;			/* status word for association */
	unsigned char status2;			/* status word for association */
	unsigned short association_id;		/* association ID */
	unsigned short offset;			/* offset of this batch of data */
	unsigned short count;			/* count of data in this packet */
    char data[DATA_SIZE];  /* string data returned with packet */
	unsigned int authenticator[AUTH_SIZE];
};

// Function prototypes
void die(char *message);

int do_ntp_query(
        unsigned char op_code, 
        unsigned short association_id,
        struct ntp_control *ntp_message
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
        struct ntp_control *ntp_message,
        ntpStatus *pval);

#endif /*OSD_NTP_STATS_H*/

