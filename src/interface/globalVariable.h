#ifndef __G_GLOABALVARIABLE_H__
#define __G_GLOABALVARIABLE_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <linux/can.h>
#include <time.h>
#include <string.h>

#include <linux/types.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/fs.h>

#include <net/if.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <stdatomic.h>

#define yinhantong //  映翰通标志位
#define SET_ERROR 0
#define SET_RECOVER 1


typedef struct
{
	/* Is this CAN (0) or CAN FD (1) */
	uint8_t ProtocolMode;

	/* Is Extended frame */
	uint8_t Extended;

	/* Length */
	uint8_t Length;

	/* RTR */
	uint8_t Remote;

	/* Error */
	uint8_t Error;

	/* Bit Rate Switch. Possible values 0 or 1. Valid only if ProtocolMode indicates CAN FD */
	uint8_t BRS;

	/* Error State Indicator.  Possible values 0 or 1. Valid only if ProtocolMode indicates CAN FD */
	uint8_t ESI;

	/* Data Length Code. Possible values 0-15. Valid only if ProtocolMode indicates CAN FD */
	uint8_t DLC;

	/* CAN ID */
	uint32_t ID;

	/* Reserved */
	uint32_t Reserved;

	/*
	  TIMESTAMP_NOT_REQUIRED is a macro that will be defined by Target teams
	  PIL, C166, FM5, xPC if they do not require the timestamp field during code
	  generation. By default, timestamp is defined. If the targets do not require
	  the timestamp field, they should define the macro TIMESTAMP_NOT_REQUIRED before
	  including this header file for code generation.
	*/
#ifndef TIMESTAMP_NOT_REQUIRED
	/* Timestamp */
	double Timestamp;
#endif

	/* Data field */
	uint8_t Data[64];

} CANFD_MESSAGE;

typedef struct
{

	/* Is Extended frame */
	uint8_t Extended;

	/* Length */
	uint8_t Length;

	/* RTR */
	uint8_t Remote;

	/* Error */
	uint8_t Error;

	/* CAN ID */
	uint32_t ID;

	/*
	  TIMESTAMP_NOT_REQUIRED is a macro that will be defined by Target teams
	  PIL, C166, FM5, xPC if they do not require the timestamp field during code
	  generation. By default, timestamp is defined. If the targets do not require
	  the timestamp field, they should define the macro TIMESTAMP_NOT_REQUIRED before
	  including this header file for code generation.
	*/
#ifndef TIMESTAMP_NOT_REQUIRED
	/* Timestamp */
	double Timestamp;
#endif

	/* Data field */
	uint8_t Data[8];

} CAN_MESSAGE;

// #########################################################################################//

// #########################################################################################//

/*
 * CAN bit-timing parameters
 */
struct can_bittiming
{
	__u32 bitrate;		/* Bit-rate in bits/second */
	__u32 sample_point; /* Sample point in one-tenth of a percent */
	__u32 tq;			/* Time quanta (TQ) in nanoseconds */
	__u32 prop_seg;		/* Propagation segment in TQs */
	__u32 phase_seg1;	/* Phase buffer segment 1 in TQs */
	__u32 phase_seg2;	/* Phase buffer segment 2 in TQs */
	__u32 sjw;			/* Synchronisation jump width in TQs */
	__u32 brp;			/* Bit-rate prescaler */
};

/*
 * CAN harware-dependent bit-timing constant
 */
struct can_bittiming_const
{
	char name[16];	 /* Name of the CAN controller hardware */
	__u32 tseg1_min; /* Time segement 1 = prop_seg + phase_seg1 */
	__u32 tseg1_max;
	__u32 tseg2_min; /* Time segement 2 = phase_seg2 */
	__u32 tseg2_max;
	__u32 sjw_max; /* Synchronisation jump width */
	__u32 brp_min; /* Bit-rate prescaler */
	__u32 brp_max;
	__u32 brp_inc;
};

/*
 * CAN clock parameters
 */
struct can_clock
{
	__u32 freq; /* CAN system clock frequency in Hz */
};

/*
 * CAN operational and error states
 */
enum can_state
{
	CAN_STATE_ERROR_ACTIVE = 0, /* RX/TX error count < 96 */
	CAN_STATE_ERROR_WARNING,	/* RX/TX error count < 128 */
	CAN_STATE_ERROR_PASSIVE,	/* RX/TX error count < 256 */
	CAN_STATE_BUS_OFF,			/* RX/TX error count >= 256 */
	CAN_STATE_STOPPED,			/* Device is stopped */
	CAN_STATE_SLEEPING,			/* Device is sleeping */
	CAN_STATE_MAX
};

/*
 * CAN bus error counters
 */
struct can_berr_counter
{
	__u16 txerr;
	__u16 rxerr;
};

/*
 * CAN controller mode
 */
struct can_ctrlmode
{
	__u32 mask;
	__u32 flags;
};

/*
 * CAN device statistics
 */
struct can_device_stats
{
	__u32 bus_error;		/* Bus errors */
	__u32 error_warning;	/* Changes to error warning state */
	__u32 error_passive;	/* Changes to error passive state */
	__u32 bus_off;			/* Changes to bus off state */
	__u32 arbitration_lost; /* Arbitration lost errors */
	__u32 restarts;			/* CAN controller re-starts */
};

/*
 * CAN netlink interface
 */
enum
{
	IFLA_CAN_UNSPEC,
	IFLA_CAN_BITTIMING,
	IFLA_CAN_BITTIMING_CONST,
	IFLA_CAN_CLOCK,
	IFLA_CAN_STATE,
	IFLA_CAN_CTRLMODE,
	IFLA_CAN_RESTART_MS,
	IFLA_CAN_RESTART,
	IFLA_CAN_BERR_COUNTER,
	__IFLA_CAN_MAX
};

#define IFLA_CAN_MAX (__IFLA_CAN_MAX - 1)

// #########################################################################################//

// #########################################################################################//

typedef struct
{
	uint16_t year;	  /**< @brief Year       */
	uint16_t month;	  /**< @brief Month      */
	uint16_t day;	  /**< @brief Day        */
	uint16_t hour;	  /**< @brief Hour       */
	uint16_t minutes; /**< @brief Minutes    */
	uint8_t seconds;  /**< @brief Seconds    */
} Rtc_Ip_TimedateType;
#endif