/* ms101_dsk.c: Micro Sistemas MS101 Floppy Disk Drive simulator

   Copyright (c) 2014-2015, Gustavo del Dago

*/

#include <stdio.h>

#include "ms101_defs.h"


t_stat dsk_svc (UNIT *uptr);
t_stat dsk_reset (DEVICE *dptr);
const char *dsk_description (DEVICE *dptr);

t_stat dsk_attach(UNIT *uptr, char *cptr);
t_stat dsk_detach(UNIT *uptr);

int32 find_unit_index (UNIT *uptr);

t_stat dsk_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr);

int32 find_unit_index (UNIT *uptr);

extern int32 PCX;
extern int32 int_req;

extern int32 saved_PC;
extern int32 SP;

extern unsigned char M[];
void show_debug_info(void);

/*

	NEC µPD372D

	IN RR0
	OUT WR0
*/

/* WRITE REGISTER 0 */
#define WR0_RST 0x80	/* Reset */
#define WR0_MBL 0x40	/* Must Be Low */
#define WR0_HLD 0x08	/* Head Load */
#define WR0_LCT 0x04	/* Low Current */
#define WR0_WFR 0x02	/* Write Fault Reset */


/* READ REGISTER 0 */
#define	RR0_ALH	0x80	/* Always High */
#define RR0_RYB	0x40	/* Drive B Ready */
#define RR0_UB1 0x20	/* UB1 (Drive B1 Selected) */
#define RR0_UB0 0x10	/* UB0 (Drive B0 Selected) */
#define RR0_ERR	0x08	/* Error */
#define RR0_TRQ 0x04	/* Timer Request */
#define RR0_IRQ 0x02	/* Index Request */
#define RR0_DRQ	0x01	/* Data Request */

/*
    Supuestamente podemos controlar 4 discos
*/
#ifdef MS101
    #define NUM_DISK_UNITS 1
    UNIT dsk_unit[] = {
        { UDATA (&dsk_svc, UNIT_ATTABLE, 0), 0}
    };
#else
    #define NUM_DISK_UNITS 4
    UNIT dsk_unit[] = {
        { UDATA (&dsk_svc, UNIT_ATTABLE, 0), 0},
        { UDATA (&dsk_svc, UNIT_ATTABLE, 0), 0},
        { UDATA (&dsk_svc, UNIT_ATTABLE, 0), 0},
        { UDATA (&dsk_svc, UNIT_ATTABLE, 0), 0}
    };
#endif

/*
    Vamos preparando los registros del PD372D y de la Shugart
*/
static int	WR0, WR1, WR2, WR3, WR4, WR5, WR6;
static int	RR0, RR1, RR2;

REG dsk_reg[] = {
	{ HRDATA (WR0, WR0, 8) },
    { HRDATA (RR0, RR0, 8) },
	{ NULL }
};


DEVICE dsk_dev = {
	"DSK", 			    /* name */
	dsk_unit, 		    /* units */
	dsk_reg, 		    /* registers */
	NULL,			    /* modifiers */
	NUM_DISK_UNITS,		/* #units */
	16, 			    /* address radix */
	8, 			        /* address width */
	1, 			        /* addr increment */
	16, 			    /* data radix */
	8,			        /* data width */
    NULL, 			    /* examine routine */
	NULL, 			    /* deposit routine */
	&dsk_reset,		    /* reset routine */
    NULL,   		    /* boot routine */
	&dsk_attach,		/* attach routine */
	&dsk_detach,		/* detach routine */
    NULL, 			    /* context */
	DEV_DEBUG,		    /* flags */
	0,			        /* debug control flags */
    NULL, 			    /* debug flag names */
	NULL, 			    /* memory size change */
	NULL, 			    /* logical name */
	&dsk_help,		    /* help routine */
	NULL,			    /* attach help routine */
	NULL,			    /* help context */
	&dsk_description	/* device description */
};

/*  Service routines to handle simlulator functions */

/* service routine - actually gets char & places in buffer */



t_stat dsk_svc (UNIT *uptr)
{

	//sim_printf ("dsk_svc\n");
    //sim_activate (&dsk_unit, dsk_unit.wait);            /* continue poll */

	return (SCPE_OK);
}

/* Reset routine */
t_stat dsk_reset (DEVICE *dptr)
{
	t_stat r;
    RR0 = RR0_ALH;	/* Always High */

	return (SCPE_OK);
}

/*  I/O instruction handlers, called from the CPU module when an
    IN or OUT instruction is issued.

    Each function is passed an 'io' flag, where 0 means a read from
    the port, and 1 means a write to the port.  On input, the actual
    input is passed as the return value, on output, 'data' is written
    to the device.
*/

/*
	NEC µPD372D

	Write Register 0
	Read  Register 0
*/

int32 port28(int32 io, int32 data)
{
	if (io == 0)
	{
		return (RR0);
	}
    else
    {
		WR0 = data;	/* R0 bit 6 = 1 (DRIVE READY) */
    }
}

const char *dsk_description (DEVICE *dptr)
{

	return ("MS101 Floppy Disk (µPD372D)");
}

/* Attach routine */
t_stat dsk_attach(UNIT *uptr, char *cptr)
{
   	return (SCPE_OK);
}


/* Detach routine */
t_stat dsk_detach(UNIT *uptr)
{
   	return (SCPE_OK);
}


t_stat dsk_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr)
{

	return (SCPE_OK);
}

t_stat dsk_boot (int32 unit_num, DEVICE *dptr)
{
	return (SCPE_OK);
}




