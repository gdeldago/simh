/* pdp8_rf.c: RF08 fixed head disk simulator

   Copyright (c) 1993-1999, Robert M Supnik

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   ROBERT M SUPNIK BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name of Robert M Supnik shall not
   be used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from Robert M Supnik.

   14-Apr-99	RMS	Changed t_addr to unsigned
   30-Mar-98	RMS	Fixed bug in RF bootstrap.

   The RF08 is a head-per-track disk.  It uses the three cycle data break
   facility.  To minimize overhead, the entire RF08 is buffered in memory.

   Two timing parameters are provided:

   rf_time	Interword timing, must be non-zero
   rf_burst	Burst mode, if 0, DMA occurs cycle by cycle; otherwise,
		DMA occurs in a burst
*/

#include "pdp8_defs.h"
#include <math.h>

/* Constants */

#define RF_NUMWD	2048				/* words/track */
#define RF_NUMTR	128				/* tracks/disk */
#define RF_NUMDK	4				/* disks/controller */
#define RF_SIZE		(RF_NUMDK * RF_NUMTR * RF_NUMWD) /* words/drive */
#define RF_WC		07750				/* word count */
#define RF_MA		07751				/* mem address */
#define RF_WMASK	(RF_NUMWD - 1)			/* word mask */

/* Parameters in the unit descriptor */

#define FUNC		u4				/* function */
#define RF_READ		3				/* read */
#define RF_WRITE	5				/* write */

/* Status register */

#define RFS_PCA		04000				/* photocell status */
#define RFS_DRE		02000				/* data req enable */
#define RFS_WLS		01000				/* write lock status */
#define RFS_EIE		00400				/* error int enable */
#define RFS_PIE		00200				/* photocell int enb */
#define RFS_CIE		00100				/* done int enable */
#define RFS_MEX		00070				/* memory extension */
#define RFS_DRL		00004				/* data late error */
#define RFS_NXD		00002				/* non-existent disk */
#define RFS_PER		00001				/* parity error */
#define RFS_ERR		(RFS_WLS + RFS_DRL + RFS_NXD + RFS_PER)
#define RFS_V_MEX	3

#define GET_MEX(x)	(((x) & RFS_MEX) << (12 - RFS_V_MEX))
#define GET_POS(x)	((int) fmod (sim_gtime() / ((double) (x)), \
			((double) RF_NUMWD)))
#define UPDATE_PCELL	if (GET_POS(rf_time) < 6) rf_sta = rf_sta | RFS_PCA; \
			else rf_sta = rf_sta & ~RFS_PCA
#define RF_INT_UPDATE	if ((rf_done && (rf_sta & RFS_CIE)) || \
			    ((rf_sta & RFS_ERR) && (rf_sta & RFS_EIE)) || \
			    ((rf_sta & RFS_PCA) && (rf_sta & RFS_PIE))) \
				int_req = int_req | INT_RF; \
			else int_req = int_req & ~INT_RF

extern int32 int_req, stop_inst;
extern unsigned int16 M[];
extern UNIT cpu_unit;
int32 rf_sta = 0;					/* status register */
int32 rf_da = 0;					/* disk address */
int32 rf_done = 0;					/* done flag */
int32 rf_wlk = 0;					/* write lock */
int32 rf_time = 10;					/* inter-word time */
int32 rf_burst = 1;					/* burst mode flag */
int32 rf_stopioe = 1;					/* stop on error */
t_stat rf_svc (UNIT *uptr);
t_stat pcell_svc (UNIT *uptr);
t_stat rf_reset (DEVICE *dptr);
t_stat rf_boot (int32 unitno);
extern t_stat sim_activate (UNIT *uptr, int32 delay);
extern t_stat sim_cancel (UNIT *uptr);

/* RF08 data structures

   rf_dev	RF device descriptor
   rf_unit	RF unit descriptor
   pcell_unit	photocell timing unit (orphan)
   rf_reg	RF register list
*/

UNIT rf_unit =
	{ UDATA (&rf_svc, UNIT_FIX+UNIT_ATTABLE+UNIT_BUFABLE+UNIT_MUSTBUF,
	RF_SIZE) };

UNIT pcell_unit = { UDATA (&pcell_svc, 0, 0) };

REG rf_reg[] = {
	{ ORDATA (STA, rf_sta, 12) },
	{ ORDATA (DA, rf_da, 20) },
	{ ORDATA (MA, M[RF_MA], 12) },
	{ ORDATA (WC, M[RF_WC], 12) },
	{ FLDATA (DONE, rf_done, 0) },
	{ FLDATA (INT, int_req, INT_V_RF) },
	{ ORDATA (WLK, rf_wlk, 32) },
	{ DRDATA (TIME, rf_time, 24), REG_NZ + PV_LEFT },
	{ FLDATA (BURST, rf_burst, 0) },
	{ FLDATA (STOP_IOE, rf_stopioe, 0) },
	{ NULL }  };

DEVICE rf_dev = {
	"RF", &rf_unit, rf_reg, NULL,
	1, 8, 20, 1, 8, 12,
	NULL, NULL, &rf_reset,
	&rf_boot, NULL, NULL };

/* IOT routines */

int32 rf60 (int32 pulse, int32 AC)
{
int32 t;

UPDATE_PCELL;						/* update photocell */
switch (pulse) {					/* decode IR<9:11> */
case 1:							/* DCMA */
	rf_da = rf_da & ~07777;				/* clear DAR<8:19> */
	return AC;
case 3:case 5:						/* DMAR, DMAW */
	rf_da = (rf_da & ~07777) | AC;			/* DAR<8:19> <- AC */
	rf_done = 0;					/* clear done */
	rf_sta = rf_sta & ~RFS_ERR;			/* clear errors */
	RF_INT_UPDATE;					/* update int req */
	rf_unit.FUNC = pulse;				/* save function */
	t = (rf_da & RF_WMASK) - GET_POS (rf_time);	/* delta to new loc */
	if (t < 0) t = t + RF_NUMWD;			/* wrap around? */
	sim_activate (&rf_unit, t * rf_time);		/* schedule op */
	return 0;					/* clear AC */
default:
	return (stop_inst << IOT_V_REASON) + AC;  }	/* end switch */
}

int32 rf61 (int32 pulse, int32 AC)
{
UPDATE_PCELL;						/* update photocell */
switch (pulse) {					/* decode IR<9:11> */
case 1:							/* DCIM */
	rf_sta = rf_sta & 07007;			/* clear STA<3:8> */
	int_req = int_req & ~INT_RF;			/* clear int req */
	sim_cancel (&pcell_unit);			/* cancel photocell */
	return AC;
case 2:							/* DSAC */
	return ((rf_da & RF_WMASK) == GET_POS (rf_time))? IOT_SKP + AC: AC;
case 5:							/* DIML */
	rf_sta = (rf_sta & 07007) | (AC & 0770);	/* STA<3:8> <- AC */
	if (rf_sta & RFS_PIE)				/* photocell int? */
		sim_activate (&pcell_unit, (RF_NUMWD - GET_POS (rf_time)) *
			rf_time);
	else sim_cancel (&pcell_unit);
	RF_INT_UPDATE;					/* update int req */
	return 0;					/* clear AC */
case 6:							/* DIMA */
	return rf_sta;					/* AC <- STA<0:11> */
default:
	return (stop_inst << IOT_V_REASON) + AC;  }	/* end switch */
}

/* IOT's, continued */

int32 rf62 (int32 pulse, int32 AC)
{
UPDATE_PCELL;						/* update photocell */
switch (pulse) {					/* decode IR<9:11> */
case 1:							/* DFSE */
	return (rf_sta & RFS_ERR)? IOT_SKP + AC: AC;
case 2:							/* DFSC */
	return (rf_done)? IOT_SKP + AC: AC;
case 3:							/* DISK */
	return (rf_done || (rf_sta & RFS_ERR))? IOT_SKP + AC: AC;
case 6:							/* DMAC */
	return rf_da & 07777;				/* AC <- DAR<0:11> */
default:
	return (stop_inst << IOT_V_REASON) + AC;  }	/* end switch */
}

int32 rf64 (int32 pulse, int32 AC)
{
UPDATE_PCELL;						/* update photocell */
switch (pulse) {					/* decode IR<9:11> */
case 1:							/* DCXA */
	rf_da = rf_da & 07777;				/* clear DAR<0:7> */
	return AC;
case 3:							/* DXAL */
	rf_da = (rf_da & 07777) | ((AC & 0377) << 12);	/* DAR<0:7> <- AC */
	return 0;					/* clear AC */
case 5:							/* DXAC */
	return ((rf_da >> 12) & 0377);			/* AC <- DAR<0:7> */
default:
	return (stop_inst << IOT_V_REASON) + AC;  }	/* end switch */
}

/* Unit service

   Note that for reads and writes, memory addresses wrap around in the
   current field.  This code assumes the entire disk is buffered.
*/

t_stat rf_svc (UNIT *uptr)
{
int32 pa, t, mex;

UPDATE_PCELL;						/* update photocell */
if ((uptr -> flags & UNIT_BUF) == 0) {			/* not buf? abort */
	rf_sta = rf_sta | RFS_NXD;
	rf_done = 1;
	RF_INT_UPDATE;					/* update int req */
	return IORETURN (rf_stopioe, SCPE_UNATT);  }

mex = GET_MEX (rf_sta);
do { 	M[RF_MA] = (M[RF_MA] + 1) & 07777;		/* incr mem addr */
	pa = mex | M[RF_MA]; 				/* add extension */
	if (uptr -> FUNC == RF_READ) {
		if (MEM_ADDR_OK (pa))			/* read, check nxm */
			M[pa] = *(((int16 *) uptr -> filebuf) + rf_da);  }
	else {	t = ((rf_da >> 15) & 030) | ((rf_da >> 14) & 07);
		if ((rf_wlk >> t) & 1) rf_sta = rf_sta | RFS_WLS;
		else {	*(((int16 *) uptr -> filebuf) + rf_da) = M[pa];
			if (((t_addr) rf_da) >= uptr -> hwmark)
				uptr -> hwmark = rf_da + 1;  }  }
	rf_da = (rf_da + 1) & 03777777;			/* incr disk addr */
	M[RF_WC] = (M[RF_WC] + 1) & 07777;  }		/* incr word count */
while ((M[RF_WC] != 0) && (rf_burst != 0));		/* brk if wc, no brst */

if (M[RF_WC] != 0)					/* more to do? */
	sim_activate (&rf_unit, rf_time);		/* sched next */
else {	rf_done = 1;					/* done */
	RF_INT_UPDATE;  }				/* update int req */
return SCPE_OK;
}

/* Photocell unit service */

t_stat pcell_svc (UNIT *uptr)
{
rf_sta = rf_sta | RFS_PCA;				/* set photocell */
if (rf_sta & RFS_PIE) {					/* int enable? */
	sim_activate (&pcell_unit, RF_NUMWD * rf_time);
	int_req = int_req | INT_RF;  }
return SCPE_OK;
}

/* Reset routine */

t_stat rf_reset (DEVICE *dptr)
{
rf_sta = rf_da = 0;
rf_done = 1;
int_req = int_req & ~INT_RF;				/* clear interrupt */
sim_cancel (&rf_unit);
sim_cancel (&pcell_unit);
return SCPE_OK;
}

/* Bootstrap routine */

#define BOOT_START 07750
#define BOOT_LEN (sizeof (boot_rom) / sizeof (int))

static const int32 boot_rom[] = {
	07600,			/* 7750, CLA CLL	; also word count */
	06603,			/* 7751, DMAR		; also address */
	06622,			/* 7752, DFSC		; done? */
	05352,			/* 7753, JMP .-1	; no */
	05752			/* 7754, JMP @.-2	; enter boot */
};

t_stat rf_boot (int32 unitno)
{
int32 i;
extern int32 saved_PC;

for (i = 0; i < BOOT_LEN; i++) M[BOOT_START + i] = boot_rom[i];
saved_PC = BOOT_START;
return SCPE_OK;
}