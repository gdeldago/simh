/* ms101_cpu.c: Micro Sistemas MS101 CPU simulator

   Copyright (c) 2014-2015, Gustavo del Dago

*/

#include <stdio.h>
#if defined (_WIN32)
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "ms101_defs.h"

#define UNIT_V_OPSTOP   (UNIT_V_UF)                     /* Stop on Invalid OP? */
#define UNIT_OPSTOP     (1 << UNIT_V_OPSTOP)
#define UNIT_V_MSIZE    (UNIT_V_UF+2)                   /* Memory Size */
#define UNIT_MSIZE      (1 << UNIT_V_MSIZE)

unsigned char M[MAXMEMSIZE];                            /* memory */
int32 A = 0;                                            /* accumulator */
int32 BC = 0;                                           /* BC register pair */
int32 DE = 0;                                           /* DE register pair */
int32 HL = 0;                                           /* HL register pair */
int32 SP = 0;                                           /* Stack pointer */
int32 C = 0;                                            /* carry flag */
int32 Z = 0;                                            /* Zero flag */
int32 AC = 0;                                           /* Aux carry */
int32 S = 0;                                            /* sign flag */
int32 P = 0;                                            /* parity flag */
int32 saved_PC = 0;                                     /* program counter */
int32 SR = 0;                                           /* switch register */
int32 INTE = 0;                                         /* Interrupt Enable */
int32 int_req = 0;                                      /* Interrupt request */

static uint32 tStates = 0;
static uint32 sliceLength = 10;
int32 PCX;                                              /* External view of PC */

extern int32 sim_int_char;
extern uint32 sim_brk_types, sim_brk_dflt, sim_brk_summ;/* breakpoint info */

/* function prototypes */

t_stat cpu_ex (t_value *vptr, t_addr addr, UNIT *uptr, int32 sw);
t_stat cpu_dep (t_value val, t_addr addr, UNIT *uptr, int32 sw);
t_stat cpu_reset (DEVICE *dptr);
t_stat cpu_set_size (UNIT *uptr, int32 val, char *cptr, void *desc);
t_stat cpu_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr);

void setarith(int32 reg);
void setlogical(int32 reg);
void setinc(int32 reg);
int32 getreg(int32 reg);
void putreg(int32 reg, int32 val);
int32 getpair(int32 reg);
int32 getpush(int32 reg);
void putpush(int32 reg, int32 data);
void putpair(int32 reg, int32 val);
void parity(int32 reg);
int32 cond(int32 con);

extern int32 port11 (int32 io, int32 data);
extern int32 port14 (int32 io, int32 data);
extern int32 port28 (int32 io, int32 data);
extern int32 port29 (int32 io, int32 data);
extern int32 port2A (int32 io, int32 data);
extern int32 port2B (int32 io, int32 data);
extern int32 port2C (int32 io, int32 data);
extern int32 port30 (int32 io, int32 data);
extern int32 port38 (int32 io, int32 data);
extern int32 port39 (int32 io, int32 data);

int32 nulldev(int32 io, int32 data);

int32 load_ROMS (void);
int32 load_ROM (char * filename, int32 offset);

/* This is the I/O configuration table.  There are 255 possible
device addresses, if a device is plugged to a port it's routine
address is here, 'nulldev' means no device is available
*/
struct idev {
    int32 (*routine)(int32, int32);
};
struct idev dev_table[256] = {
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 000 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 008 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 010 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 018 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 020 */
{&port28},  {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 028 */
{&port30},  {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 030 */
{&port38},  {&port39},  {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 038 */

{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 040 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 048 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 050 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 058 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 060 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 068 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 070 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 078 */

{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 080 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 088 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 090 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 098 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0A0 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0A8 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0B0 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0B8 */

{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0C0 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0C8 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0D0 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0D8 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0E0 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0E8 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0F0 */
{&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, {&nulldev}, /* 0F8 */

};


/* CPU data structures

   cpu_dev      CPU device descriptor
   cpu_unit     CPU unit descriptor
   cpu_reg      CPU register list
   cpu_mod      CPU modifiers list
*/

UNIT cpu_unit = { UDATA (NULL, UNIT_FIX + UNIT_BINK, MAXMEMSIZE) };

REG cpu_reg[] = {
    { HRDATA (PC, saved_PC, 16) },
    { HRDATA (A, A, 16) },
    { HRDATA (BC, BC, 16) },
    { HRDATA (DE, DE, 16) },
    { HRDATA (HL, HL, 16) },
    { HRDATA (SP, SP, 16) },
    { FLDATA (C, C, 16) },
    { FLDATA (Z, Z, 16) },
    { FLDATA (AC, AC, 16) },
    { FLDATA (S, S, 16) },
    { FLDATA (P, P, 16) },
    { FLDATA (INTE, INTE, 16) },
    { HRDATA (SR, SR, 16) },
    { HRDATA (WRU, sim_int_char, 16) },
    { NULL }
};

MTAB cpu_mod[] = {
    { 0 }
};

DEVICE cpu_dev = {
	"CPU", 			/* name */
	&cpu_unit, 		/* units */
	cpu_reg, 		/* registers */
	cpu_mod,		/* modifiers */
	1, 			    /* #units */
	16, 			/* address radix */
	16, 			/* address width */
	1, 			    /* addr increment */
	16, 			/* data radix */
	8,			    /* data width */
   	&cpu_ex, 		/* examine routine */
	&cpu_dep, 		/* deposit routine */
	&cpu_reset,		/* reset routine */
   	NULL, 			/* boot routine */
    NULL, 			/* attach routine */
	NULL,			/* detach routine */
   	NULL, 			/* context */
	DEV_DEBUG,		/* flags */
	0,			    /* debug control flags */
   	NULL, 			/* debug flag names */
	NULL, 			/* memory size change */
	NULL, 			/* logical name */
	cpu_help, 		/* help routine */
	NULL,			/* attach help routine */
	NULL, 			/* help context */
	NULL			/* device description */
};



int32 sim_instr (void)
{
    extern int32 sim_interval;
    int32 PC, IR, OP, DAR, reason, hi, lo, carry, i;

    PC = saved_PC & ADDRMASK;                           /* load local PC */
    C = C & 0x10000;
    reason = 0;

    int8 halted = 0;

    /* Real Time Simulation */
    uint32 startTime, now;
    /*  tStates contains the number of t-states executed. One t-state is executed
        in one microsecond on a 1MHz CPU. tStates is used for real-time simulations.    */

    uint32 tStatesInSlice; /* number of t-states in 10 mSec time-slice */

   /* Real Time Simulation */
   startTime = sim_os_msec();
   tStatesInSlice = sliceLength * 2000; /* MS-101 runs at 2 Megahertz */

    /*
        Main instruction fetch/decode loop
    */
    while (reason == 0)
    {

        /*
            check clock queue
        */
        if (sim_interval <= 0)
	    {
            if (reason = sim_process_event ())
		        break;
        }

        /*
            MS-101 Keyboard Interrupt
        */
        if ((int_req == 7) && (INTE))
        {
            #ifdef MS101
            SP--;
            M[SP] = (PC >> 8) & 0xff;
            SP--;
            M[SP] = PC & 0xff;
            PC = 0x38;
            #endif
            halted = 0;
            int_req = 0;
            continue;
        }

    	/*
            Hit breakpoint? stop simulation
        */
    	if (sim_brk_summ && sim_brk_test (PC, SWMASK ('E')))
        {
            reason = STOP_IBKPT;
            break;
        }

        /*
            Real Time Simulation
        */
        if (tStates >= tStatesInSlice)
        {
            startTime += sliceLength;
            tStates -= tStatesInSlice;
            if (startTime > (now = sim_os_msec()))
            {
                usleep(1000 * (startTime - now));
            }
        }

        sim_interval--;

        /*
            MS-101 Architectur halt the Micro & wait for hardware Interrupts
        */
        if (halted)
            continue;

        PCX = PC;
        IR = OP = M[PC];            /* fetch instruction */
        PC = (PC + 1) & ADDRMASK;   /* increment PC */

        if (OP == 0x76) /* HLT - Halt */
		{
		    tStates += 7;
			#ifdef MS101
			if (PCX == 0x0072)  /* MS-101 Keyboard Input Patch */
			{
				sim_printf ("HALT 0x0072\n");
				halted = 1;
				continue;
			}
			if (PCX == 0x0535)  // ms101
			{
				sim_printf ("HALT 0x0535\n");
				halted = 1;
				continue;
			}
			else if ((PCX == 0x128A) || (PCX == 0x10A0)) /* ms101 SKIP SECTOR DATA READ */
			{
				continue;
			}
			#endif
			else /* Ignore HALT Instructions */
			{
				continue;
			}
        }

       /* Handle below all operations which refer to registers or
          register pairs.  After that, a large switch statement
          takes care of all other opcodes */

        if ((OP & 0xC0) == 0x40) /* MOV r1,r2 - Move register to register */
        {
            DAR = getreg(OP & 0x07);
            putreg((OP >> 3) & 0x07, DAR);
            tStates += 5;
            continue;
        }
        if ((OP & 0xC7) == 0x06) /* MVI - Move immediate register*/
        {
            putreg((OP >> 3) & 0x07, M[PC]);
            PC++;
            tStates += 7;
            continue;
        }
        if ((OP & 0xCF) == 0x01) /* LXI - Load immediate register Pair */
        {
            DAR = M[PC] & 0x00ff;
            PC++;
            DAR = DAR | (M[PC] <<8) & 0xFF00;;
            putpair((OP >> 4) & 0x03, DAR);
            PC++;
            tStates += 10;
            continue;
        }
        if ((OP & 0xEF) == 0x0A) /* LDAX (B|D) - Load A indirect */
        {
            DAR = getpair((OP >> 4) & 0x03);
            putreg(7, M[DAR]);
            tStates += 7;
            continue;
        }
        if ((OP & 0xEF) == 0x02) /* STAX (B|D) - Store A indirect */
        {
            DAR = getpair((OP >> 4) & 0x03);
            M[DAR] = getreg(7);
            tStates += 7;
            continue;
        }

        if ((OP & 0xF8) == 0xB8) /* CMP - Compare register with A */
        {
            DAR = A & 0xFF;
            DAR -= getreg(OP & 0x07);
            setarith(DAR);
            tStates += 4;
            continue;
        }
        if ((OP & 0xC7) == 0xC2) /* JMP <condition> */
        {
            if (cond((OP >> 3) & 0x07) == 1) {
                lo = M[PC];
                PC++;
                hi = M[PC];
                PC++;
                PC = (hi << 8) + lo;
            } else {
                PC += 2;
            }
            tStates += 10;
            continue;
        }
        if ((OP & 0xC7) == 0xC4) /* CALL <condition> */
        {
            if (cond((OP >> 3) & 0x07) == 1) {
                lo = M[PC];
                PC++;
                hi = M[PC];
                PC++;
                SP--;
                M[SP] = (PC >> 8) & 0xff;
                SP--;
                M[SP] = PC & 0xff;
                PC = (hi << 8) + lo;
                tStates += 17;
            } else
            {
                PC += 2;
                tStates += 11;
            }
            continue;
        }
        if ((OP & 0xC7) == 0xC0) /* RET <condition> */
        {
            if (cond((OP >> 3) & 0x07) == 1) {
                PC = M[SP];
                SP++;
                PC |= (M[SP] << 8) & 0xff00;
                SP++;
                tStates += 11;
            }
            else
            {
                tStates += 5;
            }
            continue;
        }
        if ((OP & 0xC7) == 0xC7) /* RST - Restart */
        {
            SP--;
            M[SP] = (PC >> 8) & 0xff;
            SP--;
            M[SP] = PC & 0xff;
            PC = OP & 0x38;
            tStates += 11;
            continue;
        }

        if ((OP & 0xCF) == 0xC5) /* PUSH (B | D | H | PSW) - Push register Pair (B & C | D & E | H & L | A and Flags) on stack */
        {
            DAR = getpush((OP >> 4) & 0x03);
            SP--;
            M[SP] = (DAR >> 8) & 0xff;
            SP--;
            M[SP] = DAR & 0xff;
            tStates += 11;
            continue;
        }
        if ((OP & 0xCF) == 0xC1) /* POP (B | D | H | PSW) - Pop register Pair (B & C | D & E | H & L | A and Flags) off stack */
        {
            DAR = M[SP];
            SP++;
            DAR |= M[SP] << 8;
            SP++;
            putpush((OP >> 4) & 0x03, DAR);
            tStates += 10;
            continue;
        }
        if ((OP & 0xF8) == 0x80) /* ADD r - Add register to A */
        {
            A += getreg(OP & 0x07);
            setarith(A);
            A = A & 0xFF;
            tStates += 4;
            continue;
        }
        if ((OP & 0xF8) == 0x88) /* ADC r - Add register to A with carry */
        {
            carry = 0;
            if (C) carry = 1;
            A += getreg(OP & 0x07);
            A += carry;
            setarith(A);
            A = A & 0xFF;
            tStates += 4;
            continue;
        }
        if ((OP & 0xF8) == 0x90) /* SUB r - Subtract register from A */
        {
            A -= getreg(OP & 0x07);
            setarith(A);
            A = A & 0xFF;
            tStates += 4;
            continue;
        }
        if ((OP & 0xF8) == 0x98) /* SBB r - Subtract register from A with borrow */
        {
            carry = 0;
            if (C) carry = 1;
            A -= (getreg(OP & 0x07)) + carry ;
            setarith(A);
            A = A & 0xFF;
            tStates += 4;
            continue;
        }
        if ((OP & 0xC7) == 0x04) /* INR r - Increment register */
        {
            DAR = getreg((OP >> 3) & 0x07);
            DAR++;
            setinc(DAR);
            DAR = DAR & 0xFF;
            putreg((OP >> 3) & 0x07, DAR);
            tStates += 5;
            continue;
        }
        if ((OP & 0xC7) == 0x05) /* DCR r - Decrement register */
        {
            DAR = getreg((OP >> 3) & 0x07);
            DAR--;
            setinc(DAR);
            DAR = DAR & 0xFF;
            putreg((OP >> 3) & 0x07, DAR);
            tStates += 5;
            continue;
        }
        if ((OP & 0xCF) == 0x03) /* INX - Increment register pair */
        {
            DAR = getpair((OP >> 4) & 0x03);
            DAR++;
            DAR = DAR & 0xFFFF;
            putpair((OP >> 4) & 0x03, DAR);
            tStates += 5;
            continue;
        }
        if ((OP & 0xCF) == 0x0B) /* DCX - Decrement register pair */
        {
            DAR = getpair((OP >> 4) & 0x03);
            DAR--;
            DAR = DAR & 0xFFFF;
            putpair((OP >> 4) & 0x03, DAR);
            tStates += 5;
            continue;
        }
        if ((OP & 0xCF) == 0x09) /* DAD - Add register pair to H & L*/
        {
            HL += getpair((OP >> 4) & 0x03);
            C = 0;
            if (HL & 0x10000)
                C = 0x10000;
            HL = HL & 0xFFFF;
            tStates += 10;
            continue;
        }
        if ((OP & 0xF8) == 0xA0) /* ANA r - And register with A */
        {
            A &= getreg(OP & 0x07);
            C = 0;
            setlogical(A);
            A &= 0xFF;
            tStates += 4;
            continue;
        }
        if ((OP & 0xF8) == 0xA8) /* XRA M - Exclusive Or memory with A */
        {
            A ^= getreg(OP & 0x07);
            C = 0;
            setlogical(A);
            A &= 0xFF;
            tStates += 7;
            continue;
        }
        if ((OP & 0xF8) == 0xB0) /* ORA M - Or memory with A */
        {
            A |= getreg(OP & 0x07);
            C = 0;
            setlogical(A);
            A &= 0xFF;
            tStates += 7;
            continue;
        }



        /* The Big Instruction Decode Switch */

        switch (IR) {

        /* Logical instructions */

        case 0xFE: /* CPI - Compare immediate with A */
        {
            DAR = A & 0xFF;
            DAR -= M[PC];
            PC++;
            setarith(DAR);
            tStates += 7;
            break;
        }
        case 0xE6: /* ANI Add immediate with A */
        {
            A &= M[PC];
            PC++;
            C = AC = 0;
            setlogical(A);
            A &= 0xFF;
            tStates += 7;
            break;
        }
        case 0xEE: /* XRI Exclusive Or immediate with A */
        {
            A ^= M[PC];
            PC++;
            C = AC = 0;
            setlogical(A);
            A &= 0xFF;
            tStates += 7;
            break;
        }
        case 0xF6: /* ORI Or immediate with A */
        {
            A |= M[PC];
            PC++;
            C = AC = 0;
            setlogical(A);
            A &= 0xFF;
            tStates += 7;
            break;
        }

        /* Jump instructions */

        case 0xC3: /* JMP jump unconditional */
        {
            lo = M[PC];
            PC++;
            hi = M[PC];
            PC++;
            PC = (hi << 8) + lo;
            tStates += 10;
            break;
        }
        case 0xE9: /* PCHL Program counter to H & L */
        {
            PC = HL;
            tStates += 5;
            break;
        }
        case 0xCD: /* CALL Call unconditional */
        {
            lo = M[PC];
            PC++;
            hi = M[PC];
            PC++;
            SP--;
            M[SP] = (PC >> 8) & 0xff;
            SP--;
            M[SP] = PC & 0xff;
            PC = (hi << 8) + lo;
            tStates += 17;
            break;
        }
        case 0xC9: /* RET Return */
        {
            PC = M[SP];
            SP++;
            PC |= (M[SP] << 8) & 0xff00;
            SP++;
            tStates += 10;
            break;
        }

        /* Data Transfer Group */

        case 0x32: /* STA Store A direct */
        {
            lo = M[PC];
            PC++;
            hi = M[PC];
            PC++;
            DAR = (hi << 8) + lo;
            M[DAR] = A;
            tStates += 13;
            break;
        }
        case 0x3A: /* LDA Load A direct */
        {
            lo = M[PC];
            PC++;
            hi = M[PC];
            PC++;
            DAR = (hi << 8) + lo;
            A = M[DAR];
            tStates += 13;
            break;
        }
        case 0x22: /* SHLD Store H & L direct */
        {
            lo = M[PC];
            PC++;
            hi = M[PC];
            PC++;
            DAR = (hi << 8) + lo;
            M[DAR] = HL;
            DAR++;
            M[DAR] = (HL >>8) & 0x00ff;
            tStates += 16;
            break;
        }
        case 0x2A: /* LHLD Load H & L direct */
        {
            lo = M[PC];
            PC++;
            hi = M[PC];
            PC++;
            DAR = (hi << 8) + lo;
            HL = M[DAR];
            DAR++;
            HL = HL | (M[DAR] <<8);
            tStates += 16;
            break;
        }
        case 0xEB: /* XCHG Exchange D & E, H & L Registers */
        {
            DAR = HL;
            HL = DE;
            DE = DAR;
            tStates += 4;
            break;
        }

        /* Arithmetic Group */

        case 0xC6: /* ADI Add immedite to A */
        {
            A += M[PC];
            PC++;
            setarith(A);
            A = A & 0xFF;
            tStates += 7;
            break;
        }
        case 0xCE: /* ACI Add immedite to A with carry */
        {
            carry = 0;
            if (C) carry = 1;
            A += M[PC];
            A += carry;
            PC++;
            setarith(A);
            A = A & 0xFF;
            tStates += 7;
            break;
        }
        case 0xD6: /* SUI Subtract immediate from A */
        {
            A -= M[PC];
            PC++;
            setarith(A);
            A = A & 0xFF;
            tStates += 7;
            break;
        }
        case 0xDE: /* SBI Subtract immediate from A with borrow */
        {
            carry = 0;
            if (C) carry = 1;
            A -= (M[PC] + carry);
            PC++;
            setarith(A);
            A = A & 0xFF;
            tStates += 7;
            break;
        }
        case 0x27: /* DAA Decimal adjust A*/
        {
            DAR = A & 0x0F;
            if (DAR > 9 || AC > 0) {
                DAR += 6;
                A &= 0xF0;
                A |= DAR & 0x0F;
                if (DAR & 0x10)
                    AC = 0x10000;
                   else
                    AC = 0;
            }
            DAR = (A >> 4) & 0x0F;
            if (DAR > 9 || AC > 0) {
                DAR += 6;
                if (AC) DAR++;
                A &= 0x0F;
                A |= (DAR << 4);
            }
            if ((DAR << 4) & 0x100)
                C = 0x10000;
               else
                C = 0;
            if (A & 0x80) {
                S = 0x10000;
            } else {
                S = 0;
            }
            if ((A & 0xff) == 0)
                Z = 0x10000;
              else
                Z = 0;
            parity(A);
            A = A & 0xFF;
            tStates += 4;
            break;
        }
        case 0x07: /* RLC Rotate A left */
        {
            C = 0;
            C = (A << 9) & 0x10000;
            A = (A << 1) & 0xFF;
            if (C)
                A |= 0x01;
            tStates += 4;
            break;
        }
        case 0x0F: /* RRC Rotate A right */
        {
            C = 0;
            if ((A & 0x01) == 1)
                C |= 0x10000;
            A = (A >> 1) & 0xFF;
            if (C)
                A |= 0x80;
            tStates += 4;
            break;
        }
        case 0x17: /* RAL Rotate A left through carry */
        {
            DAR = C;
            C = 0;
            C = (A << 9) & 0x10000;
            A = (A << 1) & 0xFF;
            if (DAR)
                A |= 1;
            else
                A &= 0xFE;
            tStates += 4;
            break;
        }
        case 0x1F: /* RAL Rotate A right through carry */
        {
            DAR = C;
            C = 0;
            if ((A & 0x01) == 1)
                C |= 0x10000;
            A = (A >> 1) & 0xFF;
            if (DAR)
                A |= 0x80;
            else
                A &= 0x7F;
            tStates += 4;
            break;
        }
        case 0x2F: /* CMA Complement A */
        {
            A = ~ A;
            A &= 0xFF;
            tStates += 4;
            break;
        }
        case 0x3F: /* CMC Complement carry */
        {
            C = ~ C;
            C &= 0x10000;
            tStates += 4;
            break;
        }
        case 0x37: /* STC Set carry */
        {
            C = 0x10000;
            tStates += 4;
            break;
        }

        /* Stack, I/O & Machine Control Group */

        case 0x00: /* NOP No-operation*/
        {
            tStates += 4;
            break;
        }
        case 0xE3: /* XTHL Exchange top of stack, H & L */
        {
            lo = M[SP];
            hi = M[SP + 1];
            M[SP] = HL & 0xFF;
            M[SP + 1] = (HL >> 8) & 0xFF;
            HL = (hi << 8) + lo;
            tStates += 18;
            break;
        }
        case 0xF9: /* SPHL H & L to stack pointer */
        {
            SP = HL;
            tStates += 5;
            break;
        }

        case 0xFB: /* EI Enable interrupts */
        {
            INTE = 0x10000;
            tStates += 4;
            break;
        }
        case 0xF3: /* DI Disable interrupts */
        {
            INTE = 0;
            tStates += 4;
            break;
        }
        case 0xDB: /* IN Input */
        {
            DAR = M[PC] & 0xFF;
            PC++;
            if (DAR == 0xFF) {
                A = (SR >> 8) & 0xFF;
            } else {
                A = dev_table[DAR].routine(0, 0);
            }
            tStates += 10;
            break;
        }
        case 0xD3: /* OUT Output */
        {
            DAR = M[PC] & 0xFF;
            PC++;
            dev_table[DAR].routine(1, A);
            tStates += 10;
            break;
        }

        default:
        {
            if (cpu_unit.flags & UNIT_OPSTOP)
            {
                reason = STOP_OPCODE;
                PC--;
            }
            break;
        }

    }
}

    /* Simulation halted */
    saved_PC = PC;
    return reason;
}

/* Test an 8080 flag condition and return 1 if true, 0 if false */
int32 cond(int32 con)
{
    switch (con) {
        case 0:
            if (Z == 0) return (1);
            break;
        case 1:
            if (Z != 0) return (1);
            break;
        case 2:
            if (C == 0) return (1);
            break;
        case 3:
            if (C != 0) return (1);
            break;
        case 4:
            if (P == 0) return (1);
            break;
        case 5:
            if (P != 0) return (1);
            break;
        case 6:
            if (S == 0) return (1);
            break;
        case 7:
            if (S != 0) return (1);
            break;
        default:
            break;
    }
    return (0);
}

/* Set the <C>arry, <S>ign, <Z>ero and <P>arity flags following
   an arithmetic operation on 'reg'.
*/

void setarith(int32 reg)
{
    int32 bc = 0;

    if (reg & 0x100)
        C = 0x10000;
      else
        C = 0;
    if (reg & 0x80) {
        bc++;
        S = 0x10000;
    } else {
        S = 0;
    }
    if ((reg & 0xff) == 0)
        Z = 0x10000;
      else
        Z = 0;
    AC = 0;

        parity(reg);

}

/* Set the <C>arry, <S>ign, <Z>ero amd <P>arity flags following
   a logical (bitwise) operation on 'reg'.
*/

void setlogical(int32 reg)
{
    C = 0;
    if (reg & 0x80) {
        S = 0x10000;
    } else {
        S = 0;
    }
    if ((reg & 0xff) == 0)
        Z = 0x10000;
      else
        Z = 0;
    AC = 0;
    parity(reg);
}

/* Set the Parity (P) flag based on parity of 'reg', i.e., number
   of bits on even: P=0x10000, else P=0
*/

void parity(int32 reg)
{
    int32 bc = 0;

    if (reg & 0x01) bc++;
    if (reg & 0x02) bc++;
    if (reg & 0x04) bc++;
    if (reg & 0x08) bc++;
    if (reg & 0x10) bc++;
    if (reg & 0x20) bc++;
    if (reg & 0x40) bc++;
    if (reg & 0x80) bc++;
    P = ~(bc << 16);
    P &= 0x10000;
}

/* Set the <S>ign, <Z>ero amd <P>arity flags following
   an INR/DCR operation on 'reg'.
*/

void setinc(int32 reg)
{
    int32 bc = 0;

    if (reg & 0x80) {
        bc++;
        S = 0x10000;
    } else {
        S = 0;
    }
    if ((reg & 0xff) == 0)
        Z = 0x10000;
      else
        Z = 0;
        parity(reg);
}

/* Get an 8080 register and return it */
int32 getreg(int32 reg)
{
    switch (reg) {
        case 0:
            return ((BC >>8) & 0x00ff);
        case 1:
            return (BC & 0x00FF);
        case 2:
            return ((DE >>8) & 0x00ff);
        case 3:
            return (DE & 0x00ff);
        case 4:
            return ((HL >>8) & 0x00ff);
        case 5:
            return (HL & 0x00ff);
        case 6:
            return (M[HL]);
        case 7:
            return (A);
        default:
            break;
    }
    return 0;
}

/* Put a value into an 8080 register from memory */
void putreg(int32 reg, int32 val)
{
    switch (reg) {
        case 0:
            BC = BC & 0x00FF;
            BC = BC | (val <<8);
            break;
        case 1:
            BC = BC & 0xFF00;
            BC = BC | val;
            break;
        case 2:
            DE = DE & 0x00FF;
            DE = DE | (val <<8);
            break;
        case 3:
            DE = DE & 0xFF00;
            DE = DE | val;
            break;
        case 4:
            HL = HL & 0x00FF;
            HL = HL | (val <<8);
            break;
        case 5:
            HL = HL & 0xFF00;
            HL = HL | val;
            break;
        case 6:
            M[HL] = val & 0xff;
            break;
        case 7:
            A = val & 0xff;
        default:
            break;
    }
}

/* Return the value of a selected register pair */
int32 getpair(int32 reg)
{
    switch (reg) {
        case 0:
            return (BC);
        case 1:
            return (DE);
        case 2:
            return (HL);
        case 3:
            return (SP);
        default:
            break;
    }
    return 0;
}

/* Return the value of a selected register pair, in PUSH
   format where 3 means A& flags, not SP */
int32 getpush(int32 reg)
{
    int32 stat;

    switch (reg) {
        case 0:
            return (BC);
        case 1:
            return (DE);
        case 2:
            return (HL);
        case 3:
            stat = A << 8;
            if (S) stat |= 0x80;
            if (Z) stat |= 0x40;
            if (AC) stat |= 0x10;
            if (P) stat |= 0x04;
            stat |= 0x02;
            if (C) stat |= 0x01;
            return (stat);
        default:
            break;
    }
    return 0;
}


/* Place data into the indicated register pair, in PUSH
   format where 3 means A& flags, not SP */
void putpush(int32 reg, int32 data)
{
    switch (reg) {
        case 0:
            BC = data;
            break;
        case 1:
            DE = data;
            break;
        case 2:
            HL = data;
            break;
        case 3:
            A = (data >> 8) & 0xff;
            S = Z = AC = P = C = 0;
            if (data & 0x80) S = 0x10000;
            if (data & 0x40) Z = 0x10000;
            if (data & 0x10) AC = 0x10000;
            if (data & 0x04) P = 0x10000;
            if (data & 0x01) C = 0x10000;
            break;
        default:
            break;
    }
}


/* Put a value into an 8080 register pair */
void putpair(int32 reg, int32 val)
{
    switch (reg) {
        case 0:
            BC = val;
            break;
        case 1:
            DE = val;
            break;
        case 2:
            HL = val;
            break;
        case 3:
            SP = val;
            break;
        default:
            break;
    }
}


/* Reset routine */

t_stat cpu_reset (DEVICE *dptr)
{
	C = 0;
	Z = 0;
	saved_PC = 0;
	int_req = 0;
	sim_brk_types = sim_brk_dflt = SWMASK ('E');
	return (SCPE_OK);
}

/* Memory examine */

t_stat cpu_ex (t_value *vptr, t_addr addr, UNIT *uptr, int32 sw)
{
	if (addr >= MEMSIZE) return SCPE_NXM;
	if (vptr != NULL) *vptr = M[addr] & 0xFF;
	return (SCPE_OK);
}

/* Memory deposit */

t_stat cpu_dep (t_value val, t_addr addr, UNIT *uptr, int32 sw)
{
	if (addr >= MEMSIZE)
		return SCPE_NXM;
    	M[addr] = val & 0xFF;
	return (SCPE_OK);
}


int32 nulldev(int32 flag, int32 data)
{
    if (flag == 0)
        return (0xFF);
    return 0;
}

/* This is the binary loader.  The input file is considered to be
   a string of literal bytes with no format special format. The
   load starts at the current value of the PC.
*/


void sim_special_init (void)
{
	load_ROMS();
}

void (*sim_vm_init)(void) = &sim_special_init;


int32 load_ROMS ()
{
	#ifdef MS101
	load_ROM ("rom1.bin", 0);
	load_ROM ("rom2.bin", 0x0400);

	load_ROM ("rom5.bin", 0x1000);
	load_ROM ("rom6.bin", 0x1400);
	#endif
	return (SCPE_OK);
}

int32 load_ROM (char * filename, int32 offset)
{
	FILE * fpROM;
	int	i;
	unsigned char data;

	fpROM = fopen (filename, "r");
	if (!fpROM)
	{
		sim_printf ("%s NOT FOUND\n", filename);
		return (SCPE_OPENERR);;
	}
	for (i = 0; i < 1024; i++)
	{
		fread (&data, 1, 1, fpROM);
		M[offset + i] = data;
	}
	sim_printf ("%d Bytes from %s loaded at %04X\n", i, filename, offset);
	return (SCPE_OK);
}


t_stat cpu_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr)
{
	return (SCPE_OK);
}


