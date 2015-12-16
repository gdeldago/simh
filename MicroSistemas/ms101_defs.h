/* ms101_defs.h: Micro Sistemas MS101 simulator definitions

   Copyright (c) 2014-2015, Gustavo del Dago

*/

#include "sim_defs.h"                                   /* simulator defns */
#include "sim_video.h"

/* Memory */

#define MAXMEMSIZE      65536                           /* max memory size */
#define MEMSIZE         (cpu_unit.capac)                /* actual memory size */
#define ADDRMASK        (MAXMEMSIZE - 1)                /* address mask */
#define MEM_ADDR_OK(x)  (((uint32) (x)) < MEMSIZE)
#define MEM_VIDEO       0x7C00                          /* video memory base */

/* Simulator stop codes */

#define STOP_RSRV   1                                   /* must be 1 */
#define STOP_HALT   2                                   /* HALT */
#define STOP_IBKPT  3                                   /* breakpoint */
#define STOP_OPCODE 4
#define WAIT_INT    5

