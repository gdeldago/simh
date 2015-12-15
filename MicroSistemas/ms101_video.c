/* ms101_video.c: Micro Sistemas MS101 Video Board Simulator

   Copyright (c) 2014-2015, Gustavo del Dago

*/

#include "ms101_defs.h"



/* include video library header */
#include "sim_video.h"


#define VIDEO_XSIZE      64 * 8                         /* screen size */
#define VIDEO_YSIZE      16 * 16
#define VIDEO_MEMSIZE    VIDEO_XSIZE * VIDEO_YSIZE

#define SCREEN_BORDER    8
#define SCREEN_WIDTH     64 * 8 + SCREEN_BORDER * 2
#define SCREEN_HEIGHT    16 * 16 + SCREEN_BORDER * 2
#define SCREEN_BPP       32

/* Constants of Video Library */
#define VC_XSIZE      64 * 8
#define VC_YSIZE      16 * 16
#define VC_MEMSIZE    (1u << 16)                        /* video memory size */

/* get the colors black and white (see section for details) */
unsigned long black,white;

extern unsigned char M[];

DEVICE video_dev;

t_stat video_reset (DEVICE *dptr);
const char *video_description (DEVICE *dptr);

t_stat video_svc (UNIT *uptr);

t_stat video_detach (UNIT *uptr);
t_stat video_set_enable (UNIT *uptr, int32 val, char *cptr, void *desc);
void video_setint (int32 src);
int32 video_inta (void);
void video_clrint (int32 src);
void video_uart_int (uint32 set);
t_stat video_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr);

t_stat video_set_enable (UNIT *uptr, int32 val, char *cptr, void *desc);
static t_stat set_zoom (UNIT *uptr, int32 value, char *cptr, void *desc);
static t_stat disp_zoom (FILE *st, UNIT *uptr, int value, void *desc);

uint32 *video_buf = NULL;                               /* Video memory */

static int8 port_0 = 0;
static uint8 scale = 2;
#define DPY_WAIT 50000                                  /* 100us */

static int initialized = 0;

static int8 zoom = 1;

/* Variables para la implementacion Video Library */
uint32 *vc_map = NULL;                                  /* Scanline map */
uint32 *vc_buf = NULL;                                  /* Video memory */
uint32 *vc_lines = NULL;                                /* Video Display Lines */

uint8 rom_char[16 * 128];


UNIT video_unit = { UDATA (&video_svc, 0, 0), DPY_WAIT };

REG video_reg[] = {
    { NULL }
};

MTAB video_mod[] = {
    //{ MTAB_XTD | MTAB_VDV, 1, NULL, "ENABLE",  &video_set_enable, NULL, NULL, "Enable VIDEO" },
    //{ MTAB_XTD | MTAB_VDV, 0, NULL, "DISABLE", &video_set_enable, NULL, NULL, "Disable VIDEO" },
    /*
        El Zoom funciona en modalidad x11 hay que laburar para que funque en Video Library
    */
    //{ MTAB_XTD | MTAB_VDV | MTAB_VALR , 0, "ZOOM", "ZOOM", &set_zoom, &disp_zoom, NULL, "Set the VIDEO size zoom factor"  },
    { 0 }
    };

DEVICE video_dev = {
    "VIDEO",              /* name */
    &video_unit,          /* units */
    video_reg,            /* registers */
    video_mod,            /* modifiers */
    1,                    /* #units */
    16,                   /* address radix */
    8,                    /* address width */
    1,                    /* addr increment */
    16,                   /* data radix */
    8,                    /* data width */
    NULL,                 /* examine routine */
    NULL,                 /* deposit routine */
    &video_reset,         /* reset routine */
    NULL,                 /* boot routine */
    NULL,                 /* attach routine */
    &video_detach,        /* detach routine */
    NULL,                 /* context */
    DEV_DISABLE |
    //DEV_DIS |           /* Por ahora que arranque activo */
    DEV_DEBUG |
    DEV_DISPLAY,          /* flags */
    0,                    /* debug control flags */
    NULL,                 /* debug flag names */
    NULL,                 /* memory size change */
    NULL,                 /* logical name */
    &video_help,          /* help routine */
    NULL,                 /* attach help routine */
    NULL,                 /* help context */
    &video_description    /* device description */
};

char *video_regnames[] = {""};

/* Service routines to handle simlulator functions */


/* Service routine - actually gets char & places in buffer */

t_stat video_svc (UNIT *uptr)
{
int row, col;

#ifdef MS101
int p = 0x7C00; // MS101 Base Memory Addr.
#endif
/*
     MEM
ROW  OFFSET
0   DC00
1   DC40
2   DC80
3   DCC0
4   DD00
5   DD40
6   DD80
7   DDC0
8   DE00
9   DE40
10  DE80
11  DEC0
12  DF00
13  DF40
14  DF80
15  DFC0
*/

/* Video Library */
uint32 lines, ln, off;

int mask = 0;
int x = 0;

for (row = 0; row < 16; row++) {
    for (ln = 0; ln < 16; ln++) {
        for (col = 0; col < 64; col++) {
            mask = 0x80;
            for (x = 0; x < 8; x++) {
                off = M[p + col + row * 64] & 0x7F;

                if (rom_char[off * 16 + ln] & mask) {
                    vc_lines[(row * 16 + ln) * VC_XSIZE + col * 8 + x] = vid_mono_palette[1];
                    }
                else {
                    vc_lines[(row * 16 + ln) * VC_XSIZE + col * 8 + x] = vid_mono_palette[0];
                    }
                mask = (mask >> 1);

            }
        }
    }
}
vid_draw (0, 0, VC_XSIZE, VC_YSIZE, vc_lines); /* update screen */
vid_refresh ();

sim_activate (&video_unit, video_unit.wait); /* requeue! */
return (SCPE_OK);
}

/* Reset routine */

t_stat video_reset (DEVICE *dptr)
{
t_stat r;

int depth;

FILE *fp6571 = NULL;

int mask;
int x, y;
unsigned char data;

//sim_printf ("Video Reset\n");

if (dptr->flags & DEV_DIS) {
    free (vc_buf);
    vc_buf = NULL;
    free (vc_lines);
    vc_lines = NULL;
    free (vc_map);
    vc_map = NULL;
    return vid_close ();
}

//if (!initialized)
if (!vid_active) {
/* Video Library */

    /* Load the MCM6571 char pattern */
    fp6571 = fopen ("6571.bin", "r");
    if (!fp6571) {
        sim_printf ("6571.bin NOT FOUND\n");
        return (SCPE_OPENERR);
    }

    int read_glyphs = fread (rom_char, 16, 128, fp6571);
    if (read_glyphs!=128) {
        sim_printf ("6571.bin is %d not %d chars\n", read_glyphs, 128);
        return (SCPE_OPENERR);
    }

    fclose (fp6571);

    vid_open (dptr, VIDEO_XSIZE, VIDEO_YSIZE, 0);
    if (r != SCPE_OK)
        return r;
    vc_buf = (uint32 *) calloc (VC_MEMSIZE, sizeof (uint32));
    if (vc_buf == NULL) {
        vid_close ();
        return (SCPE_MEM);
    }
    vc_lines = (uint32 *) calloc (VC_XSIZE*VC_YSIZE, sizeof (uint32));
    if (vc_lines == NULL) {
        free (vc_buf);
        vid_close ();
        return (SCPE_MEM);
    }
    vc_map = (uint32 *) calloc (VC_XSIZE, sizeof (uint32));
    if (vc_map == NULL) {
        free (vc_lines);
        vc_lines = NULL;
        free (vc_buf);
        vid_close ();
        return (SCPE_MEM);
    }
    initialized = 1;
    sim_printf ("Video initialized (Video Library)\n");

    sim_activate (&video_unit, video_unit.wait); /* activate */
}

return (SCPE_OK);
}

t_stat video_detach (UNIT *uptr)
{
if ((video_dev.flags & DEV_DIS) == 0) {
    video_dev.flags |= DEV_DIS;
    video_reset (&video_dev);
}
return (SCPE_OK);
}

t_stat video_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr)
{
int p = 0x7C00;
int x, y;

printf ("\n");
for (y = 0; y < 16; y++)
{
    for (x = 0; x < 64; x++)
        printf ("%c", M[p++]);
    printf ("\n");
}

return (SCPE_OK);
}

const char *video_description (DEVICE *dptr)
{
return ("MS 101 Video Board");
}

static t_stat set_zoom (UNIT *uptr, int32 value, char *cptr, void *desc)
{
int val;

if (cptr == NULL)
    return SCPE_ARG;

val = atoi(cptr);

if ((val < 1) && (val > 4))
    return SCPE_ARG;

if (val != zoom) {
    zoom = val;

    initialized = 0;
    video_reset(NULL);
}

return (SCPE_OK);
}


static t_stat disp_zoom (FILE *st, UNIT *uptr, int value, void *desc)
{
sim_printf ("zoom %d\n", zoom);

return (SCPE_OK);
}

