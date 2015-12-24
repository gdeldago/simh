/* ms101_kbd.c: Micro Sistemas MS101 Keyboard simulator

   Copyright (c) 2014-2015 Gustavo del Dago

*/
#include <stdio.h>
#include <ctype.h>
#include "ms101_defs.h"

/* para Beep */
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <linux/kd.h>
#include <unistd.h>
/* para Beep */

void buzzer_on (void);
void buzzer_off (void);

t_stat kbd_svc (UNIT *uptr);
t_stat kbd_reset (DEVICE *dptr);
t_stat kbd_boot (int32 unit_num, DEVICE *dptr);
t_stat kbd_detach (UNIT *uptr);
const char *kbd_description (DEVICE *dptr);
extern int32 PCX;
extern int32 int_req;


/*
    Intel 8279 Programmable Keyboard/Display Interface
*/
#define I8279_FIFO_FULL         0x04
#define I8279_ERROR_UNDERRUN    0x10
#define I8279_ERROR_OVERRUN     0x20

unsigned char keybuf[8];
uint8 status;
uint8 keybuf_len;

int fd_buzzer = 0;

UNIT kbd_unit = { UDATA (&kbd_svc, UNIT_ATTABLE, 0), KBD_POLL_WAIT };

REG kbd_reg[] = {
    { BRDATA (BUFFER, keybuf, 16, 8, 8) },
    { HRDATA (STATUS, status, 8) },
    { NULL }
    };


DEVICE kbd_dev = {
    "KBD",              /* name */
    &kbd_unit,          /* units */
    kbd_reg,            /* registers */
    NULL,               /* modifiers */
    1,                  /* #units */
    16,                 /* address radix */
    8,                  /* address width */
    1,                  /* addr increment */
    16,                 /* data radix */
    8,                  /* data width */
    NULL,               /* examine routine */
    NULL,               /* deposit routine */
    &kbd_reset,         /* reset routine */
    NULL,               /* boot routine */
    NULL,               /* attach routine */
    NULL,               /* detach routine */
    NULL,               /* context */
    DEV_DEBUG,          /* flags */
    0,                  /* debug control flags */
    NULL,               /* debug flag names */
    NULL,               /* memory size change */
    NULL,               /* logical name */
    NULL,               /* help routine */
    NULL,               /* attach help routine */
    NULL,               /* help context */
    &kbd_description    /* device description */
    };


/*  Service routines to handle simlulator functions */

/* service routine - actually gets char & places in buffer */

t_stat kbd_svc (UNIT *uptr)
{
/*
    MS-101 Key conversion table [0x0480,0x04FF], 7 bits to 8 bits
*/
const uint8 keyboard[] = {
    0x46, 0x45, 0x44, 0x43, 0x42, 0x41, 0x23, 0xB2,
    0xFF, 0xB1, 0xBD, 0x41, 0xFF, 0xB3, 0xFF, 0xB4,
    0xFF, 0xFF, 0x2D, 0xBA, 0xFF, 0x51, 0x2E, 0x2C,
    0x4D, 0x4C, 0x4B, 0x4A, 0x4F, 0x49, 0x55, 0x2F,
    0x20, 0xB9, 0xFF, 0xBE, 0x4E, 0x56, 0x58, 0x5A,
    0x48, 0x47, 0x53, 0x50, 0x59, 0x54, 0x52, 0x57,
    0xFF, 0x40, 0xBC, 0xBB, 0xFF, 0xB8, 0xB7, 0xB6,
    0x3C, 0x2A, 0x25, 0xFF, 0xB5, 0xFF, 0xB0, 0xFF,
    0x3B, 0x29, 0x3A, 0x22, 0x21, 0xFF, 0x23, 0xB2,
    0xFF, 0xB1, 0xBD, 0xFF, 0xFF, 0xB3, 0xFF, 0xB4,
    0xFF, 0xFF, 0x20, 0xBA, 0xFF, 0x2B, 0x39, 0x38,
    0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x30,
    0x20, 0xB9, 0xFF, 0xBE, 0x28, 0x3D, 0x3F, 0xFF,
    0x27, 0x5E, 0x3E, 0x26, 0x20, 0x5C, 0x20, 0x20,
    0xFF, 0x23, 0xBC, 0xBB, 0xFF, 0xB8, 0xB7, 0xB6,
    0x2E, 0x24, 0x2C, 0xFF, 0xB5, 0xFF, 0xB0, 0xFF
    };

/* inverse of the previous table, 8 bits to 7 bits */
static int8 inv_keyboard[UINT8_MAX] = {'\0'};

int i;
int32 temp;

/* Inverse keyboard table */
if (inv_keyboard[0xFF] == 0) // once
    for (i = 0; i < 127; i++) {
        inv_keyboard[keyboard[i]] = (int8)i;
        }

//SIM_KEY_EVENT ev;
sim_activate (&kbd_unit, kbd_unit.wait);          /* continue poll */

temp = 0;
if ((temp = sim_poll_kbd ()) < SCPE_KFLAG) {
//        if (vid_poll_kb (&ev) != SCPE_OK)
//               return temp;                                  /* no char or error? */
//        else
//            if (ev.state == SIM_KEYPRESS_UP)
//                temp = '8';//ev.key;
//            else
            return temp;
    }
else {
    temp = temp - SCPE_KFLAG;                     /* Save char */
    }
if (keybuf_len < 7)
    status |= keybuf_len++;                       /* Set status */
else {
    status |= I8279_ERROR_OVERRUN;
    return 0;
    }

/* Do any special character handling here */

if (toupper(temp) == 127)   /* [BackSpace] Key */
    temp = 128 - 14;
else if (temp == 27)      /* [Escape] key*/
    temp = 128 - 10;      /* MS101_KBD_RESET */
else
    temp = 127 - inv_keyboard[toupper(temp)];

keybuf[keybuf_len-1] = temp;
status = keybuf_len;
int_req = 7;

return (SCPE_OK);
}

/* Reset routine */
t_stat kbd_reset (DEVICE *dptr)
{
status = 0;                                 /* Clear Status */
keybuf_len = 0;                             /* Reset buffer lenght */

if (fd_buzzer == 0) {
    fd_buzzer = open("/dev/console", O_WRONLY);
    if (fd_buzzer == -1)
        sim_printf ("Beep sounds are only for root\n");
    }
sim_printf ("Keyboard initialized\n");
sim_activate (&kbd_unit, kbd_unit.wait);    /* activate unit */

return (SCPE_OK);
}

/* Detach routine */
t_stat kbd_detach (UNIT *uptr)
{
sim_printf ("kbd_detach");
if (!fd_buzzer)
    close (fd_buzzer);
return (SCPE_OK);
}

/*
    INTEL 8279
*/
int32 port30(int32 io, int32 data)
{
if (io == 0)
    sim_printf ("[%04X] Read port 0x30\n", PCX);
else {
    sim_printf ("[%04X] Write port 0x30 %02X (Reset Keyboard)\n", PCX, data);
    //keybuf_len = 0;
    //status = 0;
    }
return (0);
}

/*
    Keyboard Data Port (Read)
*/
int32 port38(int32 io, int32 data)
{
if (io == 0) {
    if (keybuf_len > 0) {
        keybuf_len--;
        status = keybuf_len;
        sim_printf ("\n[%04X] Keyboard data %d (%02X) \n", PCX, keybuf[keybuf_len], keybuf[keybuf_len]);
        return (keybuf[keybuf_len]);
        }
    }
else {
    sim_printf ("[%04X] Write port 0x38 %02X\n", PCX, data);
    }
return (0);
}

/*
    Keyboard Status Port (Read/write)
*/
int32 port39(int32 io, int32 data)
{
int32 temp;

if (io == 0) {
    //sim_printf ("\n[%04X] Read port 0x39 (%02X)\n", PCX, status);
    return (status);
    }
else {
    //sim_printf ("[%04X] Write port 0x39 %02X (Config i8279)\n", PCX, data);

    switch (data >> 5) {
        case 0:     /* Keyboard/Display Mode Set */
            sim_printf ("Keyboard/Display Mode Set DD = %d KKK = %d\n", ((data >> 3) & 0x03), (data & 0x07));
            break;
        case 1:     /* Program Clock */
            sim_printf ("Program Clock PPPPP = %d\n", (data & 31));
            break;
        case 2:     /* Read FIFO/Sensor RAM */
            sim_printf ("Read FIFO/Sensor RAM\n");
            break;
        case 3:     /* Read Display RAM */
            sim_printf ("Read Display RAM\n");
            break;
        case 4:     /* Write Display RAM */
            sim_printf ("Write Display RAM\n");
            break;
        case 5:     /* Display Write Inhibit/Blanking */
            sim_printf ("Display Write Inhibit/Blanking\n");
            break;
        case 6:      /* Clear */
            //sim_printf ("[%04X] Clear %02X\n\n", PCX, data);
            if ((data & 0x01) || (data & 0x02)) {
                sim_printf ("Clear Keyboard FIFO\n");
                status = 0x00;                                 /* Status */
                keybuf_len = 0;
                }
            else if (data == 0xD0) {
                buzzer_off();
                //sim_printf ("sound off\n");
                }
            else if (data == 0xDC) {
                buzzer_on();
                //sim_printf ("sound on\n");
                }
            break;
        case 7:     /* End Interrupt/Error Mode Set */
            sim_printf ("End Interrupt/Error Mode Set\n");
            break;
        }
}
return (0);
}

const char *kbd_description (DEVICE *dptr)
{
return ("MS101 Keyboard");
}

/*
    Atenti que el PC Speaker viene desabilitado.
    sudo modprobe -v pcspkr
*/
void buzzer_on (void)
{
int freq = 750;
ioctl(fd_buzzer, KIOCSOUND, (int)(1193180/freq));
return;
}

void buzzer_off (void)
{
ioctl(fd_buzzer, KIOCSOUND, 0);
return;
}



