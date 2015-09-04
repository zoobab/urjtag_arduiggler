/*
 * $Id: usbblaster.c 1863 2010-10-19 21:10:14Z vapier $
 *
 * Arduino JTAG USB Cable Driver
 * Copyright (C) 2006 K. Waschk
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 * Written by Kolja Waschk, 2006; http://www.ixo.de
 *
 */

#include <sysdep.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ftdi.h>

#include <urjtag/cable.h>
#include <urjtag/chain.h>
#include <urjtag/cmd.h>

#include "generic.h"
#include "generic_usbconn.h"
#include "cmd_xfer.h"

#include "usbconn/libftdx.h"


const int BAUD_RATE = 115200;

const uint8_t CMD_RESET  = 0x74;
const uint8_t CMD_STATUS = 0x3F;
const uint8_t CMD_GETVER = 0x61;
const uint8_t CMD_SEND   = 0x73;
const uint8_t CMD_READ   = 0x72;
const uint8_t CMD_FORCE  = 0x66;

const int STATUS_OK   = 0x6F6B;
const int STATUS_ERR1 = 0x6531;
const int STATUS_ERR2 = 0x6532;

/* Cable params_t structure with our data */
typedef struct
{
  urj_tap_cable_cx_cmd_root_t cmd_root;
} params_t;



static int
arduiggler_get_status (urj_cable_t *cable)
{
    int ar_status = 0;
    uint8_t ar_rply;

    ar_rply = urj_tap_cable_cx_xfer_recv (cable);
    ar_status = (ar_rply << 8);
    ar_rply = urj_tap_cable_cx_xfer_recv (cable);
    ar_status |= ar_rply;

    return ar_status;
}

static int
arduiggler_connect (urj_cable_t *cable, const urj_param_t *params[])
{
    params_t *cable_params;

    /* perform urj_tap_cable_generic_usbconn_connect */
    if (urj_tap_cable_generic_usbconn_connect (cable, params) != URJ_STATUS_OK)
        return URJ_STATUS_FAIL;

    cable_params = malloc (sizeof (*cable_params));
    if (!cable_params)
    {
        urj_error_set (URJ_ERROR_OUT_OF_MEMORY, _("malloc(%zd) fails"),
                       sizeof (*cable_params));
        /* NOTE:
         * Call the underlying usbport driver (*free) routine directly
         * not urj_tap_cable_generic_usbconn_free() since it also free's cable->params
         * (which is not established) and cable (which the caller will do)
         */
        cable->link.usb->driver->free (cable->link.usb);
        return URJ_STATUS_FAIL;
    }

    urj_tap_cable_cx_cmd_init (&cable_params->cmd_root);

    /* exchange generic cable parameters with our private parameter set */
    free (cable->params);
    cable->params = cable_params;

    return URJ_STATUS_OK;
}

static int
arduiggler_init (urj_cable_t *cable)
{
    params_t *params = cable->params;
    urj_tap_cable_cx_cmd_root_t *cmd_root = &params->cmd_root;

    if (urj_tap_usbconn_open (cable->link.usb) != URJ_STATUS_OK)
        return URJ_STATUS_FAIL;

    /* need to change the default baud rate from libftdi.c
     * to the actual one used by the cable
     */
    ftdi_param_t *fp = cable->link.usb->params;
    int r = ftdi_set_baudrate(fp->fc, BAUD_RATE);

    if (r != 0) {
        urj_warning (_("cannot change baud rate\n"));
        return URJ_STATUS_FAIL;
    }

    urj_tap_cable_cx_cmd_queue (cmd_root, 0);
    urj_tap_cable_cx_cmd_push (cmd_root, CMD_RESET);
    urj_tap_cable_cx_xfer (cmd_root, NULL, cable, URJ_TAP_CABLE_COMPLETELY);

    int ar_status = arduiggler_get_status(cable);

    if (ar_status != STATUS_OK) {
        urj_warning (_("cable not initialized properly\n"));
        return URJ_STATUS_FAIL;
    }

    urj_tap_cable_cx_cmd_queue (cmd_root, 0);
    urj_tap_cable_cx_cmd_push (cmd_root, CMD_GETVER);
    urj_tap_cable_cx_xfer (cmd_root, NULL, cable, URJ_TAP_CABLE_COMPLETELY);

    char ar_swver[] = "    ";

    for (int i = 0; i < strlen(ar_swver); i++) {
      ar_swver[i] = urj_tap_cable_cx_xfer_recv (cable);
    }
    urj_log (URJ_LOG_LEVEL_NORMAL, "Arduiggler firmware: %s\n", ar_swver);

    ar_status = arduiggler_get_status(cable);

    if (ar_status != STATUS_OK) {
        urj_warning (_("cable not initialized properly\n"));
        return URJ_STATUS_FAIL;
    }
    //arduiggler_set_frequency (cable, 0);

    return URJ_STATUS_OK;
}

static void
arduiggler_cable_free (urj_cable_t *cable)
{
    params_t *params = cable->params;

    urj_tap_cable_cx_cmd_deinit (&params->cmd_root);
    urj_tap_cable_generic_usbconn_free (cable);
}

static void
arduiggler_set_frequency (urj_cable_t *cable, uint32_t new_frequency)
{
  urj_warning (_("Arduiggler does not support configurable frequency\n"));
}

static void
arduiggler_clock (urj_cable_t *cable, int tms, int tdi, int n)
{
    uint8_t ar_data;
    int ar_clk = n;

    if (tdi)
      ar_data = URJ_POD_CS_TDI;
    else
      ar_data = 0x00;

    if(tms)
      ar_data |= URJ_POD_CS_TMS;

    params_t *params = cable->params;
    urj_tap_cable_cx_cmd_root_t *cmd_root = &params->cmd_root;
    int ar_status;

    while(ar_clk > UCHAR_MAX) {
      urj_tap_cable_cx_cmd_queue (cmd_root, 0);
      urj_tap_cable_cx_cmd_push (cmd_root, CMD_SEND);
      urj_tap_cable_cx_cmd_push (cmd_root, ar_data);
      urj_tap_cable_cx_cmd_push (cmd_root, UCHAR_MAX);
      urj_tap_cable_cx_xfer (cmd_root, NULL, cable, URJ_TAP_CABLE_COMPLETELY);

      ar_status = arduiggler_get_status(cable);
      if (ar_status != STATUS_OK) {
        urj_log (URJ_LOG_LEVEL_WARNING, "arduiggler_clock - ar_status = %X\n", ar_status);
        return;
      }
      ar_clk -= UCHAR_MAX;
    }

    if(ar_clk > 0) {
      urj_tap_cable_cx_cmd_queue (cmd_root, 0);
      urj_tap_cable_cx_cmd_push (cmd_root, CMD_SEND);
      urj_tap_cable_cx_cmd_push (cmd_root, ar_data);
      urj_tap_cable_cx_cmd_push (cmd_root, (uint8_t)ar_clk);
      urj_tap_cable_cx_xfer (cmd_root, NULL, cable, URJ_TAP_CABLE_COMPLETELY);

      ar_status = arduiggler_get_status(cable);
      if (ar_status != STATUS_OK) {
        urj_log (URJ_LOG_LEVEL_WARNING, "arduiggler_clock - ar_status = %X\n", ar_status);
        return;
      }
    }
}

static int
arduiggler_get_tdo (urj_cable_t *cable)
{
    params_t *params = cable->params;
    urj_tap_cable_cx_cmd_root_t *cmd_root = &params->cmd_root;

    urj_tap_cable_cx_cmd_queue (cmd_root, 0);
    urj_tap_cable_cx_cmd_push (cmd_root, CMD_READ);
    urj_tap_cable_cx_xfer (cmd_root, NULL, cable, URJ_TAP_CABLE_COMPLETELY);

    uint8_t ar_rply = urj_tap_cable_cx_xfer_recv (cable);

    int tdo = (int)(ar_rply & 0x01);

    int ar_status = arduiggler_get_status(cable);
    //TODO: handle cable failure
    urj_log (URJ_LOG_LEVEL_DEBUG, "arduiggler_get_tdo - ar_status = %X\n", ar_status);

    return tdo;
}

static int
arduiggler_set_signal (urj_cable_t *cable, int mask, int val)
{
    params_t *params = cable->params;
    urj_tap_cable_cx_cmd_root_t *cmd_root = &params->cmd_root;

    mask &= ( URJ_POD_CS_RESET |
        URJ_POD_CS_TRST |
        URJ_POD_CS_TMS |
        URJ_POD_CS_TCK |
        URJ_POD_CS_TDI );

    urj_tap_cable_cx_cmd_queue (cmd_root, 0);
    urj_tap_cable_cx_cmd_push (cmd_root, CMD_FORCE);
    urj_tap_cable_cx_cmd_push (cmd_root, (uint8_t)(val & mask));
    urj_tap_cable_cx_xfer (cmd_root, NULL, cable, URJ_TAP_CABLE_COMPLETELY);

    int ar_status = arduiggler_get_status(cable);
    //TODO: handle cable failure
    urj_log (URJ_LOG_LEVEL_DEBUG, "arduiggler_set_signal - ar_status = %X\n", ar_status);

    return 0;
}

const const urj_cable_driver_t urj_tap_cable_arduiggler_driver = {
    "Arduiggler",
    N_("Arduino JTAG USB Cable (FT232)"),
    URJ_CABLE_DEVICE_USB,
    { .usb = arduiggler_connect, },
    urj_tap_cable_generic_disconnect,
    arduiggler_cable_free,
    arduiggler_init,
    urj_tap_cable_generic_usbconn_done,
    arduiggler_set_frequency,
    arduiggler_clock,
    arduiggler_get_tdo,
    urj_tap_cable_generic_transfer, // TODO
    arduiggler_set_signal,
    urj_tap_cable_generic_get_signal, // TODO
    urj_tap_cable_generic_flush_one_by_one,
    urj_tap_cable_generic_usbconn_help
};
URJ_DECLARE_FTDX_CABLE(0x0403, 0x6001, "", "arduiggler", arduiggler)
