//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ipmiComm'.
// It is subject to the license terms in the LICENSE.txt file found in the
// top-level directory of this distribution and at:
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
// No part of 'ipmiComm', including this file,
// may be copied, modified, propagated, or distributed except according to
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <dbAccess.h>
#include <dbAddr.h>
#include <dbDefs.h>
#include <link.h>
#include <registryFunction.h>
#include <epicsExport.h>
#include <errlog.h>
#include <aSubRecord.h>

#include <drvMch.h>

/*
 *
 *  Inputs
 *
 *	a   Raw 'type' value as defined in drvMchDef.h
 *
 * b-h are the converted 'type' values used by other facility software
 *
 *      b   'Type' value for unknown device
 *      c   'Type' value for Vadatech MCH
 *      d   'Type' value for NAT MCH
 *      e   'Type' value for Supermicro server
 *      f   'Type' value for Pentair shelf manager
 *      g   'Type' value for Artesyn shelf manager
 *      h   'Type' value for Advantech server
 *
 *  Outputs
 *
 *      a   0 if FAULT, 1 if OK
 *
 *  Logic
 *
 *      MPS requirements can be found...
 */

long subMchTypeFacility(struct aSubRecord* psub) {
    epicsFloat64 unknown    = *(epicsFloat64*)psub->b;
    epicsFloat64 vt         = *(epicsFloat64*)psub->c;
    epicsFloat64 nat        = *(epicsFloat64*)psub->d;
    epicsFloat64 supermicro = *(epicsFloat64*)psub->e;
    epicsFloat64 pentair    = *(epicsFloat64*)psub->f;
    epicsFloat64 artesyn    = *(epicsFloat64*)psub->g;
    epicsFloat64 advantech  = *(epicsFloat64*)psub->h;

    epicsFloat64 raw;

    if (psub == NULL)
        return 0;

    raw = *(epicsFloat64*)psub->a;

    if (raw == MCH_TYPE_VT)
        *((epicsFloat64*)psub->vala) = vt;

    else if (raw == MCH_TYPE_NAT)
        *((epicsFloat64*)psub->vala) = nat;

    else if (raw == MCH_TYPE_SUPERMICRO)
        *((epicsFloat64*)psub->vala) = supermicro;

    else if (raw == MCH_TYPE_PENTAIR)
        *((epicsFloat64*)psub->vala) = pentair;

    else if (raw == MCH_TYPE_ARTESYN)
        *((epicsFloat64*)psub->vala) = artesyn;

    else if (raw == MCH_TYPE_ADVANTECH)
        *((epicsFloat64*)psub->vala) = advantech;

    else if (raw == MCH_TYPE_UNKNOWN)
        *((epicsFloat64*)psub->vala) = unknown;

    return 0;
}

epicsRegisterFunction(subMchTypeFacility);
