#!/bin/bash

export EDMDATAFILES=/afs/slac/g/lcls/tools/edm/display/misc/

edm -x -m "crat=CPU:TEST" server_pc.edl &
edm -x -m "crat=CRAT:TEST" crat_atca_7slot.edl &
