/* mtcaTestMain.cpp */
/* Author:  Marty Kraimer Date:    17MAR2000 */

#include <stddef.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "epicsExit.h"
#include "epicsThread.h"

#ifdef HAVE_CEXP
#include <cexp.h>
#endif

#include "iocsh.h"

int main(int argc,char *argv[])
{
#ifdef HAVE_CEXP
	cexp_main(argc, argv);
#else
    if(argc>=2) {    
        iocsh(argv[1]);
        epicsThreadSleep(.2);
    }
    iocsh(NULL);
#endif
    epicsExit(0);
    return(0);
}
