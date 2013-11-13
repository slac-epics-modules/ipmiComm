#include <stdint.h>
#include <math.h>    /* pow */
#include <stdio.h>   /* FILE, fopen, fclose */
#include <errlog.h>
#include <string.h>  /* strcpy, memset */
#include <ctype.h>   /* toupper */

#include <drvMch.h>
#include <ipmiDef.h>
#include <ipmiMsg.h>

#undef DEBUG

#ifndef MAX
	#define MAX( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif


/*
 * Create a script to load the appropriate EPICS records for our shelf
 *
 * In this release, always create a new script based on entities and
 * sensors we discovered.
 */
void
sensorFruRecordScript(MchSys mchSys, int p)
{
FILE    *file;
uint8_t  i, fruIndex;
char     dev[30], str[30];
char     stFile[50], dbFile[50];
char     code[10], desc[40];
Fru      fru;
Sensor   sens;
int      found, inst,  n;

	/* Convert node name to PV device name */
	for ( i = 0; i < sizeof( dev ); i++ ) {
		if ( mchSys->name[i] == (int)NULL)
			break;

		if ( mchSys->name[i] == '-' )
			dev[i] = ':';
		else
			dev[i] = toupper( mchSys->name[i] );
	}
	dev[i] = '\0';

        /* Replace first part of name with "CRAT" to match PV naming conventions */
        n = strcspn( dev, ":" );
        memcpy( str, dev + n, sizeof(dev) - n);  
        sprintf( dev, "CRAT%s", str );
	sprintf( stFile, "st.%s.cmd", mchSys->name );

	errlogPrintf("Creating %s file\n", stFile);

	file = fopen( stFile, "w" ); 

	if ( file ) {	      

	       	fprintf( file, "dbLoadRecords(\"${TOP}/db/shelf.db\",\"dev=%s,link=%s\")\n", dev, mchSys->name);

		/* If we have the info to populate our records */
		if ( p ) {

			for ( i = 0; i < MAX_FRU; i++ ) {

				fru = &mchSys->fru[i];

				/* If we identified this FRU */
				if ( fru->sdr.entityInst ) {
					fprintf( file, "dbLoadRecords(\"${TOP}/db/module.db\",\"dev=%s,link=%s,code=%s,fruid=%i,inst=%i,temp=%i,fan=%i,v=%i\")\n", /* continue next line */
					dev, mchSys->name, fru->parm, fru->sdr.fruId, fru->instance, fru->tempCnt, fru->fanCnt, fru->vCnt );

					if ( (i >= UTCA_FRU_TYPE_CU_MIN) && (i <= UTCA_FRU_TYPE_CU_MAX) )
						fprintf( file, "dbLoadRecords(\"${TOP}/db/module_cu.db\",\"dev=%s,link=%s,code=%s,fruid=%i,inst=%i\")\n", /* continue next line */
						dev, mchSys->name, fru->parm, fru->sdr.fruId, fru->instance );
				}

			}	
			
			for ( i = 0; i < mchSys->sensCount; i++ ) {

				sens = &mchSys->sens[i];
				found = 0;

				/* If we've associated this sensor with a FRU and given it an instance */
				if ( sens->instance ) {

					fruIndex = sens->fruIndex;	/* can fruIndex be fruId? */
					    fru   = &mchSys->fru[fruIndex];    
					inst  = fru->instance;
					sprintf( code, "%s", fru->parm );

					switch ( sens->sdr.sensType ) {
	
						default:
							break;

						case SENSOR_TYPE_TEMP:
							sprintf( dbFile, "sensor_temp.db" ); 
							found = 1;	
							break;
	
						case SENSOR_TYPE_FAN:
							sprintf( dbFile, "sensor_fan.db" ); 
							found = 1;
							break;
		
						case SENSOR_TYPE_VOLTAGE:
							sprintf( dbFile, "sensor_voltage.db" ); 
							found = 1;
							break;

						case SENSOR_TYPE_CURRENT:
							sprintf( dbFile, "sensor_current.db" ); 
							found = 1;
							break;

						case SENSOR_TYPE_HOTSWAP_VT:
							sprintf( dbFile, "sensor_hotswap_vt.db" ); 
							found = 1;
							break;
						/* Hot swap sensor implementation under NAT is not 
                                                 * consistent; do not load NAT hot swap sensor database
						case SENSOR_TYPE_HOTSWAP_NAT:
							sprintf( dbFile, "sensor_hotswap_nat.db" ); 
							found = 1;
							break;
						*/
					}

					if ( sens->sdr.recType == SDR_TYPE_FULL_SENSOR )
					    sprintf( desc, "'%s'", sens->sdr.str );
					else
					    sprintf( desc, "'%s'", code ); /* Improve this string */

					if ( found )
						fprintf( file, "dbLoadRecords(\"${TOP}/db/%s\",\"dev=%s,link=%s,code=%s,inst=%i,sensInst=%i,fruid=%i,type=%i,desc=%s\")\n", /* continue next line */
						dbFile, dev, mchSys->name, code, inst, sens->instance, fruIndex, sens->sdr.sensType, desc );
						/*fprintf( file, "dbLoadRecords(\"${TOP}/db/%s\",\"dev=%s,link=%s,code=%s,inst=%i,sensInst=%i,sens=%i,desc=%s\")\n", /* continue next line
						dbFile, dev, mchSys->name, code, inst, sens->instance, i, desc );*/
				}
			}

		}
       		fclose(file);
	}
	else
		errlogPrintf("Failed to open %s; some or all sensor and/or FRU PVs will be missing\n", stFile);
}
