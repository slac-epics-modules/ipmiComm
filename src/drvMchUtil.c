#include <stdint.h>
#include <math.h>    /* pow */
#include <stdio.h>   /* FILE, fopen, fclose */
#include <errlog.h>
#include <string.h>  /* strcpy, memset */
#include <ctype.h>   /* toupper */

#include <drvMch.h>
#include <ipmiDef.h>

#undef DEBUG

#ifndef MAX
	#define MAX( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

void
mchCreateFile( const char *filename )
{
FILE *file;

	file = fopen( filename, "w+" );
	if ( file )
		fclose(file);
	else
		errlogPrintf("mchCreateFile: Failed to create new file %s\n", filename);
}

void
mchSensorFruGetInstance(MchSys mchSys)
{
int     f = MAX_FRU;
int     s = mchSys->sensCount;
uint8_t fruEntId[f];                         /* Entity IDs */
uint8_t fruEntInst[f];                       /* Counts of each ID */
uint8_t sensTypeInst[f][MAX_SENSOR_TYPE];    /* Counts of sensor type per ID instance */
int i, j, sensCount = 1, found, id;
Fru     fru;
Sensor  sens;

	memset( fruEntId,     0, f        );
	memset( fruEntInst,   0, f        );
	memset( sensTypeInst, 0, f*MAX_SENSOR_TYPE );

	for ( i = 0; i < f; i++ ) {

		fru   = &mchSys->fru[i];
                id    = fru->sdr.fruId;
		found = 0;

		if ( id == UTCA_FRU_TYPE_CARRIER ) {
			sprintf( fru->parm, "CR" );
			fru->instance = 1;
		}

		else if( id >= UTCA_FRU_TYPE_SHELF_MIN && id <= UTCA_FRU_TYPE_SHELF_MAX ) {
			sprintf( fru->parm, "SH" );
			fru->instance = ( id - UTCA_FRU_TYPE_SHELF_MIN ) + 1;
		}

		else if( id >= UTCA_FRU_TYPE_MCH_MIN && id <= UTCA_FRU_TYPE_MCH_MAX ) {
			sprintf( fru->parm, "MCH" );
			fru->instance = ( id - UTCA_FRU_TYPE_MCH_MIN ) + 1;
		}

		else if( id >= UTCA_FRU_TYPE_AMC_MIN && id <= UTCA_FRU_TYPE_AMC_MAX ) {
			sprintf( fru->parm, "AMC" );
			fru->instance = ( id - UTCA_FRU_TYPE_AMC_MIN ) + 1;
		}

		else if( id >= UTCA_FRU_TYPE_CU_MIN && id <= UTCA_FRU_TYPE_CU_MAX ) {
			sprintf( fru->parm, "CU" );
			fru->instance = ( id - UTCA_FRU_TYPE_CU_MIN ) + 1;
		}

		else if( id >= UTCA_FRU_TYPE_PM_MIN && id <= UTCA_FRU_TYPE_PM_MAX ) {
			sprintf( fru->parm, "PM" );
			fru->instance = ( id - UTCA_FRU_TYPE_PM_MIN ) + 1;
		}

		else if( id >= UTCA_FRU_TYPE_RTM_MIN && id <= UTCA_FRU_TYPE_RTM_MAX ) {
			sprintf( fru->parm, "RTM" );
			fru->instance = ( id - UTCA_FRU_TYPE_RTM_MIN ) + 1;
		}

		else if ( id == UTCA_FRU_TYPE_LOG_CARRIER ) {
			sprintf( fru->parm, "CR" );
			fru->instance = 1;
		}

		/* Find sensors associated with this FRU, assign each an instance based on sensor type */
		for ( j = 0; j < s; j++ ) {

			sens = &mchSys->sens[j];

			if ( mchSys->fru[sens->fruIndex].sdr.fruId == id ) {

				/* If a real entity */
				if ( fru->sdr.entityInst ) {
					sens->instance = ++sensTypeInst[i][sens->sdr.sensType]; 
					sensCount++;

					switch ( sens->sdr.sensType ) {
	
						default:
							break;

						case SENSOR_TYPE_TEMP:
							fru->tempCnt = sens->instance;
							break;
	
						case SENSOR_TYPE_FAN:
							fru->fanCnt = sens->instance;
							break;
		
						case SENSOR_TYPE_VOLTAGE:
							fru->vCnt = sens->instance;
							break;
					}
				}
			}
		}
	}

#ifdef DEBUG
printf("mchSensorFruGetInstance: Found %i matching sensors, total sensor count is %i\n",sensCount, mchSys->sensCount);
for ( i = 0; i < mchSys->sensCount; i++ ) {
	sens = &mchSys->sens[i];
	int fruIndex = sens->fruIndex;
	fru  = &mchSys->fru[fruIndex];
	printf("Sensor %i, type %02x, inst %i, FRU entId %02x, entInst %02x, sens entId %02x  entInst %02x\n",sens->sdr.number, sens->sdr.sensType, sens->instance, fru->sdr.entityId, fru->sdr.entityInst, sens->sdr.entityId, sens->sdr.entityInst);
}
#endif
}

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
char     dev[30];
char     stFile[50], dbFile[50];
char     code[10], desc[40];
Fru      fru;
Sensor   sens;
int      found, inst;

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

	sprintf( stFile, "st.%s.cmd", mchSys->name );

	errlogPrintf("Creating %s...\n", stFile);

	file = fopen( stFile, "w" ); 

	if ( file ) {	      

	       	fprintf( file, "dbLoadRecords(\"${TOP}/db/shelf.db\",\"dev=%s,link=%s\")\n", dev, mchSys->name);

		/* If we have the info to populate our records */
		if ( p ) {

			for ( i = 0; i < MAX_FRU; i++ ) {

				fru = &mchSys->fru[i];

				/* If we identified this FRU */
				if ( fru->sdr.entityInst ) {
					fprintf( file, "dbLoadRecords(\"${TOP}/db/module.db\",\"dev=%s,link=%s,code=%s,fruid=%i,inst=%i,index=%i,temp=%i,fan=%i,v=%i\")\n", /* continue next line */
					dev, mchSys->name, fru->parm, fru->sdr.fruId, fru->instance, i, fru->tempCnt, fru->fanCnt, fru->vCnt );

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

					fruIndex = sens->fruIndex;	
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

						case SENSOR_TYPE_HOT_SWAP:
							sprintf( dbFile, "sensor_hotswap.db" ); 
							found = 1;
							break;
					}

					if ( sens->sdr.recType == SDR_TYPE_FULL_SENSOR )
					    sprintf( desc, "'%s'", sens->sdr.str );
					else
					    sprintf( desc, "'%s'", code ); /* Improve this string */

					if ( found )
						fprintf( file, "dbLoadRecords(\"${TOP}/db/%s\",\"dev=%s,link=%s,code=%s,inst=%i,sensInst=%i,sens=%i,desc=%s\")\n", /* continue next line */
						dbFile, dev, mchSys->name, code, inst, sens->instance, i, desc );
				}
			}

		}
       		fclose(file);
	}
	else
		errlogPrintf("Failed to open %s; some or all sensor and/or FRU PVs will be missing\n", stFile);
}
