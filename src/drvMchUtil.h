#ifndef DRV_MCH_UTIL_H
#define DRV_MCH_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif
	           
void
mchCreateFile(const char *filename);

/* IMPORTANT: Caller must perform locking when calling this routine
 * 
 * In order to assign PV names, 
 *
 * assign each FRU an instance for its type and 
 * each sensor an instance of its type on the associated FRU
 *
 * Notes:
 *
 *  -The FRU instance does not necessarily match the entity instance because that 
 *   one is not guaranteed to start at any particular number and we 
 *   would like ours to start at 1.)
 *
 *  -Non-FRU entities are ignored for now--will implement those in later release
 *
 *  -We assume the FRU IDs follow the MicroTCA spec (see ipmiDef.h) and also 
 *   that they do not change over time, i.e. Cooling Unit 2 is always 
 *   in the same physical location on the shelf and never swaps 
 *   locations with Cooling Unit 1.
 */
void
mchSensorFruGetInstance(MchSys mchSys);

void
sensorFruRecordScript(MchSys mchSys, int p);

#ifdef __cplusplus
};
#endif

#endif
