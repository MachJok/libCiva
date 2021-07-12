#ifndef __DATAREFS_H__
#define __DATAREFS_H__

#include "acfutils/dr.h"
#include "sources.h"
#include "structs.h"

static dr_t Sim_Frame_Time;
static dr_t Sim_Paused;
static dr_t Sim_Lat;
static dr_t Sim_Lon;
static dr_t Sim_RunTime;
static dr_t Sim_Hdg_True;
static dr_t Sim_True_Track;
static dr_t Sim_Ground_Speed;
static dr_t Sim_APU_gen_on;
static dr_t Sim_ENG_gen_on;
static dr_t Sim_Timestamp;
static dr_t Sim_TAS;
static dr_t Sim_TAS_L;
static dr_t Sim_TAS_R;
static dr_t Sim_ALT_L;
static dr_t Sim_ALT_R;


//function loads required datarefs
void LoadDataRefs();
//get function for datarefs
void GetDataRefs();
//makes xplane datarefs
void MakeDataRefs();

#endif
