#include <XPLMProcessing.h>
#include <XPLMUtilities.h>
#include <acfutils/dr_cmd_reg.h>
#include <acfutils/log.h>
#include <acfutils/core.h>
#include <cstring>
#include "XPLMDefs.h"
#include "sources.h"
#include "datarefs.h"
#include "functions.h"
#include "structs.h"
#include "variables.h"
#include "navsystem.h"
#include "pos_functions.h"

#if IBM
	#include <windows.h>
#endif

#define PLUGIN_VERSION 220614.1440
bool first_floop = {true};
double nav_start_time = {0};
char sim_pos_dm[32] = {0};


static float 
iru_floop(float elapsed1, float elapsed2, int counter, void* refcon);

int
PosSetCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

static void
log_dbg_string(const char *str)
{
	XPLMDebugString(str);
}

PLUGIN_API int XPluginStart
(
	char * outName,
	char * outSig,
	char * outDesc
)
{
	strcpy(outName, "OMI IRU");
	strcpy(outSig, "omi.iru");
	strcpy(outDesc, "C-IVA IRU implementation");
	log_init(log_dbg_string, "OMI-IRU");

	LoadDataRefs();
	logMsg("Load DataRefs DONE");
	GetDataRefs();
	logMsg("Get DateRefs Done");
	dcr_init();
	logMsg("C-IVA IRU version: %f, using libacfutils-%s", PLUGIN_VERSION, libacfutils_version);
	MakeDataRefs();
	logMsg("Make Datarefs DONE");	
	
	return 1;
}

PLUGIN_API int XPluginEnable(void)
{
	
	iru_init();
	XPLMRegisterFlightLoopCallback(iru_floop, -1, NULL);
	return 1;
}

PLUGIN_API void XPluginDisable(void)
{
	XPLMUnregisterFlightLoopCallback(iru_floop,NULL);
}

PLUGIN_API void XPluginStop()
{
	dcr_fini();
	log_fini();
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) { }


static float iru_floop(float elapsed1, float elapsed2, int counter, void* refcon)
{

	
	GetDataRefs();
	if(!State_New.paused)
	{
		State_New.delta_time = State_New.runtime - State_Old.runtime;
		//sim_pos_deg_min();
		deg_min(State_New.lat, State_New.lon, sim_pos_dm, sizeof(sim_pos_dm));

		electrical_source();
		debug_set_pos();
		
		for(int i = 0; i < NUM_IRU; ++i)
		{   
			timer_functions(&State_New.IRU[i], State_New.delta_time);
			
			if(IRU[i].msu_mode > ALIGN && IRU[i].batt_capacity_sec > 0)
			{
				warn_light_logic(i);
				adc_data_in(i);				
				current_pos_update(i);
				triple_mix_logic(i);
				wpt_deg_min(i);
				current_leg_compute(i);
				leg_switch(i);
			}
			waypoint_selector_clamp(i);
			remote_priority(i);
			State_New.IRU[i] = IRU[i];
		}

		if(first_floop)
		{
			
			logMsg("Successfully Entered flight loop");
			
			first_floop = false;
		}
	}
	return -1;
}

