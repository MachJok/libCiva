#include <XPLMProcessing.h>
#include <XPLMUtilities.h>
#include <acfutils/dr_cmd_reg.h>
#include <acfutils/log.h>
#include <acfutils/core.h>
#include "sources.h"
#include "datarefs.h"
#include "functions.h"
#include "structs.h"
#include "variables.h"

#define PLUGIN_VERSION 210711.0651

bool first_floop = true;

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

        electrical_source();
        debug_set_pos();
        for(int i = 0; i < NUM_IRU; ++i)
        {
            if(IRU[i].nav_mode > 2)
            {
                current_pos_update(i);
            }
        }

        if(first_floop)
        {
            logMsg("Successfully Entered flight loop");
            first_floop = false;
        }
    }
    return -1;
}
