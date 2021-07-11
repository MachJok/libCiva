#include "datarefs.h"
#include "XPLMUtilities.h"
#include "acfutils/dr.h"
#include "acfutils/dr_cmd_reg.h"
#include "structs.h"
#include "variables.h"


void LoadDataRefs()
{
    fdr_find(&Sim_RunTime, "sim/time/total_running_time_sec");
    fdr_find(&Sim_Frame_Time, "sim/time/framerate_period");
    fdr_find(&Sim_Paused, "sim/time/paused");
    fdr_find(&Sim_Track, "sim/flightmodel/position/true_psi");
    fdr_find(&Sim_Ground_Speed, "sim/flightmodel/position/groundspeed");
    fdr_find(&Sim_Lat, "sim/flightmodel/position/latitude");
    fdr_find(&Sim_Lon, "sim/flightmodel/position/longitude");
    fdr_find(&Sim_Hdg_True, "sim/flightmodel/position/true_psi");
    fdr_find(&Sim_APU_gen_on, "sim/cockpit/electrical/generator_apu_on");
    fdr_find(&Sim_ENG_gen_on, "sim/cockpit/electrical/generator_on");
    fdr_find(&Sim_Timestamp, "sim/time/total_running_time_sec");
    fdr_find(&Sim_Paused, "sim/time/paused");

}

void GetDataRefs()
{
    State_Old = State_New;
    for(int i = 0; i < NUM_IRU; ++i)
    {
        State_Old.IRU[i] = IRU[i];
    }
    State_Old.Triple_Mix_Pos = Triple_Mix_Pos;
    State_New.paused = dr_geti(&Sim_Paused);
    State_New.frame_time = dr_getf_prot(&Sim_Frame_Time);
    State_New.runtime = dr_getf_prot(&Sim_RunTime);
    State_New.track = dr_getf_prot(&Sim_Track);
    State_New.ground_speed = dr_getf_prot(&Sim_Ground_Speed);
    State_New.lat = dr_getf_prot(&Sim_Lat);
    State_New.lon = dr_getf_prot(&Sim_Lon);
    State_New.hdg_true = dr_getf_prot(&Sim_Hdg_True);
    State_New.apu_gen_on = dr_geti(&Sim_APU_gen_on);
    dr_getvi(&Sim_ENG_gen_on, State_New.eng_gen_on, 0, 8);
    State_New.paused = dr_geti(&Sim_Paused);
    
}

void MakeDataRefs()
{
    
    for (int i = 0; i < NUM_IRU; i++)
    {
        DCR_CREATE_F64(NULL, &IRU[i].nav_mode, true, "omi/iru/%d/mode", i);
        DCR_CREATE_F64(NULL, &IRU[i].power_on, true, "omi/iru/%d/power", i);
        DCR_CREATE_F64(NULL, &IRU[i].current_pos.lat, true, "omi/iru/%d/iru_pos_lat", i);
        DCR_CREATE_F64(NULL, &IRU[i].current_pos.lon, true, "omi/iru/%d/iru_pos_lon", i);
        DCR_CREATE_F64(NULL, &IRU[i].align_pos.lat, true, "omi/iru/%d/align_lat", i);
        DCR_CREATE_F64(NULL, &IRU[i].align_pos.lon, true, "omi/iru/%d/align_lon", i);

        DCR_CREATE_F64(NULL, &IRU[i].velocity_vect.x, true, "omi/iru/%d/velocity_n", i);
        DCR_CREATE_F64(NULL, &IRU[i].velocity_vect.y, true, "omi/iru/%d/velocity_e", i);        
        DCR_CREATE_F64(NULL, &IRU[i].batt_capacity_sec, true, "omi/iru/%d/battery_secs", i);
        DCR_CREATE_F64(NULL, &IRU[i].polar_vel.x, true, "omi/iru/%d/iru_psi", i);
        DCR_CREATE_F64(NULL, &IRU[i].polar_vel.y, true, "omi/iru/%d/iru_vel", i);

        DCR_CREATE_F64(NULL, &IRU[i].pos_drift_vect.x, true, "omi/iru/%d/pos_drift_n", i);
        DCR_CREATE_F64(NULL, &IRU[i].pos_drift_vect.y, true, "omi/iru/%d/pos_drift_e", i);
        DCR_CREATE_F64(NULL, &IRU[i].polar_pos_drift.x, true, "omi/iru/%d/pos_drift_psi", i);
        DCR_CREATE_F64(NULL, &IRU[i].polar_pos_drift.y, true, "omi/iru/%d/pos_drift_vel", i);

        DCR_CREATE_F64(NULL, &IRU[i].mix_switch, true, "omi/iru/%d/triple_mix_switch", i);
        DCR_CREATE_F64(NULL, &Triple_Mix_Pos.curr_pos.lat, true, "omi/iru/%d/triple_mix_pos_lat", i);
        DCR_CREATE_F64(NULL, &Triple_Mix_Pos.curr_pos.lon, true, "omi/iru/%d/triple_mix_pos_lon", i);
        DCR_CREATE_F64(NULL, &Triple_Mix_Pos.velocity_vect.x, true, "omi/iru/%d/triple_mix_vel_n", i);
        DCR_CREATE_F64(NULL, &Triple_Mix_Pos.velocity_vect.y, true, "omi/iru/%d/triple_mix_vel_e", i);   
        DCR_CREATE_F64(NULL, &Triple_Mix_Pos.polar_vel_vect.x, true, "omi/iru/%d/triple_mix_psi", i);
        DCR_CREATE_F64(NULL, &Triple_Mix_Pos.polar_vel_vect.y, true, "omi/iru/%d/triple_mix_spd", i);   
    
        
    }
}