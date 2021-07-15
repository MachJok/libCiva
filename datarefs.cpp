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
    fdr_find(&Sim_Lat, "sim/flightmodel/position/latitude");
    fdr_find(&Sim_Lon, "sim/flightmodel/position/longitude");
    fdr_find(&Sim_Hdg_True, "sim/flightmodel/position/true_psi");
    fdr_find(&Sim_APU_gen_on, "sim/cockpit/electrical/generator_apu_on");
    fdr_find(&Sim_ENG_gen_on, "sim/cockpit/electrical/generator_on");
    fdr_find(&Sim_Timestamp, "sim/time/total_running_time_sec");
    fdr_find(&Sim_Paused, "sim/time/paused");
    fdr_find(&Sim_TAS_L,"sim/cockpit2/gauges/indicators/true_airspeed_kts_pilot");
    fdr_find(&Sim_TAS_R,"sim/cockpit2/gauges/indicators/true_airspeed_kts_copilot");
    fdr_find(&Sim_ALT_L,"sim/cockpit2/gauges/indicators/airspeed_kts_pilot");
    fdr_find(&Sim_ALT_R,"sim/cockpit2/gauges/indicators/airspeed_kts_copilot");
    fdr_find(&Sim_Ground_Speed, "sim/flightmodel/position/groundspeed");
    fdr_find(&Sim_TAS,"sim/flightmodel/position/true_airspeed");
    fdr_find(&Sim_True_Track, "sim/flightmodel/position/hpath");   

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
    State_New.lat = dr_getf_prot(&Sim_Lat);
    State_New.lon = dr_getf_prot(&Sim_Lon);
    State_New.apu_gen_on = dr_geti(&Sim_APU_gen_on);
    dr_getvi(&Sim_ENG_gen_on, State_New.eng_gen_on, 0, 8);
    State_New.paused = dr_geti(&Sim_Paused);
    State_New.tas_ms = dr_getf_prot(&Sim_TAS);
    State_New.hdg_true = dr_getf_prot(&Sim_Hdg_True);
    State_New.tas_kt_[0] = dr_getf_prot(&Sim_TAS_L);
    State_New.tas_kt_[1] = dr_getf_prot(&Sim_TAS_R);
    State_New.alt_ft_[0] = dr_getf_prot(&Sim_ALT_L);
    State_New.alt_ft_[1] = dr_getf_prot(&Sim_ALT_R);
    State_New.true_trk = dr_getf_prot(&Sim_True_Track);
    State_New.ground_speed = dr_getf_prot(&Sim_Ground_Speed);

    
}

void MakeDataRefs()
{
    DCR_CREATE_B(NULL, sim_pos_dm, sizeof(sim_pos_dm), true, "omi/iru/sim_pos_dm");
    for (int i = 0; i < NUM_IRU; i++)
    {
        
        
        DCR_CREATE_I(NULL, &IRU[i].nav_mode, true, "omi/iru/%d/mode", i);
        DCR_CREATE_I(NULL, &IRU[i].power_on, true, "omi/iru/%d/power", i);
        DCR_CREATE_I(NULL, &IRU[i].mix_switch, true, "omi/iru/%d/triple_mix_switch", i);

        DCR_CREATE_F64(NULL, &IRU[i].batt_capacity_sec, true, "omi/iru/%d/battery_secs", i);
        DCR_CREATE_F64(NULL, &IRU[i].drift_angle, true, "omi/iru/%d/drift_angle", i);
        DCR_CREATE_F64(NULL, &IRU[i].tas, true, "omi/iru/%d/tas", i);
        DCR_CREATE_F64(NULL, &IRU[i].heading_true,true,"omi/iru/%d/heading", i);
        DCR_CREATE_F64(NULL, &IRU[i].time_in_nav,true,"omi/iru/%d/time_in_nav", i);

        DCR_CREATE_VF64(NULL, (double *) &IRU[i].current_pos, 2, true, "omi/iru/%d/iru_pos", i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].align_pos, 2, true, "omi/iru/%d/align_pos", i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].pos_drift_vect, 2, true, "omi/iru/%d/pos_drift_en",i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].polar_pos_drift, 2, true, "omi/iru/%d/pos_drift_vect",i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].velocity_vect, 2, true, "omi/iru/%d/gnd_vel_en", i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].flight_vect, 2, true, "omi/iru/%d/flt_vel_en", i);        
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].polar_ground_vel, 2, true, "omi/iru/%d/ground_vel_vect", i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].polar_flight_vel, 2, true, "omi/iru/%d/flight_vel_vect", i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].polar_wind_vect, 2, true, "omi/iru/%d/wind_vect",i);
        DCR_CREATE_B(NULL, IRU[i].curr_pos_dm, sizeof(IRU[i].curr_pos_dm), true, "omi/iru/%d/current_pos_dm", i);

        DCR_CREATE_VF64(NULL, (double *) &Triple_Mix_Pos.curr_pos, 2, true, "omi/iru/%d/triple_pos",i);
        DCR_CREATE_VF64(NULL, (double *) &Triple_Mix_Pos.velocity_vect, 2, true, "omi/iru/%d/triple_vel_en",i);
        DCR_CREATE_VF64(NULL, (double *) &Triple_Mix_Pos.polar_vel_vect, 2,  true, "omi/iru/%d/triple_vel_vect", i);
        DCR_CREATE_B(NULL, Triple_Mix_Pos.curr_pos_dm, sizeof(Triple_Mix_Pos.curr_pos_dm), true, "omi/iru/%d/triple_pos_dm", i);
        

        
        /*
        *DCR_CREATE_F64(NULL, &IRU[i].current_pos.lat, true, "omi/iru/%d/iru_pos_lat", i);
        *DCR_CREATE_F64(NULL, &IRU[i].current_pos.lon, true, "omi/iru/%d/iru_pos_lon", i);
        *DCR_CREATE_F64(NULL, &IRU[i].align_pos.lat, true, "omi/iru/%d/align_lat", i);
        *DCR_CREATE_F64(NULL, &IRU[i].align_pos.lon, true, "omi/iru/%d/align_lon", i);
        *DCR_CREATE_F64(NULL, &IRU[i].velocity_vect.x, true, "omi/iru/%d/velocity_n", i);
        *DCR_CREATE_F64(NULL, &IRU[i].velocity_vect.y, true, "omi/iru/%d/velocity_e", i);
        *DCR_CREATE_F64(NULL, &IRU[i].polar_vel.x, true, "omi/iru/%d/iru_psi", i);
        *DCR_CREATE_F64(NULL, &IRU[i].polar_vel.y, true, "omi/iru/%d/iru_vel", i);
        *DCR_CREATE_F64(NULL, &IRU[i].pos_drift_vect.x, true, "omi/iru/%d/pos_drift_n", i);
        *DCR_CREATE_F64(NULL, &IRU[i].pos_drift_vect.y, true, "omi/iru/%d/pos_drift_e", i);
        *DCR_CREATE_F64(NULL, &IRU[i].polar_pos_drift.x, true, "omi/iru/%d/pos_drift_psi", i);
        *DCR_CREATE_F64(NULL, &IRU[i].polar_pos_drift.y, true, "omi/iru/%d/pos_drift_vel", i);
        *DCR_CREATE_F64(NULL, &Triple_Mix_Pos.curr_pos.lat, true, "omi/iru/%d/triple_mix_pos_lat", i);
        *DCR_CREATE_F64(NULL, &Triple_Mix_Pos.curr_pos.lon, true, "omi/iru/%d/triple_mix_pos_lon", i);
        *DCR_CREATE_F64(NULL, &Triple_Mix_Pos.velocity_vect.x, true, "omi/iru/%d/triple_mix_vel_n", i);
        *DCR_CREATE_F64(NULL, &Triple_Mix_Pos.velocity_vect.y, true, "omi/iru/%d/triple_mix_vel_e", i);
        *DCR_CREATE_F64(NULL, &Triple_Mix_Pos.polar_vel_vect.x, true, "omi/iru/%d/triple_mix_psi", i);
        *DCR_CREATE_F64(NULL, &Triple_Mix_Pos.polar_vel_vect.y, true, "omi/iru/%d/triple_mix_spd", i);
        */     
    
        
    }
}