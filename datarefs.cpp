#include "datarefs.h"
#include "XPLMUtilities.h"
#include "acfutils/dr.h"
#include "acfutils/dr_cmd_reg.h"
#include "functions.h"
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
    fdr_find(&Sim_ALT_L,"sim/cockpit2/gauges/indicators/altitude_ft_pilot");
    fdr_find(&Sim_ALT_R,"sim/cockpit2/gauges/indicators/altitude_ft_copilot");
    fdr_find(&Sim_Ground_Speed, "sim/flightmodel/position/groundspeed");
    fdr_find(&Sim_TAS,"sim/flightmodel/position/true_airspeed");
    fdr_find(&Sim_True_Track, "sim/flightmodel/position/hpath");
    fdr_find(&Sim_VH_IND_FPM2, "sim/flightmodel/position/vh_ind_fpm2");
    fdr_find(&Sim_Elev_mtr,"sim/flightmodel/position/elevation");
    fdr_find(&Sim_NAV1_Freq, "sim/cockpit/radios/nav1_freq_hz");
    fdr_find(&Sim_NAV2_Freq, "sim/cockpit/radios/nav2_freq_hz");
    fdr_find(&Sim_NAV1_DME, "sim/cockpit/radios/nav1_dme_dist_m");
    fdr_find(&Sim_NAV2_DME, "sim/cockpit/radios/nav2_dme_dist_m");
    fdr_find(&Sim_Pitot1_Fail, "sim/operation/failures/rel_pitot");
    fdr_find(&Sim_Pitot2_Fail, "sim/operation/failures/rel_pitot2");
    fdr_find(&Sim_Static1_Fail, "sim/operation/failures/rel_static");
    fdr_find(&Sim_Static2_Fail, "sim/operation/failures/rel_static2");
    fdr_find(&Sim_Num_Gens, "sim/aircraft/electrical/num_generators");
    }

void GetDataRefs()
{
    State_Old = State_New;
    for(int i = 0; i < NUM_IRU; ++i)
    {
        State_Old.IRU[i] = IRU[i];
    }

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
    State_New.vh_ind_fpm2 = dr_getf_prot(&Sim_VH_IND_FPM2);
    State_New.elev_mtr = dr_getf_prot(&Sim_Elev_mtr); 
    State_New.nav1_freq_hz = dr_getf_prot(&Sim_NAV1_Freq);
    State_New.nav2_freq_hz = dr_getf_prot(&Sim_NAV2_Freq);
    State_New.nav1_dme_nm = dr_getf_prot(&Sim_NAV1_DME);
    State_New.nav2_dme_nm = dr_getf_prot(&Sim_NAV2_DME);
    State_New.pitot1_fail = dr_geti(&Sim_Pitot1_Fail);
    State_New.pitot2_fail = dr_geti(&Sim_Pitot2_Fail);
    State_New.static1_fail = dr_geti(&Sim_Static1_Fail);
    State_New.static2_fail = dr_geti(&Sim_Static2_Fail);
    State_New.num_generators = dr_geti(&Sim_Num_Gens);
}

void MakeDataRefs()
{
    DCR_CREATE_B(NULL, &sim_pos_dm, sizeof(sim_pos_dm), true, "omi/iru/sim_pos_dm");
    DCR_CREATE_VF64(NULL, (double *) &mix_pos, 3, true, "omi/iru/triple_pos");
    

    for (int i = 0; i < NUM_IRU; i++)
    {
        //DME Position datarefs
        for(int j = 0; j < 9; j++)
        {
            DCR_CREATE_VF64(NULL, (double *) &IRU[i].dme_pos[j], 3, true, "omi/iru/%d/dme-%d_pos", i, j);
        } 
        
        DCR_CREATE_I(NULL, &IRU[i].nav_mode, true, "omi/iru/%d/mode", i);
        DCR_CREATE_I(NULL, &IRU[i].power_on, true, "omi/iru/%d/power", i);
        DCR_CREATE_I(NULL, &IRU[i].mix_switch, true, "omi/iru/%d/triple_mix_switch", i);
        DCR_CREATE_I(NULL, &IRU[i].waypoint_selector, true, "omi/iru/%d/nav/waypoint_selector", i);
        DCR_CREATE_I(NULL, &IRU[i].batt_light, true, "omi/iru/%d/battery_light", i);
        DCR_CREATE_I(NULL, &IRU[i].auto_man_switch, true, "omi/iru/%d/nav/auto_man_switch",i);

        DCR_CREATE_F64(NULL, &IRU[i].batt_capacity_sec, true, "omi/iru/%d/battery_secs", i);
        DCR_CREATE_F64(NULL, &IRU[i].drift_angle, true, "omi/iru/%d/drift_angle", i);
        DCR_CREATE_F64(NULL, &IRU[i].tas, true, "omi/iru/%d/tas", i);
        DCR_CREATE_F64(NULL, &IRU[i].heading_true,true,"omi/iru/%d/heading", i);
        DCR_CREATE_F64(NULL, &IRU[i].time_in_nav,true,"omi/iru/%d/time_in_nav", i);
        DCR_CREATE_F64(NULL, &IRU[i].dme_data.dme1_dist_comp, true, "omi/iru/%d/dme1_dist_comp", i);
        DCR_CREATE_F64(NULL, &IRU[i].dme_data.dme2_dist_comp, true, "omi/iru/%d/dme2_dist_comp", i);
        DCR_CREATE_F64(NULL, &IRU[i].current_pos.elev, true, "omi/iru/%d/iru_alt", i);
        DCR_CREATE_F64(NULL, &IRU[i].flightplan.curr_leg_dist, true, "omi/iru/%d/nav/wpt_dist", i);
        DCR_CREATE_F64(NULL, &IRU[i].flightplan.curr_leg_crs, true, "omi/iru/%d/nav/dsrtk", i);
        DCR_CREATE_F64(NULL, &IRU[i].track_ang_err, true, "omi/iru/%d/nav/tke", i);

        DCR_CREATE_VI(NULL, (int *) &IRU[i].flightplan.leg, 2, true, "omi/iru/%d/nav/leg_frm_to", i);

        DCR_CREATE_VF64(NULL, (double *) &IRU[i].current_pos, 2, true, "omi/iru/%d/iru_pos", i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].nav_pos, 2, true, "omi/iru/%d/nav_pos", i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].align_pos, 2, true, "omi/iru/%d/align_pos", i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].pos_drift_vect, 2, true, "omi/iru/%d/pos_drift_en",i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].polar_pos_drift, 2, true, "omi/iru/%d/pos_drift_vect",i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].velocity_vect, 2, true, "omi/iru/%d/gnd_vel_en", i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].flight_vect, 2, true, "omi/iru/%d/flt_vel_en", i);        
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].polar_ground_vel, 2, true, "omi/iru/%d/ground_vel_vect", i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].polar_flight_vel, 2, true, "omi/iru/%d/flight_vel_vect", i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].polar_wind_vect, 2, true, "omi/iru/%d/wind_vect",i);
        for(int j = 1; j < 9; ++j)
        {
            DCR_CREATE_VF64(NULL, (double *) &IRU[i].flightplan.waypoint_pos[j], 2, true, "omi/iru/%d/nav/waypoint%d", i, j);
            
        }
        DCR_CREATE_B(NULL, &IRU[i].waypoint_dm, sizeof(&IRU[i].waypoint_dm), true, "omi/iru/%d/nav/waypoint_dm", i);
        DCR_CREATE_B(NULL, &IRU[i].curr_pos_dm, sizeof(IRU[i].curr_pos_dm), true, "omi/iru/%d/current_pos_dm", i);

        DCR_CREATE_VF64(NULL, (double *) &IRU[i].mix_pos, 2, true, "omi/iru/%d/triple_pos",i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].mix_vect, 2, true, "omi/iru/%d/triple_vel_en",i);
        DCR_CREATE_VF64(NULL, (double *) &IRU[i].polar_mix_vel, 2,  true, "omi/iru/%d/triple_vel_vect", i);
    }
}