#ifndef __STRUCTS_H__
#define __STRUCTS_H__

#include "acfutils/geom.h"
#include "variables.h"
#include <string>

//sim state variables

struct waypoints
{
    geo_pos2_t wp[9];
    double dist[8];
    double crs[8];
};

struct IRU_t
{
    int nav_mode; //nav mode on
    int power_on; //is the unit on
    int mix_switch; //triple mix
    int batt_self_test; //battery self test light during alignment
    int dme1_update_status;
    int dme2_update_status;
    int dme1_freq_hz;
    int dme2_freq_hz;
    int waypoint_selector;
    geo_pos3_t current_pos; //computed from align pos and fwd propogation
    geo_pos3_t align_pos; //input for alignment - no cheating, bad pos = bad nav
    geo_pos3_t dme_pos[9]; //0 = dme 1
    geo_pos3_t mix_pos;
    geo_pos2_t update_pos;
    geo_pos3_t nav_pos;
    char curr_pos_dm[32];
    char mix_pos_dm[32];
    vect2_t pos_drift_vect; //unit sensor bias rectangular velocities 
    vect2_t polar_pos_drift;//unit sensor bias polar coordinates
    vect2_t velocity_vect; // current rectangular velocity vector (N, E)
    vect2_t flight_vect;
    vect2_t polar_ground_vel; //dir,mag
    vect2_t polar_flight_vel;
    vect2_t polar_wind_vect;
    vect2_t mix_vect;
    vect2_t polar_mix_vel;
    double adc1_alt;
    double adc2_alt;
    double dme1_dist_comp;
    double dme2_dist_comp;
    double heading_true; //true heading degrees
    double batt_capacity_sec; //1800 seconds of power
    double drift_angle;
    double cross_track_err;
    double track_ang_err;
    double tas;
    double time_in_nav;
};

struct Sim_State_t
{
    int paused;
    int pitot1_fail;
    int pitot2_fail;
    int static1_fail;
    int static2_fail;
    double runtime;
    double tas_kt_[2];
    double alt_ft_[2];
    double tas_ms;
    double ground_speed;
    double true_trk;
    double frame_time;
    double lat;
    double lon;
    double hdg_true;
    double vh_ind_fpm2;
    double elev_mtr;
    double nav1_freq_hz;
    double nav2_freq_hz;
    double nav1_dme_nm;
    double nav2_dme_nm;
    int apu_gen_on;
    int eng_gen_on[8];
    IRU_t IRU[NUM_IRU];
};

extern waypoints flight_plan;
extern IRU_t IRU[NUM_IRU];
extern Sim_State_t State_New;
extern Sim_State_t State_Old;

#endif