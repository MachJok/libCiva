#ifndef __STRUCTS_H__
#define __STRUCTS_H__

#include "acfutils/geom.h"
#include "variables.h"

//sim state variables

struct triple_mix_pos_t
{
    geo_pos3_t curr_pos;
    vect2_t velocity_vect;
    vect2_t polar_vel_vect;
};

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
    geo_pos3_t current_pos; //computed from align pos and fwd propogation
    geo_pos3_t align_pos; //input for alignment - no cheating, bad pos = bad nav
    //triple_mix_pos_t triple_mix_pos_wgs;
    vect2_t pos_drift_vect; //unit sensor bias rectangular velocities 
    vect2_t polar_pos_drift;//unit sensor bias polar coordinates
    vect2_t correction_vect; //for DME updating (direction, magnitude)
    vect2_t velocity_vect; // current rectangular velocity vector (N, E)
    vect2_t polar_vel; //dir,mag
    vect2_t polar_wind_vect;
    double heading_true; //true heading degrees
    double batt_capacity_sec; //1800 seconds of power
    double drift_angle;
    double cross_track_err;
    double track_ang_err;
    double tas;
};

struct Sim_State_t
{
    int paused;
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
    int apu_gen_on;
    int eng_gen_on[8];
    IRU_t IRU[NUM_IRU];
    triple_mix_pos_t Triple_Mix_Pos;
};

extern waypoints flight_plan;
extern IRU_t IRU[NUM_IRU];
extern triple_mix_pos_t Triple_Mix_Pos;
extern Sim_State_t State_New;
extern Sim_State_t State_Old;

#endif