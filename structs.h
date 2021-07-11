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


struct IRU_t
{
    double nav_mode; //nav mode on
    double power_on; //is the unit on
    double mix_switch; //triple mix
    geo_pos3_t current_pos; //computed from align pos and fwd propogation
    geo_pos3_t align_pos; //input for alignment - no cheating, bad pos = bad nav
    //triple_mix_pos_t triple_mix_pos_wgs;
    vect2_t pos_drift_vect; //unit sensor bias rectangular velocities 
    vect2_t polar_pos_drift;//unit sensor bias polar coordinates
    vect2_t correction_vect; //for DME updating (direction, magnitude)
    vect2_t velocity_vect; // current rectangular velocity vector (N, E)
    vect2_t polar_vel; //dir,mag
    double heading_true; //true heading degrees
    double batt_capacity_sec; //1800 seconds of power
    double batt_self_test; //battery self test light during alignment
};

struct Sim_State_t
{
    int paused;
    double runtime;
    double frame_time;
    double track;
    double ground_speed;
    double lat;
    double lon;
    double hdg_true;
    int apu_gen_on;
    int eng_gen_on[8];
    IRU_t IRU[NUM_IRU];
    triple_mix_pos_t Triple_Mix_Pos;
};

extern IRU_t IRU[NUM_IRU];
extern triple_mix_pos_t Triple_Mix_Pos;
extern Sim_State_t State_New;
extern Sim_State_t State_Old;

#endif