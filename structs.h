#ifndef __STRUCTS_H__
#define __STRUCTS_H__

#include "acfutils/geom.h"
#include "variables.h"
#include <string>

//sim state variables
struct leg_selector_t
{
    int from;
    int to;
};

struct flightplan_t
{
    geo_pos2_t waypoint_pos[10];
    leg_selector_t leg;
    double curr_leg_dist;
    double sel_leg_dist;
    double curr_leg_crs;
    double sel_leg_crs;
    double curr_leg_hdg;
    double next_leg_hdg;
    double time_to_fix;
};

struct dme_info_t
{
    int dme1_freq_hz;
    int dme2_freq_hz;
    double dme1_dist_comp;
    double dme2_dist_comp;
    int dme1_update_status;
    int dme2_update_status;

};

enum cdu_mode_enum_t
{
    TKGS = 0,
    HDGDA = 1,
    XTETKE = 2,
    POS = 3,
    WPT = 4,
    DISTIME = 5,
    WIND = 6,
    DSRTKSTS = 7
};

enum msu_mode_enum_t
{
    OFF = 0,
    STBY = 1,
    ALIGN = 2,
    NAV = 3,
    ATT = 4
};

enum bus_msg_enum
{
    MESSAGE_REMOTE,
    MESSAGE_UPDATE_VECT,
    MESSAGE_DME_DATA
};

struct IRU_t
{
	bool leg_switch;
	bool alert_flash;
	bool hold_on;

    int power_on; //is the unit on
    int mix_switch; //triple mix
    int batt_light; //battery light
    int waypoint_selector;
    int auto_man_switch;
    
    int warn_light;
	int alert_light;
    int remote_on;
    int remote_sender;
    int remote_receiver;
    cdu_mode_enum_t current_mode;
    msu_mode_enum_t msu_mode;
    flightplan_t flightplan;

    dme_info_t dme_data;
    geo_pos2_t update_pos2;
	geo_pos3_t update_pos3;
	geo_pos2_t hold_pos2;
    geo_pos3_t current_pos; //computed from align pos and fwd propogation
    geo_pos3_t align_pos; //input for alignment - no cheating, bad pos = bad nav
    geo_pos3_t dme_pos[9]; //0 = dme 1
    geo_pos3_t mix_pos;
    geo_pos3_t nav_pos;
    
    char curr_pos_dm[14];
    char mix_pos_dm[14];
    char nav_pos_dm[14];
    char waypoint_dm[14];

    vect3_t drift_vect_ecef;
    vect3_t vel_vect_ecef;
    vect3_t vel_vect3;
    vect3_t flight_vect_ecef;
    vect3_t wind_vect_ecef;
    vect3_t mix_vect_ecef;
    vect3_t current_pos_ecef;
	vect3_t vel_corr_vect3;

    vect2_t pos_drift_vect; //unit sensor bias rectangular velocities 
    vect2_t polar_pos_drift;//unit sensor bias polar coordinates
    vect2_t velocity_vect; // current rectangular velocity vector (N, E)
    vect2_t flight_vect;
    vect2_t polar_ground_vel; //dir,mag
    vect2_t polar_flight_vel;
    vect2_t polar_wind_vect;
    vect2_t mix_vect;
    vect2_t polar_mix_vel;
    vect2_t wind_vect_en;
	vect2_t wind_vect_hx;
    vect2_t pos_corr_dme[2];
	vect2_t vel_corr_vect2;

    double adc_alt;
    double heading_true; //true heading degrees
    double batt_capacity_sec; //1800 seconds of power
    double drift_angle;
    double cross_track_err;
    double track_ang_err;
    double tas;
    double time_in_nav;
	double time_since_update;
    };

struct Sim_State_t
{
    int paused;
    int pitot1_fail;
    int pitot2_fail;
    int static1_fail;
    int static2_fail;
    int num_generators;
    double runtime;
	double delta_time;
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

extern IRU_t IRU[NUM_IRU];
extern Sim_State_t State_New;
extern Sim_State_t State_Old;

//work on databus function for communication between units additionally the detection of changed waypoints for transmitting

#endif