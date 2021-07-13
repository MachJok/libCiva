#ifndef __FUNCTIONS_H__
#define __FUNCTIONS_H__
#include "sources.h"
#include "structs.h"
#include "variables.h"
#include "datarefs.h"
#include "GeographicLib.h"
#include <acfutils/geom.h>
#include <acfutils/perf.h>


//sets random drift vector using MT19937 and stores to IRU, called in iru_init
void set_drift_vector();

//randomizes the heading on load
void set_true_hdg_offset();

//sets the initial condition of the IRU
void iru_init();

//computes the velocity vector of the aicraft in sim and adds drift vector
void set_velocity_vect(int i);

//sets the platform true heading
void true_heading_update(int i);

//computes the wind vector for the INS, subject to loss of ADC tas input
void wind_vect_update(int i);

//calls set_velocity_vect to compute new position and checks for triple mix flags
void current_pos_update(int i);

//simulated aligning of the IRU
void align_iru(int i);

//checks generator source and selects ac source or internal 30 minute battery
void electrical_source();

//computes the average of the 3 IRU positions
void triple_mix();

//cheater function to set current pos and triple mix
void debug_set_pos();

#endif