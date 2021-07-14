#ifndef __VARIABLES_H__
#define __VARIABLES_H__
#include "sources.h"

#define NUM_IRU 3 //this is the number of IRUs the aircraft has

extern geo_pos3_t old_pos, pos3; //pos_3 will be the triple mix position
extern vect3_t ecef_pos_[NUM_IRU + 1];
extern bool first_floop;
extern bool drift;
extern double nav_start_time;

#endif