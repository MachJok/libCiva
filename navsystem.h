#ifndef __NAVSYSTEM_H__
#define __NAVSYSTEM_H__

#include "sources.h"
#include "structs.h"
#include "functions.h"

void current_leg_compute(int i);

void leg_compute(int i, int from, int to);

void leg_switch(int i);

double crosstrack_dist(geo_pos3_t wpt1, geo_pos3_t wpt2, geo_pos3_t nav_pos);


#endif