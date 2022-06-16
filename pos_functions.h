#ifndef __POS_FUNCTIONS_H__
#define __POS_FUNCTIONS_H__
#include "sources.h"
#include "variables.h"
#include "structs.h"
#include "functions.h"
#include "ecef_functions.h"

void set_drift_vector();

void update_drift_vect(int i);

void set_velocity_vect(int i);

void current_pos_update(int i);

void triple_mix();

void wind_vect_update(int i);



#endif