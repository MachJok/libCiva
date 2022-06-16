#ifndef __VARIABLES_H__
#define __VARIABLES_H__
#include "sources.h"

#define NUM_IRU 3 //this is the number of IRUs the aircraft has should be in range [1,3]
#define MIN_SPD KT2MPS(115)
extern geo_pos3_t old_pos, mix_pos; //pos_3 will be the triple mix position
extern vect3_t mix_pos_ecef;
extern bool first_floop, drift, adc1_valid, adc2_valid;;
extern char sim_pos_dm[32];



#endif