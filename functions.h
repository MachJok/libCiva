#ifndef __FUNCTIONS_H__
#define __FUNCTIONS_H__
#include "sources.h"
#include "structs.h"
#include "variables.h"
#include "datarefs.h"
#include "GeographicLib.h"
#include <acfutils/geom.h>
#include <acfutils/perf.h>

void set_drift_vector();

void set_true_hdg_offset();

void iru_init();

void set_velocity_vect(int i);

void current_pos_update(int i);

void align_iru(int i);

void electrical_source();

void triple_mix();

void debug_set_pos();

#endif