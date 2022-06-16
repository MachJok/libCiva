#include "functions.h"
#include "acfutils/geom.h"
#include "acfutils/assert.h"
#include "acfutils/helpers.h"
#include "acfutils/log.h"
#include "acfutils/math.h"
#include "acfutils/math_core.h"
#include "acfutils/perf.h"
#include "acfutils/sysmacros.h"
#include "datarefs.h"
#include "structs.h"
#include "variables.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <random>
#include <sys/types.h>
#include <type_traits>
#include <array>


geo_pos3_t old_pos{0}, mix_pos{0};
vect3_t ecef_pos_[NUM_IRU + 1]{0}, nav_pos_ecef{0}, mix_pos_ecef{0};
bool adc1_valid{0};
bool adc0_valid{0};
bool triple_mix_on{false};
double mix_weight{0};
bool drift = true;
char output[32] = {0};

static geo_pos2_t waypoint_old[9];
static geo_pos2_t waypoint_bus[9];
std::array<double, 9> waypoint_lat_old;
std::array<double, 9> waypoint_lon_old;
std::array<double, 9> waypoint_lat;
std::array<double, 9> waypoint_lon;

void set_true_hdg_offset()
{
	for (int i = 0; i < NUM_IRU; ++i)
	{
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<double> uniform_dist(0, 360);
		IRU[i].heading_true = uniform_dist(gen);
	}
}

void iru_init()
{
	drift = true;
	set_drift_vector();
	set_true_hdg_offset();
	//set_true_hdg_offset();
	
	for (int i = 0; i < NUM_IRU; ++i)
	{
		IRU[i].batt_capacity_sec = 900;
		IRU[i].flightplan.leg.to = 1;
		
		logMsg("IRU-%i Drift Vector: (%03f m/s E, %03f m/s N)", i+1, 
										IRU[i].pos_drift_vect.x, 
										IRU[i].pos_drift_vect.y);
		logMsg("IRU-%i init True Heading: %f", i+1, IRU[i].heading_true);
	}
	logMsg("IRU INIT COMPLETE");
}
//computes the actual N and E velocities of the aircraft in the sim to compute IRU velocity vectors m/s
//simulates accelerometer integration for velocity

void true_heading_update(int i)
{
	IRU[i].heading_true = State_New.hdg_true;
}



void triple_mix_logic(int i)
{
	if(!IRU[i].mix_switch || IRU[i].msu_mode != NAV || NUM_IRU < 3)
	{
		IRU[i].mix_pos = {0};
		IRU[i].mix_vect = {0};
		IRU[i].polar_mix_vel = {0};
	}
	
	if(NUM_IRU > 2 && (IRU[i].mix_switch && IRU[i].msu_mode == NAV))
	{	
		triple_mix();
		nav_pos_ecef = geo2ecef_mtr(IRU[i].nav_pos, &wgs84);

		mix_pos_ecef = geo2ecef_mtr(IRU[i].mix_pos, &wgs84);

		FILTER_IN(mix_weight, 1, State_New.frame_time, 30/State_New.frame_time);
		vect3_t new_nav_pos;

		new_nav_pos.x = wavg(nav_pos_ecef.x, mix_pos_ecef.x, mix_weight);
		new_nav_pos.y = wavg(nav_pos_ecef.y, mix_pos_ecef.y, mix_weight);
		new_nav_pos.z = wavg(nav_pos_ecef.z, mix_pos_ecef.z, mix_weight);

		IRU[i].nav_pos = ecef2geo(new_nav_pos, &wgs84);
		deg_min(IRU[i].nav_pos.lat, IRU[i].nav_pos.lon, IRU[i].nav_pos_dm, sizeof(IRU[i].nav_pos_dm));
	}
	else if(NUM_IRU < 3 || (!IRU[i].mix_switch || IRU[i].msu_mode != NAV))
	{
		triple_mix_on = false;
		IRU[i].nav_pos = IRU[i].current_pos;
		deg_min(IRU[i].nav_pos.lat, IRU[i].nav_pos.lon, IRU[i].nav_pos_dm, sizeof(IRU[i].nav_pos_dm));
		FILTER_IN(mix_weight, 0, State_New.frame_time, 30);
	}	
}



void electrical_source()
{
	int n = sizeof(State_New.eng_gen_on) / sizeof(State_New.eng_gen_on[0]);
	int num_gens = std::count(State_New.eng_gen_on, State_New.eng_gen_on + n, 1);
	int apu_gen_on = State_New.apu_gen_on;
	for(int i = 0; i < NUM_IRU; ++i)
	{
		if(!apu_gen_on && !num_gens && IRU[i].msu_mode != OFF)
		{
			FILTER_IN_LIN(IRU[i].batt_capacity_sec, 0, State_New.frame_time, 1);
			IRU[i].power_on = 0;//CHANGE THIS TO BE CONTROLED BY BATT AND GENERATOR POWER
			IRU[i].batt_light = !IRU[i].power_on;
		}

		if(IRU[i].batt_capacity_sec < 900 && (num_gens >= 1 || apu_gen_on))
		{   
			FILTER_IN_LIN(IRU[i].batt_capacity_sec, 900, State_New.frame_time, 0.1);
			IRU[i].power_on = 1;
			IRU[i].batt_light = !IRU[i].power_on;		   
		}
	}

}

bool seconds_update()
{
	bool second_update{};
	int time_int = std::floor(State_New.runtime);
	double delta_time{};
	delta_time = State_New.delta_time;
	if(delta_time >= 1.0 && !State_New.paused)
	{
		second_update = true;
		return second_update;
	}

	else
	{
		second_update = false;
		return second_update;
	}

}

void warn_light_logic(int i)
{
	bool passed_wpt{};
	if(State_Old.IRU[i].flightplan.curr_leg_dist < IRU[i].flightplan.curr_leg_dist &&
	   State_Old.IRU[i].flightplan.leg.from == IRU[i].flightplan.leg.from &&
	   State_Old.IRU[i].flightplan.leg.to == IRU[i].flightplan.leg.to
	)
	{
		passed_wpt = true;
		IRU[i].alert_flash = seconds_update();

	}
	else 
	{
		passed_wpt = false;
		IRU[i].alert_flash = 0;
	}
	
	if(!IRU[i].auto_man_switch && passed_wpt)
	{
		IRU[i].alert_light = int(IRU[i].alert_flash);
	}
	else
	{
		IRU[i].alert_light = IRU[i].flightplan.time_to_fix <= 120 ? 1 : 0;
	}
}

void debug_set_pos() //shortcut for entering position
{
	if (first_floop) 
	{
		for(int i = 0; i < NUM_IRU; ++i)
		{
			IRU[i].current_pos.lat = State_New.lat;
			IRU[i].current_pos.lon = State_New.lon;
			IRU[i].current_pos.elev = State_New.elev_mtr;
			IRU[i].msu_mode = NAV;
			IRU[i].mix_switch = 0;
		}	
	}
}

void waypoint_selector_clamp(int i)
{
	if (IRU[i].waypoint_selector > 9) {
	IRU[i].waypoint_selector = std::fmod(IRU[i].waypoint_selector, 10);
	}
}

double wgs84_rad(double lat_deg)
{
	double radius = geo2ecef_mtr({0,0,0}, &wgs84).y;
	return radius;
}

void wpt_deg_min(int i)
{
	geo_pos2_t pos;
	int wpt_num;

	wpt_num = IRU[i].waypoint_selector;
	pos = IRU[i].flightplan.waypoint_pos[wpt_num];

	deg_min(pos.lat, pos.lon, IRU[i].waypoint_dm, sizeof(IRU[i].waypoint_dm));
}

void deg_min(double lat, double lon, char *output, size_t cap)
{
	double int_dump;
	double lat_val = abs(round(lat*600)/600);
	double lon_val = abs(round(lon*600)/600);
	u_int8_t lat_deg = floor(lat_val);
	u_int8_t lon_deg = floor(lon_val);
	short lat_min = (modf(lat_val, &int_dump) * 600) > 599.49 ? 0 : trunc((modf(lat_val, &int_dump) * 600));
	short lon_min = (modf(lon_val, &int_dump) * 600) > 599.49 ? 0 : trunc((modf(lon_val, &int_dump) * 600));
	
	char NS = (lat >= 0 ? 'N' : 'S');
	char EW = (lon >= 0 ? 'E' : 'W');
	snprintf(output, cap, "%02u%03u%03u%03u%c%c", 
			lat_deg,  lat_min, lon_deg, lon_min, NS, EW);  
}

void adc_data_in(int i)
{
	double hdg = IRU[i].heading_true;
	double tas = IRU[i].tas;
	IRU[i].flight_vect = vect2_scmul(hdg2dir(hdg), tas);
	IRU[i].polar_flight_vel = {hdg, tas};
	IRU[i].flight_vect_ecef = compute_ecef_vel(IRU[i].nav_pos, IRU[i].vel_vect_ecef);

	if((i == 2 || i ==0) && IRU[i].msu_mode == NAV)
	{
		IRU[i].adc_alt = FEET2MET(State_New.alt_ft_[0]);
		IRU[i].tas = KT2MPS(State_New.tas_kt_[0]);
	}
	else if(IRU[i].msu_mode == NAV)
	{
		IRU[i].adc_alt = FEET2MET(State_New.alt_ft_[1]);
		IRU[i].tas = KT2MPS(State_New.tas_kt_[1]);
	}
	//integrate each frame vertical speed to compute altitude
  
}

bool waypoint_comp(geo_pos2_t wpt1, geo_pos2_t wpt2)
{
	bool lat_same = wpt1.lat == wpt2.lat ? true : false;
	bool lon_same = wpt1.lon == wpt2.lon ? true : false;
	return ((lat_same && lon_same) ? true : false);
}

void waypoint_transfer(int i)
{
	
	if(IRU[i].remote_sender)
	{
		for (int j = 1; j < 10; ++j) 
		{
			
			waypoint_lat_old[j-1] = State_Old.IRU[i].flightplan.waypoint_pos[j].lat;
			waypoint_lon_old[j-1] = State_Old.IRU[i].flightplan.waypoint_pos[j].lon;
			waypoint_lat[j-1] = IRU[i].flightplan.waypoint_pos[j].lat;
			waypoint_lon[j-1] = IRU[i].flightplan.waypoint_pos[j].lon;
		}
	}

	#if NUM_IRU == 1
		continue;
	#elif NUM_IRU == 2
		if(IRU[0].remote_sender && 
			(IRU[i].remote_on && IRU[i].remote_receiver == 1) && 
			!waypoint_comp(
				IRU[0].flightplan.waypoint_pos[IRU[0].waypoint_selector],
				IRU[i].flightplan.waypoint_pos[IRU[0].waypoint_selector]) &&
		)
		{
			int wpt_num = IRU[0].waypoint_selector;
			IRU[i].flightplan.waypoint_pos[wpt_num].lat = 
				IRU[0].flightplan.waypoint_pos[wpt_num].lat;
			IRU[i].flightplan.waypoint_pos[wpt_num].lon = 
				IRU[0].flightplan.waypoint_pos[wpt_num].lon;
		}
		if(IRU[1].remote_sender &&
			(IRU[i].remote_on && IRU[i].remote_receiver == 1) &&
			!waypoint_comp(
				IRU[1].flightplan.waypoint_pos[IRU[1].waypoint_selector],
				IRU[i].flightplan.waypoint_pos[IRU[1].waypoint_selector]))
		{
			int wpt_num = IRU[1].waypoint_selector;
			IRU[i].flightplan.waypoint_pos[wpt_num].lat = 
				IRU[1].flightplan.waypoint_pos[wpt_num].lat;
			IRU[i].flightplan.waypoint_pos[wpt_num].lon = 
				IRU[1].flightplan.waypoint_pos[wpt_num].lon;
		}
	#elif NUM_IRU == 3
		if(IRU[0].remote_sender &&
			(IRU[i].remote_on && IRU[i].remote_receiver == 1) &&
			!waypoint_comp(
				IRU[0].flightplan.waypoint_pos[IRU[0].waypoint_selector],
				IRU[i].flightplan.waypoint_pos[IRU[0].waypoint_selector])
			)
		{
			int wpt_num = IRU[0].waypoint_selector;
			IRU[i].flightplan.waypoint_pos[wpt_num].lat =
				IRU[0].flightplan.waypoint_pos[wpt_num].lat;
			IRU[i].flightplan.waypoint_pos[wpt_num].lon =
				IRU[0].flightplan.waypoint_pos[wpt_num].lon;
		}
		if(IRU[1].remote_sender &&
			(IRU[i].remote_on && IRU[i].remote_receiver == 1) &&
			!waypoint_comp(
				IRU[1].flightplan.waypoint_pos[IRU[1].waypoint_selector],
				IRU[i].flightplan.waypoint_pos[IRU[1].waypoint_selector])
			)
		{
			int wpt_num = IRU[1].waypoint_selector;
			IRU[i].flightplan.waypoint_pos[wpt_num].lat =
				IRU[1].flightplan.waypoint_pos[wpt_num].lat;
			IRU[i].flightplan.waypoint_pos[wpt_num].lon =
				IRU[1].flightplan.waypoint_pos[wpt_num].lon;
		}
		if(IRU[2].remote_sender &&
			(IRU[i].remote_on && IRU[i].remote_receiver == 1) &&
			!waypoint_comp(
				IRU[2].flightplan.waypoint_pos[IRU[2].waypoint_selector],
				IRU[i].flightplan.waypoint_pos[IRU[2].waypoint_selector])
			)
		{
			int wpt_num = IRU[2].waypoint_selector;
			IRU[i].flightplan.waypoint_pos[wpt_num].lat =
				IRU[2].flightplan.waypoint_pos[wpt_num].lat;
			IRU[i].flightplan.waypoint_pos[wpt_num].lon =
				IRU[2].flightplan.waypoint_pos[wpt_num].lon;
		}
	#endif					  
}

void remote_priority(int i)
{
	#if NUM_IRU == 1
		continue;

	#elif NUM_IRU == 2
		std::array <int, 2> iru_remote_old {State_Old.IRU[0].remote_on, State_Old.IRU[1].remote_on};
		std::array <int, 2> iru_remote {IRU[0].remote_on, IRU[1].remote_on};
		if (iru_remote != iru_remote_old)
		{

			if (IRU[0].remote_on && !IRU[1].remote_on)
			{
				IRU[0].remote_sender = 1;
				IRU[0].remote_receiver = 0;
				IRU[1].remote_sender = 0;
				IRU[1].remote_receiver = 1;
			}
			if (!IRU[0].remote_on && IRU[1].remote_on)
			{
				IRU[0].remote_sender = 0;
				IRU[0].remote_receiver = 1;
				IRU[1].remote_sender = 1;
				IRU[1].remote_receiver = 0;
			}
		}
		
	#elif NUM_IRU == 3

		if (IRU[0].remote_on && !IRU[1].remote_on && !IRU[2].remote_on)
		{
			IRU[0].remote_sender = 1;
			IRU[0].remote_receiver = 0;

			IRU[1].remote_sender = 0;
			IRU[1].remote_receiver = 1;

			IRU[2].remote_sender = 0;
			IRU[2].remote_receiver = 1;			
		}
		else if (!IRU[0].remote_on && IRU[1].remote_on && !IRU[2].remote_on)
		{
			IRU[0].remote_sender = 0;
			IRU[0].remote_receiver = 1;

			IRU[1].remote_sender = 1;
			IRU[1].remote_receiver = 0;
			
			IRU[2].remote_sender = 0;
			IRU[2].remote_receiver = 1;			
		}
		else if (!IRU[0].remote_on && !IRU[1].remote_on && IRU[2].remote_on)
		{
			IRU[0].remote_sender = 0;
			IRU[0].remote_receiver = 1;

			IRU[1].remote_sender = 0;
			IRU[1].remote_receiver = 1;
			
			IRU[2].remote_sender = 1;
			IRU[2].remote_receiver = 0;			
		}
		
		
		for (int j = 0; j < 3; ++j)
			{
				if (!IRU[j].remote_on && IRU[j].remote_sender == 1)
				{
					IRU[0].remote_sender = 0;
					IRU[0].remote_receiver = 0;

					IRU[1].remote_sender = 0;
					IRU[1].remote_receiver = 0;

					IRU[2].remote_sender = 0;
					IRU[2].remote_receiver = 0;	
				}
			}
						  
	#endif
	waypoint_transfer(i);

}


void timer_functions(IRU_t *iru, double delta_time)
{
  if(iru->msu_mode > ALIGN &&
	 iru->batt_capacity_sec > 0 && iru->power_on)
  {
	iru->time_in_nav += delta_time;
	iru->time_since_update += delta_time;
  }
}

