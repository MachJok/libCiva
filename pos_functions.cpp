#include "pos_functions.h"
#include "acfutils/geom.h"
#include "acfutils/helpers.h"
#include "acfutils/perf.h"
#include "acfutils/sysmacros.h"
#include "ecef_functions.h"
#include "structs.h"
#include <cmath>

void set_drift_vector()
{
	if(drift)
	{
		vect2_t drift_vector, drift_vect2;
		std::random_device rd;
		std::mt19937 gen(rd());
		double mean=0;
		double sigma = 0.675;
		for(int i = 0; i<NUM_IRU; ++i)
		{
			//normal dist centered on 0, 1.35, 0.41994

			std::normal_distribution<double> normal_dist(mean, sigma);

			IRU[i].pos_drift_vect = {KT2MPS(normal_dist(gen)),
									 KT2MPS(normal_dist(gen))}; //converts nm/hr to m/s

			IRU[i].polar_pos_drift = {dir2hdg(IRU[i].pos_drift_vect), 
									  vect2_abs(IRU[i].pos_drift_vect)};


			ASSERT(!isnan(IRU[i].pos_drift_vect.x));
			ASSERT(!isnan(IRU[i].pos_drift_vect.y));
			ASSERT(!isnan(IRU[i].polar_pos_drift.x));
			ASSERT(!isnan(IRU[i].polar_pos_drift.y));
		}
	}
}

void update_drift_vect(int i)
{
	vect3_t drift_vect3 = VECT2_TO_VECT3(IRU[i].polar_pos_drift, 0);
	IRU[i].drift_vect_ecef = vect3_add(
							compute_ecef_vel(IRU[i].current_pos, drift_vect3),
							IRU[i].vel_corr_vect3);
	ASSERT(!isnan(IRU[i].drift_vect_ecef.x));
	ASSERT(!isnan(IRU[i].drift_vect_ecef.y));
	ASSERT(!isnan(IRU[i].drift_vect_ecef.z));
}

void set_velocity_vect(int i)
{
	vect2_t sim_vel_vect2 = vect2_scmul(hdg2dir(State_New.true_trk), 
										State_New.ground_speed);
	
	IRU[i].velocity_vect = vect2_add(sim_vel_vect2, 
						   			vect2_add(IRU[i].pos_drift_vect, 
											  IRU[i].vel_corr_vect2));

	IRU[i].polar_ground_vel = {dir2hdg(IRU[i].velocity_vect), 
							   vect2_abs(IRU[i].velocity_vect)};

	IRU[i].vel_vect3 = VECT2_TO_VECT3(IRU[i].polar_ground_vel, 
									  FPM2MPS(State_New.vh_ind_fpm2));

	IRU[i].vel_vect_ecef = compute_ecef_vel(IRU[i].current_pos, 
											IRU[i].vel_vect3);

}

void current_pos_update(int i)
{
	update_drift_vect(i);
	set_velocity_vect(i);
	true_heading_update(i);
	wind_vect_update(i);

	vect3_t ppos_ecef = geo2ecef_mtr(IRU[i].current_pos, &wgs84);
	vect3_t vel_vect_dist = vect3_scmul(IRU[i].vel_vect_ecef, 
										State_New.frame_time);
	IRU[i].current_pos_ecef = vect3_add(ppos_ecef, vel_vect_dist);
	IRU[i].current_pos = ecef2geo(IRU[i].current_pos_ecef, &wgs84);

	deg_min(IRU[i].current_pos.lat, IRU[i].current_pos.lon, IRU[i].curr_pos_dm,
			 sizeof(IRU[i].curr_pos_dm));
	

}


void triple_mix()
{	
	double max_lat, max_lon, min_lat, min_lon, elev;

	//return max/min(IRU1,IRU2,IRU3)
	max_lat = std::max({IRU[0].current_pos.lat, IRU[1].current_pos.lat, 
						IRU[2].current_pos.lat});
	min_lat = std::min({IRU[0].current_pos.lat, IRU[1].current_pos.lat, 
						IRU[2].current_pos.lat});
	max_lon = std::max({IRU[0].current_pos.lon, IRU[1].current_pos.lon, 
						IRU[2].current_pos.lon});
	min_lon = std::min({IRU[0].current_pos.lon, IRU[1].current_pos.lon, 
						IRU[2].current_pos.lon});
	elev = IRU[0].current_pos.elev;
	geo_pos3_t max_pos;
	geo_pos3_t min_pos;
	max_pos = {max_lat, max_lon, elev};
	min_pos = {min_lat, min_lon, elev};
	vect3_t max_ecef, min_ecef, mid_ecef;
	
	max_ecef = geo2ecef_mtr(max_pos, &wgs84);
	min_ecef = geo2ecef_mtr(min_pos, &wgs84);

	mid_ecef = {(max_ecef.x + min_ecef.x) / 2., (max_ecef.y + min_ecef.y) / 2.,
				 (max_ecef.z + min_ecef.z) / 2.};
	mix_pos_ecef = mid_ecef;
	mix_pos = ecef2geo(mid_ecef, &wgs84);

	for (int i = 0; i < NUM_IRU; ++i)
	{
		
		IRU[i].mix_pos = mix_pos;
		deg_min(IRU[i].mix_pos.lat, IRU[i].mix_pos.lon, IRU[i].mix_pos_dm, 
				sizeof(IRU[i].mix_pos_dm));
		old_pos.lat = State_Old.IRU[i].mix_pos.lat;
		old_pos.lon = State_Old.IRU[i].mix_pos.lon;
		double trash_val, az, dist;
		az = {};
		dist = {};
		// IN:lat1, lon1, lat2, lon2 out: dist, az12, az21
		dist = gc_distance(GEO3_TO_GEO2(old_pos), GEO3_TO_GEO2(IRU[i].mix_pos));
		az = gc_point_hdg(GEO3_TO_GEO2(old_pos), GEO3_TO_GEO2(IRU[i].mix_pos));
		//geod.Inverse(old_pos.lat, old_pos.lon, IRU[i].mix_pos.lat, 
		//			IRU[i].mix_pos.lon, dist, az, trash_val);
		IRU[i].polar_mix_vel = {normalize_hdg(az), dist / State_New.frame_time};
		IRU[i].mix_vect = {vect2_scmul(hdg2dir(az), 
							dist / State_New.frame_time)};

	}
		
}

void wind_vect_update(int i)
{
	vect2_t wind_vect_en = vect2_sub(IRU[i].flight_vect, IRU[i].velocity_vect);
	vect2_t polar_wind_vect = 
			{floor(normalize_hdg(dir2hdg(wind_vect_en))),
			round(MPS2KT(vect2_abs(wind_vect_en)))};
	
	if (IRU[i].tas >= MIN_SPD && polar_wind_vect.y != 0) 
	{
		
		//gnd vector - flight vector
		IRU[i].wind_vect_en = wind_vect_en;
		double update_freq = 5.;
		
		FILTER_IN_LIN(IRU[i].polar_wind_vect.x, polar_wind_vect.x,
			State_New.frame_time, update_freq);
		FILTER_IN_LIN(IRU[i].polar_wind_vect.y, polar_wind_vect.y,
			State_New.frame_time, update_freq);
		
		IRU[i].drift_angle = IRU[i].polar_flight_vel.x - 
			IRU[i].polar_ground_vel.x > 90 ? 360 - (IRU[i].polar_flight_vel.x - 
			IRU[i].polar_ground_vel.x) : IRU[i].polar_flight_vel.x - 
			IRU[i].polar_ground_vel.x;
		
		double eff_tas = std::cos(DEG2RAD(IRU[i].drift_angle)) * 
						IRU[i].polar_flight_vel.y;

		FILTER_IN_LIN(IRU[i].wind_vect_hx.x, 
			(MPS2KT(eff_tas - IRU[i].polar_ground_vel.y)), 
			State_New.frame_time, update_freq);

		FILTER_IN_LIN(IRU[i].wind_vect_hx.y, 
			IRU[i].polar_wind_vect.y * 
			std::sin(DEG2RAD(polar_wind_vect.x - IRU[i].polar_flight_vel.x)), 
			State_New.frame_time, update_freq);
		//double hw = (int)(std::round(MPS2KT(eff_tas - IRU[i].polar_ground_vel.y)));
		vect3_t wind_vect3 = {IRU[i].wind_vect_en.x, IRU[i].wind_vect_en.y, 0};
		IRU[i].wind_vect_ecef = compute_ecef_vel(IRU[i].current_pos, wind_vect3);

		// FILTER_IN_LIN(IRU[i].drift_angle, IRU[i].polar_flight_vel.x - 
		//			 IRU[i].polar_ground_vel.x, State_New.frame_time, 1);

	}

	else
	{
		IRU[i].polar_wind_vect = {0};
	}
	
}

void pos_hold_transfer(IRU_t *iru_old, IRU_t *iru_new)
{
	if (iru_new->hold_on && !iru_old->hold_on && iru_new->current_mode == POS) 
	{
		iru_new->hold_pos2 = GEO3_TO_GEO2(iru_new->current_pos);
	}	
}

void pos_update_corr_vect(IRU_t *iru)
{

	/*
	1) current pos is held in hold_pos_2
	2) entered pos is held in update_pos_2
	3) compute distance and direction
	4) compute velocity based on time since last update
	5) compute correction vector
	6) apply correction vector to drift vector
	*/
	//meters
	double update_dist = gc_distance(iru->update_pos2, iru->hold_pos2);
	//degrees
	double direction = gc_point_hdg(iru->hold_pos2, iru->update_pos2);
	double time_sec = iru->time_since_update == 0 ? iru->time_in_nav : 
													iru->time_since_update;
	// meters per second
	double velocity = direction / time_sec;
	vect2_t update_vect2{direction, update_dist};
	vect3_t update_vect3_polar{VECT2_TO_VECT3(update_vect2, 0)};
	iru->vel_corr_vect2 = update_vect2;
	iru->vel_corr_vect3 = compute_ecef_vel(iru->current_pos, update_vect3_polar);

	
	
	//position update
	iru->current_pos = GEO2_TO_GEO3(geo_displace_dir(&wgs84, 
									GEO3_TO_GEO2(iru->current_pos), 
									hdg2dir(direction), update_dist),
									iru->adc_alt);
	iru->current_pos_ecef = geo2ecef_mtr(iru->current_pos, &wgs84);
}

void clear_corr_vect(IRU_t *iru)
{
	iru->vel_corr_vect3 = {0,0,0};
	iru->time_since_update = iru->time_in_nav;
}