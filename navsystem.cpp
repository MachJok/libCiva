#include "navsystem.h"
#include "acfutils/geom.h"
#include "acfutils/helpers.h"
#include "acfutils/perf.h"
#include "datarefs.h"
#include "ecef_functions.h"
#include "structs.h"
#include "variables.h"
#include <cmath>

bool stored_0{false};

void current_leg_compute(int i)
{
	if(IRU[i].flightplan.leg.from != IRU[i].flightplan.leg.to)
	{
		geo_pos2_t start, end, npos;
		int from, to;
		from = IRU[i].flightplan.leg.from;
		to = IRU[i].flightplan.leg.to;
		npos = GEO3_TO_GEO2(IRU[i].nav_pos);
		if(from == 0 && !stored_0)
		{
			start = IRU[i].flightplan.waypoint_pos[from];
			stored_0 = true;
		}
		else
		{
			start = IRU[i].flightplan.waypoint_pos[from];
			stored_0 = false;
		}
		end = IRU[i].flightplan.waypoint_pos[to];
		IRU[i].flightplan.curr_leg_dist = gc_distance(npos, end);
		IRU[i].flightplan.curr_leg_crs = isnan(gc_point_hdg(start, end)) ? 0 : gc_point_hdg(start, end);

		IRU[i].flightplan.time_to_fix = IRU[i].flightplan.curr_leg_dist / IRU[i].polar_ground_vel.y;

		IRU[i].cross_track_err = 
		isnan(crosstrack_dist(GEO2_TO_GEO3(start, 0), GEO2_TO_GEO3(end, 0), IRU[i].nav_pos)) ? 0 : 
		crosstrack_dist(GEO2_TO_GEO3(start, 0), GEO2_TO_GEO3(end, 0), IRU[i].nav_pos);

		if(IRU[i].polar_flight_vel.y > MIN_SPD)
		{
			vect3_t gnd_vect = VECT2_TO_VECT3(IRU[i].velocity_vect, 0);
			vect2_t fp_polar = {IRU[i].flightplan.curr_leg_crs, IRU[i].polar_ground_vel.y};
			vect3_t fp_vect = {vect2_scmul(hdg2dir(fp_polar.x),fp_polar.y).x, 
							   vect2_scmul(hdg2dir(fp_polar.x),fp_polar.y).y, 0};

			double error = RAD2DEG(acos(vect3_dotprod(gnd_vect, fp_vect) / 
								   (vect3_abs(fp_vect) * vect3_abs(gnd_vect))));
			vect3_t sign_vect = vect3_xprod(gnd_vect, fp_vect);
			int sign = sign_vect.z > 0 ? 1 : -1;
			IRU[i].track_ang_err = sign * error;							  

		}
		else
		{
			IRU[i].track_ang_err = 0;
		}
	}
	
	else
	{
		IRU[i].cross_track_err = 0;
		IRU[i].flightplan.curr_leg_dist = 0;
		IRU[i].flightplan.curr_leg_crs = 0;		
	}
}

void leg_compute(int i, int from, int to)
{
	if(from != to && to != 0)
	{
		geo_pos2_t start, end;
		start = IRU[i].flightplan.waypoint_pos[from];
		end = IRU[i].flightplan.waypoint_pos[to];
		IRU[i].flightplan.sel_leg_dist = gc_distance(start, end);
		IRU[i].flightplan.sel_leg_crs = gc_point_hdg(start, end);

		IRU[i].flightplan.time_to_fix = IRU[i].flightplan.sel_leg_dist / 
										IRU[i].polar_ground_vel.y;

		IRU[i].cross_track_err = crosstrack_dist(GEO2_TO_GEO3(start, 0), 
								 				 GEO2_TO_GEO3(end, 0), 
												 IRU[i].current_pos);

		// const fpp_t proj = ortho_fpp_init(end, IRU[i].flightplan.curr_leg_crs, &wgs84, false);
		// vect2_t ppos_proj = geo2fpp(ppos, &proj);
		// IRU[i].cross_track_err = ppos_proj.x;
	}
	else
	{
		IRU[i].cross_track_err = 0;
		IRU[i].flightplan.curr_leg_dist = 0;
		IRU[i].flightplan.curr_leg_crs = 0;		
	}
}

void leg_switch(int i)
{
	vect2_t wind_vect{}, next_flight_vect{}, next_ground_vect{};
	wind_vect = IRU[i].wind_vect_en;

	int from{}, to{};
	from = IRU[i].flightplan.leg.from;
	to = IRU[i].flightplan.leg.to;
	double bank_limit_rad = DEG2RAD(25);
	double bank_rate_rad = DEG2RAD(3);
	//compute the dir, mag of the next flight vector
	next_flight_vect = {gc_point_hdg(IRU[i].flightplan.waypoint_pos[from + 1], 
									 IRU[i].flightplan.waypoint_pos[to + 1]), 
									 IRU[i].polar_flight_vel.y};
	next_ground_vect = vect2_add(next_flight_vect, wind_vect);
	double next_drift_ang{}, next_hdg{}, delta_hdg{}, gs1{}, gs2{}, xwind{},
		tailwind{}, radius{}, distance{};
	next_drift_ang =  normalize_hdg((next_ground_vect.x - next_flight_vect.x));
	next_hdg = next_flight_vect.x - next_drift_ang;
	delta_hdg = fabs(next_hdg - IRU[i].polar_flight_vel.x);

	gs1 = IRU[i].polar_ground_vel.y;
	gs2 = next_ground_vect.y;

	radius = (gs1 * gs2) / 
		(earth_gravity_accurate(IRU[i].current_pos.lat, IRU[i].current_pos.elev) 
			* tan(bank_limit_rad));
	double bank_rate_correction_distance = (bank_limit_rad / bank_rate_rad) * 
											((gs1 + gs2)/2.0);
	distance = radius * tan(DEG2RAD(delta_hdg)/2) + bank_rate_correction_distance;

	bool in_switch_dist = (IRU[i].flightplan.curr_leg_dist <= distance + 50. &&
		IRU[i].flightplan.curr_leg_dist > distance - 50.) ? true : false;

	if (in_switch_dist && IRU[i].auto_man_switch && !IRU[i].leg_switch)
	{
		IRU[i].leg_switch = true;
		++IRU[i].flightplan.leg.from;
		++IRU[i].flightplan.leg.to;
	}
	else
	{
		IRU[i].leg_switch = false;
	}
}

double crosstrack_dist(geo_pos3_t wpt1, geo_pos3_t wpt2, geo_pos3_t nav_pos)
{
	double delta_13, theta_13, theta_12, crs_13, crs_12;
	vect3_t p1, p2, pn;
	geo_pos3_t lat;
	p1 = geo2ecef_mtr(wpt1, &wgs84);
	p1 = geo2ecef_mtr(wpt1, &wgs84);
	pn = geo2ecef_mtr(nav_pos, &wgs84);
	delta_13 = acos(vect3_dotprod(p1, pn)/(vect3_abs(p1)*vect3_abs(pn)));
	crs_13 = DEG2RAD(gc_point_hdg(GEO3_TO_GEO2(wpt1), GEO3_TO_GEO2(nav_pos)));
	crs_12 = DEG2RAD(gc_point_hdg(GEO3_TO_GEO2(wpt1), GEO3_TO_GEO2(wpt2)));
	lat = {nav_pos.lat,0,0};
	return asin(sin(delta_13) * sin(crs_13 - crs_12))*vect3_abs(geo2ecef_mtr(lat, &wgs84));
}