#include "navsystem.h"
#include "acfutils/geom.h"
#include "acfutils/helpers.h"
#include "acfutils/perf.h"
#include "datarefs.h"
#include "structs.h"
#include "variables.h"

void current_leg_compute(int i)
{
    if(IRU[i].flightplan.leg.from != IRU[i].flightplan.leg.to)
    {
        geo_pos2_t start, end, npos;
        int from, to;
        from = IRU[i].flightplan.leg.from;
        to = IRU[i].flightplan.leg.to;
        npos = {IRU[i].nav_pos.lat, IRU[i].nav_pos.lon};
        start = IRU[i].flightplan.waypoint_pos[from];
        end = IRU[i].flightplan.waypoint_pos[to];
        IRU[i].flightplan.curr_leg_dist = gc_distance(start, end);
        IRU[i].flightplan.curr_leg_crs = gc_point_hdg(start, end);
        const fpp_t proj = ortho_fpp_init(end, IRU[i].flightplan.curr_leg_crs, &wgs84, false);
        vect2_t ppos_proj = geo2fpp(npos, &proj);
        IRU[i].cross_track_err = ppos_proj.x;
        if(IRU[i].polar_flight_vel.y > 115)
        {
            IRU[i].track_ang_err = IRU[i].flightplan.curr_leg_crs - IRU[i].polar_ground_vel.x;
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
    if(from != to)
    {
        geo_pos2_t start, end, ppos;
        ppos = GEO3_TO_GEO2(IRU[i].nav_pos);
        start = IRU[i].flightplan.waypoint_pos[from];
        end = IRU[i].flightplan.waypoint_pos[to];
        IRU[i].flightplan.curr_leg_dist = gc_distance(start, end);
        IRU[i].flightplan.curr_leg_crs = gc_point_hdg(start, end);
        const fpp_t proj = ortho_fpp_init(end, IRU[i].flightplan.curr_leg_crs, &wgs84, false);
        vect2_t ppos_proj = geo2fpp(ppos, &proj);
        IRU[i].cross_track_err = ppos_proj.x;
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
    vect2_t wind_vect, next_flight_vect, next_ground_vect;
    wind_vect = IRU[i].wind_vect;

    int from, to;
    from = (IRU[i].flightplan.leg.from + 1) % 10 ;
    to = (IRU[i].flightplan.leg.to + 1) % 10 ;

    //compute the dir, mag of the next flight vector
    next_flight_vect = {gc_point_hdg(IRU[i].flightplan.waypoint_pos[from], 
                                     IRU[i].flightplan.waypoint_pos[to]), 
                        IRU[i].polar_flight_vel.y};
    next_ground_vect = vect2_add(next_flight_vect, wind_vect);
    double next_drift_ang, next_hdg, delta_hdg, gs1, gs2, xwind, tailwind, radius, distance;
    next_drift_ang =  normalize_hdg((next_ground_vect.x - next_flight_vect.x));
    next_hdg = next_flight_vect.x - next_drift_ang;
    delta_hdg = fabs(next_hdg - IRU[i].polar_flight_vel.x);

    gs1 = IRU[i].polar_ground_vel.y;
    gs2 = next_ground_vect.y;

    radius = (gs1 * gs2) / (EARTH_GRAVITY * tan(DEG2RAD(25)));
    distance = radius * tan(DEG2RAD(delta_hdg)/2);

    if (IRU[i].flightplan.curr_leg_dist < distance && !IRU[i].leg_switch)
    {
        IRU[i].leg_switch = 1;
        ++IRU[i].flightplan.leg.from;
        ++IRU[i].flightplan.leg.to;        
    }
    else 
    {
        IRU[i].leg_switch = 0;
    }
}