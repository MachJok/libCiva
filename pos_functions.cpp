#include "pos_functions.h"
#include "acfutils/geom.h"
#include "ecef_functions.h"
#include "structs.h"

void set_drift_vector()
{
    if(drift)
    {   
        vect2_t drift_vector;
        double drift_mag{};
        std::random_device rd;
        std::mt19937 gen(rd());
        for(int i = 0; i<NUM_IRU; ++i)
        {
            //normal dist centered on 0, 1.35, 0.41994
            std::normal_distribution<double> normal_dist(0.57, 0.39);

            drift_mag = KT2MPS(normal_dist(gen)); //converts nm/hr to m/s

            std::mt19937 gen2(rd());
            std::uniform_real_distribution<double> uniform_dist(0, 360);
            double drift_dir = uniform_dist(gen2);
            if(drift_mag < 0)
            {
                drift_dir = normalize_hdg(drift_dir + 180);
                drift_mag = -drift_mag;
            }            
            //vel E and N
            IRU[i].pos_drift_vect = vect2_scmul(hdg2dir(drift_dir), drift_mag);
            IRU[i].polar_pos_drift = {drift_dir, drift_mag};


            ASSERT(!isnan(IRU[i].pos_drift_vect.x));
            ASSERT(!isnan(IRU[i].pos_drift_vect.y));
            ASSERT(!isnan(IRU[i].drift_vect_ecef.x));
            ASSERT(!isnan(IRU[i].drift_vect_ecef.y));
            ASSERT(!isnan(IRU[i].drift_vect_ecef.z));
        }
    }
}

void update_drift_vect(int i)
{
    vect3_t drift_vect3 = VECT2_TO_VECT3(IRU[i].polar_pos_drift, 0);
    IRU[i].drift_vect_ecef = compute_ecef_vel(IRU[i].current_pos, drift_vect3);
}

void set_velocity_vect(int i)
{
    vect2_t sim_vel_vect2 = vect2_scmul(hdg2dir(State_New.true_trk), 
                                        State_New.ground_speed);
    IRU[i].velocity_vect = vect2_add(sim_vel_vect2, IRU[i].pos_drift_vect);

    IRU[i].polar_ground_vel = {dir2hdg(IRU[i].velocity_vect), vect2_abs(IRU[i].velocity_vect)};

    IRU[i].vel_vect3 = VECT2_TO_VECT3(IRU[i].polar_ground_vel, FPM2MPS(State_New.vh_ind_fpm2));

    IRU[i].vel_vect_ecef = compute_ecef_vel(IRU[i].current_pos, IRU[i].vel_vect3);

}

void current_pos_update(int i)
{   
    update_drift_vect(i);
    set_velocity_vect(i);
    true_heading_update(i);
    wind_vect_update(i);

    IRU[i].time_in_nav = State_New.runtime - nav_start_time;

    vect3_t ppos_ecef = geo2ecef_mtr(IRU[i].current_pos, &wgs84);
    vect3_t vel_vect_dist = vect3_scmul(IRU[i].vel_vect_ecef, State_New.frame_time) ;
    IRU[i].current_pos_ecef = vect3_add(ppos_ecef, vel_vect_dist);
    IRU[i].current_pos = ecef2geo(IRU[i].current_pos_ecef, &wgs84);

    deg_min(IRU[i].current_pos.lat, IRU[i].current_pos.lon, IRU[i].curr_pos_dm,
             sizeof(IRU[i].curr_pos_dm));
    
    IRU[i].flightplan.waypoint_pos[0] = {IRU[i].nav_pos.lat, IRU[i].nav_pos.lon};

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
        //            IRU[i].mix_pos.lon, dist, az, trash_val);
        IRU[i].polar_mix_vel = {normalize_hdg(az), dist / State_New.frame_time};
        IRU[i].mix_vect = {vect2_scmul(hdg2dir(az), 
                            dist / State_New.frame_time)};

    }
        
}


void wind_vect_update(int i)
{
    if(IRU[i].tas > MIN_SPD )
    {
        //gnd vector - flight vector
        IRU[i].wind_vect = vect2_sub(IRU[i].flight_vect, 
                                        IRU[i].velocity_vect);
        IRU[i].polar_wind_vect = {floor(normalize_hdg(dir2hdg(IRU[i].wind_vect))), 
                                    round(MPS2KT(vect2_abs(IRU[i].wind_vect)))};
        vect3_t wind_vect3 = {IRU[i].wind_vect.x, IRU[i].wind_vect.y, 0};
        IRU[i].wind_vect_ecef = compute_ecef_vel(IRU[i].current_pos, wind_vect3);
        if(IRU[i].polar_wind_vect.y > 0)
        {
            FILTER_IN_LIN(IRU[i].drift_angle, IRU[i].polar_flight_vel.x - 
                        IRU[i].polar_ground_vel.x, State_New.frame_time, 1);
        }
        else
        {
            IRU[i].polar_wind_vect = {0};
        }
    }

    else
    {
        IRU[i].polar_wind_vect = {0};
    }

}
