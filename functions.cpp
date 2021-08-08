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


geo_pos3_t old_pos{0}, mix_pos{0};
vect3_t ecef_pos_[NUM_IRU + 1]{0}, nav_pos_ecef{0}, mix_pos_ecef{0};
bool adc1_valid{0};
bool adc0_valid{0};
bool triple_mix_on{false};
double mix_weight{0};
bool drift = true;
char output[32] = {0};


geo_pos2_t amy_displace(const ellip_t *ellip, geo_pos2_t pos, double truehdg, double dist) {
    ASSERT(ellip);
    if(IS_NULL_GEO_POS2(pos)) return NULL_GEO_POS2;
    
    static const vect3_t up = VECT3(0, 0, 1);
    double dist_r = dist / EARTH_MSL;
    double angle = DEG2RAD(truehdg);
    
    vect3_t norm_start = vect3_unit(geo2ecef_mtr(GEO2_TO_GEO3(pos, 0), ellip), NULL);
    vect3_t east = vect3_unit(vect3_xprod(up, norm_start), NULL);
    vect3_t north = vect3_unit(vect3_xprod(norm_start, east), NULL);
    vect3_t dir =  vect3_add(vect3_scmul(north, cos(angle)), vect3_scmul(east, sin(angle)));
    vect3_t norm_end = vect3_add(vect3_scmul(norm_start, cos(dist_r)), vect3_scmul(dir, sin(dist_r)));
    
    geo_pos3_t out = ecef2geo(vect3_set_abs(norm_end, EARTH_MSL), ellip);
    return GEO3_TO_GEO2(out);
}

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
        }
    }
    if(!drift)
    {
        for(int i = 0; i < NUM_IRU; ++i)
        {
            IRU[i].pos_drift_vect = {0};
            IRU[i].polar_pos_drift = {0};
        }
    }
}

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
        IRU[i].current_pos.elev = 0;
        IRU[i].align_pos.elev = 0;
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
void set_velocity_vect(int i)
{
    vect2_t sim_vel_vect;
    sim_vel_vect = vect2_scmul(hdg2dir(State_New.true_trk), 
                                        State_New.ground_speed);
    IRU[i].velocity_vect = vect2_add(sim_vel_vect, IRU[i].pos_drift_vect);

    // IRU[i].polar_ground_vel.x = normalize_hdg(dir2hdg(IRU[i].velocity_vect));
    // IRU[i].polar_ground_vel.y = vect2_abs(IRU[i].velocity_vect) / State_New.frame_time;
    // IRU[i].polar_flight_vel = {IRU[i].heading_true, IRU[i].tas};

    IRU[i].polar_ground_vel = {dir2hdg(IRU[i].velocity_vect), vect2_abs(IRU[i].velocity_vect)};

}

void deg_min(double lat, double lon, char *output, size_t cap)
{
    int lat_degrees = abs(lat);
    double lat_minutes = (abs(lat) - floor(abs(lat)))*60;
    int lon_degrees = abs(lon);
    double lon_minutes = (abs(lon) - floor(abs(lon)))*60;;
    if ((float)(((int)(lat_minutes*10 + 0.5))/10) > 59.9)
    {
        ++lat_degrees;
        lat_minutes = 0;
    }

    if ((float)(((int)(lon_minutes*10 + 0.5))/10) > 59.9)
    {
        ++lon_degrees;
        lon_minutes = 0;
    }
    char NS = (lat >= 0 ? 'N' : 'S');
    char EW = (lon >= 0 ? 'E' : 'W');
    snprintf(output, cap, "%c%02d* %02.1f', %c%03d* %02.1f'", 
            NS, lat_degrees,  lat_minutes, EW, lon_degrees, lon_minutes);  
}


void true_heading_update(int i)
{
    IRU[i].heading_true = State_New.hdg_true;
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
void current_pos_update(int i)
{   
    set_velocity_vect(i);
    true_heading_update(i);
    wind_vect_update(i);

    IRU[i].time_in_nav = State_New.runtime - nav_start_time;

    //recompute current IRU position always
    geo_pos2_t new_pos, ppos;
    ppos = GEO3_TO_GEO2(IRU[i].current_pos);
           //geo_displace(*ellip,  pos, hdg, dist)
    double az = IRU[i].polar_ground_vel.x;
    double dist = IRU[i].polar_ground_vel.y * State_New.frame_time;
    geod.Direct(ppos.lat, ppos.lon, az, dist, IRU[i].current_pos.lat, IRU[i].current_pos.lon);
    //new_pos = amy_displace(&wgs84, ppos, az, dist);
    //IRU[i].current_pos = {new_pos.lat, new_pos.lon, IRU[i].current_pos.elev};

    deg_min(IRU[i].current_pos.lat, IRU[i].current_pos.lon, IRU[i].curr_pos_dm,
             sizeof(IRU[i].curr_pos_dm));
    
    IRU[i].flightplan.waypoint_pos[0] = {IRU[i].nav_pos.lat, IRU[i].nav_pos.lon};

}

void triple_mix_logic(int i)
{
    if(!IRU[i].mix_switch || IRU[i].nav_mode != 3 || NUM_IRU < 3)
    {
        IRU[i].mix_pos = {0};
        IRU[i].mix_vect = {0};
        IRU[i].polar_mix_vel = {0};
    }
    
    if(NUM_IRU > 2 && (IRU[i].mix_switch && IRU[i].nav_mode == 3))
    {    
        triple_mix();
        nav_pos_ecef = geo2ecef_mtr(IRU[i].nav_pos, &wgs84);

        mix_pos_ecef = geo2ecef_mtr(IRU[i].mix_pos, &wgs84);

        FILTER_IN(mix_weight, 1, State_New.frame_time, 30);
        vect3_t new_nav_pos;

        new_nav_pos.x = wavg(nav_pos_ecef.x, mix_pos_ecef.x, mix_weight);
        new_nav_pos.y = wavg(nav_pos_ecef.y, mix_pos_ecef.y, mix_weight);
        new_nav_pos.z = wavg(nav_pos_ecef.z, mix_pos_ecef.z, mix_weight);

        IRU[i].nav_pos = ecef2geo(new_nav_pos, &wgs84);
        deg_min(IRU[i].nav_pos.lat, IRU[i].nav_pos.lon, IRU[i].nav_pos_dm, sizeof(IRU[i].nav_pos_dm));
    }
    else if(NUM_IRU < 3 || (!IRU[i].mix_switch || IRU[i].nav_mode != 3))
    {
        triple_mix_on = false;
        IRU[i].nav_pos = IRU[i].current_pos;
        deg_min(IRU[i].nav_pos.lat, IRU[i].nav_pos.lon, IRU[i].nav_pos_dm, sizeof(IRU[i].nav_pos_dm));
    }    
}

void adc_data_in(int i)
{
    double hdg = IRU[i].heading_true;
    double tas = IRU[i].tas;
    IRU[i].flight_vect = vect2_scmul(hdg2dir(hdg), tas);
    IRU[i].polar_flight_vel = {hdg, tas};

    if(i == 2 && IRU[i].nav_mode == 3)
    {
        IRU[i].adc1_alt = FEET2MET(State_New.alt_ft_[0]);
        IRU[i].tas = KT2MPS(State_New.tas_kt_[0]);
    }
    else if(IRU[i].nav_mode == 3)
    {
        IRU[i].adc2_alt = FEET2MET(State_New.alt_ft_[i]);
        IRU[i].tas = KT2MPS(State_New.tas_kt_[i]);
    }
    //integrate each frame vertical speed to compute altitude
    IRU[i].current_pos.elev += FEET2MET(State_New.vh_ind_fpm2) * State_New.frame_time;
  
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

void electrical_source()
{
    int n = sizeof(State_New.eng_gen_on) / sizeof(State_New.eng_gen_on[0]);
    int num_gens = std::count(State_New.eng_gen_on, State_New.eng_gen_on + n, 1);
    for(int i = 0; i < NUM_IRU; ++i)
    {
        if(!State_New.apu_gen_on && num_gens < 1 && IRU[i].nav_mode > 0)
        {
            FILTER_IN_LIN(IRU[i].batt_capacity_sec, 0, State_New.frame_time, 1);
            IRU[i].power_on = 0;//CHANGE THIS TO BE CONTROLED BY BATT AND GENERATOR POWER
            IRU[i].batt_light = !IRU[i].power_on;
        }

        if(IRU[i].batt_capacity_sec < 900 && num_gens >= 1)
        {   
            FILTER_IN_LIN(IRU[i].batt_capacity_sec, 900, State_New.frame_time, 1);
            IRU[i].power_on = 1;
            IRU[i].batt_light = !IRU[i].power_on;           
        }
    }

}

void warn_light_logic(int i)
{
    IRU[i].warn_light = IRU[i].flightplan.time_to_fix <= 120 ? 1 : 0;
}

void debug_set_pos() //shortcut for entering position
{
    if (first_floop) 
    {
        for(int i = 0; i < NUM_IRU; ++i)
        {
            IRU[i].current_pos.lat = State_New.lat;
            IRU[i].current_pos.lon = State_New.lon;
            IRU[i].nav_mode = 3;
            IRU[i].mix_switch = 0;
        }    
    }
}

void adc_validity_check()
{

}

void waypoint_selector_clamp(int i)
{
    if (IRU[i].waypoint_selector > 9) {
    IRU[i].waypoint_selector = std::fmod(IRU[i].waypoint_selector, 10);
    }
}

double wgs84_rad(double lat_deg)
{
    double sin_lat = sin(DEG2RAD(lat_deg));
    double radius = wgs84.a / sqrt(1 - wgs84.ecc2 * POW2(sin_lat));
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

//work on the remote transfer function
void remote_transfer()
{
    
}