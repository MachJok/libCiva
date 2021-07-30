#include "functions.h"
#include "Geodesic.hpp"
#include <acfutils/geom.h>
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
            //normal dist centered on 0
            std::normal_distribution<double> normal_dist(1.35, 0.41994);
            drift_mag = KT2MPS(normal_dist(gen)); //converts nm/hr to m/s
            if(drift_mag < 0)
            {
                drift_mag = -drift_mag;
            }
            std::mt19937 gen2(rd());
            std::uniform_real_distribution<double> uniform_dist(0, 360);
            double drift_dir = uniform_dist(gen2);
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
        
        logMsg("IRU-%i Drift Vector: (%03f, %03f)", i+1, 
                                        IRU[i].pos_drift_vect.x, 
                                        IRU[i].pos_drift_vect.y);
        logMsg("IRU-%i init True Heading: %f", i+1, IRU[i].heading_true);
    }
    logMsg("IRU INIT COMPLETE: ");
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
    double hdg = IRU[i].heading_true;
    double tas = IRU[i].tas;
    IRU[i].flight_vect = vect2_scmul(hdg2dir(hdg), tas);
    IRU[i].polar_flight_vel.x = hdg;
    IRU[i].polar_flight_vel.y = tas;
    IRU[i].polar_ground_vel.x = dir2hdg(IRU[i].velocity_vect);
    IRU[i].polar_ground_vel.y = vect2_abs(IRU[i].velocity_vect);

}

void deg_min(double lat, double lon, char *output, size_t cap)
{
    int lat_degrees = abs(lat);
    double lat_minutes = (abs(lat) - floor(abs(lat)))*60;
    int lon_degrees = abs(lon);
    double lon_minutes = (abs(lon) - floor(abs(lon)))*60;;
    while (lat_minutes > 59.9)
    {
        lat_minutes = clamp(lon_minutes, 0, 59.9);
    }
    while (lon_minutes > 59.9)
    {
        lon_minutes = clamp(lon_minutes, 0, 59.9);
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
    
   double min_spd = KT2MPS(115);



    if(IRU[i].tas > min_spd )
    {
        //gnd vector - flight vector
        vect2_t wind_vector = vect2_sub(IRU[i].flight_vect, 
                                        IRU[i].velocity_vect);
        IRU[i].polar_wind_vect = {floor(normalize_hdg(dir2hdg(wind_vector))), 
                                    round(MPS2KT(vect2_abs(wind_vector)))};
        //{normalize_hdg(dir2hdg(wind_vector)) + 180, vect2_abs(wind_vector)};
        // IRU[i].polar_wind_vect.x = (normalize_hdg(dir2hdg(wind_vector)));
        // IRU[i].polar_wind_vect.y = (vect2_abs(wind_vector));
        FILTER_IN_LIN(IRU[i].drift_angle, IRU[i].polar_flight_vel.x - 
                        IRU[i].polar_ground_vel.x, State_New.frame_time, 30);
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

    //compute distance travelled in 1 frame and direction
    //MAKE NEW FUNCTION since velocities are now MAG-N/MAG-E not DIR/MAG

    IRU[i].time_in_nav = State_New.runtime - nav_start_time;


    double az = IRU[i].polar_ground_vel.x;
    double dist = vect2_abs(IRU[i].velocity_vect) * State_New.frame_time;

    //recompute current IRU position always
    geod.Direct(IRU[i].current_pos.lat, IRU[i].current_pos.lon, az, dist, IRU[i].current_pos.lat, 
                IRU[i].current_pos.lon);
    deg_min(IRU[i].current_pos.lat, IRU[i].current_pos.lon, IRU[i].curr_pos_dm,
             sizeof(IRU[i].curr_pos_dm));

    
    //Altitude and TAS input into IRU. ADC1 -> IRU1/3, ADC 2 -> IRU2
    for(int i = 0; i < NUM_IRU; ++i)
    {
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

    if(!IRU[i].mix_switch || IRU[i].nav_mode != 3 || NUM_IRU < 3)
    {
        IRU[i].mix_pos = {0};
        IRU[i].mix_vect = {0};
        IRU[i].polar_mix_vel = {0};
    }
    
    if(NUM_IRU > 2 && (IRU[i].mix_switch && IRU[i].nav_mode == 3))
    {    
        triple_mix();
        ecef_convert_geo_pos3(IRU[i].nav_pos, nav_pos_ecef);
        ecef_convert_geo_pos3(IRU[i].mix_pos, mix_pos_ecef);

        FILTER_IN(mix_weight, 1, State_New.frame_time, 120 / State_New.frame_time);
        vect3_t new_nav_pos;

        new_nav_pos.x = wavg(nav_pos_ecef.x, mix_pos_ecef.x, mix_weight);
        new_nav_pos.y = wavg(nav_pos_ecef.y, mix_pos_ecef.y, mix_weight);
        new_nav_pos.z = wavg(nav_pos_ecef.z, mix_pos_ecef.z, mix_weight);

        IRU[i].nav_pos = ecef2geo(new_nav_pos, &wgs84);
    }
    else if(NUM_IRU < 3 && (!IRU[i].mix_switch && IRU[i].nav_mode != 3))
    {
        triple_mix_on = false;
        IRU[i].nav_pos = IRU[i].current_pos;
    }

}

void ecef_convert_geo_pos2(geo_pos2_t pos, vect3_t &ecef_pos)
{
    ecef.Forward(pos.lat, pos.lon, 0, ecef_pos.x, ecef_pos.y, ecef_pos.z);
}

void ecef_convert_geo_pos3(geo_pos3_t pos, vect3_t &ecef_pos)
{
    ecef.Forward(pos.lat, pos.lon, pos.elev, ecef_pos.x, ecef_pos.y, ecef_pos.z);
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
    max_pos = {min_lat, min_lon, elev};
    vect3_t max_ecef, min_ecef, mid_ecef;
    ecef_convert_geo_pos3(max_pos, max_ecef);
    ecef_convert_geo_pos3(min_pos, min_ecef);
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
        geod.Inverse(old_pos.lat, old_pos.lon, IRU[i].mix_pos.lat, 
                    IRU[i].mix_pos.lon, dist, az, trash_val);
        IRU[i].polar_mix_vel = {normalize_hdg(az), dist / State_New.frame_time};
        IRU[i].mix_vect = {vect2_scmul(hdg2dir(az), 
                            dist / State_New.frame_time)};

    }
        
}

void electrical_source()
{
    for(int i = 0; i < NUM_IRU; ++i)
    {
        for(int j = 0; j < sizeof(State_New.eng_gen_on); ++j)
        {
            if ((!State_New.apu_gen_on && !State_New.eng_gen_on[j]) && 
                IRU[i].nav_mode > 0) 
            {
                FILTER_IN_LIN(IRU[i].batt_capacity_sec, 0, 
                                State_New.frame_time, 1);
                IRU[i].power_on = 0;
            }
            else if(IRU[i].batt_capacity_sec <= 900 && 
                    (State_New.apu_gen_on || State_New.eng_gen_on[j]))
            {
                FILTER_IN_LIN(IRU[i].batt_capacity_sec, 900, 
                                State_New.frame_time, .5);
                IRU[i].power_on = 1;
            }
        }
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

