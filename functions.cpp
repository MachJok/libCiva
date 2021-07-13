#include "functions.h"
#include "Geodesic.hpp"
#include <acfutils/geom.h>
#include "acfutils/assert.h"
#include "acfutils/helpers.h"
#include "acfutils/log.h"
#include "acfutils/math.h"
#include "acfutils/perf.h"
#include "acfutils/sysmacros.h"
#include "datarefs.h"
#include "structs.h"
#include "variables.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <random>
#include <type_traits>


geo_pos3_t old_pos{0};
geo_pos3_t pos3{0}; //pos_3 will be the triple mix position
vect3_t ecef_pos_[NUM_IRU + 1] = {0};
triple_mix_pos_t Triple_Mix_Pos = {0};
bool drift = true;

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
            bool try_again = true;
            while (try_again)
            {
                std::normal_distribution<double> normal_dist(1.35, 0.41994);
                drift_mag = KT2MPS(normal_dist(gen)); //converts nm/hr to m/s
                if(drift_mag > 0)
                {
                    try_again = false;
                }
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
    //set_true_hdg_offset();
    
    for (int i = 0; i < NUM_IRU; ++i)
    {
        IRU[i].batt_capacity_sec = 900;
        IRU[i].current_pos.elev = 0;
        IRU[i].align_pos.elev = 0;
        
        logMsg("IRU-%i Drift Vector: (%02f, %02f)", i+1, IRU[i].pos_drift_vect.x, IRU[i].pos_drift_vect.y);
        logMsg("IRU-%i init True Heading: %f", i+1, IRU[i].heading_true);
    }
    logMsg("IRU INIT COMPLETE: ");
}
//computes the actual N and E velocities of the aircraft in the sim to compute IRU velocity vectors m/s
//simulates accelerometer integration for velocity
void set_velocity_vect(int i)
{
    vect2_t sim_vel_vect;
    sim_vel_vect = vect2_scmul(hdg2dir(State_New.true_trk), State_New.ground_speed);
    IRU[i].velocity_vect = vect2_add(sim_vel_vect, IRU[i].pos_drift_vect);

    // IRU[i].polar_ground_vel.x = normalize_hdg(dir2hdg(IRU[i].velocity_vect));
    // IRU[i].polar_ground_vel.y = vect2_abs(IRU[i].velocity_vect) / State_New.frame_time;
    // IRU[i].polar_flight_vel = {IRU[i].heading_true, IRU[i].tas};
    double hdg = IRU[i].heading_true;
    double tas = IRU[i].tas;
    IRU[i].flight_vect = vect2_scmul(hdg2dir(hdg), tas);
    IRU[i].polar_flight_vel.x = dir2hdg(IRU[i].flight_vect);
    IRU[i].polar_flight_vel.y = vect2_abs(IRU[i].flight_vect);
    IRU[i].polar_ground_vel.x = dir2hdg(IRU[i].velocity_vect);
    IRU[i].polar_ground_vel.y = vect2_abs(IRU[i].velocity_vect);

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
        vect2_t wind_vector = vect2_sub(IRU[i].flight_vect, IRU[i].velocity_vect);
        IRU[i].polar_wind_vect = {floor(normalize_hdg(dir2hdg(wind_vector))), round(MPS2KT(vect2_abs(wind_vector)))};
        //{normalize_hdg(dir2hdg(wind_vector)) + 180, vect2_abs(wind_vector)};
        // IRU[i].polar_wind_vect.x = (normalize_hdg(dir2hdg(wind_vector)));
        // IRU[i].polar_wind_vect.y = (vect2_abs(wind_vector));
        FILTER_IN_LIN(IRU[i].drift_angle, IRU[i].polar_flight_vel.x - IRU[i].polar_ground_vel.x, State_New.frame_time, 30);
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

    double az = IRU[i].polar_ground_vel.x;
    double dist = vect2_abs(IRU[i].velocity_vect) * State_New.frame_time;
    double old_lat = IRU[i].current_pos.lat;
    double old_lon = IRU[i].current_pos.lon;
    //in: lat1, lat2, crs, dist, out: new lat, new lon
    geod.Direct(old_lat, old_lon, az, dist, IRU[i].current_pos.lat, IRU[i].current_pos.lon);
    
    //Altitude and TAS input into IRU. ADC1 -> IRU1/3, ADC 2 -> IRU2
    for(int i = 0; i < NUM_IRU; ++i)
    {
        if(i == 2)
        {
            IRU[i].current_pos.elev = FEET2MET(State_New.alt_ft_[0]);
            IRU[i].tas = KT2MPS(State_New.tas_kt_[0]);
        }
        else
        {
            IRU[i].current_pos.elev = FEET2MET(State_New.alt_ft_[i]);
            IRU[i].tas = KT2MPS(State_New.tas_kt_[i]);
        }
    }

    if(NUM_IRU > 2)
    {    
        if(!IRU[0].mix_switch || !IRU[1].mix_switch || !IRU[2].mix_switch)
        {
            Triple_Mix_Pos.curr_pos = {0};
            Triple_Mix_Pos.velocity_vect = {0};
            Triple_Mix_Pos.polar_vel_vect = {0};
        }
        else if(IRU[0].mix_switch && IRU[1].mix_switch && IRU[2].mix_switch)
        {
            triple_mix();
        }
    }

}

void triple_mix()
{    
    for(int i = 0; i < NUM_IRU; ++i) //convert current position from each IRU into ECEF coordinates at 0 altitude
    {
        ecef_pos_[i] = geo2ecef_mtr(IRU[i].current_pos, &wgs84);
    }
    //compute the unweighted average of the 3 ECEF positions as the triple mix
    //convert back to wgs84 and store
    ecef_pos_[3].x = (ecef_pos_[0].x + ecef_pos_[1].x + ecef_pos_[2].x) / 3.;
    ecef_pos_[3].y = (ecef_pos_[0].y + ecef_pos_[1].y + ecef_pos_[2].y) / 3.;
    ecef_pos_[3].z = (ecef_pos_[0].z + ecef_pos_[1].z + ecef_pos_[2].z) / 3.;
    pos3 = ecef2geo(ecef_pos_[3], &wgs84);
    
    for(int i = 0; i < NUM_IRU; ++i)
    {
        //assign values for easier input
        Triple_Mix_Pos.curr_pos = pos3;
        old_pos.lat = State_Old.Triple_Mix_Pos.curr_pos.lat;
        old_pos.lon = State_Old.Triple_Mix_Pos.curr_pos.lon;
        double trash_val, az, dist;
        az = {};
        dist = {};
        // IN:lat1, lon1, lat2, lon2 out: dist, az12, az21
        geod.Inverse(old_pos.lat, old_pos.lon, pos3.lat, pos3.lon, dist, az, trash_val);
        Triple_Mix_Pos.polar_vel_vect.x = az;
        Triple_Mix_Pos.polar_vel_vect.y = dist / State_New.frame_time;
        Triple_Mix_Pos.polar_vel_vect.x = normalize_hdg(Triple_Mix_Pos.polar_vel_vect.x);
        Triple_Mix_Pos.velocity_vect.x = dist * sin(DEG2RAD(az)) / State_New.frame_time;
        Triple_Mix_Pos.velocity_vect.y = dist * cos(DEG2RAD(az)) / State_New.frame_time;
    }    
}

void electrical_source()
{
    for(int i = 0; i < NUM_IRU; ++i)
    {
        for(int j = 0; j < sizeof(State_New.eng_gen_on); ++j)
        {
            if ((!State_New.apu_gen_on && !State_New.eng_gen_on[j]) && IRU[i].nav_mode > 0) 
            {
                FILTER_IN_LIN(IRU[i].batt_capacity_sec, 0, 900, 1);
                IRU[i].power_on = 0;
            }
            else if(IRU[i].batt_capacity_sec <= 900 && (State_New.apu_gen_on || State_New.eng_gen_on[j]))
            {
                FILTER_IN_LIN(IRU[i].batt_capacity_sec, 900, State_New.frame_time, .5);
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
            IRU[i].mix_switch = 1;
        }    
    }
}


// void align_iru(int i)
// {
//     IRU[i].time_since_align = 0;
//     if (IRU[i].power_on && (IRU[i].align_switch == 2 && IRU[i].batt_capacity_sec > 0) && State_New.ground_speed < 0.5)
//     {
//         double align_start_time = State_New.runtime;
//         IRU[i].time_since_align = State_New.runtime - align_start_time;
//         if (IRU[i].time_since_align < 60) 
//         {
//             IRU[i].current_mode = 8;
//             IRU[i].batt_self_test = 1;
//         }
//         if(IRU[i].time_since_align == 60 && IRU[i].current_mode == 8 && !IRU[i].align_pos_enterd)
//         {
//             --IRU[i].current_mode;
//             IRU[i].batt_self_test = 0;
//         }
//         if(IRU[i].time_since_align >= 60 && IRU[i].time_since_align <= 450 && IRU[i].align_pos_enterd)
//         {
//             --IRU[i].current_mode;
//             IRU[i].batt_self_test = 0;
//             double heading_time_start = State_New.runtime;
//             double delta_hdg_sec = 10;
//             double secs_to_align_hdg = abs(IRU[i].heading_true - State_New.hdg_true)/delta_hdg_sec;
//             double delta_t = State_New.runtime - heading_time_start;
//             double align_fraction = delta_t / secs_to_align_hdg;
//             IRU[i].heading_true = wavg(IRU[i].heading_true, State_New.hdg_true, align_fraction);
//         }
//         if(IRU[i].time_since_align == 450)
//         {
//             //set up a scaling vector where at 5 drift = 2nm/hr and at 0 drift = random bias
//             double max_drift_ms = KT2MPS(2);
//             double min_drift_ms = IRU[i].pos_drift_vect.y;
//             IRU[i].pos_drift_vect.y = max_drift_ms;
//             FILTER_IN(IRU[i].pos_drift_vect.y, min_drift_ms, State_New.frame_time, 255);
//             IRU[i].current_mode = 5;
//             double next_mode_time = 204;
//             double time_start = State_New.runtime;
//             double mode_delta_t = State_New.runtime-time_start;
//             if (mode_delta_t == next_mode_time && IRU[i].current_mode >= 0) 
//             {
//                 --IRU[i].current_mode;
//             }
//         }
//     }

//     else if (IRU[i].power_on && (IRU[i].align_switch == 2 && IRU[i].batt_capacity_sec > 0) && State_New.ground_speed >= 0.5) 
//     {
//         IRU[i].warn_light_cdu = 1
//     }

// }