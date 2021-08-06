#include "dme_functions.h"
#include "acfutils/assert.h"
#include "acfutils/geom.h"
#include "acfutils/perf.h"
#include "structs.h"
#include "variables.h"
#include <algorithm>
#include <cmath>

bool dme1_compare_valid;
bool dme2_compare_valid;

static void compare_dme_freq()
{
  dme1_compare_valid = 
    (State_New.nav1_freq_hz == IRU[0].dme_data.dme1_freq_hz ? true : false);
  dme2_compare_valid = 
    (State_New.nav2_freq_hz == IRU[0].dme_data.dme2_freq_hz ? true : false);
}

vect2_t iru_comp_pos_diff_dme(double dir, double dme_dist_comp)
{
    return vect2_scmul(hdg2dir(dir), dme_dist_comp);
}

void iru_dme_pos_corr_comp(int i)
{
    vect3_t IRU_curr_pos_ecef[3], DME_pos_ecef[2];
    double dme_dir_1, dme_dir_2, dump;
    //compute the direction and ground distance between the current pos
    //and the entered dme station coordinates
    for (int i = 0; i < NUM_IRU; ++i)
    {
        //convert internal pos and DME pos to ecef
        IRU_curr_pos_ecef[i] = geo2ecef_mtr(IRU[i].current_pos, &wgs84);
        DME_pos_ecef[0] = geo2ecef_mtr(IRU[i].dme_pos[0], &wgs84);
        DME_pos_ecef[1] = geo2ecef_mtr(IRU[i].dme_pos[1], &wgs84);
        
        //compute the direction and ground distance between the current pos
        //and the entered dme station coordinates
        IRU[i].dme_data.dme1_dist_comp = gc_distance(GEO3_TO_GEO2(IRU[i].current_pos),GEO3_TO_GEO2(IRU[i].dme_pos[0]));
        IRU[i].dme_data.dme2_dist_comp = gc_distance(GEO3_TO_GEO2(IRU[i].current_pos),GEO3_TO_GEO2(IRU[i].dme_pos[1]));


        // geod.Inverse(IRU[i].current_pos.lat, IRU[i].current_pos.lon, 
        //              IRU[i].dme_pos[0].lat, IRU[i].dme_pos[0].lon, 
        //              IRU[i].dme_data.dme1_dist_comp, dme_dir_1, dump);

        // geod.Inverse(IRU[i].current_pos.lat, IRU[i].current_pos.lon, 
        //              IRU[i].dme_pos[1].lat, IRU[i].dme_pos[1].lon, 
        //              IRU[i].dme_data.dme2_dist_comp, dme_dir_2, dump);
    }
    
    //compute the E and N position corrections to apply to the current position
    #if NUM_IRU == 1

        IRU[0].pos_corr_dme[0] = iru_comp_pos_diff_dme(dme_dir_1, IRU[0].dme_data.dme1_dist_comp);
        IRU[0].pos_corr_dme[1] = iru_comp_pos_diff_dme(dme_dir_2, IRU[0].dme_data.dme2_dist_comp);

    #elif NUM_IRU == 2

        IRU[0].pos_corr_dme[0] = iru_comp_pos_diff_dme(dme_dir_1, IRU[0].dme_data.dme1_dist_comp);
        IRU[0].pos_corr_dme[1] = iru_comp_pos_diff_dme(dme_dir_2, IRU[0].dme_data.dme2_dist_comp);
        IRU[1].pos_corr_dme[0] = iru_comp_pos_diff_dme(dme_dir_2, IRU[1].dme_data.dme2_dist_comp);
        IRU[1].pos_corr_dme[1] = iru_comp_pos_diff_dme(dme_dir_2, IRU[1].dme_data.dme2_dist_comp);

    #elif NUM_IRU == 3

        IRU[0].pos_corr_dme[0] = iru_comp_pos_diff_dme(dme_dir_1, IRU[0].dme_data.dme1_dist_comp);
        IRU[1].pos_corr_dme[1] = iru_comp_pos_diff_dme(dme_dir_2, IRU[1].dme_data.dme2_dist_comp);
        IRU[2].pos_corr_dme[0] = iru_comp_pos_diff_dme(dme_dir_1, IRU[2].dme_data.dme1_dist_comp);
        IRU[2].pos_corr_dme[1] = iru_comp_pos_diff_dme(dme_dir_2, IRU[2].dme_data.dme1_dist_comp); 
    #else
    #error "Invalid NUM_IRU value. NUM_IRU must be in range [1,3]"
    #endif
}

void dme_pos_corr()
{
    double DME1 = State_New.nav1_dme_nm;
    double DME2 = State_New.nav2_dme_nm;
}

void iru_dme_pos_update(int i)
{
    for (int i; i < NUM_IRU; ++i)
    {
        
    }
}
