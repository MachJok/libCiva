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
int num_deltas = NUM_IRU > 2 ? 4 : 2;

static void compare_dme_freq()
{
  dme1_compare_valid = 
    (State_New.nav1_freq_hz == IRU[0].dme1_freq_hz ? true : false);
  dme2_compare_valid = 
    (State_New.nav2_freq_hz == IRU[0].dme2_freq_hz ? true : false);
}

void iru_dme_pos_update(int i)
{
    vect3_t IRU_curr_pos_ecef[3], DME_pos_ecef[2];
    double dme_dir_1, dme_dir_2, dump;
    for (int i = 0; i < NUM_IRU; ++i)
    {
        //convert internal pos and DME pos to ecef
        ecef_convert_geo_pos3(IRU[i].current_pos, IRU_curr_pos_ecef[i]);

        ecef_convert_geo_pos3(IRU[i].dme_pos[0], DME_pos_ecef[0]);

        ecef_convert_geo_pos3(IRU[i].dme_pos[1], DME_pos_ecef[1]);
        
        //compute the direction and ground distance between the current pos
        //and the dme station
        geod.Inverse(IRU[i].current_pos.lat, IRU[i].current_pos.lon, 
                     IRU[i].dme_pos[0].lat, IRU[i].dme_pos[0].lon, 
                     IRU[i].dme1_dist_comp, dme_dir_1, dump);
        geod.Inverse(IRU[i].current_pos.lat, IRU[i].current_pos.lon, 
                     IRU[i].dme_pos[1].lat, IRU[i].dme_pos[1].lon, 
                     IRU[i].dme2_dist_comp, dme_dir_2, dump);
    }
    vect2_t pos_delta[num_deltas];
//compute the E and N position corrections to apply to the current position        
    switch (NUM_IRU) 
    {
        case 1: 
            
            pos_delta[0] = vect2_scmul(hdg2dir(dme_dir_1), IRU[0].dme1_dist_comp);
            pos_delta[1] = vect2_scmul(hdg2dir(dme_dir_2), IRU[0].dme2_dist_comp);

        case 2:
            pos_delta[0] = vect2_scmul(hdg2dir(dme_dir_1), IRU[0].dme1_dist_comp);
            pos_delta[1] = vect2_scmul(hdg2dir(dme_dir_2), IRU[0].dme2_dist_comp);
            pos_delta[2] = vect2_scmul(hdg2dir(dme_dir_1), IRU[1].dme1_dist_comp);
            pos_delta[3] = vect2_scmul(hdg2dir(dme_dir_2), IRU[1].dme2_dist_comp);

        case 3:
            pos_delta[0] = vect2_scmul(hdg2dir(dme_dir_1), IRU[0].dme1_dist_comp);
            pos_delta[1] = vect2_scmul(hdg2dir(dme_dir_2), IRU[1].dme2_dist_comp);
            pos_delta[2] = vect2_scmul(hdg2dir(dme_dir_1), IRU[2].dme1_dist_comp);
            pos_delta[3] = vect2_scmul(hdg2dir(dme_dir_2), IRU[2].dme2_dist_comp);
            
        default:
            ASSERT(NUM_IRU < 4);
            ASSERT(NUM_IRU > 0);
    }


}
