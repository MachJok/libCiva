#include "structs.h"
#include "variables.h"

IRU_t IRU[NUM_IRU] = {0};
void IRU_INIT_STATE()
{
    for (int i = 0; i < NUM_IRU; ++i)
    {
        IRU[i].flightplan.leg.from = 0;
        IRU[i].flightplan.leg.from = 1;
    }
}

Sim_State_t State_Old = {0};
Sim_State_t State_New = {0};
