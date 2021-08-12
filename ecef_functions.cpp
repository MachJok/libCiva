#include "ecef_functions.h"

void compute_enu(geo_pos3_t ppos,  const ellip_t *ellip, vect3_t &e_unit, vect3_t &n_unit, vect3_t &u_unit)
{
    static const vect3_t z = VECT3(0, 0, 1);
    vect3_t pos = geo2ecef_mtr(ppos, ellip);
    e_unit = vect3_unit(vect3_xprod(z, pos), NULL);
    n_unit = vect3_unit(vect3_xprod(pos, e_unit), NULL);
    u_unit = vect3_unit(pos, NULL);
}

vect3_t compute_ecef_vel(geo_pos3_t ppos, vect3_t polar_vel_vect)
{
    static const vect3_t z = VECT3(0, 0, 1);
    vect3_t pos = geo2ecef_mtr(ppos, &wgs84);
    vect3_t e_unit = vect3_unit(vect3_xprod(z, pos), NULL);
    vect3_t n_unit = vect3_unit(vect3_xprod(pos, e_unit), NULL);
    vect3_t u_unit = vect3_unit(pos, NULL);
        
    vect3_t u_vel = vect3_set_abs(u_unit, polar_vel_vect.z);
    vect3_t e_vel = vect3_set_abs(e_unit, sin(DEG2RAD(polar_vel_vect.x)) * polar_vel_vect.y);
    vect3_t n_vel = vect3_set_abs(n_unit, cos(DEG2RAD(polar_vel_vect.x)) * polar_vel_vect.y);

    double x_vel = u_vel.x + e_vel.x + n_vel.x;
    double y_vel = u_vel.y + e_vel.y + n_vel.y;
    double z_vel = u_vel.z + e_vel.z + n_vel.z;
    vect3_t ecef_vel = {x_vel, y_vel, z_vel};
    return ecef_vel;
}
