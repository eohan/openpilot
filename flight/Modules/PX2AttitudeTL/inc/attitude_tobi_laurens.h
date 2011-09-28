/*
 * attitude_tobi_laurens.h
 *
 *  Created on: May 31, 2011
 *      Author: pixhawk
 */

#ifndef ATTITUDE_TOBI_LAURENS_H_
#define ATTITUDE_TOBI_LAURENS_H_

#include "matrix.h"
//#include "mav_vect.h"

void vect_norm(float_vect3 *vect);

void vect_cross_product(const float_vect3 *a, const float_vect3 *b, float_vect3 *c);

void attitude_tobi_laurens_update_a(void);

void attitude_tobi_laurens_init(void);

void attitude_tobi_laurens(const float_vect3 *accel, const float_vect3 *mag, const float_vect3 *gyro);

void attitude_tobi_laurens_get_euler(float_vect3 * angles);
#endif /* ATTITUDE_TOBI_LAURENS_H_ */
