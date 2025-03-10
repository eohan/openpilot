/*
 * attitude_tobi_laurens.c
 *
 *  Created on: May 31, 2011
 *      Author: pixhawk
 */

/*
 * attitude_tobi_laurens.c
 *
 *  Created on: 21.12.2010
 *      Author: Laurens Mackay
 */
#include "attitude_tobi_laurens.h"
#include "kalman.h"

//#include "mavlink_debug.h"
//#include "sensors.h"
#include "math.h"
//#include "altitude_speed.h"
//#include "transformation.h"
//#include "gps_transformations.h"

//#include "mavlink_types.h"
//extern mavlink_system_t mavlink_system;
//
//#include "mavlink.h"

//void debug_vect3(const char* string, const float_vect3 vect)
//{
//	debug_vect(string, vect.x, vect.y, vect.z);
//}

//#define VELOCITY_HOLD 0.999f
//#define ACCELERATION_HOLD 0.99f
//#define VELOCITY_HOLD 1.0f
//#define ACCELERATION_HOLD 1.0f
#define TIME_STEP (1.0f / 200.0f)

kalman_t attitude_tobi_laurens_kal;

void vect_norm(float_vect3 *vect)
{
	float length = sqrt(
			vect->x * vect->x + vect->y * vect->y + vect->z * vect->z);
	if (length != 0)
	{
		vect->x /= length;
		vect->y /= length;
		vect->z /= length;
	}
}



void vect_cross_product(const float_vect3 *a, const float_vect3 *b,
		float_vect3 *c)
{
	c->x = a->y * b->z - a->z * b->y;
	c->y = a->z * b->x - a->x * b->z;
	c->z = a->x * b->y - a->y * b->x;
}

void attitude_tobi_laurens_update_a(void)
{
	// for acc
	// Idendity matrix already in A.
	M(attitude_tobi_laurens_kal.a, 0, 1) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 11);
	M(attitude_tobi_laurens_kal.a, 0, 2) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 10);

	M(attitude_tobi_laurens_kal.a, 1, 0) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 11);
	M(attitude_tobi_laurens_kal.a, 1, 2) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 9);

	M(attitude_tobi_laurens_kal.a, 2, 0) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 10);
	M(attitude_tobi_laurens_kal.a, 2, 1) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 9);

	// for mag
	// Idendity matrix already in A.
	M(attitude_tobi_laurens_kal.a, 3, 4) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 11);
	M(attitude_tobi_laurens_kal.a, 3, 5) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 10);

	M(attitude_tobi_laurens_kal.a, 4, 3) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 11);
	M(attitude_tobi_laurens_kal.a, 4, 5) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 9);

	M(attitude_tobi_laurens_kal.a, 5, 3) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 10);
	M(attitude_tobi_laurens_kal.a, 5, 4) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 9);

}

void attitude_tobi_laurens_init(void)
{
	//X Kalmanfilter
	//initalize matrices

	static m_elem kal_a[12 * 12] =
	{ 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0f, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0f, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0f };

	static m_elem kal_c[9 * 12] =
	{ 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 1.0f, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 1.0f, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 1.0f };



#define FACTOR 0.5
#define FACTORstart 1


//	static m_elem kal_gain[12 * 9] =
//	{ 		0.004 , 0    ,   0    ,   0    ,   0    ,   0    ,   0   ,    0    ,   0,
//			0   ,    0.004 , 0   ,    0   ,    0   ,    0   ,    0   ,    0   ,    0,
//			0   ,    0    ,   0.004 , 0   ,    0   ,    0   ,    0   ,    0   ,    0,
//			0   ,    0    ,   0   ,    0.015, 	0   ,    0   ,    0   ,    0   ,    0,
//			0   ,    0   ,    0   ,    0    ,   0.015, 	 0   ,    0   ,    0   ,    0,
//			0   ,    0    ,   0   ,    0    ,   0   ,    0.015, 	  0   ,    0   ,    0,
//			0.0000 , +0.00002,0   ,    0 , 		0, 		 0,  	  0,  	   0    ,   0,
//			-0.00002,0    ,   0   ,    0 , 		0, 		 0,  	  0,  	   0, 	    0,
//			0,    	 0 ,	  0   ,    0,  	    0,		 0,  	  0,  	   0, 	    0,
//			0  ,     0    ,   0   ,    0   ,    0    ,   0   ,    0.4 ,   0   ,    0,
//			0   ,    0   ,    0   ,    0   ,    0    ,   0   ,    0    ,   0.4 ,   0,
//			0   ,    0   ,    0   ,    0   ,    0   ,    0   ,    0    ,   0    ,   0.4
//	};

	static m_elem kal_gain[12 * 9] =
	{ 		0.0006f , 0    ,   0    ,   0    ,   0    ,   0    ,   0   ,    0    ,   0,
			0   ,    0.0006f , 0   ,    0   ,    0   ,    0   ,    0   ,    0   ,    0,
			0   ,    0    ,   0.0006f , 0   ,    0   ,    0   ,    0   ,    0   ,    0,
			0   ,    0    ,   0   ,    0.015f, 	0   ,    0   ,    0   ,    0   ,    0,
			0   ,    0   ,    0   ,    0    ,   0.015f, 	 0   ,    0   ,    0   ,    0,
			0   ,    0    ,   0   ,    0    ,   0   ,    0.015f, 	  0   ,    0   ,    0,
			0.0000f , +0.00002f,0   ,    0 , 		0, 		 0,  	  0,  	   0    ,   0,
			-0.00002f,0    ,   0   ,    0 , 		0, 		 0,  	  0,  	   0, 	    0,
			0,    	 0 ,	  0   ,    0,  	    0,		 0,  	  0,  	   0, 	    0,
			0  ,     0    ,   0   ,    0   ,    0    ,   0   ,    0.6f ,   0   ,    0,
			0   ,    0   ,    0   ,    0   ,    0    ,   0   ,    0    ,   0.6f ,   0,
			0   ,    0   ,    0   ,    0   ,    0   ,    0   ,    0    ,   0    ,   0.6f
	};
	//offset update only correct if not upside down.

#define K 10.0f*TIME_STEP

	static m_elem kal_gain_start[12 * 9] =
	{ K, 0, 0, 0, 0, 0, 0, 0, 0,

			0, K, 0, 0, 0, 0, 0, 0, 0,

			0, 0, K, 0, 0, 0, 0, 0, 0,

			0, 0, 0, K, 0, 0, 0, 0, 0,

			0, 0, 0, 0, K, 0, 0, 0, 0,

			0, 0, 0, 0, 0, K, 0, 0, 0,

			0, 0, 0, 0, 0, 0, K, 0, 0,

			0, 0, 0, 0, 0, 0, 0, K, 0,

			0, 0, 0, 0, 0, 0, 0, 0, K,

			0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 0 };



	static m_elem kal_x_apriori[12 * 1] =
	{  };


	//---> initial states sind aposteriori!? ---> fehler
	static m_elem kal_x_aposteriori[12 * 1] =
	{ 0.0f, 0.0f, -1.0f, 0.6f, 0.0f, 0.8f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	kalman_init(&attitude_tobi_laurens_kal, 12, 9, kal_a, kal_c,
			kal_gain_start, kal_gain, kal_x_apriori, kal_x_aposteriori, 1000);

}

void attitude_tobi_laurens(const float_vect3 *accel, const float_vect3 *mag, const float_vect3 *gyro)
{
	//Transform accelerometer used in all directions
	//	float_vect3 acc_nav;
	//body2navi(&global_data.accel_si, &global_data.attitude, &acc_nav);

	// Kalman Filter

	//Calculate new linearized A matrix
	attitude_tobi_laurens_update_a();

	kalman_predict(&attitude_tobi_laurens_kal);

	//correction update

	m_elem measurement[9] =
	{ };
	m_elem mask[9] =
	{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

//	float_vect3 acc;
//	float_vect3 mag;
//	float_vect3 gyro;



	//	acc.x = global_data.accel_raw.x * 9.81f /690;
	//	acc.y = global_data.accel_raw.y * 9.81f / 690;
	//	acc.z = global_data.accel_raw.z * 9.81f / 690;

//	acc.x = global_data.accel_raw.x;
//	acc.y = global_data.accel_raw.y;
//	acc.z = global_data.accel_raw.z;



	//	mag.x = (global_data.magnet_corrected.x ) * 1.f / 510.f;
	//	mag.y = (global_data.magnet_corrected.y) * 1.f / 510.f;
	//	mag.z = (global_data.magnet_corrected.z) * 1.f / 510.f;
//	mag.x = (global_data.magnet_corrected.x ) ;
//	mag.y = (global_data.magnet_corrected.y) ;
//	mag.z = (global_data.magnet_corrected.z) ;


//	gyro.x = -(global_data.gyros_raw.x-global_data.param[PARAM_GYRO_OFFSET_X]) * 0.000955;
//	gyro.y = (global_data.gyros_raw.y-global_data.param[PARAM_GYRO_OFFSET_Y]) * 0.000955;
//	gyro.z = -(global_data.gyros_raw.z-global_data.param[PARAM_GYRO_OFFSET_Z]) * 0.001010;

//	gyro.x = -(global_data.gyros_raw.x-global_data.param[PARAM_GYRO_OFFSET_X]) * 0.001008;
//	gyro.y = (global_data.gyros_raw.y-global_data.param[PARAM_GYRO_OFFSET_Y]) * 0.001008;
//	gyro.z = -(global_data.gyros_raw.z-global_data.param[PARAM_GYRO_OFFSET_Z]) * 0.001010;





	measurement[0] = accel->x;
	measurement[1] = accel->y;
	measurement[2] = accel->z;

	measurement[3] = mag->x;
	measurement[4] = mag->y;
	measurement[5] = mag->z;

	measurement[6] = gyro->x;
	measurement[7] = gyro->y;
	measurement[8] = gyro->z;

	//Put measurements into filter


//	static int j = 0;
//	if (j >= 3)
//	{
//		j = 0;
//
//		mask[3]=1;
//		mask[4]=1;
//		mask[5]=1;
//		j=0;
//
//	}else{
//		j++;}

	kalman_correct(&attitude_tobi_laurens_kal, measurement, mask);

}
void attitude_tobi_laurens_get_all(float_vect3 * euler, float_vect3 * rates, float_vect3 * x_n_b, float_vect3 * y_n_b, float_vect3 * z_n_b){
	//debug

	// save outputs
	float_vect3 kal_acc;
	float_vect3 kal_mag;
//	float_vect3 kal_w0, kal_w;

	kal_acc.x = kalman_get_state(&attitude_tobi_laurens_kal, 0);
	kal_acc.y = kalman_get_state(&attitude_tobi_laurens_kal, 1);
	kal_acc.z = kalman_get_state(&attitude_tobi_laurens_kal, 2);

	kal_mag.x = kalman_get_state(&attitude_tobi_laurens_kal, 3);
	kal_mag.y = kalman_get_state(&attitude_tobi_laurens_kal, 4);
	kal_mag.z = kalman_get_state(&attitude_tobi_laurens_kal, 5);

//	kal_w0.x = kalman_get_state(&attitude_tobi_laurens_kal, 6);
//	kal_w0.y = kalman_get_state(&attitude_tobi_laurens_kal, 7);
//	kal_w0.z = kalman_get_state(&attitude_tobi_laurens_kal, 8);
//
//	kal_w.x = kalman_get_state(&attitude_tobi_laurens_kal, 9);
//	kal_w.y = kalman_get_state(&attitude_tobi_laurens_kal, 10);
//	kal_w.z = kalman_get_state(&attitude_tobi_laurens_kal, 11);

	rates->x = kalman_get_state(&attitude_tobi_laurens_kal, 9);
	rates->y = kalman_get_state(&attitude_tobi_laurens_kal, 10);
	rates->z = kalman_get_state(&attitude_tobi_laurens_kal, 11);



//	kal_w = kal_w;		// XXX hack to silence compiler warning
//	kal_w0 = kal_w0;	// XXX hack to silence compiler warning



	//debug_vect("magn", mag);

	//float_vect3 x_n_b, y_n_b, z_n_b;
	z_n_b->x = -kal_acc.x;
	z_n_b->y = -kal_acc.y;
	z_n_b->z = -kal_acc.z;
	vect_norm(z_n_b);
	vect_cross_product(z_n_b, &kal_mag, y_n_b);
	vect_norm(y_n_b);

	vect_cross_product(y_n_b, z_n_b, x_n_b);



	//save euler angles
	euler->x = atan2f(z_n_b->y, z_n_b->z);
	euler->y = -asinf(z_n_b->x);
	euler->z = atan2f(y_n_b->x, x_n_b->x);

//	static int i = 10;
//	if (i++ >= 10)
//	{
//		i = 0;
//		//send the angles
//
//		debug_vect3("kal_w0", kal_w0);
//		debug_vect3("kal_w", kal_w);
//		debug_vect3("kal_acc",kal_acc);
//		debug_vect3("kal_mag", kal_mag);
//	}
}
