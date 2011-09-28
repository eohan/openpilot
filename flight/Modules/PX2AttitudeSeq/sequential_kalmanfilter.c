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
#include "sequential_kalmanfilter.h"
//#include "kalman.h"
#include "matrix.h"

#include "mavlink_debug.h"
//#include "sensors.h"
#include "math.h"
//#include "altitude_speed.h"
//#include "transformation.h"
//#include "gps_transformations.h"
#include "mavlink.h"


//m_elem aH[9*12] = {
//
//		1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//
//		0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//
//		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//
//		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
//
//		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
//
//		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
//
//		0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0,
//
//		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0,
//
//		0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1 };


///////////////////////////////////////////////////////
// PROTOTYPES

void sequential_kalmanfilter_predict(void);

void sequential_kalmanfilter_update(int *elements,int nr_of_elements,m_elem measurement, m_elem R);

void sequential_kalmanfilter_get_measurements(m_elem *measurements);

void sequential_kalmanfilter_vect_norm(float_vect3 *vect);

void sequential_kalmanfilter_vect_cross_product(const float_vect3 *a, const float_vect3 *b,float_vect3 *c);

//////////////////////////////////////////////////////
// GLOBAL VARIABLES

#define TIME_STEP (1.0f / 200.0f)

#define Q_A  0.00009f
#define Q_M  0.00009f
#define Q_W0 0.00009f
#define Q_W  0.00009f

m_elem aP[12*12] = {0};
m_elem ax[12*1] = {0};
m_elem aR[9*1] = {19.1198 , 7.0098, 7.7522,600.3000, 600.30000 , 600.30000, 0.0018, 0.0028, 0.0015 };

m_elem aQ[12*1] = {Q_A , Q_A, Q_A, Q_M, Q_M, Q_M, Q_W0, Q_W0, Q_W0, Q_W, Q_W, Q_W };

matrix_t P;
matrix_t x;
matrix_t R_diag;


void sequential_kalmanfilter_vect_norm(float_vect3 *vect)
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



void sequential_kalmanfilter_vect_cross_product(const float_vect3 *a, const float_vect3 *b,
		float_vect3 *c)
{
	c->x = a->y * b->z - a->z * b->y;
	c->y = a->z * b->x - a->x * b->z;
	c->z = a->x * b->y - a->y * b->x;
}

void sequential_kalmanfilter_predict(void)
{

	//create a Temp matrix for the complicated computations
	m_elem aTemp[12*12];
	matrix_t Temp;
	Temp = matrix_create(12,12,aTemp);

	//create the F matrix for prediction of x
	m_elem aF[12*12] = {0};
	matrix_t F;
	F = matrix_create(12,12,aF);

	for (int i = 0;i<12;i++)
	{
		M(F, i, i) =  1.0f;
	}

	M(F, 0, 1) =  TIME_STEP * M(x,11,0);
	M(F, 0, 2) = -TIME_STEP * M(x,10,0);

	M(F, 1, 0) = -TIME_STEP * M(x,11,0);
	M(F, 1, 2) =  TIME_STEP * M(x,9,0);

	M(F, 2, 0) =  TIME_STEP * M(x,10,0);
	M(F, 2, 1) = -TIME_STEP * M(x,9,0);

	// for mag
	// Idendity matrix already in A.
	M(F, 3, 4) =  TIME_STEP * M(x,11,0);
	M(F, 3, 5) = -TIME_STEP * M(x,10,0);

	M(F, 4, 3) = -TIME_STEP * M(x,11,0);
	M(F, 4, 5) =  TIME_STEP * M(x,9,0);

	M(F, 5, 3) =  TIME_STEP * M(x,10,0);
	M(F, 5, 4) = -TIME_STEP * M(x,9,0);

	//temp vector for x
	m_elem ax_temp[12*1];
	matrix_t x_temp;
	x_temp = matrix_create(12,1,ax_temp);

	//prediction of the state
	//matrix_mult(F,x,x_temp); // x_temp=F*x
//	for (int i = 0;i<12;i++)
//	{
//		M(x,i,0)=M(x_temp,i,0);
//	}

	M(x_temp,0,0)=TIME_STEP * M(x,11,0) * M(x,1,0) - TIME_STEP * M(x,10,0) * M(x,2,0);
	M(x_temp,1,0)=TIME_STEP * M(x, 9,0) * M(x,2,0) - TIME_STEP * M(x,11,0) * M(x,0,0);
	M(x_temp,2,0)=TIME_STEP * M(x,10,0) * M(x,0,0) - TIME_STEP * M(x, 9,0) * M(x,1,0);
	M(x_temp,3,0)=TIME_STEP * M(x,11,0) * M(x,4,0) - TIME_STEP * M(x,10,0) * M(x,5,0);
	M(x_temp,4,0)=TIME_STEP * M(x, 9,0) * M(x,5,0) - TIME_STEP * M(x,11,0) * M(x,3,0);
	M(x_temp,5,0)=TIME_STEP * M(x,10,0) * M(x,3,0) - TIME_STEP * M(x, 9,0) * M(x,4,0);

	//F linearization for the P matrix update
	M(F, 0, 10) =  -TIME_STEP * M(x,2,0);
	M(F, 0, 11) = 	TIME_STEP * M(x,1,0);

	M(F, 1, 9)  = 	TIME_STEP * M(x,2,0);
	M(F, 1, 11) =  -TIME_STEP * M(x,0,0);

	M(F, 2, 9)  =  -TIME_STEP * M(x,1,0);
	M(F, 2, 10) = 	TIME_STEP * M(x,0,0);

	// for mag
	// Idendity matrix already in A.
	M(F, 3, 10) =  -TIME_STEP * M(x,5,0);
	M(F, 3, 11) = 	TIME_STEP * M(x,4,0);

	M(F, 4, 9)  =  	TIME_STEP * M(x,5,0);
	M(F, 4, 11) =  -TIME_STEP * M(x,3,0);

	M(F, 5, 9)  =  -TIME_STEP * M(x,4,0);
	M(F, 5, 10) = 	TIME_STEP * M(x,3,0);

	//prediction of the covariance matrix P
	matrix_mult(F,P,Temp); // Temp=F*P_apo;
	matrix_mult_trans(Temp,F,P);  //P_apr=Temp*F'=F*P_apo*F';
	for(int i=0; i<12; i++)
	{
		M(P,i,i)+=aQ[i];
	}

}


void sequential_kalmanfilter_init(void)
{
	R_diag = matrix_create(9,1,aR);
	P = matrix_create(12,12,aP);
	x = matrix_create(12,1,ax);

	//set the initial values for the P matrices and the states x (P diag =1; x = 1)

	for(int i=0;i<12;i++)
	{
		M(P,i,i)=1.0f;
	}

	for(int i=0;i<12;i++)
	{
		M(x,i,0)=0.0f;
	}

}


void sequential_kalmanfilter_update(int *elements,int nr_of_elements,m_elem measurement, m_elem R)
{

	// "Calculate" H*P*H'
	m_elem P_elem=0;
	for(int i=0; i< nr_of_elements; i++)
	{
		for(int j=0; j< nr_of_elements; j++)
		{
			P_elem += M(P,elements[i],elements[j]);
		}
	}

	//Calculate S
	m_elem S = P_elem + R;

	m_elem aK[12*1]={0};
	matrix_t K = matrix_create(12,1,aK);
	matrix_t temp = matrix_create(12,1,0);

	for(int i=0; i< nr_of_elements; i++)
	{
		temp.a = &aP[elements[i]*12];
		matrix_add(K,temp,K);
	}

	//computation of y
	m_elem y = measurement;
	for(int i=0; i< nr_of_elements; i++)
	{
		y -= M(x,elements[i],0);
	}

	matrix_mult_scalar(1/S,K,K);

	for(int i=0; i< 12; i++)
	{
		M(x,i,0) += M(K,i,0)*y;
	}

	//calculate H*P
	m_elem aHP[12]={0};
	temp = matrix_create(1,12,0);
	matrix_t HP = matrix_create(1,12,aHP);
	for(int i=0; i< nr_of_elements; i++)
	{
		temp.a = &aP[elements[i]*12];
		matrix_add(temp,HP,HP);
	}

	m_elem aKHP[12*12];
	matrix_t KHP = matrix_create(12,12,aKHP);
	matrix_mult(K,HP,KHP);
	matrix_sub(P,KHP,P);
}

void sequential_kalmanfilter_get_measurements(m_elem *measurements)
{
	measurements[0] = 0;//global_data.accel_raw.x;
	measurements[1] =  0;//global_data.accel_raw.y;
	measurements[2] =  0;//global_data.accel_raw.z;

	measurements[3] = 0;// (global_data.magnet_corrected.x ) ;
	measurements[4] = 0;// (global_data.magnet_corrected.y) ;
	measurements[5] = 0;// (global_data.magnet_corrected.z) ;

	measurements[6] = 0;// -(global_data.gyros_raw.x-global_data.param[PARAM_GYRO_OFFSET_X]) * 0.001008;
	measurements[7] = 0;// (global_data.gyros_raw.y-global_data.param[PARAM_GYRO_OFFSET_Y]) * 0.001008;
	measurements[8] = 0;// -(global_data.gyros_raw.z-global_data.param[PARAM_GYRO_OFFSET_Z]) * 0.001010;
}


void sequential_kalmanfilter(void)
{
	//uint64_t loop_start_time = sys_time_clock_get_time_usec();

	//allocate the measurement vector
	m_elem measurements[9];

	//Prediction step
	sequential_kalmanfilter_predict();

//	uint64_t loop_stop_predict = sys_time_clock_get_time_usec();

	//Read out the measurements
	sequential_kalmanfilter_get_measurements(measurements);

	//update step (sequential)
	for (int i = 0; i<6;i++)
	{
		int elements[1] = {i};
		sequential_kalmanfilter_update(elements,1,measurements[i],M(R_diag,i,0));
	}
	for (int i = 6; i<9;i++)
	{
		int elements[2] = {i, i+3};
		sequential_kalmanfilter_update(elements,2,measurements[i],M(R_diag,i,0));
	}

//	uint64_t loop_stop_update = sys_time_clock_get_time_usec();

	// save outputs
	float_vect3 kal_acc, kal_mag, kal_w0, kal_w;

	kal_acc.x = M(x,0,0);
	kal_acc.y = M(x,1,0);
	kal_acc.z = M(x,2,0);;

	kal_mag.x = M(x,3,0);
	kal_mag.y = M(x,4,0);;
	kal_mag.z = M(x,5,0);

	kal_w0.x = M(x,6,0);
	kal_w0.y = M(x,7,0);
	kal_w0.z = M(x,8,0);

	kal_w.x = M(x,9,0);
	kal_w.y = M(x,10,0);
	kal_w.z = M(x,11,0);







	float_vect3 x_n_b, y_n_b, z_n_b;
	z_n_b.x = -kal_acc.x;
	z_n_b.y = -kal_acc.y;
	z_n_b.z = -kal_acc.z;
	sequential_kalmanfilter_vect_norm(&z_n_b);
	sequential_kalmanfilter_vect_cross_product(&z_n_b, &kal_mag, &y_n_b);
	sequential_kalmanfilter_vect_norm(&y_n_b);

	sequential_kalmanfilter_vect_cross_product(&y_n_b, &z_n_b, &x_n_b);



	//save euler angles
//	global_data.attitude.x = atan2(z_n_b.y, z_n_b.z);
//	global_data.attitude.y = -asin(z_n_b.x);
//	global_data.attitude.z = atan2(y_n_b.x, x_n_b.x);

//	uint64_t loop_stop_time = sys_time_clock_get_time_usec();
	static int i = 10;
	if (i++ >= 50)
	{
		i = 0;
		//send the angles

//		kal_w.x = loop_stop_predict - loop_start_time;
//		kal_w.y = loop_stop_update - loop_stop_predict;
//		kal_w.z = loop_stop_time - loop_start_time;
//		debug_vect("kal_w0", kal_w0);
//		debug_vect("kal_w", kal_w);
	}



}
