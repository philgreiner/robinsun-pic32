/*!
* \file kalman_gr1.cc
* \brief Function to update position by merging odometry and triangulation
*/

#include "kalman_gr1.h"

NAMESPACE_INIT(ctrlGr1); 

void kalman(CtrlStruct * cvs)
{	
	/* -------------- Prediction update ---------------- */
	odometry_estimate(cvs);

	/* -------------- Measurement update --------------- */
	measurement_update(cvs);
}

void measurement_update(CtrlStruct * cvs)
{
	kalmanStruct *kal;

	/* 
		 Step 1 : Observation 
				
			--> Acquisition of the results "z" from exteroceptive sensors ("alpha" and "r")
	*/

	//triangulation(cvs);
	acquire_triang_measurements(cvs);
	

	/* 
		Step 2 : Measurement prediction  
	
			--> Predict the values "z_pred" that the measurements shoud give ("alpha_pred" and "r_pred")
			--> Compute the Jacobian "H" (and its transposed)
	*/
    
    int i,j,k;

    // Shortcuts
    double *position_odo = cvs->state->position_odo;
    double *alpha 		 = cvs->state->alpha;
    double *r   		 = cvs->state->r;

    for (i=0; i<3; i=i+1)
    {
        kal->alpha_pred[i] = alpha[i] - position_odo[2];
        kal->r_pred[i]     = r[i] - (position_odo[0]*cos(alpha[i]) + position_odo[1]*sin(alpha[i]));
    }

    // Jacobian Matrix H and its transposed Ht
    double H[6][3] =  {{0.0           , 0.0           ,-1.0},
          			   {-cos(alpha[0]), -sin(alpha[0]), 0.0},
     	  			   {0.0           , 0.0           ,-1.0},
                       {-cos(alpha[1]), -sin(alpha[1]), 0.0},
    	  			   {0.0           , 0.0           ,-1.0},
          			   {-cos(alpha[2]), -sin(alpha[2]), 0.0}};

	for(i=0; i<3; i=i+1)
		for(j=0; j<6; j=j+1)
		{
			kal->H[j][i] = H[j][i];									
			kal->H_transposed[i][j] = H[j][i];						
		}


    /* 
    	Step 3 : Matching  

			--> Compute the innovation "v" defined as "v = z - z_pred"
			--> Then compute its covariance "sigma_inn" 
    */
    
    for (i=0; i<3; i=i+1)
    {
        kal->v[0][i] = alpha[0] - kal->alpha_pred[i];
        kal->v[1][i] = r[0] - kal->r_pred[i];
        kal->v[2][i] = alpha[1] - kal->alpha_pred[i];
        kal->v[3][i] = r[1] - kal->r_pred[i];
        kal->v[4][i] = alpha[2] - kal->alpha_pred[i];
        kal->v[5][i] = r[2] - kal->r_pred[i];
    }

    measure_covariance_compute(cvs, kal);	// Compute measure_covariance (alias R)

    innovation_covariance_compute(cvs,kal);


    /* --------------  Step 4 : Estimation -------------- */

    // Kalman gain and its transposed
    kalman_gain_compute(cvs,kal);
    
    for(i=0; i<6; i=i+1)
		for(j=0; j<3; j=j+1)
			kal->Kt_transposed[i][j] = kal->Kt[j][i];

    // Final update of the mean and covariance
	for(i=0; i<3; i=i+1)
		for(j=0; j<3; j=j+1)
		{
			// cvs->state->position = cvs->state->position_odo + kal->Kt * kal->v;
			// cvs->state->covariance = cvs->state->covariance_odo - kal->Kt * kal->innovation_covariance * kal->Kt_transposed;
			cvs->state->position[i] = cvs->state->position_odo[i];
    		cvs->state->covariance[i][j] = cvs->state->covariance_odo[i][j];
		}
    
}

void acquire_triang_measurements(CtrlStruct * cvs) 															
{
	CtrlIn *ivs;
    ivs = cvs->inputs;

	/* If rising_index_fixed > falling_index_fixed, the laser is currently ON a beacon.
	the angles that we should look at are therefore the angles before the last rising edge (to look only at 'complete' beacons). */
	int rising_index = ivs->rising_index_fixed;
	int falling_index = ivs->falling_index_fixed;
	if (rising_index > falling_index) rising_index--;

	/* Now look at the three last detected beacons to compute position. */
	double rising[3], falling[3];
	int rising_index_1, rising_index_2;

	if(rising_index == 0){
        rising_index_1 = NB_STORE_EDGE - 1;
        rising_index_2 = NB_STORE_EDGE - 2;
	}
	else if (rising_index == 1){
        rising_index_1 = 0;
        rising_index_2 = NB_STORE_EDGE - 1;
	}
	else {
        rising_index_1 = rising_index - 1;
        rising_index_2 = rising_index - 2;
	}

	rising[0] = ivs->last_rising_fixed[rising_index_2];
	rising[1] = ivs->last_rising_fixed[rising_index_1];
	rising[2] = ivs->last_rising_fixed[rising_index];
	falling[0] = ivs->last_falling_fixed[rising_index_2];
	falling[1] = ivs->last_falling_fixed[rising_index_1];
	falling[2] = ivs->last_falling_fixed[rising_index];

	/* Define the angle of each beacon as the mean between rising and falling edge */
	double alpha[3];
	alpha[0] = (rising[0] + falling[0]) / 2.0;
	alpha[1] = (rising[1] + falling[1]) / 2.0;
	alpha[2] = (rising[2] + falling[2]) / 2.0;

    // Completion of the Kalman structure
    double coveredAngle[3];
    coveredAngle[0] = (rising[0] - falling[0]) / 2.0;
    coveredAngle[1] = (rising[1] - falling[1]) / 2.0;
    coveredAngle[2] = (rising[2] - falling[2]) / 2.0;
    cvs->state->alpha[0] = alpha[0];
    cvs->state->alpha[1] = alpha[1];
    cvs->state->alpha[2] = alpha[2];
    cvs->state->r[0] = cvs->param->R_beacon / sin(coveredAngle[0]/2.0);
    cvs->state->r[1] = cvs->param->R_beacon / sin(coveredAngle[1]/2.0);
    cvs->state->r[2] = cvs->param->R_beacon / sin(coveredAngle[2]/2.0);
}

void measure_covariance_compute(CtrlStruct * cvs, kalmanStruct * kal)
{
	double sigma_aa[3], sigma_ar[3], sigma_ra[3], sigma_rr[3];
	int i,j;

	double *alpha = cvs->state->alpha;
	double *r     = cvs->state->r;
	double *alpha_pred = kal->alpha_pred;
	double *r_pred     = kal->r_pred;

	/* --------- Fill the non-diagonal matrices with 0's ----------- */

	for (i = 2; i < 6 ; i++)
	{
		// Fill 0's from lines 0 and 1
		kal->measure_covariance[0][i] = 0.0;
		kal->measure_covariance[1][i] = 0.0;
	}
	for (i = 0; i < 6 ; i++)
	{
		// Fill 0's from lines 2 and 3
		kal->measure_covariance[2][i] = 0.0;
		kal->measure_covariance[3][i] = 0.0;
		if (i == 1)
			i = i+2;
	}
	for (i = 0; i < 4 ; i++)
	{
		// Fill 0's from lines 4 and 5
		kal->measure_covariance[4][i] = 0.0;
		kal->measure_covariance[5][i] = 0.0;
	}

	/* --------- Fill the diagonal matrices with 2x2 submatrices ----------- */

	// Computation of the coefficients
	double var;						
	for (i = 0; i < 3; i++)
	{
		var = (alpha[i] - alpha_pred[i]);
		sigma_aa[i] = var*var;

		sigma_ar[i] = (alpha[i] - alpha_pred[i])*(r[i] - r_pred[i]);
		sigma_ra[i] = sigma_ar[i];

		var = (r[i] - r_pred[i]);
		sigma_rr[i] = var*var;
	}
	
	// Fill the matrix
	kal->measure_covariance[0][0] = sigma_aa[0];
	kal->measure_covariance[2][2] = sigma_aa[1];
	kal->measure_covariance[4][4] = sigma_aa[2];

	kal->measure_covariance[0][1] = sigma_ar[0];
	kal->measure_covariance[2][3] = sigma_ar[1];
	kal->measure_covariance[4][5] = sigma_ar[2];

	kal->measure_covariance[1][0] = sigma_ra[0];
	kal->measure_covariance[3][2] = sigma_ra[1];
	kal->measure_covariance[5][4] = sigma_ra[2];

	kal->measure_covariance[1][1] = sigma_rr[0];
	kal->measure_covariance[3][3] = sigma_rr[1];
	kal->measure_covariance[5][5] = sigma_rr[2];

}

void innovation_covariance_compute(CtrlStruct * cvs, kalmanStruct * kal)						
{
	int i,j;
	for(i=0; i<6; i=i+1)
		for(j=0; j<6; j=j+1)
			kal->innovation_covariance[i][j] = kal->H[i][j];					// TO BE MODIFIED WHEN MULTIPLICATION IS OK
}	// kal->innovation_covariance = kal->H * cvs->state->covariance_odo * kal->H_transposed + kal->measure_covariance;		
 
void kalman_gain_compute(CtrlStruct * cvs, kalmanStruct * kal)
{
	int i,j;
	for(i=0; i<6; i=i+1)
		for(j=0; j<6; j=j+1)
			kal->Kt[i][j] = cvs->state->covariance_odo[i][j];					// TO BE MODIFIED WHEN MULTIPLICATION IS OK
	//kal->Kt = cvs->state->covariance_odo * kal->H_transposed * inverse(kal);					
}

void inverse(double matrix [6][6], kalmanStruct * kal)									// TO DO
{	
	// TO IMPLEMENT
}

void mult_matrix()																		// TO DO
{
	// TO IMPLEMENT	
}


NAMESPACE_CLOSE();