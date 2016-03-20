/*!
* \file kalman_gr1.cc
* \brief Function to update position by merging odometry and triangulation
*/

#include "kalman_gr1.h"

NAMESPACE_INIT(ctrlGr1);

void kalman(CtrlStruct * cvs)
{
	kalmanStruct *kal;
	kal = (kalmanStruct*) malloc(sizeof(kalmanStruct));

	/* -------------- Prediction update ---------------- */
	odometry_estimate(cvs);

	/* -------------- Measurement update --------------- */
	measurement_update(cvs,kal);
}


void measurement_update(CtrlStruct * cvs, kalmanStruct * kal)
{
	int i,j;
	/*
		 Step 1 : Observation

			--> Acquisition of the results "z" from exteroceptive sensors ("alpha" and "r")
	*/
	triangulation(cvs);

	/*
		Step 2 : Measurement prediction

			--> Predict the values "z_pred" that the measurements shoud give ("alpha_pred" and "r_pred")
			--> Compute the Jacobian "H" (and its transposed)
	*/

	// Jacobian Matrix H = identity and its transposed is the same
	for (i = 0; i < 3; i=i+1)
		for (j = 0; j < 3; j=j+1)
			if (i==j)
			{
				kal->measure_covariance[i][j] = 1.0;
				kal->H[i][j] = 1.0;
			}
			else
			{
				kal->measure_covariance[i][j] = 0.0;
				kal->H[i][j] = 0.0;
			}

    /*
    	Step 3 : Matching

			--> Compute the innovation "v" defined as "v = z - z_pred = position_triang - position_odo"
			--> Then compute its covariance "sigma_inn"
    */

	for (i=0; i<3; i=i+1)
		kal->v[i] = cvs->state->position_triang[i] - cvs->state->position_odo[i];
	//printf("x_err: %f \t y_err: %f \t theta_err: %f \n",kal->v[0],kal->v[1],kal->v[2]);

    innovation_covariance_compute(cvs,kal);


    /* --------------  Step 4 : Estimation -------------- */

    // Kalman gain and its transposed
    kalman_gain_compute(cvs,kal);

    // Final update of the mean and covariance
    update_position(cvs,kal);

}

void innovation_covariance_compute(CtrlStruct * cvs, kalmanStruct * kal)
{
	int i,j;
	for(i=0; i<3; i=i+1)
		for(j=0; j<3; j=j+1)
		{
			kal->innovation_covariance[i][j] = cvs->state->covariance_odo[i][j] + kal->measure_covariance[i][j];
			//printf("innovation_covariance[%d][%d]: %f \n",i,j,kal->innovation_covariance[i][j]);
		}
}

void kalman_gain_compute(CtrlStruct * cvs, kalmanStruct * kal)
{
	int i,j;

	// Compute the inverse of the covariance
	inverse(&(kal->innovation_covariance[0]),kal);
	/*
	printf("Inverse covariance: \n");
	for (i = 0; i < 3; i=i+1)
	{
		for (j = 0; j < 3; j=j+1)
			printf("\t %f",kal->inverse_innovation_covariance[i][j]);
		printf("\n");
	}
	*/

	// Kt = Pt*H*sigma_inv = Pt*sigma_inv since H is the identity matrix
 	mult_matrix33_33(&(cvs->state->covariance_odo[0]),&(kal->inverse_innovation_covariance[0]),&(kal->Kt[0]));
 	/*
 	printf("Kt: \n");
	for (i = 0; i < 3; i=i+1)
	{
		for (j = 0; j < 3; j=j+1)
			printf("\t %f",kal->Kt[i][j]);
		printf("\n");
	}
	*/
}

void inverse(double matrix [3][3], kalmanStruct * kal)
{
	int i,j;
 	double determinant=0.0, var;

 	for(i=0;i<3;i=i+1)
 	{
 		var = 0.0;
 		var = matrix[1][(i+1)%3]*matrix[2][(i+2)%3];
 		var = var - matrix[1][(i+2)%3]*matrix[2][(i+1)%3];
     	determinant = determinant + (matrix[0][i] * var);
    }

	for(i=0;i<3;i=i+1)
    	for(j=0;j<3;j=j+1)
    	{
    		var = 0.0;
    		var = matrix[(i+1)%3][(j+1)%3] * matrix[(i+2)%3][(j+2)%3];
    		var = var - matrix[(i+1)%3][(j+2)%3]*matrix[(i+2)%3][(j+1)%3];
    		kal->inverse_innovation_covariance[j][i] = var / determinant;
    	}
}

void update_position(CtrlStruct * cvs, kalmanStruct * kal)
{
	int i,j;
	double intermediate0[3];

	// Update of the mean : x = x_odo + (K * v)
	mult_matrix33_31(&(kal->Kt[0]),kal->v,intermediate0);

	for(i=0;i<3;i=i+1)
		cvs->state->position[i] = cvs->state->position_odo[i] + intermediate0[i];
	/*
	printf("Intermediate: \n");
	for (i = 0; i < 3; i=i+1)
	{
			printf("\t %f",intermediate0[i]);
	}
	printf("\n");
	*/

	// Update of the covariance : cov = cov_odo - (K * sigma * K_transp)
	double intermediate1[3][3], intermediate2 [3][3];
	double Kt_transposed[3][3];

	mult_matrix33_33(&(kal->Kt[0]),&(kal->innovation_covariance[0]),&(intermediate1[0]));
	for(i=0;i<3;i=i+1)
    	for(j=0;j<3;j=j+1)
    		Kt_transposed[j][i] = kal->Kt[i][j];
    mult_matrix33_33(&(intermediate1[0]),&(Kt_transposed[0]),&(intermediate2[0]));

	for(i=0;i<3;i=i+1)
    	for(j=0;j<3;j=j+1)
    		cvs->state->covariance[i][j] = cvs->state->covariance_odo[i][j] - intermediate2[i][j];
    	/*
    printf("Covariance: \n");
	for (i = 0; i < 3; i=i+1)
	{
		for (j = 0; j < 3; j=j+1)
			printf("\t %f",cvs->state->covariance[i][j]);
		printf("\n");
	}
	*/
}

void mult_matrix33_31(double A[3][3], double B[3], double result[3])																		// TO DO
{
	// How to call function ? --> mult_matrix33(&A[0],B,result);

 	int i,j;

 	for(i=0;i<3;i=i+1)
 	{
 		result[i] = 0.0;
  		for(j=0;j<3;j=j+1)
  			result[i] = result[i] + A[i][j]*B[j];
   	}
}

NAMESPACE_CLOSE();
