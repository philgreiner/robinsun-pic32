/*!
 * \file odometry_gr1.cc
 * \brief Functions to update position using odometry
 */

#include "odometry_gr1.h"

NAMESPACE_INIT(ctrlGr1);

void odometry_estimate(CtrlStruct * cvs) {
    double deltaT = cvs->inputs->t - cvs->state->lastT;
    double drivenR, drivenL, R, b, drivenCenter, drivenCenter2, angleDiff, angleDiff2;
    int i, j;

    #ifdef SIMU_PROJECT
        R = 0.03; // Wheel radius [m]
        b = 0.225; // Distance between left and right wheels [m]
    #else
        #ifdef ROBINSUN
            int wheels = 0;
            R = (wheels) ? 0.0325 : 0.02215;
            b = (wheels) ? 0.129 : 0.22;
        #else
            R = 0.03;
            b = 0.225;
        #endif
    #endif

    // Total distance moved for each wheel (30mm = wheel radius)
    #ifdef MINIBOT
        drivenR = -cvs->inputs->r_wheel_speed * deltaT * R;
        drivenL = -cvs->inputs->l_wheel_speed * deltaT * R;
    #else
        #ifdef ROBINSUN
            drivenR = cvs->inputs->odo_r_speed * deltaT * R;
            drivenL = cvs->inputs->odo_l_speed * deltaT * R;
        #else
            drivenR = cvs->inputs->r_wheel_speed * deltaT * R;
            drivenL = cvs->inputs->l_wheel_speed * deltaT * R;
        #endif  
    #endif

    drivenCenter = (drivenR + drivenL) / 2.0;

    #ifdef MINIBOT
        angleDiff = 1.21 * (drivenR - drivenL) / b;
    #else
        angleDiff = (drivenR - drivenL) / b;
    #endif

    // We consider the whole movement is done in the new direction
    // Update of the mean
    if (fabs(cvs->inputs->odo_r_speed * R) < 1.0 && fabs(cvs->inputs->odo_l_speed * R) < 1.0) {
        cvs->state->position_odo[2] = cvs->state->position[2] + angleDiff;
        if (cvs->state->position_odo[2] > M_PI)
            cvs->state->position_odo[2] -= 2 * M_PI;
        if (cvs->state->position_odo[2] < -M_PI)
            cvs->state->position_odo[2] += 2 * M_PI;

        cvs->state->position_odo[0] = cvs->state->position[0] + drivenCenter * cos(cvs->state->position[2]);
        cvs->state->position_odo[1] = cvs->state->position[1] + drivenCenter * sin(cvs->state->position[2]);
    }

    #ifdef ODO_ONLY
        cvs->state->position[0] = cvs->state->position_odo[0];
        cvs->state->position[1] = cvs->state->position_odo[1];
        cvs->state->position[2] = cvs->state->position_odo[2];
    #endif

    /* ------------------------ Kalman Filter ------------------------ */
    #ifdef KALMAN

        angleDiff2 = cvs->state->position_odo[2] - angleDiff / 2.0;
        drivenCenter2 = drivenCenter / (2.0 * b);

        // Jacobian of f(x,u) wrt x and its transposed
        double Fx[3][3] = {
            {1.0, 0.0, -drivenCenter * sin(angleDiff2)},
            {0.0, 1.0, drivenCenter * cos(angleDiff2)},
            {0.0, 0.0, 1.0}};

        double Fx_transposed [3][3];
        for (i = 0; i < 3; i = i + 1)
            for (j = 0; j < 3; j = j + 1)
                Fx_transposed[i][j] = Fx[j][i];

        // Jacobian of f(x,u) wrt u and its transposed
        double Fu[3][2] = {
            {cos(angleDiff2) / 2.0 - drivenCenter2 * sin(angleDiff2), cos(angleDiff2) / 2.0 + drivenCenter2 * sin(angleDiff2)},
            {sin(angleDiff2) / 2.0 + drivenCenter2 * cos(angleDiff2), sin(angleDiff2) / 2.0 - drivenCenter2 * cos(angleDiff2)},
            {1.0 / b, -1.0 / b}};

        double Fu_transposed[2][3];
        for (i = 0; i < 2; i = i + 1)
            for (j = 0; j < 3; j = j + 1)
                Fu_transposed[i][j] = Fu[j][i];

        // Covariance of the noise associated to the motion model
        double Q[2][2] = {
            {cvs->param->kr * fabs(drivenR), 0.0},
            {0.0, cvs->param->kl * fabs(drivenL)}};

        // P_est = Fx * P * Fx' + Fu * Q * Fu'

        double intermediate1[3][3];
        double intermediate2[3][3];
        double intermediate3[3][2];
        double intermediate4[3][3];

        mult_matrix33_33(&(Fx[0]), &(cvs->state->covariance[0]), &(intermediate1[0]));
        mult_matrix33_33(&(intermediate1[0]), &(Fx_transposed[0]), &(intermediate2[0]));

        mult_matrix32_22(&(Fu[0]), &(Q[0]), &(intermediate3[0]));
        mult_matrix32_23(&(intermediate3[0]), &(Fu_transposed[0]), &(intermediate4[0]));

        // cvs->state->covariance_odo = Fx * cvs->state->covariance * Fx_transposed + Fu * Q * Fu_transposed;
        for (i = 0; i < 3; i = i + 1)
            for (j = 0; j < 3; j = j + 1)
                cvs->state->covariance_odo[i][j] = intermediate2[i][j] + intermediate4[i][j];

    #endif
}

/*
 * 	MULT_MATRIX33_33 :
 *		This function makes a multiplication with following size specs { (3x3) * (3x3) } = (3x3)
 */
void mult_matrix33_33(double A[3][3], double B[3][3], double result[3][3]) // TO DO
{
    // How to call function ? --> mult_matrix33(&A[0],&B[0],&result[0]);

    int i, j, k;

    for (i = 0; i < 3; i = i + 1)
        for (j = 0; j < 3; j = j + 1) {
            result[i][j] = 0.0;
            for (k = 0; k < 3; k = k + 1)
                result[i][j] = result[i][j] + A[i][k] * B[k][j];
        }
}

/*
 * 	MULT_MATRIX32_22 :
 *		This function makes a multiplication with following size specs { (3x2) * (2x2) } = (3x2)
 */
void mult_matrix32_22(double A[3][2], double B[2][2], double result[3][2]) {
    // How to call function ? --> mult_matrix33(&A[0],&B[0],&result[0]);

    int i, j, k;

    for (i = 0; i < 3; i = i + 1)
        for (j = 0; j < 2; j = j + 1) {
            result[i][j] = 0.0;
            for (k = 0; k < 2; k = k + 1)
                result[i][j] = result[i][j] + A[i][k] * B[k][j];
        }
}

/*
 * 	MULT_MATRIX32_23 :
 *		This function makes a multiplication with following size specs { (3x2) * (2x3) } = (3x3)
 */
void mult_matrix32_23(double A[3][2], double B[2][3], double result[3][3]) {
    // How to call function ? --> mult_matrix33(&A[0],&B[0],&result[0]);

    int i, j, k;

    for (i = 0; i < 3; i = i + 1)
        for (j = 0; j < 3; j = j + 1) {
            result[i][j] = 0.0;
            for (k = 0; k < 2; k = k + 1)
                result[i][j] = result[i][j] + A[i][k] * B[k][j];
        }
}

NAMESPACE_CLOSE();