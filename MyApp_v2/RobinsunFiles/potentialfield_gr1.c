/*!
* \file potentialfield_gr1.cc
* \brief File description
*/

#include "namespace_ctrl.h"
#include "CtrlStruct_gr1.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "potentialfield_gr1.h"
#include "triangulation_gr1.h"

#ifdef SIMU_PROJECT
// Simulation and results
    #include "robot_id.h"
#endif

NAMESPACE_INIT(ctrlGr1); // where X should be replaced by your group number

#ifdef POTENTIAL

/*! \brief Allocates the coordinates of an obstacle edge
*
* \param[in] qstart, qend, step, start_index, qconst : starting coordinate, ending coordinate, value of the step,
*                                                      index to start in array obstacle, value of the constant coordinate
* \param[out] outputs written in obstacle;
*/
int edge_allocate(double *obstacle, double qstart, double qend, double step, double qconst, int XY, int start_index)
{
	int n_iter = (int) floor(fabs(qstart - qend) / step);     // Number of iterations to perform
	int index = start_index;

	if (qstart > qend) {
		step = -step;
	}

	int i;
	for (i = 0; i<n_iter + 1; i = i + 1)
	{
		if (XY == 0) { // The variable coordinate is along x
			obstacle[index] = qstart + (i*step);
			obstacle[index + 1] = qconst;
		}
		else {
			obstacle[index + 1] = qstart + (i*step);
			obstacle[index] = qconst;
		}
		index = index + 2;
	}
	return index - 2;
}

/*! \brief Allocates the coordinates of a diagonal (45 deg) obstacle edge
*
* \param[in] qstartx, qendx, qstarty, qendy, step, start_index: starting coordinate, ending coordinate, value of the step,
*                                                      index to start in array obstacle, value of the constant coordinate
* \param[out] outputs written in obstacle;
*/
int diag_allocate(double *obstacle, double qstartx, double qendx, double qstarty, double qendy, double step, int start_index)
{
    double stepx, stepy;
    int n_iter = (int) floor(fabs(qstartx - qendx) / step);     // Number of iterations to perform
	int index = start_index;

	if (qstartx > qendx)    stepx = -step;
	else                    stepx = step;

	if (qstarty > qendy)    stepy = -step;
	else                    stepy = step;

	int i;
	for (i = 0; i<n_iter + 1; i = i + 1)
	{
        obstacle[index] = qstartx + (i*stepx);
        obstacle[index + 1] = qstarty + (i*stepy);
		index = index + 2;
	}

	return index - 2;
}

void potential_Field_Init(CtrlStruct *cvs)
{
    /* Allocation of the obstacles coordinates */
	cvs->param->nb_edges = 147;
	double* edges = cvs->param->obstacle_edges;
	int start_ind = 0;
	int last_ind = 0;
	last_ind = edge_allocate(edges, -1.25, 1.25, 0.1, 1.0, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 1.0, 0.75, 0.05, 1.25, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 1.25, 1.5, 0.05, 0.75, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 0.75, -1.0, 0.05, 1.5, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 1.5, 0.7, 0.1, -1.0, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -1.0, -0.8, 0.1, 0.7, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 0.7, -0.7, 0.1, -0.8, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -0.8, -1.0, 0.1, -0.7, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -0.7, -1.5, 0.1, -1.0, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -1.0, 0.75, 0.05, -1.5, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -1.5, -1.25, 0.05, 0.75, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 0.75, 1.0, 0.05, -1.25, 0, start_ind);

    cvs->param->nb_center = 49;
    double* center = cvs->param->obstacle_center;

	start_ind = 0;
	last_ind = 0;
	last_ind = edge_allocate(center, 0.65, -0.65, 0.1, -0.28, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(center, -0.28, -0.2, 0.02, -0.65, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(center, -0.65, 0.65, 0.1, -0.2, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(center, -0.2, -0.28, 0.02, 0.65, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(center, -0.25, 0.35, 0.1, 0.05, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(center, -0.05, 0.05, 0.05, 0.35, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(center, 0.35, -0.25, 0.1, -0.05, 0, start_ind);
	start_ind = last_ind;

	cvs->param->nb_purple = 14;
    double* purple = cvs->param->obstacle_purpleZone;
    start_ind = 0;
	last_ind = 0;
	last_ind = diag_allocate(purple, -0.2, 0.45, -0.65, 0.0, 0.05, start_ind);

	cvs->param->nb_green = 14;
    double* green = cvs->param->obstacle_greenZone;
    start_ind = 0;
	last_ind = 0;
	last_ind = diag_allocate(green, -0.2, 0.45, 0.65, 0.0, 0.05, start_ind);

	/* Moving obstacles (opponents) */
    cvs->param->nb_robot1 = 21;
    cvs->param->k_robot1 = 17.0;
    cvs->param->rho_robot1 = 0.2;
    cvs->param->nb_robot2 = 21;
    cvs->param->k_robot2 = 17.0;
    cvs->param->rho_robot2 = 0.2;

	/* Initialization of the parameters of the potential field path planning */
	cvs->param->k_edge = 8.5;
	cvs->param->rho_edge = 0.25;
	cvs->param->k_center = 13.5;
	cvs->param->rho_center = 0.25;
	cvs->param->k_purple = 15;
	cvs->param->rho_purple = 0.3;
	cvs->param->k_green = 15;
	cvs->param->rho_green = 0.3;

	cvs->param->rho_att = 0.1;
	cvs->param->k_att_conic = 200.0;
	cvs->param->k_att_quad = 200.0/0.1;

	cvs->param->K_SpeedX = 0.035;
	cvs->param->K_SpeedRot = 7.5;

    cvs->param->Kp_pot = 3.57;
	cvs->param->Ki_pot = 0.85;

	cvs->state->errorAngle = 0.0;
	cvs->state->lastT_pot = 0.0;
}

/*! \brief Calculate the minimal distance between the robot and an obstacle
*
* \param[in] position, obstacle : position of the robot, obstacle coordinates
* \param[out] outputs written in coord, Euclidian distance;
*/
double min_Distance(double *position, double *obstacle, double *coord, int nbr_elements)
{
	double d = 0.0; //Euclidian distance
	double dist = 0.0;
	double x = position[0];
	double y = position[1];
	double x_obst, y_obst;

	int index = 0;
	int i;
	for (i = 0; i<nbr_elements; i = i + 1)
	{
		x_obst = obstacle[index];
		y_obst = obstacle[index + 1];

		if (i == 0) {
			dist = sqrt(((x - x_obst)*(x - x_obst)) + ((y - y_obst)*(y - y_obst)));
			d = dist;
			coord[0] = x_obst;
			coord[1] = y_obst;
		}
		else {
			dist = sqrt(((x - x_obst)*(x - x_obst)) + ((y - y_obst)*(y - y_obst)));
			if (dist<d) {
				d = dist;
				coord[0] = x_obst;
				coord[1] = y_obst;
			}
		}
		index = index + 2;
	}
	return d;
}

/*! \brief Calculate the angle between the robot orientation and the closest point of the obstacle
*
* \param[in] position, coord_obstacle : position of the robot, obstacle coordinates
* \param[out] angle in rads
*/
double angle_Obstacle(double *position, double *coord_obstacle)
{
    double angle = 0.0;         // Absolute orientation of the obstacle
    double rel_angle = 0.0;     // Orientation of the obstacle wrt the robot
    double delta_x = coord_obstacle[0] - position[0];
    double delta_y = coord_obstacle[1] - position[1];

    if((delta_x < 0)&&(delta_y > 0)) {              // 2nd quadrant
            angle = atan(delta_y/delta_x) + M_PI;
        }
        else if ((delta_x < 0)&&(delta_y < 0)) {    // 3rd quadrant
            angle = atan(delta_y/delta_x) - M_PI;
        }
        else {
            angle = atan(delta_y/delta_x);
        }

        rel_angle = angle - position[2];
        if (rel_angle > M_PI)          rel_angle = rel_angle - (2*M_PI);
        else if (rel_angle < -M_PI)    rel_angle = rel_angle + (2*M_PI);

        return rel_angle;
}

/*! \brief Calculate the reference speed using a potential field
*
* \param[in] position : position of the robot
* \param[out] omegaref : Reference speed of the wheels
*/
void potential_Field(CtrlStruct *cvs)
{
    CtrlIn *ivs = cvs->inputs;
	double x = cvs->state->position_odo[0];
	double y = cvs->state->position_odo[1];
	double orientation = cvs->state->position_odo[2];

	double x_goal = cvs->state->goal_position[0];
	double y_goal = cvs->state->goal_position[1];
	double d = sqrt((x - x_goal)*(x - x_goal) + (y - y_goal)*(y - y_goal));

	/* Compute the smallest distance and the coordinates of the closest point in the edge and the central obstacle */
	double d_edge;
	double coord_edge[2];
	double d_center;
	double coord_center[2];
	double d_purple;
	double coord_purple[2];
	double d_green;
	double coord_green[2];

	d_edge = min_Distance(cvs->state->position, cvs->param->obstacle_edges, coord_edge, cvs->param->nb_edges);
	d_center = min_Distance(cvs->state->position, cvs->param->obstacle_center, coord_center, cvs->param->nb_center);
	d_purple = min_Distance(cvs->state->position, cvs->param->obstacle_purpleZone, coord_purple, cvs->param->nb_purple);
	d_green = min_Distance(cvs->state->position, cvs->param->obstacle_greenZone, coord_green, cvs->param->nb_green);

	#ifdef SIMU_PROJECT
        int n_robots = cvs->inputs->nb_opponents;
        double d_robot1;
        double coord_robot1[2];
        if(n_robots > 0) {
            // Robot coordinates
            double xrobot1, yrobot1;
            xrobot1 = cvs->state->opponent_position[0];
            yrobot1 = cvs->state->opponent_position[1];

            // Dynamic obstacle definition
            double* robot1 = cvs->param->obstacle_robot1;
            int start_ind = 0;
            int last_ind = 0;
            last_ind = edge_allocate(robot1, xrobot1-0.125, xrobot1+0.125, 0.05, yrobot1-0.125, 0, start_ind);
            start_ind = last_ind;
            last_ind = edge_allocate(robot1, yrobot1-0.125, yrobot1+0.125, 0.05, xrobot1+0.125, 1, start_ind);
            start_ind = last_ind;
            last_ind = edge_allocate(robot1, xrobot1+0.125, xrobot1-0.125, 0.05, yrobot1+0.125, 0, start_ind);
            start_ind = last_ind;
            last_ind = edge_allocate(robot1, yrobot1+0.125, yrobot1-0.125, 0.05, xrobot1-0.125, 1, start_ind);

            //Obstacle distance and coordinates
            d_robot1 = min_Distance(cvs->state->position, cvs->param->obstacle_robot1, coord_robot1, cvs->param->nb_robot1);
        }

        double d_robot2;
        double coord_robot2[2];
        if(n_robots > 1) {
            // Robot coordinates
            double xrobot2, yrobot2;
            xrobot2 = cvs->state->opponent_position[2];
            yrobot2 = cvs->state->opponent_position[3];

            // Dynamic obstacle definition
            double* robot2 = cvs->param->obstacle_robot2;
            int start_ind = 0;
            int last_ind = 0;
            last_ind = edge_allocate(robot2, xrobot2-0.125, xrobot2+0.125, 0.05, yrobot2-0.125, 0, start_ind);
            start_ind = last_ind;
            last_ind = edge_allocate(robot2, yrobot2-0.125, yrobot2+0.125, 0.05, xrobot2+0.125, 1, start_ind);
            start_ind = last_ind;
            last_ind = edge_allocate(robot2, xrobot2+0.125, xrobot2-0.125, 0.05, yrobot2+0.125, 0, start_ind);
            start_ind = last_ind;
            last_ind = edge_allocate(robot2, yrobot2+0.125, yrobot2-0.125, 0.05, xrobot2-0.125, 1, start_ind);

            //Obstacle distance and coordinates
            d_robot2 = min_Distance(cvs->state->position, cvs->param->obstacle_robot2, coord_robot2, cvs->param->nb_robot2);
        }
	#endif

	/* Calculate the forces using the gradient of the potential field */
	double F_x_att, F_x_edge, F_x_center, F_x_purple, F_x_green;
	double F_y_att, F_y_edge, F_y_center, F_y_purple, F_y_green;
	double k_att_conic = cvs->param->k_att_conic;
	double k_att_quad = cvs->param->k_att_quad;
	double rho_att = cvs->param->rho_att;
	double k_edge = cvs->param->k_edge;
	double rho_edge = cvs->param->rho_edge;
	double k_center = cvs->param->k_center;
	double rho_center = cvs->param->rho_center;
	double k_purple = cvs->param->k_purple;
	double rho_purple = cvs->param->rho_purple;
	double k_green = cvs->param->k_green;
	double rho_green = cvs->param->rho_green;
	double k_robot1 = cvs->param->k_robot1;
	double rho_robot1 = cvs->param->rho_robot1;
	double k_robot2 = cvs->param->k_robot2;
	double rho_robot2 = cvs->param->rho_robot2;

	#ifdef SIMU_PROJECT
        int purple_team = ((cvs->inputs->robot_id == ROBOT_B)||(cvs->inputs->robot_id == ROBOT_R));
        int green_team = ((cvs->inputs->robot_id == ROBOT_Y)||(cvs->inputs->robot_id == ROBOT_W));
    #else
        int purple_team = 1;
        int green_team = 0;
    #endif

    int GoToBase = (cvs->state->next_objective == BASE) || (cvs->state->objectives_on_robot == 0);

    // Variation of the repulsive fields according to the position
    if((cvs->state->next_objective == BASE) && (x > -0.05)) {
        cvs->param->k_center = 11.5;
        cvs->param->rho_center = 0.1;
    }
    else if (x > 0.3) {
        cvs->param->k_center = 9.0;
        cvs->param->rho_center = 0.125;
    }
    else {
        cvs->param->k_center = 12.5;
        cvs->param->rho_center = 0.275;
    }

    if((cvs->state->next_objective == BASE) && (x > -0.05)) {
        cvs->param->k_edge = 11.0;
        cvs->param->rho_edge = 0.15;
    }
    else if((x > 0.4) && (fabs(y) > 0.75)) {
        cvs->param->k_edge = 11.0;
        cvs->param->rho_edge = 0.225;
    }
    else {
        cvs->param->k_edge = 9.0;
        cvs->param->rho_edge = 0.325;
    }

    // Forces
	if (d < rho_att) {
		F_x_att = -k_att_quad * (x - x_goal);
		F_y_att = -k_att_quad * (y - y_goal);
	}
	else {
		F_x_att = -k_att_conic * (x - x_goal) / d;
		F_y_att = -k_att_conic * (y - y_goal) / d;
	}

	if (d_edge < rho_edge) {
		F_x_edge = k_edge*((1 / d_edge) - (1 / rho_edge))*((x - coord_edge[0]) / (d_edge*d_edge*d_edge));
		F_y_edge = k_edge*((1 / d_edge) - (1 / rho_edge))*((y - coord_edge[1]) / (d_edge*d_edge*d_edge));
	}
	else {
		F_x_edge = 0.0;
		F_y_edge = 0.0;
	}

	if (d_center < rho_center) {
		F_x_center = k_center*((1 / d_center) - (1 / rho_center))*((x - coord_center[0]) / (d_center*d_center*d_center));
		F_y_center = k_center*((1 / d_center) - (1 / rho_center))*((y - coord_center[1]) / (d_center*d_center*d_center));
	}
	else {
		F_x_center = 0.0;
		F_y_center = 0.0;
	}

	if ((d_purple < rho_purple) && (green_team || (purple_team && !GoToBase)))
    {
		F_x_purple = k_purple*((1 / d_purple) - (1 / rho_purple))*((x - coord_purple[0]) / (d_purple*d_purple*d_purple));
		F_y_purple = k_purple*((1 / d_purple) - (1 / rho_purple))*((y - coord_purple[1]) / (d_purple*d_purple*d_purple));
	}
	else {
		F_x_purple = 0.0;
		F_y_purple = 0.0;
	}

	if ((d_green < rho_green) && (purple_team || (green_team && !GoToBase)))
    {
		F_x_green = k_green*((1 / d_green) - (1 / rho_green))*((x - coord_green[0]) / (d_green*d_green*d_green));
		F_y_green = k_green*((1 / d_green) - (1 / rho_green))*((y - coord_green[1]) / (d_green*d_green*d_green));
	}
	else {
		F_x_green = 0.0;
		F_y_green = 0.0;
	}

	#ifdef SIMU_PROJECT
        double F_x_robot1, F_y_robot1;
        if(n_robots > 0) {
            if (d_robot1 < rho_robot1)
            {
                printf("Robot obstacle \n");
                F_x_robot1 = k_robot1*((1 / d_robot1) - (1 / rho_robot1))*((x - coord_robot1[0]) / (d_robot1*d_robot1*d_robot1));
                F_y_robot1 = k_robot1*((1 / d_robot1) - (1 / rho_robot1))*((y - coord_robot1[1]) / (d_robot1*d_robot1*d_robot1));
            }
            else {
                F_x_robot1 = 0.0;
                F_y_robot1 = 0.0;
            }
        }
        else {
                F_x_robot1 = 0.0;
                F_y_robot1 = 0.0;
        }

        double F_x_robot2, F_y_robot2;
        if(n_robots > 1) {
            if (d_robot2 < rho_robot2)
            {
                F_x_robot2 = k_robot2*((1 / d_robot2) - (1 / rho_robot2))*((x - coord_robot2[0]) / (d_robot2*d_robot2*d_robot2));
                F_y_robot2 = k_robot2*((1 / d_robot2) - (1 / rho_robot2))*((y - coord_robot2[1]) / (d_robot2*d_robot2*d_robot2));
            }
            else {
                F_x_robot2 = 0.0;
                F_y_robot2 = 0.0;
            }
        }
        else {
                F_x_robot2 = 0.0;
                F_y_robot2 = 0.0;
            }
    #endif

	double Fx, Fy;      // Forces in the inertial frame
	double FxR, FyR;    // Forces in the robot frame
	double K_SpeedX = cvs->param->K_SpeedX;
	double K_SpeedRot = cvs->param->K_SpeedRot;
	double angle;
	double omega, vlin, omega_R, omega_L;

	Fx = F_x_att + F_x_edge + F_x_center + F_x_purple + F_x_green + F_x_robot1 + F_x_robot2;
	Fy = F_y_att + F_y_edge + F_y_center + F_y_purple + F_y_green + F_y_robot1 + F_y_robot2;

	FxR = Fx*cos(orientation) + Fy*sin(orientation);
	FyR = -Fx*sin(orientation) + Fy*cos(orientation);

	if((FxR < 0)&&(FyR > 0)) { // 2nd quadrant
        angle = atan(FyR/FxR) + M_PI;
    }
    else if ((FxR < 0)&&(FyR < 0)) { // 3rd quadrant
        angle = atan(FyR/FxR) - M_PI;
    }
    else {
        angle = atan(FyR/FxR);
    }

    //PI controller for the orientation
    if (ivs->t < 0.1) {
        cvs->state->errorAngle = 0.0;
        angle = 0;
    }
    else if(fabs(angle) < M_PI){
       cvs->state->errorAngle += -angle*(ivs->t - cvs->state->lastT_pot);
    }

    cvs->state->lastT_pot = ivs->t;

	//Limit the integral error (anti-windup)
	cvs->state->errorAngle = (cvs->state->errorAngle*cvs->param->Ki_pot> 3*M_PI) ? (3*M_PI/cvs->param->Ki_pot) : (cvs->state->errorAngle);
	cvs->state->errorAngle = (cvs->state->errorAngle*cvs->param->Ki_pot< -3*M_PI) ? (-3*M_PI/cvs->param->Ki_pot) : (cvs->state->errorAngle);

	#ifdef POTENTIAL_CONSOLE
        char msg[1024];
        sprintf(msg, "x: %.3f; y: %.3f; theta: %.3f; angle = %.3f, errorAngle: %.3f\n", x, y, orientation*(180.0/M_PI), angle*(180.0/M_PI), cvs->state->errorAngle*(180.0/M_PI));
        MyConsole_SendMsg(msg);
    #endif

	// PI control
	omega = (-angle*cvs->param->Kp_pot) + cvs->state->errorAngle*cvs->param->Ki_pot;
	omega = (omega > 3*M_PI) ? (3*M_PI) : (omega);
	omega = (omega < -3*M_PI) ? (-3*M_PI) : (omega);

	vlin = 3.5*M_PI*(fabs(FxR)/200.0);
	vlin = (vlin > 2*M_PI) ? (2*M_PI) : (vlin);
	vlin = (vlin < -2*M_PI) ? (-2*M_PI) : (vlin);
	if(fabs(vlin) < M_PI) {
        vlin = M_PI;
	}

    if(FxR < 0)
    {
        omega_R = M_PI_4 - omega;
        omega_L = M_PI_4 + omega;
    }
    else {
        omega_R = vlin - omega;
        omega_L = vlin + omega;
    }

	if (isnan(omega_R) || (d < 0.05))
	{
		cvs->state->omegaref[R_ID] = 0.0;
	}
	else {
		cvs->state->omegaref[R_ID] = lowpass(cvs->state->omegaref[R_ID], omega_R, TAU_POT, cvs);
	}

	if (isnan(omega_L) || (d < 0.05))
	{
		cvs->state->omegaref[L_ID] = 0.0;
	}
	else {
		cvs->state->omegaref[L_ID] = lowpass(cvs->state->omegaref[L_ID], omega_L, TAU_POT, cvs);
	}
}

#endif

NAMESPACE_CLOSE();